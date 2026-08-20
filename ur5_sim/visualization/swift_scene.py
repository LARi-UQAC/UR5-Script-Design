"""
ur5_sim/visualization/swift_scene.py — Gestion de la scène 3D Swift.

Responsabilités :
  - Lancement du backend Swift (navigateur WebGL).
  - Attachement des maillages fin d'effecteur (FT-300 + 2F-85 + Support_doigt).
  - Triade de repère base, marqueur TCP sphère.
  - Cycle de vie de la scène : montage, battement WebSocket, poussée de
    pose par frame, réouverture de l'onglet, démontage.

Toutes ces fonctions sont stateless ; elles retournent des handles que
l'appelant conserve pour les mises à jour de pose à chaque frame.
"""

from __future__ import annotations

import shutil
import threading
import time as _time
import webbrowser
from pathlib import Path

import numpy as np
import roboticstoolbox as rtb

try:
    from swift import Swift
    _SWIFT_AVAILABLE = True
except ImportError:
    Swift = None  # type: ignore[assignment]
    _SWIFT_AVAILABLE = False

try:
    import trimesh as _trimesh
    _HAS_TRIMESH = True
except ImportError:
    _HAS_TRIMESH = False

from ur5_sim.config import (
    END_LINK,
    ROBOTIQ_MESH_DIR,
    SUPPORT_TOOL_LOCAL_RPY,
    SUPPORT_TOOL_LOCAL_XYZ,
    SUPPORT_TOOL_MESH_PATH,
)
# Ré-exportés : le socket UDP a migré vers ipc_live.py, mais ces deux noms
# restent importables depuis ce module (tests/test_udp_ipc.py les y prend).
from ur5_sim.visualization.ipc_live import (  # noqa: F401
    get_tcp_live_socket,
    send_tcp_live,
)
from ur5_sim.kinematics.transforms import link_world_T, se3, tcp_tool_offset
from ur5_sim.meshes import build_endeffector_meshes


# ---------------------------------------------------------------------------
# Lancement Swift
# ---------------------------------------------------------------------------

def launch_swift_env(robot: rtb.Robot):
    """Lance Swift et enregistre le robot. Retourne l'env ou None."""
    if not _SWIFT_AVAILABLE or Swift is None:
        print("[swift_scene] swift non installé — mode 2D seulement.")
        return None
    try:
        env = Swift()
        env.launch(realtime=True, headless=False)
        env.add(robot)
        _time.sleep(1.5)
        try:
            env.step(0)
        except Exception:
            pass
        try:
            base_z = 0.4
            env.set_camera_pose([2.0, 2.0, base_z], [0.0, 0.0, base_z])
        except Exception as exc:
            print(f"[swift_scene] set_camera_pose échoué ({exc!r})")
        return env
    except Exception as exc:
        print(f"[swift_scene] Swift launch échoué ({exc!r}) — mode 2D.")
        return None


# ---------------------------------------------------------------------------
# Attachement des maillages fin d'effecteur
# ---------------------------------------------------------------------------

def _attach_endeffector_direct_meshes(robot: rtb.Robot, env, sg) -> list[dict]:
    """Chemin de repli : attache les CAD RobotIQ directement sans trimesh."""
    mesh_dir = ROBOTIQ_MESH_DIR
    if not mesh_dir.exists():
        print(f"[swift_scene] Répertoire RobotIQ introuvable ({mesh_dir})")
        return []

    handles: list[dict] = []
    attach_link = "tool0"
    mount_T = np.eye(4)
    q_ref = getattr(robot, "q", None)
    if q_ref is None or len(q_ref) == 0:
        q_ref = robot.qr
    tool_T = link_world_T(robot, q_ref, attach_link)

    ft_top_z = 0.0348
    coupling_thickness = 0.0139
    gripper_base_z = ft_top_z + coupling_thickness

    def _add_mesh(path: Path, T_local: np.ndarray, scale=None):
        if not path.exists():
            return
        kwargs = {}
        if scale is not None:
            kwargs["scale"] = scale
        shape = sg.Mesh(filename=str(path), **kwargs)
        T_eff = mount_T @ T_local
        shape.T = tool_T @ T_eff
        env.add(shape, collision_alpha=1.0)
        handles.append({"shape": shape, "name": attach_link, "T_local": T_eff})

    _add_mesh(mesh_dir / "robotiq_ft300.STL", np.eye(4), scale=[0.001, 0.001, 0.001])
    _add_mesh(
        mesh_dir / "robotiq_ft300-G-062-COUPLING_G-50-4M6-1D6_20181119.STL",
        se3([0.0, 0.0, ft_top_z]),
        scale=[0.001, 0.001, 0.001],
    )
    _add_mesh(
        mesh_dir / "robotiq_gripper_coupling.stl",
        se3([0.0, 0.0, ft_top_z + coupling_thickness / 2 + 0.007]),
        scale=[0.001, 0.001, 0.001],
    )
    T_to_base = se3([0.0, 0.0, gripper_base_z])
    _add_mesh(mesh_dir / "robotiq_arg2f_85_base_link.dae", T_to_base)
    if SUPPORT_TOOL_MESH_PATH.exists():
        T_support = T_to_base @ se3(SUPPORT_TOOL_LOCAL_XYZ, SUPPORT_TOOL_LOCAL_RPY)
        _add_mesh(SUPPORT_TOOL_MESH_PATH, T_support, scale=[0.001, 0.001, 0.001])

    pi = float(np.pi)
    for reflect, side_rot in [(1, pi), (-1, 0.0)]:
        T_ok = T_to_base @ se3([0, reflect * -0.0306011, 0.054904], [0, 0, side_rot])
        T_of = T_ok @ se3([0, 0.0315, -0.0041])
        T_if = T_of @ se3([0, 0.0061, 0.0471])
        T_pad = T_if @ se3([0, -0.0220, 0.0324])
        T_ik = T_to_base @ se3([0, reflect * -0.0127, 0.06142], [0, 0, side_rot])
        _add_mesh(mesh_dir / "robotiq_arg2f_85_outer_knuckle.dae", T_ok)
        _add_mesh(mesh_dir / "robotiq_arg2f_85_outer_finger.dae", T_of)
        _add_mesh(mesh_dir / "robotiq_arg2f_85_inner_finger.dae", T_if)
        _add_mesh(mesh_dir / "robotiq_arg2f_85_pad.dae", T_pad)
        _add_mesh(mesh_dir / "robotiq_arg2f_85_inner_knuckle.dae", T_ik)

    return handles


def attach_endeffector_to_swift(
    robot: rtb.Robot, env
) -> tuple[list[dict], object]:
    """Attache FT-300 + 2F-85. Retourne (handles, temp_dir)."""
    if env is None:
        return [], None
    try:
        import spatialgeometry as sg
    except Exception as exc:
        print(f"[swift_scene] spatialgeometry indisponible ({exc!r})")
        return [], None
    import shutil
    import tempfile

    attach_link = "tool0"
    mount_T = np.eye(4)
    mesh_dicts = build_endeffector_meshes(attach_link=attach_link)
    if not mesh_dicts:
        print("[swift_scene] Pipeline trimesh vide — tentative CAD directe.")
        handles = _attach_endeffector_direct_meshes(robot, env, sg)
        if handles:
            print(f"[swift_scene] {len(handles)} pièces ajoutées (CAD directe).")
            return handles, None
        print("[swift_scene] Aucun maillage fin d'effecteur généré.")
        return [], None

    temp_dir = Path(tempfile.mkdtemp(prefix="ur5_ee_"))
    handles: list[dict] = []
    q_ref = getattr(robot, "q", None)
    if q_ref is None or len(q_ref) == 0:
        q_ref = robot.qr

    for i, m in enumerate(mesh_dicts):
        verts = np.asarray(m["verts"], dtype=float)
        faces = np.asarray(m["faces"], dtype=int)
        color = tuple(m.get("color", (0.75, 0.75, 0.75)))
        link_name = str(m.get("name", "tool0"))
        tri = _trimesh.Trimesh(vertices=verts, faces=faces, process=False)
        mesh_file = temp_dir / f"ee_{i:03d}.stl"
        tri.export(str(mesh_file))
        try:
            shape = sg.Mesh(filename=str(mesh_file), color=color)
        except TypeError:
            shape = sg.Mesh(filename=str(mesh_file))
            if hasattr(shape, "color"):
                shape.color = color
        shape.T = link_world_T(robot, q_ref, link_name) @ mount_T
        env.add(shape, collision_alpha=1.0)
        handles.append({"shape": shape, "name": link_name, "T_local": mount_T})

    print(f"[swift_scene] {len(handles)} pièces ajoutées (FT-300 + 2F-85).")
    return handles, temp_dir


def attach_base_axes_to_swift(env, length: float = 0.25):
    """Ajoute une triade RGB à l'origine du repère de base. Retourne l'handle."""
    if env is None:
        return None
    try:
        import spatialgeometry as sg
    except Exception as exc:
        print(f"[swift_scene] spatialgeometry indisponible ({exc!r})")
        return None
    try:
        axes = sg.Axes(length=length)
        axes.T = np.eye(4)
        env.add(axes)
    except Exception as exc:
        print(f"[swift_scene] Base axes échoué ({exc!r}).")
        return None
    tip_radius = max(0.012, length * 0.05)
    for color, pos in [
        ((1.0, 0.0, 0.0, 1.0), (length, 0.0, 0.0)),
        ((0.0, 1.0, 0.0, 1.0), (0.0, length, 0.0)),
        ((0.0, 0.4, 1.0, 1.0), (0.0, 0.0, length)),
    ]:
        try:
            sphere = sg.Sphere(radius=tip_radius, color=color)
        except TypeError:
            sphere = sg.Sphere(radius=tip_radius)
            if hasattr(sphere, "color"):
                sphere.color = color
        T = np.eye(4)
        T[:3, 3] = pos
        sphere.T = T
        try:
            env.add(sphere, collision_alpha=1.0)
        except TypeError:
            env.add(sphere)
    return axes


def attach_tcp_marker_to_swift(env, robot: rtb.Robot):
    """Ajoute une sphère bleue au TCP. Retourne l'handle."""
    if env is None:
        return None
    try:
        import spatialgeometry as sg
    except Exception as exc:
        print(f"[swift_scene] spatialgeometry indisponible ({exc!r})")
        return None
    try:
        sphere = sg.Sphere(radius=0.006, color=(0.1, 0.4, 1.0, 1.0))
    except TypeError:
        sphere = sg.Sphere(radius=0.006)
        if hasattr(sphere, "color"):
            sphere.color = (0.1, 0.4, 1.0, 1.0)
    try:
        q_ref = getattr(robot, "q", None)
        if q_ref is None or len(q_ref) == 0:
            q_ref = robot.qr
        tool_offset = tcp_tool_offset().A
        sphere.T = np.asarray(
            robot.fkine(q_ref, end=END_LINK).A @ tool_offset, dtype=float,
        )
        env.add(sphere, collision_alpha=1.0)
    except Exception as exc:
        print(f"[swift_scene] TCP marker échoué ({exc!r}).")
        return None
    return sphere


# ---------------------------------------------------------------------------
# Cycle de vie de la scène (monté / battement / poussée de pose / démontage)
#
# Ces cinq fonctions viennent de ``viewer.visualize`` : elles y étaient
# inlinées et constituaient la moitié « câblage Swift » du fichier.
# ---------------------------------------------------------------------------

def setup_swift_scene(robot: rtb.Robot, surface: dict | None) -> dict:
    """Lance Swift et attache tout ce que la scène 3D contient.

    Retourne ``{"env", "ee_handles", "ee_temp_dir", "surface_handle",
    "tcp_marker"}``; ``env`` vaut None quand Swift est indisponible, et
    l'appelant retombe alors en mode 2D.
    """
    from ur5_sim.visualization.surface import attach_surface_to_swift

    print("Launching Swift backend (browser tab will open)...")
    env = launch_swift_env(robot)
    ee_handles: list[dict] = []
    ee_temp_dir = None
    surface_handle = None
    tcp_marker = None
    if env is not None:
        print("  Swift ready - the 3D view runs in your browser.")
        attach_base_axes_to_swift(env)
        ee_handles, ee_temp_dir = attach_endeffector_to_swift(robot, env)
        tcp_marker = attach_tcp_marker_to_swift(env, robot)
        if surface is not None:
            surface_handle = attach_surface_to_swift(env, surface)
            if surface_handle is not None:
                print(
                    f"  Test surface added: "
                    f"{surface['w_m'] * 1000:.0f} x {surface['h_m'] * 1000:.0f} mm."
                )
        try:
            env.step(0)
        except Exception:
            pass
    else:
        print("  matplotlib-only mode: 2D panels only, no 3D rendering.")
    return {
        "env": env,
        "ee_handles": ee_handles,
        "ee_temp_dir": ee_temp_dir,
        "surface_handle": surface_handle,
        "tcp_marker": tcp_marker,
    }


def start_heartbeat(env) -> tuple[list[bool], threading.Thread | None]:
    """Swift WebSocket keepalive: start NOW, before matplotlib setup.

    Creating the figure + axes + RadioButtons can take 5-15 s on Windows.
    During that time the asyncio event loop is blocked in outq.get() and
    cannot fire WebSocket pings - the browser tab would close otherwise.
    """
    active = [True]

    def _swift_heartbeat() -> None:
        while active[0]:
            if env is not None:
                try:
                    env.step(0)
                except Exception:
                    pass
            _time.sleep(0.4)

    thread: threading.Thread | None = None
    if env is not None:
        thread = threading.Thread(target=_swift_heartbeat, daemon=True)
        thread.start()
    return active, thread


def stop_heartbeat(active: list[bool], thread: threading.Thread | None) -> None:
    """Hand the keepalive back to the matplotlib timer."""
    active[0] = False
    if thread is not None:
        thread.join(timeout=1.0)


def push_pose_to_swift(
    env,
    robot: rtb.Robot,
    q,
    ee_handles: list[dict],
    tcp_marker,
    tool_offset_A,
) -> None:
    """Pousse une configuration articulaire dans la scène 3D.

    Ne rattrape aucune exception : les deux appelants historiques les
    traitaient différemment (l'un les imprime, l'autre les ignore), donc la
    décision reste chez eux.
    """
    robot.q = q
    for h in ee_handles:
        T_link = link_world_T(robot, q, h["name"])
        T_local = h.get("T_local", np.eye(4))
        h["shape"].T = T_link @ T_local
    if tcp_marker is not None:
        tcp_marker.T = np.asarray(
            robot.fkine(q, end=END_LINK).A @ tool_offset_A,
            dtype=float,
        )
    env.step(0)


def reopen_swift_tab(env, robot: rtb.Robot, trajectory, frame_index: int) -> None:
    """Rouvre l'onglet navigateur de Swift (bouton « Reouvrir 3D »)."""
    # Swift's URL is stored on the backend; re-opening helps when the user
    # accidentally closed the browser tab during playback.
    try:
        url = (
            getattr(env, "swift_path", None)
            or getattr(env, "url", None)
            or getattr(env, "_url", None)
            or getattr(env, "server_url", None)
        )
        if url:
            print(f"[viewer] Reouvrir 3D: opening {url}")
            webbrowser.open(url, new=1)
        else:
            print("[viewer] Reouvrir 3D: URL introuvable sur l'objet Swift, tentative de reveil puis localhost.")
            # Fallback 1: trigger another step so Swift can re-emit its URL.
            robot.q = trajectory[frame_index]
            env.step(0)
            # Fallback 2: try common local URLs used by Swift backend.
            webbrowser.open("http://127.0.0.1:8080", new=1)
    except Exception as exc:  # pragma: no cover
        print(f"[viewer] Cannot re-open Swift tab: {exc!r}")


def teardown_swift_scene(env, ee_temp_dir) -> None:
    """Ferme Swift et efface les maillages temporaires."""
    if env is not None:
        try:
            env.close()
        except Exception:  # pragma: no cover
            pass
    if ee_temp_dir is not None:
        try:
            shutil.rmtree(ee_temp_dir, ignore_errors=True)
        except Exception:  # pragma: no cover
            pass
