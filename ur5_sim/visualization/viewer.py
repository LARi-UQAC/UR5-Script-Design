"""Hybrid Swift + matplotlib viewer for the UR5 trajectory replay.

Architecture
------------
* **Swift backend (WebGL, browser tab)** — handles the 3D rendering of the
  UR5 robot. Swift ships with the UR5 URDF + meshes, gives GPU-accelerated
  real-time playback similar in feel to RoboDK / URSim, and supports user
  camera control natively (orbit/pan/zoom).
* **matplotlib (native window)** — keeps the two 2D panels (XYZ vs time
  and XY trail), the configuration selector (IK branches), the START/STOP
  toggle, and the HUD text.

Compute stage assembles a per-frame buffer of TCP positions and joint
configurations once per selected IK branch. Display stage is wall-clock
driven: every tick pushes ``robot.q = trajectory[frame]`` to Swift and
updates the matplotlib line data. Slow renders drop frames instead of
stalling the simulation clock.

The simulation starts in the STOP state; the user must click START to
begin the playback.
"""

from __future__ import annotations

import shutil
import threading
import time
import webbrowser
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle as MplCircle, Polygon as MplPolygon
from matplotlib.widgets import Button, RadioButtons

try:
    from swift import Swift as _Swift_unused  # noqa: F401  kept for optional dep check
except ImportError:  # pragma: no cover - optional dependency
    pass

from ur5_sim.config import (
    CONTACT_SNAP_TOL_M,
    DT,
    END_LINK,
    FORCE_Z_TARGET_N,
    REPO_ROOT,
    SCRIPT_PATH,
    SIM_SPEED,
    SIM_TRAJ_ROT_Y_RAD,
    SURFACE_CLEARANCE_M,
    SURFACE_COLOR_RGBA,
    SURFACE_ENABLE_CLAMP,
    URSCRIPT_MAX_TCP_SPEED_MPS,
    P_ANCHOR_OLD_RAW,
    P_REF_RAW,
)
from ur5_sim.ipc_config import TCP_LIVE_HOST, TCP_LIVE_PORT
from ur5_sim.kinematics.ik import run_ik
from ur5_sim.kinematics.motion import densify_segments
from ur5_sim.kinematics.transforms import (
    link_world_T,
    rotate_translation_y,
    tcp_tool_offset,
)
from ur5_sim.parsing.urscript import (
    parse_motion_segments,
    transform,
    urscript_pose,
)
from ur5_sim.visualization.interactions import attach_pan, attach_scroll_zoom
from ur5_sim.visualization.surface import (
    apply_surface_constraint,
    attach_surface_to_swift,
    compute_probe_points_world,
    compute_test_points_world,
    make_corners_xy_polygon,
    test_point_radius_world_m,
)
from ur5_sim.visualization.swift_scene import (
    attach_base_axes_to_swift,
    attach_endeffector_to_swift,
    attach_tcp_marker_to_swift,
    launch_swift_env,
    send_tcp_live,
)

try:
    import trimesh
    _HAS_TRIMESH = True
except ImportError:
    _HAS_TRIMESH = False


def visualize(
    robot: rtb.Robot,
    trajectories: list[tuple[str, list[np.ndarray]]],
    dt: float = DT,
    cycle_per_frame: list[int] | None = None,
    plate_xy_per_frame: list[tuple[float, float]] | None = None,
    surface: dict | None = None,
    in_contact_per_frame: list[bool] | None = None,
) -> None:
    """Run the live animation with multi-configuration selection.

    Parameters
    ----------
    robot:
        roboticstoolbox UR5 model.
    trajectories:
        List of ``(label, joint_trajectory)`` pairs - one entry per IK
        branch that reaches the initial pose. The label appears in the
        configuration selector.
    dt:
        Nominal time step between successive joint configurations.
    cycle_per_frame:
        Optional list aligned with the trajectory length, containing the
        1-based index of the URScript cycle each frame belongs to
        (``0`` for frames that fall outside any ``def cycle_N():`` block).
        When provided, the viewer publishes a live TCP status to
        ``tcp_live.json`` for cross-process consumption by the design UI.
    surface:
        Optional surface frame returned by
        :func:`ur5_sim.visualization.surface.compute_surface_frame`. When
        present, the test plate is rendered both in Swift (50x50x2 mm
        cuboid) and in the matplotlib XY panel (semi-transparent polygon),
        and the HUD marks each frame as contact (green) or transit (blue).
    in_contact_per_frame:
        Optional list aligned with the trajectory; ``True`` between
        ``force_mode(...)`` and ``end_force_mode()``. Drives the HUD
        ``F_Z`` field (6.0 N during contact, 0.0 N during transit) and the
        live marker colour. Defaults to all-False if omitted.
    """
    if not trajectories:
        raise ValueError("visualize() requires at least one trajectory")

    n_frames = len(trajectories[0][1])
    times = np.arange(n_frames) * dt
    if cycle_per_frame is None:
        cycle_per_frame = [0] * n_frames
    if len(cycle_per_frame) < n_frames:
        cycle_per_frame = list(cycle_per_frame) + [0] * (n_frames - len(cycle_per_frame))
    elif len(cycle_per_frame) > n_frames:
        cycle_per_frame = list(cycle_per_frame[:n_frames])
    n_cycles_detected = max(cycle_per_frame) if cycle_per_frame else 0

    if plate_xy_per_frame is None:
        plate_xy_per_frame = [(0.0, 0.0)] * n_frames
    if len(plate_xy_per_frame) < n_frames:
        plate_xy_per_frame = list(plate_xy_per_frame) + [(0.0, 0.0)] * (
            n_frames - len(plate_xy_per_frame)
        )
    elif len(plate_xy_per_frame) > n_frames:
        plate_xy_per_frame = list(plate_xy_per_frame[:n_frames])

    if in_contact_per_frame is None:
        in_contact_per_frame = [False] * n_frames
    if len(in_contact_per_frame) < n_frames:
        in_contact_per_frame = list(in_contact_per_frame) + [False] * (
            n_frames - len(in_contact_per_frame)
        )
    elif len(in_contact_per_frame) > n_frames:
        in_contact_per_frame = list(in_contact_per_frame[:n_frames])

    # --- 3D rendering : Swift (browser tab, WebGL) ---
    print("Launching Swift backend (browser tab will open)...")
    env = launch_swift_env(robot)
    ee_handles = []
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

    # Swift WebSocket keepalive: start NOW, before matplotlib setup.
    # Creating the figure + axes + RadioButtons can take 5-15 s on Windows.
    # During that time the asyncio event loop is blocked in outq.get() and
    # cannot fire WebSocket pings — the browser tab would close otherwise.
    _hb_active = [True]

    def _swift_heartbeat() -> None:
        while _hb_active[0]:
            if env is not None:
                try:
                    env.step(0)
                except Exception:
                    pass
            time.sleep(0.4)

    _hb_thread: threading.Thread | None = None
    if env is not None:
        _hb_thread = threading.Thread(target=_swift_heartbeat, daemon=True)
        _hb_thread.start()

    # --- 2D panels + UI : matplotlib ---
    fig = plt.figure(figsize=(12, 6))
    gs = GridSpec(1, 2, wspace=0.25, left=0.07, right=0.78, top=0.88, bottom=0.13)
    ax_xyz = fig.add_subplot(gs[0, 0])
    ax_xy = fig.add_subplot(gs[0, 1])

    ax_xyz.set_xlim(0, times[-1] if n_frames > 1 else 1.0)
    ax_xyz.set_xlabel("Temps (s)")
    ax_xyz.set_ylabel("Position TCP (m)")
    ax_xyz.set_title("X, Y, Z vs temps")
    ax_xyz.grid(True, alpha=0.3)
    line_x, = ax_xyz.plot([], [], "r-", lw=1.2, label="X")
    line_y, = ax_xyz.plot([], [], "g-", lw=1.2, label="Y")
    line_z, = ax_xyz.plot([], [], "b-", lw=1.2, label="Z")
    ax_xyz.legend(loc="upper right")

    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.set_title("Trajectoire X-Y (vue dessus)")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.grid(True, alpha=0.3)
    # Trail XY colore par cycle : une ligne distincte par cycle URScript
    # detecte, plus un marqueur de la position courante. Les couleurs
    # matchent celles utilisees par le design UI (circulaires : bleu/orange/vert ;
    # rectilignes : rouge/violet/brun).
    cycle_palette = [
        "#888888", # Gris pour Cycle 0 (Initialisation/Palpage)
        "#1f77b4", "#ff7f0e", "#2ca02c",
        "#d62728", "#9467bd", "#8c564b",
        "#7f7f7f", "#bcbd22", "#17becf",
    ]
    max_palette_cycles = max(6, n_cycles_detected if n_cycles_detected else 6)
    cycle_xy_lines: list = []
    for c_i in range(max_palette_cycles + 1):
        label = "Initialisation" if c_i == 0 else f"Cycle {c_i}"
        ln, = ax_xy.plot(
                [], [],
                color=cycle_palette[c_i % len(cycle_palette)],
                lw=1.0, alpha=0.9,
                label=label,
            )
        cycle_xy_lines.append(ln)
    # Legende construite plus bas, apres l'ajout des etoiles de sondage, pour
    # que l'entree "Sondage" y figure.

    # Empreinte de la surface de test sur la vue XY (polygone semi-transparent).
    surface_xy = make_corners_xy_polygon(surface)
    if surface_xy is not None:
        ax_xy.add_patch(
            MplPolygon(
                surface_xy,
                closed=True,
                facecolor=SURFACE_COLOR_RGBA[:3] + (0.15,),
                edgecolor=SURFACE_COLOR_RGBA[:3] + (0.9,),
                linewidth=1.2,
                zorder=2,
            )
        )

    # 9-point test pattern (cf. meshes/TestMeasure.PNG). Marks are drawn
    # as static circles on top of the plate polygon so the operator can
    # see the live trajectory crossing each measurement point.
    if surface is not None:
        try:
            p_anchor_old_tp = urscript_pose(*P_ANCHOR_OLD_RAW)
            p_ref_tp = urscript_pose(*P_REF_RAW)
            tp_radius_m = test_point_radius_world_m(surface)
            for label, xyz in compute_test_points_world(p_anchor_old_tp, p_ref_tp):
                ax_xy.add_patch(
                    MplCircle(
                        (float(xyz[0]), float(xyz[1])),
                        radius=tp_radius_m,
                        facecolor="none",
                        edgecolor="#c66a3a",
                        linewidth=1.2,
                        zorder=3,
                    )
                )
                ax_xy.text(
                    float(xyz[0]), float(xyz[1]), str(label),
                    color="#1f4f8a", fontsize=8, fontweight="bold",
                    ha="center", va="center", zorder=4,
                )
        except Exception as exc:  # pragma: no cover - non-fatal overlay
            print(f"[viewer] test-point overlay skipped: {exc!r}")

    # Poses de sondage (probe_surface_plane) : etoiles noires sur la plaque,
    # aux 3 points que le robot palpe pour mesurer le plan. Memes anchors /
    # meme chaine plate->monde que les points de test, donc co-localises.
    if surface is not None:
        try:
            p_anchor_old_pb = urscript_pose(*P_ANCHOR_OLD_RAW)
            p_ref_pb = urscript_pose(*P_REF_RAW)
            for k, xyz in enumerate(
                compute_probe_points_world(p_anchor_old_pb, p_ref_pb)
            ):
                ax_xy.plot(
                    float(xyz[0]), float(xyz[1]),
                    marker="*", linestyle="None",
                    color="black", markersize=12, zorder=5,
                    label="Sondage" if k == 0 else "_nolegend_",
                )
        except Exception as exc:  # pragma: no cover - non-fatal overlay
            print(f"[viewer] probe-point overlay skipped: {exc!r}")

    ax_xy.legend(loc="best", fontsize=7, frameon=True)

    xy_now, = ax_xy.plot([], [], "o", ms=6, zorder=6,
                        markerfacecolor="black", markeredgecolor="black")

    sim_text = fig.text(
        0.42, 0.96, "", ha="center", va="center",
        fontsize=10, family="monospace",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="0.6"),
    )

    labels = [t[0] for t in trajectories]
    radio_ax = fig.add_axes([0.80, 0.50, 0.18, 0.42])
    radio_ax.set_title("Configurations IK", fontsize=10, loc="left")
    radio = RadioButtons(radio_ax, labels=labels, active=0)
    for label_widget in radio.labels:
        label_widget.set_fontsize(8)

    button_ax = fig.add_axes([0.80, 0.40, 0.18, 0.06])
    btn = Button(button_ax, "START", color="#cce5cc", hovercolor="#a6d6a6")

    status_text = fig.text(
        0.80, 0.36, "STATE = STOP", fontsize=10, family="monospace", color="#a02020",
    )

    swift_btn_ax = fig.add_axes([0.80, 0.28, 0.18, 0.05])
    swift_btn = Button(
        swift_btn_ax,
        "Reouvrir 3D" if env is not None else "Swift indispo",
        color="#cce5e5" if env is not None else "#e5e5e5",
        hovercolor="#a6d6d6" if env is not None else "#e5e5e5",
    )

    state = {
        "idx": 0,
        "running": False,
        "trajectory": None,
        "tcp_pts": None,
        "xs": None, "ys": None, "zs": None,
        "timer": None,
    }

    p_anchor_old = urscript_pose(*P_ANCHOR_OLD_RAW)
    p_ref = urscript_pose(*P_REF_RAW)
    tool_offset = tcp_tool_offset()
    tool_offset_A = tool_offset.A

    # Cross-thread handoff for the background recompute. The worker thread fills
    # ``done``/``payload``/``error`` under ``lock``; the matplotlib timer
    # (main thread) polls and finalises. Nothing here touches matplotlib/Swift.
    recompute = {
        "active": False,
        "lock": threading.Lock(),
        "done": 0,
        "total": 1,
        "ready": False,
        "payload": None,
        "error": None,
    }

    def compute_state(idx: int) -> None:
        traj = trajectories[idx][1]
        tcp_pts = np.zeros((len(traj), 3))
        for i, q in enumerate(traj):
            tcp_pts[i] = (robot.fkine(q, end=END_LINK).A @ tool_offset_A)[:3, 3]
        xs_, ys_, zs_ = tcp_pts[:, 0], tcp_pts[:, 1], tcp_pts[:, 2]
        state["idx"] = idx
        state["trajectory"] = traj
        state["tcp_pts"] = tcp_pts
        state["xs"], state["ys"], state["zs"] = xs_, ys_, zs_
        # Indices des frames par cycle (1-based) -> base pour les trails colores.
        cycle_arr = np.asarray(cycle_per_frame[: len(traj)], dtype=int)
        state["cycle_indices"] = [
            np.where(cycle_arr == c)[0]
            for c in range(max_palette_cycles + 1)
        ]

        ax_xyz.set_ylim(
            min(xs_.min(), ys_.min(), zs_.min()) - 0.05,
            max(xs_.max(), ys_.max(), zs_.max()) + 0.05,
        )
        pad = 0.05
        ax_xy.set_xlim(xs_.min() - pad, xs_.max() + pad)
        ax_xy.set_ylim(ys_.min() - pad, ys_.max() + pad)

        # Reset des trails colores par cycle.
        for ln in cycle_xy_lines:
            ln.set_data([], [])

        # Push initial joint config to Swift so the robot is visible at q[0].
        if env is not None and len(traj) > 0:
            try:
                robot.q = traj[0]
                for h in ee_handles:
                    T_link = link_world_T(robot, traj[0], h["name"])
                    T_local = h.get("T_local", np.eye(4))
                    h["shape"].T = T_link @ T_local
                if tcp_marker is not None:
                    tcp_marker.T = np.asarray(
                        robot.fkine(traj[0], end=END_LINK).A @ tool_offset_A,
                        dtype=float,
                    )
                env.step(0)
            except Exception as exc:  # pragma: no cover
                print(f"[viewer] Swift step failed: {exc!r}")

    print("Computing trajectory buffer for configuration #1...")
    t0 = time.perf_counter()
    compute_state(0)
    print(f"  ready in {time.perf_counter() - t0:.1f} s.")

    total_sim_time = max(n_frames - 1, 1) * dt
    clock_t0 = [time.perf_counter()]
    paused_sim_t = [0.0]
    last_drawn = [-1]
    paint_count = [0]
    paint_start = [time.perf_counter()]
    last_wall: list[float | None] = [None]
    last_dt_real = [0.0]
    dt_real_window: list[float] = []
    dt_real_avg = [0.0]
    fps_text = [""]

    def render_frame(frame: int) -> None:
        xs_, ys_, zs_ = state["xs"], state["ys"], state["zs"]
        traj = state["trajectory"]
        # 3D : push joint config to Swift.
        if env is not None and traj is not None:
            try:
                robot.q = traj[frame]
                for h in ee_handles:
                    T_link = link_world_T(robot, traj[frame], h["name"])
                    T_local = h.get("T_local", np.eye(4))
                    h["shape"].T = T_link @ T_local
                if tcp_marker is not None:
                    tcp_marker.T = np.asarray(
                        robot.fkine(traj[frame], end=END_LINK).A @ tool_offset_A,
                        dtype=float,
                    )
                env.step(0)
            except Exception:  # pragma: no cover
                pass
        # 2D matplotlib lines.
        line_x.set_data(times[: frame + 1], xs_[: frame + 1])
        line_y.set_data(times[: frame + 1], ys_[: frame + 1])
        line_z.set_data(times[: frame + 1], zs_[: frame + 1])
        # Trail XY par cycle : on ne trace que les frames deja jouees.
        cycle_indices = state.get("cycle_indices")
        if cycle_indices is not None:
            for c_i, idxs in enumerate(cycle_indices):
                visible_idxs = idxs[idxs <= frame]
                if len(visible_idxs) > 0:
                    cycle_xy_lines[c_i].set_data(
                        xs_[visible_idxs], ys_[visible_idxs]
                    )
                else:
                    cycle_xy_lines[c_i].set_data([], [])
        xy_now.set_data([xs_[frame]], [ys_[frame]])
        # Marqueur live colore selon le regime force_mode et l'ecart au plan.
        in_contact = (
            in_contact_per_frame[frame]
            if frame < len(in_contact_per_frame) else False
        )
        surface_depth_m = 0.0
        if surface is not None:
            surface_depth_m = float(
                np.dot(
                    surface["normal"],
                    np.array([xs_[frame], ys_[frame], zs_[frame]])
                    - surface["center"],
                )
            )
        tol = CONTACT_SNAP_TOL_M
        if in_contact:
            marker_color = "red" if abs(surface_depth_m) > tol else "#1faa00"
        else:
            marker_color = "red" if surface_depth_m < -tol else "#1f6dff"
        xy_now.set_markerfacecolor(marker_color)
        xy_now.set_markeredgecolor(marker_color)
        # IPC : publish TCP state to the design UI.
        cycle_idx = cycle_per_frame[frame] if frame < len(cycle_per_frame) else 0
        plate_xy = (
            plate_xy_per_frame[frame]
            if frame < len(plate_xy_per_frame)
            else (0.0, 0.0)
        )
        trail_anchor_m = []
        if cycle_indices is not None and cycle_idx < len(cycle_indices):
            cycle_visible = cycle_indices[cycle_idx]
            cycle_visible = cycle_visible[cycle_visible <= frame]
            trail_anchor_m = [
                [
                    float(plate_xy_per_frame[int(k)][0]),
                    float(plate_xy_per_frame[int(k)][1]),
                ]
                for k in cycle_visible
                if int(k) < len(plate_xy_per_frame)
            ]
        send_tcp_live({
            "running": bool(state["running"]),
            "cycle": int(cycle_idx),
            "frame": int(frame),
            "n_frames": int(n_frames),
            # World TCP from forward kinematics (P_REF frame, m).
            "x_world": float(xs_[frame]),
            "y_world": float(ys_[frame]),
            "z_world": float(zs_[frame]),
            # Script pose in P_ANCHOR_OLD frame (m). The design UI
            # inverts plate_to_robot() on these to recover plate mm.
            "x_anchor_m": float(plate_xy[0]),
            "y_anchor_m": float(plate_xy[1]),
            "trail_anchor_m": trail_anchor_m,
            "n_cycles": int(n_cycles_detected),
            # Regulateur de force / surface (surrogate cinematique).
            "in_contact": bool(in_contact),
            "force_z_n": float(FORCE_Z_TARGET_N if in_contact else 0.0),
            "surface_depth_mm": float(surface_depth_m * 1000.0),
            "ts": time.time(),
        })
        # Memorise pour write_hud.
        state["_last_in_contact"] = in_contact
        state["_last_surface_depth_m"] = surface_depth_m

    def write_hud(frame: int, sim_elapsed: float) -> None:
        run_flag = "RUN " if state["running"] else "STOP"
        cfg_label = trajectories[state["idx"]][0]
        wall_elapsed = (time.perf_counter() - clock_t0[0]) if state["running"] else paused_sim_t[0]
        sync_err_ms = (wall_elapsed - sim_elapsed) * 1000.0
        backend_tag = "Swift" if env is not None else "2D"
        current_cycle = cycle_per_frame[frame] if frame < len(cycle_per_frame) else 0
        in_contact = bool(state.get("_last_in_contact", False))
        surface_depth_mm = float(state.get("_last_surface_depth_m", 0.0)) * 1000.0
        f_z = FORCE_Z_TARGET_N if in_contact else 0.0
        sim_text.set_text(
            f"[{run_flag}] {backend_tag}  cfg={cfg_label[:22]:<22s}  |  "
            f"cycle {current_cycle}/{n_cycles_detected}  |  "
            f"F_Z = {f_z:4.1f} N  dz = {surface_depth_mm:+6.2f} mm  |  "
            f"PC t = {wall_elapsed:6.3f} s  |  "
            f"SIM t = {sim_elapsed:6.3f} / {total_sim_time:5.2f} s  |  "
            f"delta = {sync_err_ms:+6.1f} ms  |  "
            f"frame {frame:>3d}/{n_frames - 1}  |  "
            f"dt_real = {last_dt_real[0] * 1000:5.1f} ms  "
            f"avg = {dt_real_avg[0] * 1000:5.1f} ms  "
            f"target = {dt * 1000:5.1f} ms{fps_text[0]}"
        )

    def _recompute_worker() -> None:
        """Re-parse the script and solve IK for every branch, off the GUI thread.

        Pure numpy/kinematics only (no matplotlib, no Swift). Uses a private
        UR5 instance so it never races the main thread's ``robot.q`` writes.
        Results and progress are published under ``recompute['lock']``; the
        timer's ``tick`` consumes them on the main thread.
        """
        try:
            worker_robot = rtb.models.UR5()
            segments_latest = parse_motion_segments(SCRIPT_PATH)
            parsed_latest, _events = densify_segments(
                segments_latest, DT, URSCRIPT_MAX_TCP_SPEED_MPS,
            )
            if not parsed_latest:
                with recompute["lock"]:
                    recompute["error"] = f"no poses parsed from {SCRIPT_PATH}"
                    recompute["ready"] = True
                return

            poses_xform_latest = []
            for lineno, pose, _cycle, in_contact in parsed_latest:
                pose_tf = transform(urscript_pose(*pose), p_anchor_old, p_ref)
                pose_tf = rotate_translation_y(pose_tf, SIM_TRAJ_ROT_Y_RAD)
                if surface is not None and SURFACE_ENABLE_CLAMP:
                    pose_tf, _kind, _depth = apply_surface_constraint(
                        pose_tf, surface, in_contact, SURFACE_CLEARANCE_M,
                    )
                poses_xform_latest.append((lineno, pose_tf))

            new_cycle = [cyc for _l, _p, cyc, _ic in parsed_latest]
            new_plate = [(p[0], p[1]) for _l, p, _c, _ic in parsed_latest]
            new_contact = [ic for _l, _p, _c, ic in parsed_latest]

            n_poses = len(poses_xform_latest)
            grand_total = max(1, len(trajectories) * n_poses)
            with recompute["lock"]:
                recompute["total"] = grand_total
                recompute["done"] = 0

            refreshed = []
            base = 0
            for lbl, traj_old in trajectories:
                q0_seed = (
                    traj_old[0]
                    if (traj_old is not None and len(traj_old) > 0)
                    else worker_robot.qr
                )

                def _prog(done_branch, _total_branch, _base=base):
                    with recompute["lock"]:
                        recompute["done"] = _base + done_branch

                traj_new, _fails = run_ik(
                    worker_robot, poses_xform_latest, q0_seed, progress=_prog,
                )
                refreshed.append((lbl, traj_new))
                base += n_poses

            with recompute["lock"]:
                recompute["payload"] = {
                    "trajectories": refreshed,
                    "cycle": new_cycle,
                    "plate": new_plate,
                    "contact": new_contact,
                }
                recompute["ready"] = True
        except Exception as exc:  # pragma: no cover - surfaced to the HUD
            with recompute["lock"]:
                recompute["error"] = repr(exc)
                recompute["ready"] = True

    def _finalize_recompute(payload: dict) -> None:
        """Swap in the freshly computed trajectory (main thread only)."""
        nonlocal n_frames, times, total_sim_time, cycle_per_frame
        nonlocal plate_xy_per_frame, n_cycles_detected, in_contact_per_frame
        trajectories[:] = payload["trajectories"]
        cycle_per_frame = payload["cycle"]
        plate_xy_per_frame = payload["plate"]
        in_contact_per_frame = payload["contact"]
        n_cycles_detected = max(cycle_per_frame) if cycle_per_frame else 0
        n_frames = len(trajectories[0][1])
        times = np.arange(n_frames) * dt
        total_sim_time = max(n_frames - 1, 1) * dt
        ax_xyz.set_xlim(0, times[-1] if n_frames > 1 else 1.0)
        if state["idx"] >= len(trajectories):
            state["idx"] = 0
        compute_state(state["idx"])

    def tick() -> None:
        # Background recompute in flight (script changed): keep Swift + the GUI
        # alive, show progress, and finalise on this main thread when done.
        if recompute["active"]:
            if env is not None:
                try:
                    env.step(0)
                except Exception:
                    pass
            with recompute["lock"]:
                done = recompute["done"]
                total = recompute["total"]
                ready = recompute["ready"]
                error = recompute["error"]
                payload = recompute["payload"]
            if not ready:
                pct = int(100 * done / total) if total else 0
                sim_text.set_text(
                    f"Recomputing trajectory... {pct}% ({done}/{total} IK)"
                )
                fig.canvas.draw_idle()
                return
            recompute["active"] = False
            if error is not None or payload is None:
                print(f"[viewer] recompute failed ({error}); keeping buffer.")
                sim_text.set_text("Recompute failed - previous buffer kept.")
                _reset_playback_to_start()
                set_start()
                fig.canvas.draw_idle()
                return
            _finalize_recompute(payload)
            _reset_playback_to_start()
            set_start()
            fig.canvas.draw_idle()
            return

        if not state["running"]:
            # On maintient la boucle Swift active pour l'interactivite (camera)
            # meme quand la simulation est a l'arret. Cela evite que le
            # serveur WebSocket de Swift ne se ferme par inactivite.
            if env is not None:
                try:
                    env.step(0)
                except Exception as e:
                    print(f"[viewer] Swift env.step(0) failed in idle mode: {e!r}")
            return
        wall = time.perf_counter()
        sim_elapsed = (wall - clock_t0[0]) * SIM_SPEED + paused_sim_t[0]
        if sim_elapsed > total_sim_time:
            # Fin de trajectoire : on MAINTIENT l'effecteur sur la derniere pose
            # (le retrait Z+3 cm au-dessus du dernier waypoint) et on arrete la
            # lecture, au lieu de boucler vers la frame 0 (debut de trajectoire).
            # Reproduit le comportement reel : apres etalement() le programme
            # s'arrete sur le retrait, laissant la plaque accessible.
            # Un nouveau START rejoue depuis le debut (cf. on_button ->
            # _reset_playback_to_start).
            sim_elapsed = total_sim_time
            frame = n_frames - 1
            if frame != last_drawn[0]:
                last_drawn[0] = frame
                render_frame(frame)
                write_hud(frame, sim_elapsed)
            set_stop()
            fig.canvas.draw_idle()
            return
        frame = min(int(sim_elapsed / dt), n_frames - 1)
        if frame == last_drawn[0]:
            return

        if last_wall[0] is not None:
            last_dt_real[0] = wall - last_wall[0]
            dt_real_window.append(last_dt_real[0])
            if len(dt_real_window) > 20:
                del dt_real_window[0]
            dt_real_avg[0] = sum(dt_real_window) / len(dt_real_window)
        last_wall[0] = wall

        paint_count[0] += 1
        if wall - paint_start[0] >= 1.0:
            fps = paint_count[0] / (wall - paint_start[0])
            fps_text[0] = f"  |  draw {fps:5.1f} fps"
            paint_count[0] = 0
            paint_start[0] = wall

        last_drawn[0] = frame
        render_frame(frame)
        write_hud(frame, sim_elapsed)
        fig.canvas.draw_idle()

    def set_stop() -> None:
        # STOP is always a hard stop; next START must restart from frame 0.
        paused_sim_t[0] = 0.0
        state["running"] = False
        btn.label.set_text("START")
        btn.color = "#cce5cc"
        btn.hovercolor = "#a6d6a6"
        status_text.set_text("STATE = STOP")
        status_text.set_color("#a02020")

    def _reset_playback_to_start() -> None:
        # Efface toutes les anciennes traces et repositionne la simulation au debut.
        for ln in cycle_xy_lines:
            ln.set_data([], [])
        line_x.set_data([], [])
        line_y.set_data([], [])
        line_z.set_data([], [])
        xy_now.set_data([], [])

        paused_sim_t[0] = 0.0
        clock_t0[0] = time.perf_counter()
        last_drawn[0] = -1
        last_wall[0] = None
        dt_real_window.clear()
        dt_real_avg[0] = 0.0
        last_dt_real[0] = 0.0

        render_frame(0)
        write_hud(0, 0.0)

    def set_start() -> None:
        state["running"] = True
        clock_t0[0] = time.perf_counter()
        last_wall[0] = None
        dt_real_window.clear()
        dt_real_avg[0] = 0.0
        last_dt_real[0] = 0.0
        btn.label.set_text("STOP")
        btn.color = "#f5c6c6"
        btn.hovercolor = "#e89999"
        status_text.set_text("STATE = RUN")
        status_text.set_color("#207020")

    def on_radio(label: str) -> None:
        if recompute["active"]:
            return
        idx = labels.index(label)
        set_stop()
        sim_text.set_text("Rebuilding buffer for selected configuration...")
        fig.canvas.draw_idle()
        plt.pause(0.01)
        print(f"Rebuilding buffer for configuration '{label}'...")
        t0 = time.perf_counter()
        compute_state(idx)
        print(f"  ready in {time.perf_counter() - t0:.1f} s.")
        paused_sim_t[0] = 0.0
        clock_t0[0] = time.perf_counter()
        last_drawn[0] = -1
        render_frame(0)
        write_hud(0, 0.0)
        fig.canvas.draw_idle()

    def on_button(_event) -> None:
        # STOP branch: a running playback is halted immediately.
        if state["running"]:
            set_stop()
            write_hud(0, 0.0)
            fig.canvas.draw_idle()
            return
        # A background recompute is already in flight -> ignore extra clicks.
        if recompute["active"]:
            fig.canvas.draw_idle()
            return
        # START always re-reads ``etalement.script`` and re-runs IK for every
        # branch, so the viewer never replays a trajectory that is out of date
        # with the file on disk (e.g. after the design UI re-exports it). The
        # work runs off the GUI thread; ``tick`` shows the progress and swaps
        # the fresh buffer in when the worker finishes (no freeze).
        set_stop()
        recompute["active"] = True
        recompute["ready"] = False
        recompute["error"] = None
        recompute["payload"] = None
        recompute["done"] = 0
        recompute["total"] = 1
        sim_text.set_text("Recomputing trajectory... 0%")
        status_text.set_text("STATE = BUSY")
        status_text.set_color("#a06000")
        btn.label.set_text("...")
        fig.canvas.draw_idle()
        worker = threading.Thread(target=_recompute_worker, daemon=True)
        worker.start()

    def on_swift_btn(_event) -> None:
        if env is None:
            print("[viewer] Reouvrir 3D: Swift indisponible (env=None).")
            return
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
                robot.q = state["trajectory"][last_drawn[0] if last_drawn[0] >= 0 else 0]
                env.step(0)
                # Fallback 2: try common local URLs used by Swift backend.
                webbrowser.open("http://127.0.0.1:8080", new=1)
        except Exception as exc:  # pragma: no cover
            print(f"[viewer] Cannot re-open Swift tab: {exc!r}")

    radio.on_clicked(on_radio)
    btn.on_clicked(on_button)
    swift_btn.on_clicked(on_swift_btn)

    # Navigation souris sur la vue X-Y (vue dessus) uniquement : molette = zoom
    # centre sur le curseur (haut = zoom avant, comme Swift), glisser-bouton
    # gauche = translation. Pas de rotation (plan 2D). Limite a ax_xy pour ne
    # pas perturber le graphe X,Y,Z-vs-temps ni les widgets.
    attach_scroll_zoom(fig, [ax_xy], [])
    attach_pan(fig, ax_xy)

    render_frame(0)
    write_hud(0, 0.0)

    print(
        f"Viewer ready ({len(trajectories)} configurations). "
        f"Click START to play. Close the matplotlib window to exit."
    )
    timer = fig.canvas.new_timer(interval=30) # Intervalle de 30ms (environ 33 FPS)
    timer.add_callback(tick)
    state["timer"] = timer
    timer.start()
    # Matplotlib timer now owns the heartbeat — stop the background thread.
    _hb_active[0] = False
    if _hb_thread is not None:
        _hb_thread.join(timeout=1.0)
    plt.show()

    # Cleanup on window close : flag the IPC file so the design UI hides
    # its live marker even if the simulator window closes mid-cycle.
    send_tcp_live({
        "running": False,
        "cycle": 0,
        "frame": 0,
        "n_frames": int(n_frames),
        "x_world": 0.0,
        "y_world": 0.0,
        "z_world": 0.0,
        "x_anchor_m": 0.0,
        "y_anchor_m": 0.0,
        "trail_anchor_m": [],
        "n_cycles": int(n_cycles_detected),
        "in_contact": False,
        "force_z_n": 0.0,
        "surface_depth_mm": 0.0,
        "ts": time.time(),
    })
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
