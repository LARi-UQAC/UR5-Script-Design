"""End-effector chain : RobotIQ FT-300 force sensor and 2F-85 electric gripper.

Meshes come from the ros-industrial-attic/robotiq repository. The chain of
fixed joint origins reproduced here is taken verbatim from
``robotiq_arg2f_85_model_macro.xacro``; only the actuated joints are held at
zero (closed gripper pose).
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from ur5_sim.config import (
    ROBOTIQ_MESH_DIR,
    SUPPORT_TOOL_LOCAL_RPY,
    SUPPORT_TOOL_LOCAL_XYZ,
    SUPPORT_TOOL_MESH_PATH,
    TARGET_FACES_DEFAULT,
    TARGET_FACES_FINGER,
    TARGET_FACES_FT300,
    TARGET_FACES_GRIPPER_BASE,
)
from ur5_sim.kinematics.transforms import se3
from ur5_sim.meshes.colors import extract_color
from ur5_sim.meshes.decimation import decimate

try:
    import trimesh
    _HAS_TRIMESH = True
except ImportError:
    _HAS_TRIMESH = False


def _load_real_mesh(filename: Path, target_faces: int = TARGET_FACES_DEFAULT):
    """Load a RobotIQ mesh, decimate, auto-scale mm to m if needed."""
    mesh = trimesh.load(str(filename), force="mesh")
    mesh = decimate(mesh, target_faces)
    verts = np.asarray(mesh.vertices, dtype=float)
    faces = np.asarray(mesh.faces, dtype=int)
    if np.abs(verts).max() > 1.0:
        verts = verts * 0.001
    color = extract_color(mesh)
    return verts, faces, color


def build_endeffector_meshes(
    attach_link: str = "tool0",
    mesh_dir: Optional[Path] = None,
) -> list[dict]:
    """Assemble the FT-300 + 2F-85 mesh stack mounted on ``attach_link``.

    Returns a list of mesh dicts using the same schema as
    ``load_link_meshes`` so the viewer can treat both layers uniformly.
    """
    if not _HAS_TRIMESH:
        return []
    if mesh_dir is None:
        mesh_dir = ROBOTIQ_MESH_DIR
    if not mesh_dir.exists():
        print(f"  RobotIQ mesh dir not found ({mesh_dir}) - skipping end-effector.")
        return []

    out: list[dict] = []
    ft_top_z = 0.0348
    coupling_thickness = 0.0139
    gripper_base_z = ft_top_z + coupling_thickness

    def Tv(verts, T):
        return verts @ T[:3, :3].T + T[:3, 3]

    def push(verts, faces, color):
        out.append({"name": attach_link, "verts": verts, "faces": faces, "color": color})

    v, f, c = _load_real_mesh(mesh_dir / "robotiq_ft300.STL", TARGET_FACES_FT300)
    push(v, f, c)

    coupling_a = mesh_dir / "robotiq_ft300-G-062-COUPLING_G-50-4M6-1D6_20181119.STL"
    if coupling_a.exists():
        v, f, c = _load_real_mesh(coupling_a)
        v = v.copy()
        v[:, 2] += ft_top_z
        push(v, f, c)

    grip_coup = mesh_dir / "robotiq_gripper_coupling.stl"
    if grip_coup.exists():
        v, f, c = _load_real_mesh(grip_coup)
        v = v.copy()
        v[:, 2] += ft_top_z + coupling_thickness / 2 + 0.007
        push(v, f, c)

    T_to_base = se3([0.0, 0.0, gripper_base_z])

    v, f, c = _load_real_mesh(
        mesh_dir / "robotiq_arg2f_85_base_link.dae",
        TARGET_FACES_GRIPPER_BASE,
    )
    push(Tv(v, T_to_base), f, c)

    # Optional custom support inserted between fingers.
    if SUPPORT_TOOL_MESH_PATH.exists():
        v, f, c = _load_real_mesh(SUPPORT_TOOL_MESH_PATH, TARGET_FACES_DEFAULT)
        T_support = T_to_base @ se3(SUPPORT_TOOL_LOCAL_XYZ, SUPPORT_TOOL_LOCAL_RPY)
        push(Tv(v, T_support), f, c)

    pi = float(np.pi)
    finger_chains: list[tuple[str, np.ndarray]] = []
    for reflect, side_rot in [(1, pi), (-1, 0.0)]:
        T_ok = T_to_base @ se3([0, reflect * -0.0306011, 0.054904], [0, 0, side_rot])
        T_of = T_ok @ se3([0, 0.0315, -0.0041])
        T_if = T_of @ se3([0, 0.0061, 0.0471])
        T_pad = T_if @ se3([0, -0.0220, 0.0324])
        T_ik = T_to_base @ se3([0, reflect * -0.0127, 0.06142], [0, 0, side_rot])
        finger_chains.extend([
            ("robotiq_arg2f_85_outer_knuckle.dae", T_ok),
            ("robotiq_arg2f_85_outer_finger.dae", T_of),
            ("robotiq_arg2f_85_inner_finger.dae", T_if),
            ("robotiq_arg2f_85_pad.dae", T_pad),
            ("robotiq_arg2f_85_inner_knuckle.dae", T_ik),
        ])

    for fname, T in finger_chains:
        path = mesh_dir / fname
        if not path.exists():
            continue
        v, f, c = _load_real_mesh(path, TARGET_FACES_FINGER)
        push(Tv(v, T), f, c)

    return out
