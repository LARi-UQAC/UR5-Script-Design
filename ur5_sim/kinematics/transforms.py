"""Pure-numpy SE(3) helpers used by the mesh placement code.

These three functions deliberately depend only on numpy so they can be
imported in any layer (parsing, meshes, visualization) without dragging
spatialmath or matplotlib into modules that should remain headless.
"""

from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np


def rpy_to_R(rpy: Sequence[float]) -> np.ndarray:
    """Roll-pitch-yaw (URDF convention) to a 3x3 rotation matrix.

    Equivalent to ``Rz(yaw) @ Ry(pitch) @ Rx(roll)`` applied to a column
    vector.
    """
    rx, ry, rz = rpy
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def se3(xyz: Sequence[float], rpy: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    """Assemble a 4x4 homogeneous transform from a translation and rpy."""
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(rpy)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def link_world_T(robot, q: Iterable[float], link_name: str) -> np.ndarray:
    """World-frame 4x4 transform of the named link given joint vector ``q``.

    Falls back to the identity matrix when the link name is unknown so a
    missing label degrades gracefully instead of raising.
    """
    try:
        return robot.fkine(q, end=link_name).A
    except Exception:
        return np.eye(4)


def tcp_tool_offset():
    """Return the SE3 transform from ``tool0`` to the TCP (finger tip).

    rtb's UR5 URDF bakes the 82.3 mm flange offset into the ``tool0`` link
    (visible via ``fkine(q, end='tool0')``). The remaining transform is a
    pure translation along tool0 Z covering the FT-300 + coupling + 2F-85
    + silicone finger stack (``TCP_TOOL_Z_M`` in ``ur5_sim.config``).

    The simulator uses this to convert between TCP targets (what
    ``etalement.script`` emits, what the operator calibrates with
    ``set_tcp``) and tool0 poses (what ``ikine_LM`` solves for when
    ``end='tool0'``).
    """
    from spatialmath import SE3
    from ur5_sim.config import TCP_TOOL_Z_M
    return SE3(0.0, 0.0, TCP_TOOL_Z_M)


def rotate_translation_y(pose, angle_rad: float):
    """Rotate the translation component of an SE3 around world Y.

    Orientation (rotation matrix) is preserved. Used by the simulation
    pipeline to remap replayed TCP motion onto the XY plane to match the
    design UI - see ``SIM_TRAJ_ROT_Y_RAD`` in :mod:`ur5_sim.config`.

    Imported locally from ``spatialmath`` to keep this module dependency
    light when callers only need the numpy helpers above.
    """
    from spatialmath import SE3  # local import to avoid module-level cost

    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    x, y, z = pose.t
    t_rot = [c * x + s * z, y, -s * x + c * z]
    return SE3.Rt(pose.R, t_rot)
