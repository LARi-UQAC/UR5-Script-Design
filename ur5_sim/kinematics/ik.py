"""Sequential inverse kinematics with joint-limit reporting.

Each pose is solved with a Levenberg-Marquardt solver seeded by the
previous solution to keep the joint trajectory continuous. Failures - either
non-convergence or violation of the UR5 ``qlim`` envelope - are accumulated
into a list of triples that the reporting layer turns into a punch list.

Targets received here are **TCP poses** (finger tip frame). They are
converted internally to ``wrist_3_link`` poses via :func:`tcp_tool_offset`
before being handed to ``ikine_LM`` because the rtb UR5 chain ends at
``wrist_3_link`` and does not bake the flange + tool offset into the URDF.
"""

from __future__ import annotations

from typing import Callable, Optional

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

from ur5_sim.config import END_LINK
from ur5_sim.kinematics.transforms import tcp_tool_offset


def run_ik(
    robot: rtb.Robot,
    poses_xform: list[tuple[int, SE3]],
    q_seed: np.ndarray,
    progress: Optional[Callable[[int, int], None]] = None,
) -> tuple[list[np.ndarray], list[tuple[int, str, object]]]:
    """Solve IK for each transformed pose, return the joint trajectory and failures.

    Parameters
    ----------
    robot:
        The roboticstoolbox UR5 model whose kinematic envelope is being
        checked.
    poses_xform:
        Sequence of ``(line_number, target_SE3)`` pairs, already transformed
        from the source ``P_ANCHOR_OLD`` frame to the active ``P_REF`` frame.
    q_seed:
        Initial joint configuration; usually ``robot.qr``. Seeding with the
        previous solution avoids configuration jumps between adjacent
        strokes.
    progress:
        Optional callback invoked once per pose as ``progress(done, total)``
        with ``1 <= done <= total == len(poses_xform)``. Used by the viewer's
        background recompute to drive a "Recomputing k/N" HUD without touching
        the GUI from the solver. ``None`` (default) keeps the original
        behaviour.

    Returns
    -------
    trajectory:
        List of joint vectors ``q`` aligned with ``poses_xform``. On IK
        failure the previous configuration is repeated so the trajectory
        stays the right length.
    failures:
        List of ``(line_number, kind, detail)``. ``kind`` is ``IK_FAIL``
        when the solver did not converge or ``JOINT_LIMIT`` when at least
        one joint went outside ``robot.qlim``. ``detail`` is either the
        target Cartesian position or the per-joint margin in degrees.
    """
    q = q_seed.copy()
    trajectory: list[np.ndarray] = []
    failures: list[tuple[int, str, object]] = []
    qlim_lo, qlim_hi = robot.qlim
    tool_offset_inv = tcp_tool_offset().inv()
    n_total = len(poses_xform)

    for k, (lineno, target) in enumerate(poses_xform):
        target_tool0 = target * tool_offset_inv
        sol = robot.ikine_LM(
            target_tool0, q0=q, end=END_LINK, ilimit=200, tol=1e-4,
        )
        if not sol.success:
            failures.append((lineno, "IK_FAIL", tuple(np.round(target.t, 4))))
            trajectory.append(q.copy())
            if progress is not None:
                progress(k + 1, n_total)
            continue
        q_new = sol.q
        below = q_new < qlim_lo
        above = q_new > qlim_hi
        if np.any(below) or np.any(above):
            margin_deg = np.degrees(
                np.minimum(q_new - qlim_lo, qlim_hi - q_new)
            )
            failures.append(
                (lineno, "JOINT_LIMIT", tuple(np.round(margin_deg, 2)))
            )
        q = q_new
        trajectory.append(q.copy())
        if progress is not None:
            progress(k + 1, n_total)

    return trajectory, failures
