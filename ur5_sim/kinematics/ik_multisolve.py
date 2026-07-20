"""Enumerate the discrete IK branches that reach a Cartesian target pose.

The UR5 is a 6-DOF arm whose inverse kinematics admits up to eight
geometric solutions for a generic target (shoulder left/right, elbow
up/down, wrist up/down). The Levenberg-Marquardt solver in roboticstoolbox
converges to whichever solution is nearest to the seed, so we sample a
diverse pool of seeds, run IK from each, and deduplicate the survivors.
"""

from __future__ import annotations

import itertools

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

from ur5_sim.config import END_LINK
from ur5_sim.kinematics.transforms import tcp_tool_offset


# UR5 inverse kinematics has at most eight geometric branches, switched by the
# sign of the base, shoulder, elbow and wrist-1 joints. Sampling those four
# joints at two levels each (2^4 = 16 seeds) and holding the rest mid-range
# covers every branch with margin, while a full 3^6 = 729-seed grid spent ~700
# extra Levenberg-Marquardt solves discovering nothing new (the dominant cost of
# the first viewer launch). See ../../tcp_live trajectory cache for the second
# half of that launch-time fix.
_BRANCH_JOINTS: tuple[int, ...] = (0, 1, 2, 4)
_BRANCH_LEVELS: tuple[float, ...] = (0.25, 0.75)
_MID_LEVEL: float = 0.5


def enumerate_configurations(
    robot: rtb.Robot,
    target_pose: SE3,
    n_random_seeds: int = 0,
    dedup_tol: float = 0.10,
) -> list[np.ndarray]:
    """Return every distinct joint configuration that reaches ``target_pose``.

    Parameters
    ----------
    robot:
        The roboticstoolbox UR5 model.
    target_pose:
        Cartesian target as a spatialmath ``SE3``.
    n_random_seeds:
        Number of extra uniform random seeds drawn inside ``qlim`` on top of
        the 16 canonical branch seeds. Defaults to ``0`` - the deterministic
        seeds already span every UR5 branch. Raise it only to chase a
        suspected rare branch, at the cost of solver time.
    dedup_tol:
        Two solutions whose L2 distance in joint space is below this
        threshold (radians) are considered the same branch.

    Returns
    -------
    list[np.ndarray]
        Sorted list of joint vectors, each respecting the UR5 ``qlim``.
    """
    qlim_lo, qlim_hi = robot.qlim
    span = qlim_hi - qlim_lo

    seeds: list[np.ndarray] = []
    for combo in itertools.product(_BRANCH_LEVELS, repeat=len(_BRANCH_JOINTS)):
        frac = np.full(6, _MID_LEVEL)
        for joint, level in zip(_BRANCH_JOINTS, combo):
            frac[joint] = level
        seeds.append(qlim_lo + frac * span)
    if n_random_seeds > 0:
        rng = np.random.default_rng(0)
        for _ in range(n_random_seeds):
            seeds.append(qlim_lo + rng.random(6) * span)

    # ``target_pose`` is given in the TCP (finger tip) frame; convert to
    # the tool0 frame because the IK targets that link.
    target_tool0 = target_pose * tcp_tool_offset().inv()

    solutions: list[np.ndarray] = []
    for q0 in seeds:
        sol = robot.ikine_LM(
            target_tool0, q0=q0, end=END_LINK, ilimit=200, tol=1e-5,
        )
        if not sol.success:
            continue
        q = sol.q
        if np.any(q < qlim_lo - 1e-6) or np.any(q > qlim_hi + 1e-6):
            continue
        if any(np.linalg.norm(q - s) < dedup_tol for s in solutions):
            continue
        solutions.append(q)

    solutions.sort(key=lambda q: (q[0], q[1], q[2]))
    return solutions


def describe_configuration(q: np.ndarray) -> str:
    """Short human label for a configuration based on joint signs.

    The UR5 branch structure can be summarised by the sign of three
    geometric quantities: shoulder offset, elbow angle, wrist pitch.
    The label is meant to be readable in a UI dropdown, not exact.
    """
    shoulder = "L" if q[0] < 0 else "R"
    elbow = "U" if q[2] < 0 else "D"
    wrist = "U" if q[4] < 0 else "D"
    return f"Shoulder-{shoulder}/Elbow-{elbow}/Wrist-{wrist}"
