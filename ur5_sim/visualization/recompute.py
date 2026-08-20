"""
ur5_sim/visualization/recompute.py - re-parse the script and re-solve IK, off the GUI thread.

Moved verbatim out of ``viewer.visualize``'s ``_recompute_worker`` closure. The
block was already documented there as pure numpy/kinematics, so it is the one
part of the viewer that owes nothing to matplotlib or Swift and can live on its
own. START re-runs it every time so the viewer never replays a trajectory that
is out of date with ``etalement.script`` on disk.

The caller (``ur5_sim.visualization.playback``) owns the shared ``recompute``
dict and its lock; this module only fills it.
"""

from __future__ import annotations

import roboticstoolbox as rtb

from ur5_sim.config import (
    DT,
    SCRIPT_PATH,
    SIM_TRAJ_ROT_Y_RAD,
    SURFACE_CLEARANCE_M,
    SURFACE_ENABLE_CLAMP,
    URSCRIPT_MAX_TCP_SPEED_MPS,
)
from ur5_sim.kinematics.ik import run_ik
from ur5_sim.kinematics.motion import densify_segments
from ur5_sim.kinematics.transforms import rotate_translation_y
from ur5_sim.parsing.urscript import (
    parse_motion_segments,
    transform,
    urscript_pose,
)
from ur5_sim.visualization.surface import apply_surface_constraint


def recompute_all_branches(
    trajectories: list[tuple[str, list]],
    surface: dict | None,
    p_anchor_old,
    p_ref,
    recompute: dict,
) -> None:
    """
    --------------------------------------------------------------------------
    Purpose:
        Re-parse the script and solve IK for every branch, off the GUI thread.

        Pure numpy/kinematics only (no matplotlib, no Swift). Uses a private
        UR5 instance so it never races the main thread's ``robot.q`` writes.
        Results and progress are published under ``recompute['lock']``; the
        timer's ``tick`` consumes them on the main thread.

    Inputs:
        trajectories (list): current ``(label, joint_trajectory)`` pairs; read
            only, for the branch labels and the IK seeds.
        surface (dict | None): surface frame, or None when clamping is off.
        p_anchor_old: legacy anchor pose (SE3).
        p_ref: current reference pose (SE3).
        recompute (dict): shared handoff dict carrying ``lock``, ``done``,
            ``total``, ``ready``, ``payload`` and ``error``.

    Outputs:
        None. Everything is published into ``recompute``.
    --------------------------------------------------------------------------
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
