"""Command-line entry point.

Three modes:
    --check       : parse the script, run the IK sweep, print a text report.
    --visualize   : same as --check, then open the matplotlib viewer.
    --identity    : override P_REF with P_ANCHOR_OLD (used to validate the
                    refactor itself by reproducing the original behaviour).
"""

from __future__ import annotations

import argparse
import sys

import roboticstoolbox as rtb

from ur5_sim.config import (
    CONTACT_SNAP_TOL_M,
    DT,
    P_ANCHOR_OLD_RAW,
    P_REF_RAW,
    SCRIPT_PATH,
    SIM_PROBE_ENABLE,
    SIM_TRAJ_ROT_Y_RAD,
    SURFACE_CLEARANCE_M,
    SURFACE_ENABLE_CLAMP,
    SURFACE_FORCE_TARGET_DEPTH_M,
    SURFACE_FORCE_TARGET_TOL_M,
    URSCRIPT_MAX_TCP_SPEED_MPS,
    settings_summary,
)
from ur5_sim.emulate import (
    add_rtde_arguments,
    build_rtde_server,
    penetration_from_depth,
    poses_to_xyzrpy,
    run_emulation,
    wants_rtde,
)
from ur5_sim.kinematics.ik import run_ik
from ur5_sim.kinematics.ik_multisolve import (
    describe_configuration,
    enumerate_configurations,
)
from ur5_sim.kinematics.motion import densify_segments
from ur5_sim.kinematics.transforms import rotate_translation_y
from ur5_sim.parsing.urscript import (
    parse_motion_segments,
    parse_tcp_speed_globals,
    transform,
    urscript_pose,
)
from ur5_sim.probe import run_probe_simulation
from ur5_sim.reporting.text_report import report
from ur5_sim.visualization.surface import (
    apply_surface_constraint,
    compute_surface_frame,
)


def _is_force_target_depth(kind: str, depth_m: float) -> bool:
    """Return True if a SURFACE_DEVIATION matches the deliberate force-mode target.

    The generator emits a ``pose_contact_deep`` waypoint at ``z =
    ROBOT_Z_SURFACE - FORCE_CONTACT_DEPTH`` so the on-robot force regulator
    has authority to stop the descent before mechanical contact. The
    simulator has no force model, so the pre-snap pose sits exactly
    ``FORCE_CONTACT_DEPTH`` below the plane. We treat that depth as
    "expected" rather than a deviation : the real robot never goes there,
    the regulator does its job. Genuine bugs (wrong pose, wrong frame)
    produce depths outside the +/- tolerance window and still surface.
    """
    if kind != "SURFACE_DEVIATION":
        return False
    return (
        -SURFACE_FORCE_TARGET_DEPTH_M - SURFACE_FORCE_TARGET_TOL_M
        <= depth_m
        <= -SURFACE_FORCE_TARGET_DEPTH_M + SURFACE_FORCE_TARGET_TOL_M
    )


def _run_speed_limit_check() -> list[tuple[int, str, object]]:
    """Verify TCP-speed globals stay under URSCRIPT_MAX_TCP_SPEED_MPS.

    Reads the ``global <NAME> = <value>`` declarations at the top of the
    script and emits ``SPEED_LIMIT_EXCEEDED`` event(s) for any movel speed
    above the PolyScope plafond. Used to catch the case where a user edits
    the .script by hand and skips the generator clamp.
    """
    speeds = parse_tcp_speed_globals(SCRIPT_PATH)
    events: list[tuple[int, str, object]] = []
    for name, value in sorted(speeds.items()):
        if value > URSCRIPT_MAX_TCP_SPEED_MPS:
            events.append((0, "SPEED_LIMIT_EXCEEDED",
                           f"{name} = {value:.4f} m/s > "
                           f"URSCRIPT_MAX_TCP_SPEED_MPS = "
                           f"{URSCRIPT_MAX_TCP_SPEED_MPS:.4f} m/s"))
    return events


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="ur5_sim",
        description="Offline validation and replay for the UR5 etalement trajectory.",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Parse + IK + report (default behaviour).",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="After the check, open the matplotlib three-panel viewer.",
    )
    parser.add_argument(
        "--identity",
        action="store_true",
        help="Force P_REF = P_ANCHOR_OLD so the refactor itself can be tested.",
    )
    add_rtde_arguments(parser)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    # D'ou viennent les valeurs qui vont servir a valider, et quand elles ont
    # ete lues : sans cette ligne, un rapport vert ne dit pas contre quels
    # reglages il est vert (plan_variables_UI.md, sections 3.3 et 8).
    print(settings_summary())
    print()

    p_anchor_old = urscript_pose(*P_ANCHOR_OLD_RAW)
    if args.identity:
        p_ref = p_anchor_old
        print("[--identity] P_REF overridden to P_ANCHOR_OLD\n")
    else:
        p_ref = urscript_pose(*P_REF_RAW)

    segments = parse_motion_segments(SCRIPT_PATH)
    if not segments:
        print(
            f"ERROR: no movel/movej pose literals found in {SCRIPT_PATH}",
            file=sys.stderr,
        )
        return 2
    # Densify each segment into DT-sized substeps so the viewer animates
    # at the wall-clock velocity declared on every movel line.
    parsed, segment_events = densify_segments(
        segments, DT, URSCRIPT_MAX_TCP_SPEED_MPS,
    )

    poses_xform = []
    for lineno, pose, _cycle, _in_contact in parsed:
        pose_tf = transform(urscript_pose(*pose), p_anchor_old, p_ref)
        pose_tf = rotate_translation_y(pose_tf, SIM_TRAJ_ROT_Y_RAD)
        poses_xform.append((lineno, pose_tf))
    cycle_per_frame: list[int] = [cycle for _lineno, _pose, cycle, _ in parsed]
    in_contact_per_frame: list[bool] = [
        in_contact for _lineno, _pose, _cycle, in_contact in parsed
    ]
    # ``plate_xy_per_frame`` keeps the X/Y as they appear in the script
    # (post ``plate_to_robot`` mapping, expressed in metres in the
    # P_ANCHOR_OLD frame). The design UI inverts ``plate_to_robot`` to
    # recover the plate-frame mm coordinates used by its cycle subplots.
    plate_xy_per_frame: list[tuple[float, float]] = [
        (pose[0], pose[1]) for _lineno, pose, _cycle, _ in parsed
    ]
    n_cycles = max(cycle_per_frame) if cycle_per_frame else 0
    # Reporting layer still consumes (lineno, pose) pairs.
    parsed_for_report = [(lineno, pose) for lineno, pose, _cycle, _ in parsed]

    # ------------------------------------------------------------------
    # Surface de test + contrainte de force 6 N (surrogate cinematique).
    # Pendant les frames ``in_contact`` (entre force_mode / end_force_mode),
    # le TCP est plaque sur le plan de la surface. En transit, il est
    # uniquement clampe par le dessous. Toute deviation est consignee dans
    # ``surface_events`` et propagee a ``report`` cote text_report.
    # ------------------------------------------------------------------
    surface_frame = compute_surface_frame(p_anchor_old, p_ref)
    surface_events: list[tuple[int, str, object]] = []
    # Depth below the plane, positive downward, per densified frame. Only the
    # RTDE force surrogate reads it, and it has to be captured here: after the
    # clamp the tool rides the plane, so the deliberate recontact overshoot -
    # the one event that makes the synthesised Fz look like a real trial - is
    # no longer recoverable from the poses.
    penetration_per_frame: list[float] = [0.0] * len(poses_xform)
    if SURFACE_ENABLE_CLAMP:
        # Audit on the ORIGINAL (un-densified) segment poses. Densified
        # substeps are pure SE3 slerp interpolants between two parsed
        # poses; any pre-snap deviation they exhibit is by construction
        # an artifact of the interpolation (e.g. the recontact descent
        # slerps from +Z_TRANSIT down through the plane to
        # -FORCE_CONTACT_DEPTH and would otherwise spam dozens of
        # SURFACE_DEVIATION events per cycle). Only the script-declared
        # waypoints can carry a genuine surface violation.
        for seg in segments:
            pose_tf_seg = transform(
                urscript_pose(*seg.pose), p_anchor_old, p_ref,
            )
            pose_tf_seg = rotate_translation_y(pose_tf_seg, SIM_TRAJ_ROT_Y_RAD)
            _, kind, depth = apply_surface_constraint(
                pose_tf_seg, surface_frame, seg.in_contact,
                SURFACE_CLEARANCE_M,
            )
            if abs(depth) > CONTACT_SNAP_TOL_M and not _is_force_target_depth(
                kind, depth,
            ):
                surface_events.append(
                    (seg.lineno, kind, round(depth * 1000.0, 3)),  # mm signe
                )

        # Apply the clamp silently on the densified buffer so the IK
        # solver and the viewer see a trajectory that rides on the
        # surface during contact (no event collection here).
        constrained = []
        penetration_per_frame = []
        for (lineno, pose_tf), in_contact in zip(
            poses_xform, in_contact_per_frame,
        ):
            pose_out, _kind, depth = apply_surface_constraint(
                pose_tf, surface_frame, in_contact, SURFACE_CLEARANCE_M,
            )
            constrained.append((lineno, pose_out))
            penetration_per_frame.append(
                penetration_from_depth(in_contact, depth))
        poses_xform = constrained

    # Frame i is commanded at i * DT seconds into the run: DT is the
    # densification step, so this is the trajectory's own time base.
    frame_times = [i * DT for i in range(len(poses_xform))]

    if args.emulate:
        # Returns BEFORE the IK sweep on purpose. That sweep costs about half
        # a minute on the trial script, and the operator in the other terminal
        # is waiting for the socket, not for a validation report. Run
        # ``--check`` when the report is what is wanted.
        return run_emulation(
            poses_to_xyzrpy(poses_xform), frame_times,
            in_contact_per_frame, penetration_per_frame, args,
        )

    # ------------------------------------------------------------------
    # Sondage 3 points (L4) : rejoue probe_surface_plane() contre un plan
    # virtuel parametre dans config.SIM_PROBE_*. Verifie la reachabilite
    # des poses d'approche, l'intersection descente / plan, la
    # reconstruction MEAS_FRAME et le residu post-apply_correction.
    #
    # DESACTIVE - A REVOIR (rework futur) : le sondage 3 points est INCORRECT
    # (fixe en Z). SIM_PROBE_ENABLE = False => probe_events reste vide. La
    # fonction run_probe_simulation et ur5_sim/probe.py sont conservees (non
    # supprimees) pour le rework. L'export emet desormais probe_surface_z.
    # ------------------------------------------------------------------
    probe_events = run_probe_simulation(segments) if SIM_PROBE_ENABLE else []
    speed_events = _run_speed_limit_check()

    robot = rtb.models.UR5()
    trajectory, failures = run_ik(robot, poses_xform, robot.qr)
    failures = (
        speed_events
        + segment_events
        + probe_events
        + surface_events
        + failures
    )
    report(parsed_for_report, failures)

    if args.visualize:
        first_target = poses_xform[0][1]
        print("\nEnumerating IK branches for the initial pose...")
        configs = enumerate_configurations(robot, first_target)
        if not configs:
            print("  no analytic branches found - falling back to the default seed.")
            configs = [robot.qr]
        else:
            print(f"  found {len(configs)} configuration(s).")

        labelled_trajectories: list[tuple[str, list]] = []
        for i, q0 in enumerate(configs, start=1):
            traj_i, fails_i = run_ik(robot, poses_xform, q0)
            tag = describe_configuration(q0)
            label = f"#{i} {tag} ({len(fails_i)} fail)"
            print(f"  [{i}/{len(configs)}] {label}")
            labelled_trajectories.append((label, traj_i))

        from ur5_sim.visualization.viewer import visualize
        print(f"\nDetected {n_cycles} cycle(s) in the script.")
        rtde_server = (
            build_rtde_server(args.rtde_port) if wants_rtde(args) else None
        )
        visualize(
            robot,
            labelled_trajectories,
            cycle_per_frame=cycle_per_frame,
            plate_xy_per_frame=plate_xy_per_frame,
            surface=surface_frame,
            in_contact_per_frame=in_contact_per_frame,
            poses_xyzrpy_per_frame=poses_to_xyzrpy(poses_xform),
            penetration_per_frame=penetration_per_frame,
            rtde_server=rtde_server,
        )

    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
