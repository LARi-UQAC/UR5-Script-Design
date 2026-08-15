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

import numpy as np
import roboticstoolbox as rtb

from ur5_sim.config import (
    CONTACT_SNAP_TOL_M,
    DT,
    P_ANCHOR_OLD_RAW,
    P_REF_RAW,
    SCRIPT_PATH,
    SIM_PROBE_ENABLE,
    SIM_PROBE_PLATE_DZ_M,
    SIM_PROBE_PLATE_TILT_X_RAD,
    SIM_PROBE_PLATE_TILT_Y_RAD,
    SIM_PROBE_RESIDUAL_TOL_M,
    SIM_PROBE_TILT_MAX_RAD,
    SIM_TRAJ_ROT_Y_RAD,
    SURFACE_CLEARANCE_M,
    SURFACE_ENABLE_CLAMP,
    SURFACE_FORCE_TARGET_DEPTH_M,
    SURFACE_FORCE_TARGET_TOL_M,
    URSCRIPT_MAX_TCP_SPEED_MPS,
    settings_summary,
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
    parse_nhat,
    parse_nominal_frame,
    parse_probe_blocks,
    parse_tcp_speed_globals,
    transform,
    urscript_pose,
)
from ur5_sim.probe import (
    apply_correction,
    build_virtual_plate,
    compute_meas_frame,
    signed_distance_to_plane,
    simulate_probe_descent,
)
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


def _run_probe_simulation(
    segments_for_residual: list,
) -> list[tuple[int, str, object]]:
    """Simulate ``probe_surface_plane()`` against a virtual plate.

    Returns a list of ``(lineno, kind, detail)`` events consumed by
    :func:`ur5_sim.reporting.text_report.report`. Skips silently with a
    ``PROBE_SKIPPED`` event if the script predates the 3-point probe block.
    """
    blocks = parse_probe_blocks(SCRIPT_PATH)
    nominal = parse_nominal_frame(SCRIPT_PATH)
    nhat = parse_nhat(SCRIPT_PATH)
    if not blocks or nominal is None or nhat is None:
        return [(0, "PROBE_SKIPPED",
                 "script lacks probe_surface_plane() block - sim coverage off")]
    if len(blocks) != 3:
        return [(blocks[0][1], "PROBE_SKIPPED",
                 f"expected 3 probe points, got {len(blocks)}")]

    virtual_plate = build_virtual_plate(
        nominal_frame_pose6=nominal,
        nhat_world=nhat,
        dz_m=SIM_PROBE_PLATE_DZ_M,
        tilt_x_rad=SIM_PROBE_PLATE_TILT_X_RAD,
        tilt_y_rad=SIM_PROBE_PLATE_TILT_Y_RAD,
    )

    events: list[tuple[int, str, object]] = []
    contacts: list[tuple[float, ...]] = []
    for idx, lineno, approach, floor in blocks:
        cp = simulate_probe_descent(approach, floor, virtual_plate)
        if cp is None:
            events.append((lineno, "PROBE_NO_CONTACT",
                           f"P{idx}: descente n'intersecte pas le plan virtuel "
                           f"(dz={SIM_PROBE_PLATE_DZ_M*1000:+.1f} mm, "
                           f"tilt_x={SIM_PROBE_PLATE_TILT_X_RAD:+.4f} rad, "
                           f"tilt_y={SIM_PROBE_PLATE_TILT_Y_RAD:+.4f} rad)"))
            return events
        contacts.append(cp)
        events.append((lineno, "PROBE_OK",
                       f"P{idx} contact a z = {cp[2]*1000:+.3f} mm "
                       f"(approche z = {approach[2]*1000:+.3f} mm)"))

    try:
        meas_frame, tilt_rad = compute_meas_frame(
            contacts[0], contacts[1], contacts[2], nhat, nominal,
        )
    except ValueError as exc:
        events.append((blocks[0][1], "PROBE_NO_CONTACT", str(exc)))
        return events

    if tilt_rad > SIM_PROBE_TILT_MAX_RAD:
        events.append((blocks[0][1], "PROBE_TILT_EXCEEDED",
                       f"tilt mesure = {tilt_rad:.4f} rad > "
                       f"SIM_PROBE_TILT_MAX_RAD = {SIM_PROBE_TILT_MAX_RAD:.4f} rad"))
        return events

    # Validation : prendre le premier waypoint in_contact du cycle 1 qui est
    # sur le plan nominal (ROBOT_Z_SURFACE), lui appliquer apply_correction
    # Python et verifier qu'il atterrit sur le plan virtuel a
    # SIM_PROBE_RESIDUAL_TOL_M pres. Le tout premier in_contact emis par le
    # generateur est la "descente profonde" (z = ROBOT_Z_SURFACE -
    # FORCE_CONTACT_DEPTH = ~5 mm sous le plan) : le controleur de force
    # arrete reellement le robot au contact, mais le sim sans physique
    # verrait un faux residu. On filtre donc les poses sous le plan nominal
    # (le sim ne corrige que les vrais waypoints de trajectoire).
    sample_lineno = None
    sample_pose = None
    n = np.asarray((nhat[0], nhat[1], nhat[2]), dtype=float)
    o = np.asarray(nominal[:3], dtype=float)
    # Iterate over the ORIGINAL parsed segments (not the densified buffer):
    # SE3-slerp substeps between transit_in and contact_deep would otherwise
    # appear at z = +Z_TRANSIT above the plane and corrupt the residual.
    for seg in segments_for_residual:
        if seg.cycle_idx != 1 or not seg.in_contact:
            continue
        p_xyz = np.asarray(seg.pose[:3], dtype=float)
        signed = float(np.dot(p_xyz - o, n))
        if signed < -1e-4:  # >0.1 mm sous le plan nominal = cible force, skip
            continue
        sample_lineno = seg.lineno
        sample_pose = seg.pose
        break
    if sample_pose is None:
        events.append((blocks[0][1], "PROBE_SKIPPED",
                       "no in_contact waypoint in cycle 1 - residual not checked"))
        return events

    corrected = apply_correction(sample_pose, meas_frame, nominal)
    residual_m = signed_distance_to_plane(corrected.t, virtual_plate)
    if abs(residual_m) > SIM_PROBE_RESIDUAL_TOL_M:
        events.append((sample_lineno, "PROBE_RESIDUAL",
                       f"residu = {residual_m*1000:+.4f} mm > tol = "
                       f"{SIM_PROBE_RESIDUAL_TOL_M*1000:.4f} mm "
                       f"(tilt reconstruit = {tilt_rad:.4f} rad)"))
    else:
        events.append((sample_lineno, "PROBE_OK",
                       f"residu post-correction = {residual_m*1000:+.4f} mm "
                       f"(tilt reconstruit = {tilt_rad:.4f} rad)"))
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
        for (lineno, pose_tf), in_contact in zip(
            poses_xform, in_contact_per_frame,
        ):
            pose_out, _kind, _depth = apply_surface_constraint(
                pose_tf, surface_frame, in_contact, SURFACE_CLEARANCE_M,
            )
            constrained.append((lineno, pose_out))
        poses_xform = constrained

    # ------------------------------------------------------------------
    # Sondage 3 points (L4) : rejoue probe_surface_plane() contre un plan
    # virtuel parametre dans config.SIM_PROBE_*. Verifie la reachabilite
    # des poses d'approche, l'intersection descente / plan, la
    # reconstruction MEAS_FRAME et le residu post-apply_correction.
    #
    # DESACTIVE - A REVOIR (rework futur) : le sondage 3 points est INCORRECT
    # (fixe en Z). SIM_PROBE_ENABLE = False => probe_events reste vide. La
    # fonction _run_probe_simulation et ur5_sim/probe.py sont conservees (non
    # supprimees) pour le rework. L'export emet desormais probe_surface_z.
    # ------------------------------------------------------------------
    probe_events = _run_probe_simulation(segments) if SIM_PROBE_ENABLE else []
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
        visualize(
            robot,
            labelled_trajectories,
            cycle_per_frame=cycle_per_frame,
            plate_xy_per_frame=plate_xy_per_frame,
            surface=surface_frame,
            in_contact_per_frame=in_contact_per_frame,
        )

    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
