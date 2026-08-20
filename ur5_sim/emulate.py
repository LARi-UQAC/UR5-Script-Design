"""
Command-line glue between ur5_sim and the RTDE emulator.

Holds the argparse flags, the default resolution, the server construction and
the headless ``--emulate`` entry point, so ``cli.py`` gains four call sites
instead of a hundred lines (that file already sits at the workspace per-file
token ceiling).

The buffers handed to the server are the SURFACE-CLAMPED poses, the same array
``run_ik`` receives. A CSV recorded from this stream can therefore be compared
against the commanded path rather than against an idealised one.

Design: ../docs/superpower/specs/spec_rtde_emulator.md
"""

from __future__ import annotations

import argparse

from ur5_sim.config import (
    FORCE_MODEL_FRICTION_MU,
    FORCE_MODEL_NOISE_N,
    FORCE_MODEL_SEED,
    FORCE_MODEL_STIFFNESS_N_PER_M,
    FORCE_MODEL_TAU_S,
    FORCE_Z_TARGET_N,
    RTDE_EMU_HOST,
    RTDE_EMU_IDLE_S,
    RTDE_EMU_PORT,
    RTDE_EMU_RATE_HZ,
    RTDE_EMU_TRANSITION_PACKETS,
)
from ur5_sim.force_model import ForceModel
from ur5_sim.rtde_server import RtdeServer, run_headless


def add_rtde_arguments(parser: argparse.ArgumentParser) -> None:
    """
    --------------------------------------------------------------------------
    Purpose:
        Declare the emulator flags on the ur5_sim parser. No host flag is
        offered on purpose: the server must stay unreachable from the lab
        VLAN, so the bind address is the RTDE_EMU_HOST constant and nothing
        on the command line can widen it.

    Inputs:
        parser (argparse.ArgumentParser): the ur5_sim parser.

    Outputs:
        None. The parser is mutated in place.
    --------------------------------------------------------------------------
    """
    parser.add_argument(
        "--rtde-serve", dest="rtde_serve", action="store_true", default=None,
        help="serve the RTDE emulator (default: on with --visualize)")
    parser.add_argument(
        "--no-rtde-serve", dest="rtde_serve", action="store_false",
        help="do not open the RTDE emulator socket")
    parser.add_argument(
        "--rtde-port", type=int, default=RTDE_EMU_PORT,
        help=f"RTDE emulator port (default {RTDE_EMU_PORT}, loopback only)")
    parser.add_argument(
        "--emulate", action="store_true",
        help="headless real-time RTDE emulation, no Swift and no matplotlib")
    parser.add_argument(
        "--runs", type=int, default=1,
        help="with --emulate: number of consecutive runs")
    parser.add_argument(
        "--pause-at", type=float, default=None,
        help="with --emulate: pause once at this simulation time, seconds")


def wants_rtde(args: argparse.Namespace) -> bool:
    """
    --------------------------------------------------------------------------
    Purpose:
        Decide whether to open the emulator socket. On by default with
        --visualize and implied by --emulate; never with --check, which has no
        real-time pacing, so a three-minute protocol would stream in seconds
        and produce timestamps resembling no trial.

    Inputs:
        args (argparse.Namespace): parsed command line.

    Outputs:
        wanted (bool): True to start the RTDE server.
    --------------------------------------------------------------------------
    """
    if getattr(args, "emulate", False):
        return True
    if not getattr(args, "visualize", False):
        # Nothing on the --check path drives the run state, so a socket
        # opened here would sit at STOPPED and record nothing. Say so
        # instead of honouring the flag into a dead end.
        if getattr(args, "rtde_serve", False):
            print("[rtde-emu] --rtde-serve ignored: --check has no real-time "
                  "pacing. Use --visualize or --emulate.")
        return False
    if getattr(args, "rtde_serve", None) is not None:
        return bool(args.rtde_serve)
    return True


def build_rtde_server(port: int) -> "RtdeServer | None":
    """
    --------------------------------------------------------------------------
    Purpose:
        Build and start the emulator with the documented force surrogate.
        A busy port degrades to None instead of raising: the visualizer must
        run with or without the emulator.

    Inputs:
        port (int): TCP port on RTDE_EMU_HOST; 0 asks the OS for a free one.

    Outputs:
        server (RtdeServer | None): started server, or None if unavailable.
    --------------------------------------------------------------------------
    """
    model = ForceModel(
        stiffness_n_per_m=FORCE_MODEL_STIFFNESS_N_PER_M,
        tau_s=FORCE_MODEL_TAU_S,
        friction_mu=FORCE_MODEL_FRICTION_MU,
        noise_n=FORCE_MODEL_NOISE_N,
        seed=FORCE_MODEL_SEED,
        target_n=FORCE_Z_TARGET_N,
    )
    server = RtdeServer(
        host=RTDE_EMU_HOST, port=port, rate_hz=RTDE_EMU_RATE_HZ,
        force_model=model,
        transition_packets=RTDE_EMU_TRANSITION_PACKETS,
    )
    print("[rtde-emu] force values are a documented surrogate with plausible, "
          "NOT measured parameters (ur5_sim/config.py FORCE_MODEL_*)")
    return server if server.start() else None


def poses_to_xyzrpy(
    poses_xform: list[tuple[int, object]],
) -> list[tuple[float, ...]]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Flatten the ``(lineno, SE3)`` pairs the IK consumes into the six-value
        RTDE ``actual_TCP_pose`` form. Orientation is the axis-angle rotation
        vector URScript uses, so ``SO3.EulerVec`` (used to read the script) and
        ``SE3.eulervec()`` (used here) are exact inverses.

    Inputs:
        poses_xform (list): (line number, TCP pose SE3) pairs, surface-clamped.

    Outputs:
        poses (list[tuple[float, ...]]): one (x, y, z, rx, ry, rz) per frame.
    --------------------------------------------------------------------------
    """
    out: list[tuple[float, ...]] = []
    for _lineno, pose in poses_xform:
        x, y, z = (float(v) for v in pose.t)
        rx, ry, rz = (float(v) for v in pose.eulervec())
        out.append((x, y, z, rx, ry, rz))
    return out


def penetration_from_depth(in_contact: bool, depth: float) -> float:
    """
    --------------------------------------------------------------------------
    Purpose:
        Normalise what apply_surface_constraint reports into the
        positive-downward depth ForceModel expects. During contact the value
        is the signed pre-snap offset along the surface normal (negative below
        the plane, which is where the deliberate recontact overshoot sits);
        during transit it is already a positive penetration.

    Inputs:
        in_contact (bool): True between force_mode and end_force_mode.
        depth (float): third element of apply_surface_constraint's result, m.

    Outputs:
        penetration (float): depth below the plane in metres, never negative.
    --------------------------------------------------------------------------
    """
    return max(0.0, -float(depth) if in_contact else float(depth))


def run_emulation(
    poses_xyzrpy: list[tuple[float, ...]],
    times: list[float],
    in_contact: list[bool],
    penetration_m: list[float],
    args: argparse.Namespace,
) -> int:
    """
    --------------------------------------------------------------------------
    Purpose:
        Serve one trajectory in real time, headless, for as many consecutive
        runs as asked. This is the path a lab operator uses to exercise
        datalogger/rtde_fallback_monitor.exe with no robot and no browser.

    Inputs:
        poses_xyzrpy (list): per-frame (x, y, z, rx, ry, rz), clamped.
        times (list[float]): per-frame simulation timestamps, seconds.
        in_contact (list[bool]): per-frame contact flag.
        penetration_m (list[float]): per-frame depth below the plane, m.
        args (argparse.Namespace): parsed command line (rtde_port, runs,
            pause_at).

    Outputs:
        code (int): 0 on a completed emulation, 1 if the port was unavailable.
    --------------------------------------------------------------------------
    """
    server = build_rtde_server(args.rtde_port)
    if server is None:
        return 1
    total_sim_time = float(times[-1]) if times else 0.0
    print(f"[rtde-emu] {len(poses_xyzrpy)} frames, {total_sim_time:.2f} s per "
          f"run, {max(1, int(args.runs))} run(s); Ctrl-C to stop")
    try:
        server.load_run(
            poses=poses_xyzrpy, times=times,
            in_contact=in_contact, penetration_m=penetration_m,
        )
        run_headless(
            server=server, total_sim_time=total_sim_time,
            runs=args.runs, pause_at=args.pause_at, idle_s=RTDE_EMU_IDLE_S,
        )
    except KeyboardInterrupt:
        print("\n[rtde-emu] interrupted")
    finally:
        server.stop()
    return 0
