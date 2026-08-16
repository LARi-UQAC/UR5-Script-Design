"""
acq_emulator.py - development-machine mirror of the acquisition daemon peer.

Dev machine only. Never copied to the robot controller, and never imported
by onrobot/acq_logger_daemon.py (the daemon stays plain-stdlib, Python
2.7-compatible; this file may use type hints, f-strings, and import
ur5_sim freely - the reverse import is not allowed). It is the
acquisition-side counterpart of the fake RTDE server in
datalogger/tests/test_rtde_fallback_monitor.c, and it is what turns the
offline verification of plan_acq_datalogger.md, sections 3-ter and 7, into
a real end-to-end run instead of a unit test.

Two independent legs, each its own process (three-shell workflow, see
--help):

  --ft                 A fake FT-300 TCP server on port 63351 (the port
                        the daemon's FTReader connects out to), emitting
                        one Robotiq-format text frame every 10 ms.
  --robot <script>      A fake robot client that parses the real program
                        with ur5_sim.parsing.parse_poses and replays its
                        poses into the daemon's log port (50100) at 50 Hz,
                        alternating the CB3 thread's 16/24 ms tick pattern,
                        then sends the counting sentinel and "STOP".

The two legs never share a process, a clock, or an IPC channel, so the
--robot leg embeds its own force values (extended 7-field sample: the
ACQ_USE_INTERNAL_FORCE fallback wire format the daemon already parses)
computed from the real in_contact flag ur5_sim.parsing.parse_poses reports
for each pose. That keeps the produced CSV deterministic and reproducible
regardless of process-launch timing; the --ft leg is a separate, honest
exercise of the FT-300 wire format and reconnect path, not a dependency of
the --robot leg's force column.
"""

from __future__ import annotations

import argparse
import math
import random
import socket
import sys
import time
from pathlib import Path
from typing import Iterator, Optional

# onrobot/ is a plain subdirectory (no __init__.py, by design - it must
# stay import-invisible to the daemon), so "python onrobot/acq_emulator.py"
# puts onrobot/ itself on sys.path, not the repo root, and ur5_sim (a
# top-level sibling package) would not be found. Bootstrap the repo root
# onto sys.path before importing it, so the file runs the same way whether
# invoked as "python onrobot/acq_emulator.py" or from inside onrobot/.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ur5_sim.parsing import parse_poses  # noqa: E402 - see sys.path bootstrap above

DEFAULT_LOG_PORT = 50100
DEFAULT_FT_PORT = 63351
DEFAULT_RATE_HZ = 50.0
TICK_S = 0.008           # CB3 thread tick, 8 ms (plan_acq_datalogger.md sec. 4)
FORCE_TARGET_N = 6.0     # matches design.params.FORCE_Z_TARGET's default
FT_FRAME_PERIOD_S = 0.010  # one Robotiq frame every 10 ms
_RNG_SEED = 20260815     # fixed so a produced CSV is reproducible

_rng = random.Random(_RNG_SEED)


def _force_at(t: float, in_contact: bool) -> tuple[float, float, float]:
    """
    --------------------------------------------------------------------------
    Purpose:
        PLACEHOLDER force source: constant target plus a slow sine wiggle
        plus seeded noise. This is the ONE call site ur5_sim/force_model.py
        replaces, in one line, once the feat/rtde-emulator plan is
        implemented and merged (plan_acq_datalogger.md sec. 3-ter). The
        values returned here are plausible, not measured - nothing on this
        dev machine has ever touched an FT-300 sensor.
    Inputs:
        t (float): elapsed time in seconds (tick time for --robot, wall
            time since connection for --ft).
        in_contact (bool): True selects the ~6 N contact regime, False the
            near-zero transit regime.
    Outputs:
        force (tuple[float, float, float]): (Fx, Fy, Fz) in newtons.
    --------------------------------------------------------------------------
    """
    noise_x = (_rng.random() - 0.5) * 0.2
    noise_y = (_rng.random() - 0.5) * 0.2
    noise_z = (_rng.random() - 0.5) * 0.2
    if in_contact:
        fz = -(FORCE_TARGET_N + 0.3 * math.sin(2.0 * math.pi * 0.5 * t)) + noise_z
        return (noise_x, noise_y, fz)
    return (noise_x, noise_y, noise_z)


def _sample_periods(rate_hz: float) -> Iterator[float]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Yield successive inter-sample periods (s). At the default 50 Hz
        this reproduces the CB3 thread's exact 2-tick / 3-tick alternation
        (16 ms, 24 ms, mean 20.000 ms, plan sec. 4.3) so the Time column
        the --robot leg produces steps exactly like the real on-robot
        thread. Any other --rate is a plain constant period: the 16/24 ms
        alternation is a fact about the CB3 8 ms tick, not a property of an
        arbitrary sampling rate.
    Inputs:
        rate_hz (float): target sampling rate.
    Outputs:
        period (float): seconds until the next sample, yielded forever.
    --------------------------------------------------------------------------
    """
    if abs(rate_hz - DEFAULT_RATE_HZ) < 1e-9:
        long_tick = False
        while True:
            n_ticks = 3 if long_tick else 2
            long_tick = not long_tick
            yield n_ticks * TICK_S
    else:
        period = 1.0 / rate_hz
        while True:
            yield period


def _safe_close(closable) -> None:
    """Close a socket (or socket-file), ignoring any error."""
    try:
        closable.close()
    except OSError:
        pass


def _stream_ft_frames(conn: socket.socket, period_s: float) -> None:
    """
    --------------------------------------------------------------------------
    Purpose:
        Stream fake Robotiq FT-300 frames on an accepted connection, one
        every period_s, until the peer drops or the process is
        interrupted. Contact state follows an arbitrary fixed-period
        square wave: this leg has no channel to the --robot leg's real
        in_contact schedule (separate process, no shared clock, no IPC),
        so it is an independent placeholder exercise of the wire format,
        not a source of truth for the produced CSV's force column.
    Inputs:
        conn (socket.socket): accepted client connection (the daemon's
            FTReader).
        period_s (float): seconds between frames.
    Outputs:
        none (writes to conn until it closes or raises).
    --------------------------------------------------------------------------
    """
    start = time.perf_counter()
    square_wave_period_s = 4.0
    try:
        while True:
            t = time.perf_counter() - start
            in_contact = (t % square_wave_period_s) < (square_wave_period_s / 2.0)
            fx, fy, fz = _force_at(t, in_contact)
            line = f"({fx:.6f},{fy:.6f},{fz:.6f},0.000000,0.000000,0.000000)\n"
            conn.sendall(line.encode("ascii"))
            time.sleep(period_s)
    except OSError:
        print("[emulator] FT-300 client disconnected")
    finally:
        _safe_close(conn)


def run_ft_server(port: int) -> None:
    """Bind 127.0.0.1:port and serve fake FT-300 clients until Ctrl+C."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", port))
    server.listen(1)
    print(f"[emulator] fake FT-300 listening on 127.0.0.1:{port}")
    try:
        while True:
            conn, addr = server.accept()
            print(f"[emulator] FT-300 client connected from {addr}")
            _stream_ft_frames(conn, FT_FRAME_PERIOD_S)
    except KeyboardInterrupt:
        print("[emulator] fake FT-300 stopping")
    finally:
        _safe_close(server)


def run_robot_client(
    script_path: Path,
    port: int,
    rate_hz: float,
    max_samples: Optional[int],
) -> int:
    """
    --------------------------------------------------------------------------
    Purpose:
        Parse script_path with ur5_sim.parsing.parse_poses and replay its
        poses into the daemon's log server at 127.0.0.1:port, at rate_hz
        (default 50 Hz, tick-alternated per _sample_periods), embedding
        each pose's force via _force_at(t, in_contact). Finishes with the
        counting sentinel and the literal "STOP" (plan sec. 3, 3-ter),
        then reads and prints the daemon's reply.
    Inputs:
        script_path (Path): the real URScript program to replay.
        port (int): daemon log-server port.
        rate_hz (float): pacing rate; see _sample_periods.
        max_samples (int or None): cap on the number of poses sent.
    Outputs:
        exit_code (int): 0 if the daemon's reply starts with "OK", else 1.
    --------------------------------------------------------------------------
    """
    poses = parse_poses(script_path)
    if not poses:
        print(f"[emulator] no movel/movej pose found in {script_path}", file=sys.stderr)
        return 1
    if max_samples is not None:
        poses = poses[:max_samples]

    try:
        sock = socket.create_connection(("127.0.0.1", port), timeout=5.0)
    except OSError as exc:
        print(f"[emulator] cannot reach daemon on 127.0.0.1:{port}: {exc}", file=sys.stderr)
        return 1

    sock_file = sock.makefile("rb")
    try:
        period_gen = _sample_periods(rate_hz)
        t = 0.0
        next_deadline = time.perf_counter()
        sent = 0
        for _lineno, pose, _cycle_idx, in_contact in poses:
            period = next(period_gen)
            t += period
            next_deadline += period
            now = time.perf_counter()
            if next_deadline > now:
                time.sleep(next_deadline - now)
            x, y, z = pose[0], pose[1], pose[2]
            fx, fy, fz = _force_at(t, in_contact)
            line = f"[{t:.6f},{x:.6f},{y:.6f},{z:.6f},{fx:.6f},{fy:.6f},{fz:.6f}]\n"
            sock.sendall(line.encode("ascii"))
            sent += 1
        # Sentinelle de comptage, puis STOP litteral - miroir exact de
        # design/export.py._acq_stop_lines() (plan sec. 3, 3-ter).
        sock.sendall(f"[-1.0,{sent},0.0,0.0]\n".encode("ascii"))
        sock.sendall(b"STOP\n")
        raw_reply = sock_file.readline()
        reply = raw_reply.decode("ascii", "replace").strip()
        print(f"[emulator] daemon reply: {reply}")
        return 0 if reply.startswith("OK") else 1
    finally:
        _safe_close(sock_file)
        _safe_close(sock)


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the CLI parser; --help documents the two/three-shell workflow."""
    parser = argparse.ArgumentParser(
        prog="acq_emulator.py",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=(
            "Development-machine emulator for the acquisition daemon peer\n"
            "(onrobot/acq_logger_daemon.py). Dev machine only - never\n"
            "copied to the robot controller.\n\n"
            "Three-shell workflow:\n"
            "  shell 1: python onrobot/acq_logger_daemon.py\n"
            "  shell 2: python onrobot/acq_emulator.py --ft\n"
            "  shell 3: python onrobot/acq_emulator.py --robot etalement.script\n"
        ),
    )
    parser.add_argument(
        "--ft", action="store_true",
        help="Run the fake FT-300 TCP server on --ft-port (blocks until Ctrl+C).",
    )
    parser.add_argument(
        "--robot", metavar="SCRIPT", type=Path, default=None,
        help="Parse SCRIPT and replay its poses into the daemon's log port, then STOP.",
    )
    parser.add_argument(
        "--samples", type=int, default=None,
        help="Cap the number of poses --robot sends (exercise the 11700 buffer cap "
             "without waiting ~234 s).",
    )
    parser.add_argument(
        "--rate", type=float, default=DEFAULT_RATE_HZ,
        help="Override the --robot pacing rate in Hz (default: %(default)s).",
    )
    parser.add_argument(
        "--port", type=int, default=DEFAULT_LOG_PORT,
        help="Daemon log-server port for --robot (default: %(default)s).",
    )
    parser.add_argument(
        "--ft-port", type=int, default=DEFAULT_FT_PORT,
        help="Fake FT-300 server port for --ft (default: %(default)s).",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    """Dispatch to the --ft server or the --robot client per the parsed args."""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if not args.ft and args.robot is None:
        parser.error("choose --ft or --robot (run them in separate shells; see --help)")
    if args.ft:
        run_ft_server(args.ft_port)
        return 0
    return run_robot_client(args.robot, args.port, args.rate, args.samples)


if __name__ == "__main__":
    sys.exit(main())
