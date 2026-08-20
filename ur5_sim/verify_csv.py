"""
Check a CSV recorded by datalogger/rtde_fallback_monitor.exe against the
trajectory the simulator commanded.

The primary check is geometric - the distance from each recorded point to the
commanded polyline - because it survives a pause. Controller time keeps
advancing while simulation time freezes, so a purely time-indexed comparison
would report a false failure on any paused run.

This module also carries the CLI glue for ``--verify-csv`` (the newest-file
lookup and the report printer). ``ur5_sim/cli.py`` sits near the workspace
per-file token ceiling, so the glue lives here instead, alongside the checks
it drives, rather than growing that file further.
"""

from __future__ import annotations

import argparse
import glob
import math
import os
from typing import Sequence


def parse_monitor_csv(path: str) -> tuple[list[str], list[tuple[float, ...]]]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Split a monitor CSV into its comment header and its data rows.

    Inputs:
        path (str): CSV written by the monitor.

    Outputs:
        header (list[str]): the leading comment lines.
        rows (list[tuple[float, ...]]): (Time, Fx, Fy, Fz, X, Y, Z) per row.
    --------------------------------------------------------------------------
    """
    header: list[str] = []
    rows: list[tuple[float, ...]] = []
    with open(path, "r", encoding="ascii") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                header.append(line)
            elif line.startswith("Time,"):
                continue
            else:
                rows.append(tuple(float(v) for v in line.split(",")))
    return header, rows


def _segment_distance(
    p: tuple[float, float, float],
    a: Sequence[float],
    b: Sequence[float],
) -> float:
    abx, aby, abz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    apx, apy, apz = p[0] - a[0], p[1] - a[1], p[2] - a[2]
    denom = abx * abx + aby * aby + abz * abz
    t = 0.0 if denom <= 0.0 else (apx * abx + apy * aby + apz * abz) / denom
    t = min(1.0, max(0.0, t))
    dx = apx - abx * t
    dy = apy - aby * t
    dz = apz - abz * t
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def distance_to_polyline(
    point: tuple[float, float, float],
    polyline: Sequence[Sequence[float]],
) -> float:
    """Shortest distance from a point to a polyline, metres."""
    if len(polyline) == 1:
        a = polyline[0]
        return math.dist(point, (a[0], a[1], a[2]))
    return min(
        _segment_distance(point, polyline[i], polyline[i + 1])
        for i in range(len(polyline) - 1)
    )


def verify(
    csv_path: str,
    polyline: Sequence[Sequence[float]],
    tol_m: float,
) -> dict:
    """
    --------------------------------------------------------------------------
    Purpose:
        Check that every recorded point lies on the commanded path, that time
        increases, and report the effective rate and any pause gaps.

    Inputs:
        csv_path (str): CSV written by the monitor.
        polyline (Sequence[Sequence[float]]): commanded XYZ, surface-clamped.
        tol_m (float): maximum acceptable deviation, metres.

    Outputs:
        report (dict): passed, rows, max_dev_m, rms_dev_m, rate_hz,
            time_monotonic, gaps, simulated_source.
    --------------------------------------------------------------------------
    """
    header, rows = parse_monitor_csv(csv_path)
    if not rows:
        return {"passed": False, "rows": 0, "max_dev_m": float("inf"),
                "rms_dev_m": float("inf"), "rate_hz": 0.0,
                "time_monotonic": False, "gaps": [], "simulated_source": False,
                "reason": "no data rows"}

    deviations = [
        distance_to_polyline((r[4], r[5], r[6]), polyline) for r in rows
    ]
    max_dev = max(deviations)
    rms_dev = math.sqrt(sum(d * d for d in deviations) / len(deviations))

    times = [r[0] for r in rows]
    monotonic = all(b > a for a, b in zip(times, times[1:]))
    span = times[-1] - times[0]
    rate = (len(times) - 1) / span if span > 0 else 0.0

    # A gap well beyond the 20 ms grid is a pause, not a dropout.
    gaps = [(a, b) for a, b in zip(times, times[1:]) if (b - a) > 0.100]

    simulated = any("127.0.0.1" in line or "SIMULATED SOURCE" in line
                    for line in header)

    return {
        "passed": bool(max_dev <= tol_m and monotonic),
        "rows": len(rows),
        "max_dev_m": max_dev,
        "rms_dev_m": rms_dev,
        "rate_hz": rate,
        "time_monotonic": monotonic,
        "gaps": gaps,
        "simulated_source": simulated,
    }


def add_verify_csv_argument(parser: argparse.ArgumentParser) -> None:
    """
    --------------------------------------------------------------------------
    Purpose:
        Declare ``--verify-csv`` on the ur5_sim parser.

    Inputs:
        parser (argparse.ArgumentParser): the ur5_sim parser.

    Outputs:
        None. The parser is mutated in place.
    --------------------------------------------------------------------------
    """
    parser.add_argument(
        "--verify-csv", nargs="?", const="auto", default=None,
        metavar="PATH",
        help="check a monitor CSV against the commanded trajectory "
             "('auto' takes the newest ACQ_rtde_*.csv)")


def _newest_recorded_csv() -> str | None:
    """Newest ACQ_rtde_*.csv under datalogger/sim_runs/, then the cwd."""
    candidates: list[str] = []
    for folder in (os.path.join("datalogger", "sim_runs"), "."):
        candidates.extend(glob.glob(os.path.join(folder, "ACQ_rtde_*.csv")))
    if not candidates:
        return None
    return max(candidates, key=os.path.getmtime)


def run_verify_csv_cli(arg: str, polyline: Sequence[Sequence[float]]) -> int:
    """
    --------------------------------------------------------------------------
    Purpose:
        Resolve the CSV path (or the newest recorded one), run ``verify``
        against the commanded polyline, and print the report ``cli.py``'s
        ``--verify-csv`` branch hands back as its process exit code.

    Inputs:
        arg (str): CLI value - 'auto' or an explicit path.
        polyline (Sequence[Sequence[float]]): commanded XYZ, surface-clamped.

    Outputs:
        code (int): 0 if the CSV passed, 1 otherwise (missing file included).
    --------------------------------------------------------------------------
    """
    path = _newest_recorded_csv() if arg == "auto" else arg
    if path is None:
        print("[verify] no ACQ_rtde_*.csv found")
        return 1

    report = verify(path, polyline, tol_m=1e-5)
    print(f"[verify] {path}")
    print(f"[verify]   rows          : {report['rows']}")
    print(f"[verify]   max deviation : {report['max_dev_m'] * 1000:.4f} mm")
    print(f"[verify]   rms deviation : {report['rms_dev_m'] * 1000:.4f} mm")
    print(f"[verify]   effective rate: {report['rate_hz']:.2f} Hz")
    print(f"[verify]   time monotonic: {report['time_monotonic']}")
    for a, b in report["gaps"]:
        print(f"[verify]   pause gap     : {a:.3f} s -> {b:.3f} s")
    if report["simulated_source"]:
        print("[verify]   SOURCE        : SIMULATED (ur5_sim emulator), "
              "not robot data")
    print(f"[verify] {'PASS' if report['passed'] else 'FAIL'}")
    return 0 if report["passed"] else 1
