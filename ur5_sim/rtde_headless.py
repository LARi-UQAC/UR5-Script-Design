"""
Headless driver for the RTDE emulator - runs without a GUI.

``ur5_sim --visualize`` drives the emulator's run state from the matplotlib
timer, so the two behaviours the C monitor cares most about (a second run must
open a second CSV, a pause must NOT split one) can only be checked by a human
clicking START / PAUSE / STOP. :func:`run_headless` drives the same
``set_run_state`` contract from a plain loop, so a test or an unattended shell
can exercise those file boundaries with no browser and no figure.

Lives beside ``rtde_server.py`` rather than inside it only because that file
already sits at the workspace per-file token ceiling; ``rtde_server`` re-exports
this name, so ``ur5_sim.rtde_server.run_headless`` is the public spelling and
the split is invisible to callers. Same reason, same shape as the earlier
``rtde_wire.py`` split.

Design: ../docs/superpower/specs/spec_rtde_emulator.md
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # typing only, so the runtime import graph stays flat
    from ur5_sim.rtde_server import RtdeServer

# Wall-clock granularity of the driver loop, seconds. Well under the emitter's
# own 8 ms period at 125 Hz, so the state the emitter reads is never more than
# one tick stale, and small enough that ``pause_at`` lands within a tick of the
# requested simulation time.
_TICK_S: float = 0.01


def run_headless(
    server: "RtdeServer",
    total_sim_time: float,
    runs: int,
    pause_at: float | None,
    idle_s: float,
) -> None:
    """
    --------------------------------------------------------------------------
    Purpose:
        Drive the server through complete runs in real time with no GUI, so an
        unattended script can exercise the monitor's file boundaries. Each run
        is STOPPED, then PLAYING for total_sim_time, then STOPPED again, with
        an optional single pause partway.

    Inputs:
        server (RtdeServer): a started server.
        total_sim_time (float): duration of one run, seconds.
        runs (int): number of consecutive runs.
        pause_at (float | None): simulation time to pause once at, or None.
        idle_s (float): STOPPED dwell between runs, and the pause hold.

    Outputs:
        None.
    --------------------------------------------------------------------------
    """
    n_runs = max(1, int(runs))
    for run_index in range(n_runs):
        server.set_run_state(running=False, sim_time=0.0, finished=True)
        time.sleep(idle_s)

        sim_t, paused_done = 0.0, pause_at is None
        wall0 = time.perf_counter()
        while sim_t < total_sim_time:
            sim_t = time.perf_counter() - wall0
            server.set_run_state(running=True, sim_time=sim_t, finished=False)
            if not paused_done and sim_t >= float(pause_at):
                # PAUSED, not STOPPED: the monitor must keep writing the same
                # file across this hold. Re-anchoring the wall clock to the
                # banked sim time is what makes it a resume and not a restart.
                server.set_run_state(
                    running=False, sim_time=sim_t, finished=False)
                time.sleep(idle_s)
                wall0 = time.perf_counter() - sim_t
                paused_done = True
            time.sleep(_TICK_S)

        server.set_run_state(
            running=False, sim_time=total_sim_time, finished=True)
        time.sleep(idle_s)
        print(f"[rtde-emu] run {run_index + 1}/{n_runs} complete")
