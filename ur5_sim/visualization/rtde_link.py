"""
ur5_sim/visualization/rtde_link.py - the viewer's side of the RTDE emulator.

Two calls, and the rule that maps one onto the other:

* ``load(poses, times, in_contact, penetration)`` hands the emulator the
  trajectory. It is called once before the animation starts, and again after a
  background recompute lands, so a re-exported ``etalement.script`` is streamed
  instead of the retired buffer.
* ``publish(running, sim_time, finished)`` reports the playback state. The
  mapping is the one datalogger/rtde_fallback_monitor.c keys its file
  boundaries on:

  - STOP  -> ``finished=True``  -> STOPPING then STOPPED. The monitor closes
    the CSV; the next START opens a new one. This is right, because the
    viewer's STOP discards the elapsed time and the next START replays from
    frame 0 - a different run.
  - PAUSE -> ``running=False`` with the banked simulation time still positive
    -> PAUSING then PAUSED. The monitor keeps the SAME file growing, which is
    right, because RESUME continues where the pause stopped
    (``playback_clock.PlaybackClock``).
  - START / every animation tick -> ``running=True`` -> PLAYING.

Both calls are no-ops when no server is served, so the playback layer calls
them unconditionally and never has to branch on ``rtde_server is None``.
Neither ever raises: they run inside matplotlib widget callbacks and the timer
tick, where an exception would take the GUI down with it.

Design: ../../docs/superpower/specs/spec_rtde_emulator.md
"""

from __future__ import annotations

from typing import Any, Callable, Sequence


def build_rtde_link(rtde_server: Any) -> dict[str, Callable[..., None]]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Build the two closures the viewer uses to drive the RTDE emulator,
        degraded to no-ops when the emulator is not being served.

    Inputs:
        rtde_server (RtdeServer | None): a started server, or None.

    Outputs:
        link (dict): ``{"load": fn, "publish": fn}``.
    --------------------------------------------------------------------------
    """

    def load(
        poses: Sequence[Sequence[float]] | None,
        times: Sequence[float],
        in_contact: Sequence[bool],
        penetration: Sequence[float] | None,
    ) -> None:
        # The emulator must receive the SAME surface-clamped poses the IK
        # sees, so a CSV recorded from the stream can be checked against the
        # commanded path rather than against an idealised one. load_run copies
        # and coerces every element, so passing the live buffers is safe.
        if rtde_server is None or poses is None:
            return
        try:
            rtde_server.load_run(
                poses=poses,
                times=times,
                in_contact=in_contact,
                penetration_m=penetration
                if penetration is not None else [0.0] * len(times),
            )
        except Exception as exc:  # pragma: no cover - degraded, never fatal
            print(f"[viewer] RTDE load_run failed: {exc!r}")

    def publish(running: bool, sim_time: float, finished: bool) -> None:
        if rtde_server is None:
            return
        try:
            rtde_server.set_run_state(
                running=running, sim_time=sim_time, finished=finished)
        except Exception as exc:  # pragma: no cover - degraded, never fatal
            print(f"[viewer] RTDE set_run_state failed: {exc!r}")

    return {"load": load, "publish": publish}
