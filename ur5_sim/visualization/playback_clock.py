"""
Playback clock behind the viewer's START / PAUSE / STOP controls.

Extracted from viewer.py so the timing rules can be tested without a
matplotlib figure. The distinction it encodes is the one the RTDE emulator
needs: a PAUSE keeps elapsed simulation time (same run, so the monitor must
not split the CSV), a STOP discards it (next START replays from frame 0,
which is the viewer's long-standing documented behavior).
"""

from __future__ import annotations


class PlaybackClock:
    """Wall-clock to simulation-time mapping with pause support."""

    def __init__(self, speed: float = 1.0) -> None:
        self._speed = float(speed)
        self._running = False
        self._paused = False
        self._offset = 0.0        # simulation time banked by earlier segments
        self._anchor = 0.0        # wall time the current segment started

    @property
    def running(self) -> bool:
        return self._running

    @property
    def paused(self) -> bool:
        return self._paused

    def start(self, now: float) -> None:
        """Start, or resume from a pause without losing banked time."""
        if self._running:
            return
        self._anchor = float(now)
        self._running = True
        self._paused = False

    def pause(self, now: float) -> None:
        """Bank the elapsed simulation time and hold it."""
        if not self._running:
            return
        self._offset = self.elapsed(now)
        self._running = False
        self._paused = True

    def stop(self) -> None:
        """Hard stop: discard banked time so the next start replays from zero."""
        self._offset = 0.0
        self._running = False
        self._paused = False

    def elapsed(self, now: float) -> float:
        """Simulation time, seconds."""
        if not self._running:
            return self._offset
        return (float(now) - self._anchor) * self._speed + self._offset
