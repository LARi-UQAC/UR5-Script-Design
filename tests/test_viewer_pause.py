"""Tests for the viewer's PAUSE control, without opening a window.

The plan expected this to be checkable only by launching the visualizer, since
the timing lives in closures over a matplotlib figure. It is not: with the Agg
backend and ``time.perf_counter`` replaced by a controllable counter, the whole
chain (``build_display`` -> ``build_playback`` -> ``controls``) runs headless
and the three rules can be asserted on exact numbers instead of eyeballed.

The rules, the same ones :class:`ur5_sim.visualization.playback_clock.PlaybackClock`
states in isolation:

* PAUSE freezes simulation time, however long the wall clock runs on.
* RESUME continues from the pause point rather than replaying.
* STOP discards the banked time, so the next START replays from frame 0.

No Swift: ``env`` is None throughout, which is the viewer's documented
matplotlib-only mode.
"""

from __future__ import annotations

import sys
import unittest
import unittest.mock as mock
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib  # noqa: E402

matplotlib.use("Agg")

import numpy as np  # noqa: E402
import roboticstoolbox as rtb  # noqa: E402

from ur5_sim.visualization.mpl_display import build_display  # noqa: E402
from ur5_sim.visualization.playback import build_playback  # noqa: E402

DT = 0.05
N_FRAMES = 40
T_ORIGIN = 1000.0


class ViewerPauseTests(unittest.TestCase):
    """Drive the real callbacks with a fake clock. Expect ~5 s for the UR5 load."""

    def setUp(self) -> None:
        self.now = [T_ORIGIN]
        robot = rtb.models.UR5()
        q0 = np.array(robot.qr, dtype=float)
        traj = [q0 + np.array([0.001 * i, 0.0, 0.0, 0.0, 0.0, 0.0])
                for i in range(N_FRAMES)]
        trajectories = [("branche A", traj)]
        scene = {"env": None, "ee_handles": [], "ee_temp_dir": None,
                 "surface_handle": None, "tcp_marker": None}
        self._patch = mock.patch("time.perf_counter", lambda: self.now[0])
        self._patch.start()
        self.addCleanup(self._patch.stop)
        self.display = build_display(
            trajectories, N_FRAMES, DT, 1, None, None,
        )
        self.addCleanup(matplotlib.pyplot.close, self.display["fig"])
        self.playback = build_playback(
            robot, trajectories, DT, None,
            [1] * N_FRAMES, [(0.0, 0.0)] * N_FRAMES, [False] * N_FRAMES,
            1, scene, self.display,
        )
        # START without the recompute round trip the button would trigger.
        # set_start is the same transition the button reaches once the
        # background IK finishes; it re-anchors the clock, which a bare
        # ``state["running"] = True`` would not.
        self.playback["set_start"]()

    # -- helpers reading the HUD, which is what an operator would read --------

    def _sim_t(self) -> float:
        text = self.display["sim_text"].get_text()
        return float(text.split("SIM t = ")[1].split("/")[0])

    def _frame(self) -> int:
        text = self.display["sim_text"].get_text()
        return int(text.split("frame ")[1].split("/")[0])

    def _advance(self, seconds: float) -> None:
        self.now[0] += seconds
        self.playback["tick"]()

    def test_pause_freezes_simulation_time(self) -> None:
        self._advance(0.60)
        self.assertAlmostEqual(self._sim_t(), 0.60, places=6)
        frozen_frame = self._frame()

        self.playback["on_pause_button"](None)
        self.assertFalse(self.playback["state"]["running"])
        self.assertTrue(self.playback["state"]["paused"])

        self._advance(4.40)  # a long wall-clock gap while paused
        self.assertEqual(self._frame(), frozen_frame)
        self.assertAlmostEqual(self._sim_t(), 0.60, places=6)

    def test_resume_continues_instead_of_replaying(self) -> None:
        self._advance(0.60)
        self.playback["on_pause_button"](None)
        self.now[0] += 4.40                      # paused, wall clock runs on
        self.playback["on_pause_button"](None)   # RESUME
        self.assertTrue(self.playback["state"]["running"])
        self.assertFalse(self.playback["state"]["paused"])

        self._advance(0.15)
        # 0.60 banked + 0.15 played, not 0.15 from a replay.
        self.assertAlmostEqual(self._sim_t(), 0.75, places=6)
        self.assertEqual(self._frame(), 15)

    def test_hud_drift_stays_zero_across_a_pause(self) -> None:
        # PC t must exclude the pause, or `delta` reads as minus the banked
        # time for the rest of the run and stops meaning frame drift.
        self._advance(0.60)
        self.playback["on_pause_button"](None)
        self.now[0] += 4.40
        self.playback["on_pause_button"](None)
        self._advance(0.15)
        text = self.display["sim_text"].get_text()
        drift_ms = float(text.split("delta = ")[1].split(" ms")[0])
        self.assertAlmostEqual(drift_ms, 0.0, places=6)

    def test_stop_discards_the_banked_time(self) -> None:
        self._advance(0.60)
        self.playback["on_pause_button"](None)
        self.playback["on_button"](None)         # STOP while paused
        self.assertFalse(self.playback["state"]["running"])
        self.assertFalse(self.playback["state"]["paused"])

        self.playback["set_start"]()             # next START
        self._advance(0.20)
        # 0.20 played, with the 0.60 discarded rather than carried over.
        self.assertAlmostEqual(self._sim_t(), 0.20, places=6)

    def test_pause_button_label_tracks_the_state(self) -> None:
        label = self.display["pause_btn"].label
        self.assertEqual(label.get_text(), "PAUSE")
        self._advance(0.30)
        self.playback["on_pause_button"](None)
        self.assertEqual(label.get_text(), "RESUME")
        self.assertEqual(self.display["status_text"].get_text(), "STATE = PAUSE")
        self.playback["on_pause_button"](None)
        self.assertEqual(label.get_text(), "PAUSE")
        self.playback["on_button"](None)         # STOP resets it too
        self.assertEqual(label.get_text(), "PAUSE")

    def test_pause_is_a_no_op_while_stopped(self) -> None:
        self.playback["state"]["running"] = False
        self.playback["on_pause_button"](None)
        self.assertFalse(self.playback["state"]["running"])
        self.assertFalse(self.playback["state"]["paused"])
        self.assertEqual(self.display["pause_btn"].label.get_text(), "PAUSE")


if __name__ == "__main__":
    unittest.main()
