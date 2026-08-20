"""
Tests that the viewer reports its run state to the RTDE emulator.

Covers the half of the wiring the plan verifies by hand with a browser and the
C monitor: the emulator receives the trajectory once, and every START / PAUSE /
RESUME / STOP reaches it with the arguments that decide the monitor's file
boundaries. No Swift, no browser, no socket - matplotlib runs under Agg and the
server is a recording stub, so what is asserted is the viewer's side of the
contract rather than the wire.

The arguments matter more than they look: set_run_state maps
``finished=True`` to STOPPED (the monitor closes its CSV) and ``running=False``
with a positive banked time to PAUSED (the monitor keeps ONE file). Publishing
a pause as a stop would split a run in two, which is the exact defect this
whole plan exists to make visible.
"""

import unittest

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import roboticstoolbox as rtb  # noqa: E402

from ur5_sim.config import DT  # noqa: E402
from ur5_sim.visualization.mpl_display import build_display  # noqa: E402
from ur5_sim.visualization.playback import build_playback  # noqa: E402
from ur5_sim.visualization.rtde_link import build_rtde_link  # noqa: E402

N_FRAMES = 4


class _RecordingServer:
    """Stands in for RtdeServer: records what the viewer asks of it."""

    def __init__(self) -> None:
        self.runs: list[dict] = []
        self.states: list[tuple[bool, float, bool]] = []

    def load_run(self, poses, times, in_contact, penetration_m) -> None:
        self.runs.append({
            "poses": list(poses), "times": list(times),
            "in_contact": list(in_contact), "penetration_m": list(penetration_m),
        })

    def set_run_state(self, running: bool, sim_time: float, finished: bool) -> None:
        self.states.append((bool(running), float(sim_time), bool(finished)))


class ViewerRtdeWiringTests(unittest.TestCase):

    def _playback(self, server) -> dict:
        robot = rtb.models.UR5()
        trajectories = [("#1 test", [robot.qr.copy() for _ in range(N_FRAMES)])]
        display = build_display(trajectories, N_FRAMES, DT, 1, None, None)
        self.addCleanup(plt.close, display["fig"])
        link = build_rtde_link(server)
        link["load"](
            [(0.0, 0.5, 0.05, 3.14159, 0.0, 0.0)] * N_FRAMES,
            [i * DT for i in range(N_FRAMES)],
            [True] * N_FRAMES,
            [0.0] * N_FRAMES,
        )
        return build_playback(
            robot, trajectories, DT, None,
            [1] * N_FRAMES, [(0.0, 0.0)] * N_FRAMES, [True] * N_FRAMES,
            1, {"env": None, "ee_handles": {}, "tcp_marker": None}, display,
            link["publish"], link["load"],
        )

    def test_the_trajectory_reaches_the_emulator_once(self) -> None:
        server = _RecordingServer()
        self._playback(server)
        self.assertEqual(len(server.runs), 1)
        run = server.runs[0]
        self.assertEqual(len(run["poses"]), N_FRAMES)
        self.assertEqual(len(run["times"]), N_FRAMES)
        self.assertEqual(len(run["penetration_m"]), N_FRAMES)

    def test_stop_is_published_as_finished(self) -> None:
        server = _RecordingServer()
        playback = self._playback(server)
        server.states.clear()
        playback["set_stop"]()
        self.assertEqual(server.states, [(False, 0.0, True)])

    def test_pause_is_published_as_not_finished(self) -> None:
        """A pause must NOT read as a stop: the monitor keeps one file."""
        server = _RecordingServer()
        playback = self._playback(server)
        playback["set_start"]()
        server.states.clear()
        playback["set_pause"]()
        self.assertEqual(len(server.states), 1)
        running, _sim_t, finished = server.states[0]
        self.assertFalse(running)
        self.assertFalse(finished)

    def test_resume_after_pause_republishes_running(self) -> None:
        server = _RecordingServer()
        playback = self._playback(server)
        playback["set_start"]()
        playback["set_pause"]()
        server.states.clear()
        playback["set_start"]()
        self.assertEqual(len(server.states), 1)
        self.assertTrue(server.states[0][0])
        self.assertFalse(server.states[0][2])

    def test_no_server_is_a_no_op_not_a_crash(self) -> None:
        """The visualizer must run with or without the emulator."""
        link = build_rtde_link(None)
        link["load"]([(0.0,) * 6], [0.0], [False], [0.0])
        link["publish"](True, 1.0, False)


if __name__ == "__main__":
    unittest.main()
