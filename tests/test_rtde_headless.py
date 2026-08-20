"""
Tests for the RTDE emulator's headless driver (ur5_sim/rtde_headless.py).

The driver is what makes the two remaining pendant behaviours checkable with
no GUI and no robot: two runs must produce two files, and a pause must not
split one. Both are asserted here from the runtime_state sequence a client
actually receives, which is what datalogger/rtde_fallback_monitor.c keys its
file boundaries on.

_FakeMonitor is imported rather than duplicated: it performs the same
handshake the C monitor performs, and one copy means one thing to fix if the
handshake ever changes.
"""

import threading
import time
import unittest

from ur5_sim import rtde_server as rs

from tests.test_rtde_server import _FakeMonitor


class HeadlessRunnerTests(unittest.TestCase):
    """
    The headless runner is what makes the two remaining pendant behaviors
    checkable unattended: two runs producing two files, and a pause that does
    not split one.
    """

    POSES = [(float(i) * 0.01, 0.0, 0.3, 0.0, -3.1416, 0.0) for i in range(6)]
    TIMES = [i * 0.05 for i in range(6)]

    def _server_and_client(self):
        server = rs.RtdeServer(host="127.0.0.1", port=0, rate_hz=125.0)
        self.assertTrue(server.start())
        self.addCleanup(server.stop)
        server.load_run(
            poses=self.POSES, times=self.TIMES,
            in_contact=[True] * 6, penetration_m=[0.0] * 6,
        )
        client = _FakeMonitor(server.port)
        self.addCleanup(client.close)
        client.handshake()
        return server, client

    def _collect_states(self, client, seconds: float) -> list:
        seen, deadline = [], time.time() + seconds
        while time.time() < deadline:
            _, _, _, state = client.next_sample()
            if not seen or seen[-1] != state:
                seen.append(state)
        return seen

    def test_two_runs_emit_two_stopped_to_playing_edges(self) -> None:
        server, client = self._server_and_client()
        thread = threading.Thread(
            target=rs.run_headless,
            kwargs=dict(server=server, total_sim_time=0.25, runs=2,
                        pause_at=None, idle_s=0.2),
            daemon=True)
        thread.start()
        seen = self._collect_states(client, 2.0)
        thread.join(timeout=3.0)

        edges = sum(
            1 for a, b in zip(seen, seen[1:])
            if a == rs.RT_STOPPED and b == rs.RT_PLAYING
        )
        self.assertEqual(edges, 2)

    def test_pause_at_emits_the_pause_sequence_without_stopping(self) -> None:
        server, client = self._server_and_client()
        thread = threading.Thread(
            target=rs.run_headless,
            kwargs=dict(server=server, total_sim_time=0.5, runs=1,
                        pause_at=0.2, idle_s=0.2),
            daemon=True)
        thread.start()
        seen = self._collect_states(client, 2.0)
        thread.join(timeout=3.0)

        self.assertIn(rs.RT_PAUSED, seen)
        # No STOPPED between the pause and the resume: it is still one run.
        pause_i = seen.index(rs.RT_PAUSED)
        resume_i = len(seen) - 1 - seen[::-1].index(rs.RT_PLAYING)
        self.assertGreater(resume_i, pause_i)
        self.assertNotIn(rs.RT_STOPPED, seen[pause_i:resume_i])


if __name__ == "__main__":
    unittest.main()
