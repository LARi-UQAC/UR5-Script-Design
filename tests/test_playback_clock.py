"""
Tests for the playback clock behind the viewer's START / PAUSE / STOP.

PAUSE must preserve elapsed simulation time; STOP must discard it, keeping the
viewer's documented behavior that the next START replays from frame 0.
"""

import unittest

from ur5_sim.visualization.playback_clock import PlaybackClock


class PlaybackClockTests(unittest.TestCase):

    def test_starts_idle_at_zero(self) -> None:
        clock = PlaybackClock()
        self.assertFalse(clock.running)
        self.assertFalse(clock.paused)
        self.assertEqual(clock.elapsed(now=100.0), 0.0)

    def test_elapsed_advances_while_running(self) -> None:
        clock = PlaybackClock()
        clock.start(now=10.0)
        self.assertTrue(clock.running)
        self.assertAlmostEqual(clock.elapsed(now=12.5), 2.5, places=9)

    def test_pause_freezes_elapsed(self) -> None:
        clock = PlaybackClock()
        clock.start(now=10.0)
        clock.pause(now=12.5)
        self.assertFalse(clock.running)
        self.assertTrue(clock.paused)
        self.assertAlmostEqual(clock.elapsed(now=99.0), 2.5, places=9)

    def test_resume_continues_from_the_pause_point(self) -> None:
        clock = PlaybackClock()
        clock.start(now=10.0)
        clock.pause(now=12.5)
        clock.start(now=50.0)
        self.assertTrue(clock.running)
        self.assertFalse(clock.paused)
        self.assertAlmostEqual(clock.elapsed(now=51.0), 3.5, places=9)

    def test_stop_discards_elapsed(self) -> None:
        clock = PlaybackClock()
        clock.start(now=10.0)
        clock.pause(now=12.5)
        clock.stop()
        self.assertFalse(clock.running)
        self.assertFalse(clock.paused)
        self.assertEqual(clock.elapsed(now=99.0), 0.0)

    def test_start_after_stop_replays_from_zero(self) -> None:
        clock = PlaybackClock()
        clock.start(now=10.0)
        clock.stop()
        clock.start(now=20.0)
        self.assertAlmostEqual(clock.elapsed(now=20.75), 0.75, places=9)

    def test_speed_factor_scales_elapsed(self) -> None:
        clock = PlaybackClock(speed=2.0)
        clock.start(now=0.0)
        self.assertAlmostEqual(clock.elapsed(now=1.0), 2.0, places=9)

    def test_double_pause_is_harmless(self) -> None:
        clock = PlaybackClock()
        clock.start(now=0.0)
        clock.pause(now=1.0)
        clock.pause(now=5.0)
        self.assertAlmostEqual(clock.elapsed(now=9.0), 1.0, places=9)


if __name__ == "__main__":
    unittest.main()
