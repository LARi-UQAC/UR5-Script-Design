"""
Tests for the RTDE emulator's wire layer.

The byte layout asserted here is duplicated in datalogger/rtde_fallback_monitor.c
by necessity (two languages, one protocol). Pinning it from both sides turns a
silent drift into a failing test instead of wrong numbers in a lab CSV.
"""

import struct
import unittest

from ur5_sim import rtde_server as rs


class WireConstantsTests(unittest.TestCase):
    """Every value here must equal the C constant of the same name."""

    def test_payload_size_and_field_offsets(self) -> None:
        self.assertEqual(rs.RTDE_PAYLOAD_SIZE, 108)
        self.assertEqual(rs.FIELD_OFF_TIMESTAMP, 0)
        self.assertEqual(rs.FIELD_OFF_TCP_POSE, 8)
        self.assertEqual(rs.FIELD_OFF_TCP_FORCE, 56)
        self.assertEqual(rs.FIELD_OFF_RUNTIME_STATE, 104)

    def test_recipe_matches_the_monitor(self) -> None:
        self.assertEqual(
            rs.RTDE_OUTPUT_RECIPE,
            "timestamp,actual_TCP_pose,actual_TCP_force,runtime_state",
        )
        self.assertEqual(rs.RTDE_OUTPUT_TYPES, "DOUBLE,VECTOR6D,VECTOR6D,UINT32")

    def test_package_type_codes(self) -> None:
        self.assertEqual(rs.RTDE_REQUEST_PROTOCOL_VERSION, 86)
        self.assertEqual(rs.RTDE_TEXT_MESSAGE, 77)
        self.assertEqual(rs.RTDE_DATA_PACKAGE, 85)
        self.assertEqual(rs.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, 79)
        self.assertEqual(rs.RTDE_CONTROL_PACKAGE_START, 83)

    def test_runtime_state_enumeration(self) -> None:
        self.assertEqual(
            (rs.RT_STOPPING, rs.RT_STOPPED, rs.RT_PLAYING,
             rs.RT_PAUSING, rs.RT_PAUSED, rs.RT_RESUMING),
            (0, 1, 2, 3, 4, 5),
        )


class EncoderTests(unittest.TestCase):

    def test_header_is_big_endian_size_then_type(self) -> None:
        pkt = rs.encode_packet(rs.RTDE_CONTROL_PACKAGE_START, b"\x01")
        self.assertEqual(len(pkt), 4)
        self.assertEqual(pkt[0], 0x00)
        self.assertEqual(pkt[1], 0x04)          # total size includes the header
        self.assertEqual(pkt[2], rs.RTDE_CONTROL_PACKAGE_START)
        self.assertEqual(pkt[3], 0x01)

    def test_empty_payload_gives_a_three_byte_packet(self) -> None:
        pkt = rs.encode_packet(rs.RTDE_CONTROL_PACKAGE_START, b"")
        self.assertEqual(pkt, b"\x00\x03\x53")

    def test_data_payload_size_and_decoded_values(self) -> None:
        pose = (0.412345, -0.298765, 0.101234, 0.0, -3.1416, 0.0)
        force = (-0.123456, 0.234567, -6.012345, 0.0, 0.0, 0.0)
        body = rs.encode_data_payload(123456.789, pose, force, rs.RT_PLAYING)

        self.assertEqual(len(body), rs.RTDE_PAYLOAD_SIZE)
        # Decode field by field at the offsets the C monitor uses.
        (ts,) = struct.unpack_from(">d", body, rs.FIELD_OFF_TIMESTAMP)
        got_pose = struct.unpack_from(">6d", body, rs.FIELD_OFF_TCP_POSE)
        got_force = struct.unpack_from(">6d", body, rs.FIELD_OFF_TCP_FORCE)
        (state,) = struct.unpack_from(">I", body, rs.FIELD_OFF_RUNTIME_STATE)

        self.assertAlmostEqual(ts, 123456.789, places=9)
        self.assertEqual(got_pose, pose)
        self.assertEqual(got_force, force)
        self.assertEqual(state, rs.RT_PLAYING)

    def test_known_byte_sequence_for_one_point_zero(self) -> None:
        """Pins big-endian IEEE-754 against a hand-checkable literal."""
        body = rs.encode_data_payload(1.0, (0.0,) * 6, (0.0,) * 6, rs.RT_STOPPED)
        self.assertEqual(
            body[rs.FIELD_OFF_TIMESTAMP:rs.FIELD_OFF_TIMESTAMP + 8],
            b"\x3f\xf0\x00\x00\x00\x00\x00\x00",
        )
        self.assertEqual(
            body[rs.FIELD_OFF_RUNTIME_STATE:rs.FIELD_OFF_RUNTIME_STATE + 4],
            b"\x00\x00\x00\x01",
        )


class RunStateMachineTests(unittest.TestCase):
    """
    A real controller passes through PAUSING / RESUMING / STOPPING rather than
    jumping between the stable states. Emitting them drives the monitor through
    the same enum sequence its own C suite asserts pair by pair.
    """

    def _drain(self, machine: "rs.RunStateMachine", n: int) -> list:
        return [machine.next_state() for _ in range(n)]

    def test_starts_stopped(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        self.assertEqual(self._drain(m, 3), [rs.RT_STOPPED] * 3)

    def test_start_goes_straight_to_playing(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        m.request(rs.RT_PLAYING)
        self.assertEqual(self._drain(m, 3), [rs.RT_PLAYING] * 3)

    def test_pause_passes_through_pausing(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        m.request(rs.RT_PLAYING)
        m.next_state()
        m.request(rs.RT_PAUSED)
        self.assertEqual(
            self._drain(m, 4),
            [rs.RT_PAUSING, rs.RT_PAUSING, rs.RT_PAUSED, rs.RT_PAUSED],
        )

    def test_resume_passes_through_resuming(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        m.request(rs.RT_PLAYING)
        m.next_state()
        m.request(rs.RT_PAUSED)
        self._drain(m, 3)
        m.request(rs.RT_PLAYING)
        self.assertEqual(
            self._drain(m, 4),
            [rs.RT_RESUMING, rs.RT_RESUMING, rs.RT_PLAYING, rs.RT_PLAYING],
        )

    def test_stop_passes_through_stopping(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        m.request(rs.RT_PLAYING)
        m.next_state()
        m.request(rs.RT_STOPPED)
        self.assertEqual(
            self._drain(m, 4),
            [rs.RT_STOPPING, rs.RT_STOPPING, rs.RT_STOPPED, rs.RT_STOPPED],
        )

    def test_requesting_the_current_state_changes_nothing(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        m.request(rs.RT_PLAYING)
        m.next_state()
        m.request(rs.RT_PLAYING)
        self.assertEqual(self._drain(m, 2), [rs.RT_PLAYING] * 2)

    def test_repeated_request_during_a_transition_is_ignored(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        m.request(rs.RT_PLAYING)
        m.next_state()
        m.request(rs.RT_PAUSED)
        m.request(rs.RT_PAUSED)      # operator double-click must not restart it
        self.assertEqual(
            self._drain(m, 4),
            [rs.RT_PAUSING, rs.RT_PAUSING, rs.RT_PAUSED, rs.RT_PAUSED],
        )


class InterpolationTests(unittest.TestCase):
    """
    Translation is interpolated; orientation is taken from the nearest frame,
    because the exported trajectory holds orientation constant within a cycle.
    """

    POSES = [
        (0.0, 0.0, 0.0, 0.0, -3.1416, 0.0),
        (1.0, 2.0, 3.0, 0.0, -3.1416, 0.0),
        (2.0, 4.0, 6.0, 0.0, -3.0000, 0.0),
    ]
    TIMES = [0.0, 0.05, 0.10]

    def test_exact_frame_times_return_that_frame(self) -> None:
        for i, t in enumerate(self.TIMES):
            got = rs.interpolate_pose(self.POSES, self.TIMES, t)
            self.assertEqual(tuple(got), tuple(self.POSES[i]))

    def test_midpoint_translation_is_the_average(self) -> None:
        got = rs.interpolate_pose(self.POSES, self.TIMES, 0.025)
        self.assertAlmostEqual(got[0], 0.5, places=9)
        self.assertAlmostEqual(got[1], 1.0, places=9)
        self.assertAlmostEqual(got[2], 1.5, places=9)

    def test_quarter_point_translation(self) -> None:
        got = rs.interpolate_pose(self.POSES, self.TIMES, 0.0625)
        self.assertAlmostEqual(got[0], 1.25, places=9)
        self.assertAlmostEqual(got[1], 2.50, places=9)
        self.assertAlmostEqual(got[2], 3.75, places=9)

    def test_orientation_comes_from_the_nearer_frame(self) -> None:
        before_half = rs.interpolate_pose(self.POSES, self.TIMES, 0.060)
        after_half = rs.interpolate_pose(self.POSES, self.TIMES, 0.090)
        self.assertAlmostEqual(before_half[4], -3.1416, places=9)
        self.assertAlmostEqual(after_half[4], -3.0000, places=9)

    def test_clamps_outside_the_trajectory(self) -> None:
        self.assertEqual(
            tuple(rs.interpolate_pose(self.POSES, self.TIMES, -5.0)),
            tuple(self.POSES[0]),
        )
        self.assertEqual(
            tuple(rs.interpolate_pose(self.POSES, self.TIMES, 99.0)),
            tuple(self.POSES[-1]),
        )

    def test_single_frame_trajectory(self) -> None:
        got = rs.interpolate_pose([self.POSES[0]], [0.0], 1.0)
        self.assertEqual(tuple(got), tuple(self.POSES[0]))


if __name__ == "__main__":
    unittest.main()
