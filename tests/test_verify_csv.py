"""
Tests for the recorded-CSV checker.

The geometric check is the primary one because it survives a pause: controller
time keeps advancing while simulation time freezes, so any purely time-indexed
comparison would report a false failure on a paused run.
"""

import os
import tempfile
import unittest

from ur5_sim.verify_csv import (
    distance_to_polyline,
    parse_monitor_csv,
    verify,
)

CSV_TEXT = """# Robot Model: UR5 CB3
# Data Source: RTDE fallback monitor (192.168.4.14)
# Robot RTDE Endpoint: 127.0.0.1:30004
# Time Column: RTDE timestamp field, relative to the first sample of this file (s)
Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ
0.000,-0.100000,0.000000,-6.000000,0.000000,0.000000,0.300000
0.020,-0.100000,0.000000,-6.000000,0.050000,0.000000,0.300000
0.040,-0.100000,0.000000,-6.000000,0.100000,0.000000,0.300000
"""

POLYLINE = [
    (0.0, 0.0, 0.3),
    (0.1, 0.0, 0.3),
]


class ParseTests(unittest.TestCase):

    def setUp(self) -> None:
        fd, self.path = tempfile.mkstemp(suffix=".csv")
        with os.fdopen(fd, "w", encoding="ascii") as handle:
            handle.write(CSV_TEXT)
        self.addCleanup(os.remove, self.path)

    def test_separates_header_from_rows(self) -> None:
        header, rows = parse_monitor_csv(self.path)
        self.assertTrue(any("Robot Model" in line for line in header))
        self.assertEqual(len(rows), 3)

    def test_row_values(self) -> None:
        _, rows = parse_monitor_csv(self.path)
        self.assertAlmostEqual(rows[1][0], 0.020, places=9)
        self.assertAlmostEqual(rows[1][3], -6.0, places=9)
        self.assertAlmostEqual(rows[1][4], 0.05, places=9)
        self.assertAlmostEqual(rows[1][6], 0.30, places=9)


class DistanceTests(unittest.TestCase):

    def test_point_on_the_line_is_zero(self) -> None:
        self.assertAlmostEqual(
            distance_to_polyline((0.05, 0.0, 0.3), POLYLINE), 0.0, places=12)

    def test_point_off_the_line(self) -> None:
        self.assertAlmostEqual(
            distance_to_polyline((0.05, 0.002, 0.3), POLYLINE), 0.002, places=12)

    def test_point_beyond_an_end_uses_the_endpoint(self) -> None:
        self.assertAlmostEqual(
            distance_to_polyline((0.2, 0.0, 0.3), POLYLINE), 0.1, places=12)

    def test_single_point_polyline(self) -> None:
        self.assertAlmostEqual(
            distance_to_polyline((0.0, 0.0, 0.4), [(0.0, 0.0, 0.3)]), 0.1, places=12)


class VerifyTests(unittest.TestCase):

    def setUp(self) -> None:
        fd, self.path = tempfile.mkstemp(suffix=".csv")
        with os.fdopen(fd, "w", encoding="ascii") as handle:
            handle.write(CSV_TEXT)
        self.addCleanup(os.remove, self.path)

    def test_matching_csv_passes(self) -> None:
        report = verify(self.path, POLYLINE, tol_m=1e-5)
        self.assertTrue(report["passed"])
        self.assertEqual(report["rows"], 3)
        self.assertLess(report["max_dev_m"], 1e-5)
        self.assertTrue(report["time_monotonic"])

    def test_offset_trajectory_fails(self) -> None:
        shifted = [(x, y + 0.01, z) for (x, y, z) in POLYLINE]
        report = verify(self.path, shifted, tol_m=1e-5)
        self.assertFalse(report["passed"])
        self.assertGreater(report["max_dev_m"], 1e-5)

    def test_reports_the_effective_rate(self) -> None:
        report = verify(self.path, POLYLINE, tol_m=1e-5)
        self.assertAlmostEqual(report["rate_hz"], 50.0, places=3)

    def test_flags_a_simulated_source(self) -> None:
        report = verify(self.path, POLYLINE, tol_m=1e-5)
        self.assertTrue(report["simulated_source"])
