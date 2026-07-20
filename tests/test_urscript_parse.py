"""Smoke tests for the URScript parsing layer."""

from __future__ import annotations

import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ur5_sim.config import P_ANCHOR_OLD_RAW, P_REF_RAW, SCRIPT_PATH
from ur5_sim.parsing.urscript import (
    parse_poses,
    parse_poses_legacy,
    transform,
    urscript_pose,
)


class UrscriptParseTests(unittest.TestCase):
    def test_parse_extracts_all_poses(self):
        parsed = parse_poses(SCRIPT_PATH)
        self.assertGreater(len(parsed), 100)
        for lineno, pose, cycle, in_contact in parsed:
            self.assertIsInstance(lineno, int)
            self.assertEqual(len(pose), 6)
            self.assertIsInstance(cycle, int)
            self.assertIsInstance(in_contact, bool)

    def test_parse_poses_legacy_drops_in_contact(self):
        parsed = parse_poses_legacy(SCRIPT_PATH)
        self.assertGreater(len(parsed), 100)
        for entry in parsed:
            self.assertEqual(len(entry), 3)

    def test_force_mode_toggles_in_contact_flag(self):
        script = textwrap.dedent(
            """\
            def cycle_1():
              movel(p[0.1, 0.2, 0.3, 0.0, 1.5708, 0.0], a=1, v=0.1)
              force_mode(get_actual_tcp_pose(), [0,0,1,0,0,0], [0,0,-6.0,0,0,0], 2, [0.002,0.002,0.04,0.35,0.35,0.35])
              movel(p[0.11, 0.21, 0.05, 0.0, 1.5708, 0.0], a=1, v=0.05, r=0.002)
              movel(p[0.12, 0.22, 0.05, 0.0, 1.5708, 0.0], a=1, v=0.05, r=0.002)
              end_force_mode()
              movel(p[0.13, 0.23, 0.30, 0.0, 1.5708, 0.0], a=1, v=0.1)
            end
            """
        )
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".script", delete=False, encoding="utf-8",
        ) as f:
            f.write(script)
            tmp = Path(f.name)
        try:
            parsed = parse_poses(tmp)
        finally:
            tmp.unlink(missing_ok=True)

        contact_flags = [in_contact for _l, _p, _c, in_contact in parsed]
        cycles = [cycle for _l, _p, cycle, _i in parsed]
        self.assertEqual(contact_flags, [False, True, True, False])
        self.assertEqual(cycles, [1, 1, 1, 1])

    def test_parse_supports_apply_correction_wrapper(self):
        script = textwrap.dedent(
            """\
            def cycle_1():
              movel(apply_correction(p[-0.011, 0.610, -0.296, 0.0, 1.5708, 0.0], 0.01, 0.005), a=1, v=0.1)
            end
            """
        )
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".script", delete=False, encoding="utf-8",
        ) as f:
            f.write(script)
            tmp = Path(f.name)
        try:
            parsed = parse_poses(tmp)
        finally:
            tmp.unlink(missing_ok=True)
        self.assertEqual(len(parsed), 1)
        pose = parsed[0][1]
        self.assertAlmostEqual(pose[0], -0.011)
        self.assertAlmostEqual(pose[1], 0.610)
        self.assertAlmostEqual(pose[2], -0.296)

    def test_transform_is_identity_when_anchors_match(self):
        anchor = urscript_pose(*P_ANCHOR_OLD_RAW)
        p = urscript_pose(0.198247, -0.304185, 0.050000, 3.14159, 0.0, 0.0)
        out = transform(p, anchor, anchor)
        self.assertTrue((abs(out.A - p.A) < 1e-9).all())

    def test_transform_relocates_origin_to_p_ref(self):
        anchor = urscript_pose(*P_ANCHOR_OLD_RAW)
        ref = urscript_pose(*P_REF_RAW)
        # An identity offset in the source frame must land exactly at P_REF.
        out = transform(anchor, anchor, ref)
        self.assertTrue((abs(out.A - ref.A) < 1e-9).all())


if __name__ == "__main__":
    unittest.main()
