"""Guards on the two tracked robot artifacts, etalement.script and its acq twin.

Every other export test in this suite checks a pair it generated itself, in a
temp directory, from the defaults of design/params.py. That proves the
generator correct and says nothing about the files actually committed, which
are what gets loaded on the pendant. F11 in
docs/superpower/plans/erreur_hors_datalogger.md is precisely that gap: the
tracked acq twin had been regenerated from defaults beside an original
exported from the UI, so the two carried different trajectories - 346 poses
against 661 - and every test still passed.

These tests read the tracked files at the repo root, on purpose.
"""

from __future__ import annotations

import unittest
from pathlib import Path

import design.params as params
from design.settings import get_settings
from ur5_sim.parsing import parse_poses

_REPO_ROOT = Path(__file__).resolve().parents[1]
_BASE_PATH = _REPO_ROOT / "etalement.script"
_ACQ_PATH = _REPO_ROOT / "etalement_acq.script"


class TrackedArtifactPairTests(unittest.TestCase):
    """The shipped pair must describe one and the same motion."""

    def test_both_artifacts_exist(self):
        self.assertTrue(_BASE_PATH.is_file(), "%s is missing" % _BASE_PATH.name)
        self.assertTrue(_ACQ_PATH.is_file(), "%s is missing" % _ACQ_PATH.name)

    def test_tracked_pair_carries_identical_motion(self):
        """Guarantee 2 of the acquisition plan, applied to the shipped files.

        The line number is deliberately excluded from the comparison: the
        logger thread is inserted above def etalement(), which shifts every
        line below it without moving a single pose. Everything else must
        match exactly - pose, cycle index and in_contact flag.
        """
        base = parse_poses(_BASE_PATH)
        acq = parse_poses(_ACQ_PATH)

        self.assertEqual(
            len(base), len(acq),
            "The tracked pair no longer describes the same trajectory: "
            "%s has %d poses, %s has %d. One of them was regenerated without "
            "the other. Re-export BOTH from the same UI state (--export writes "
            "the pair), or rebuild the twin from the original with "
            "_build_acq_lines(). See F11 in erreur_hors_datalogger.md."
            % (_BASE_PATH.name, len(base), _ACQ_PATH.name, len(acq)))

        for index, (base_entry, acq_entry) in enumerate(zip(base, acq)):
            self.assertEqual(
                base_entry[1:], acq_entry[1:],
                "Pose %d differs between the tracked artifacts: base=%r "
                "acq=%r" % (index, base_entry[1:], acq_entry[1:]))

    def test_acq_twin_is_within_the_polyscope_memory_budget(self):
        budget = get_settings().urscript_max_bytes
        size = _ACQ_PATH.stat().st_size
        self.assertLessEqual(
            size, budget,
            "%s is %d bytes, over the %d-byte PolyScope budget (%.1f%%)."
            % (_ACQ_PATH.name, size, budget, 100.0 * size / budget))

    def test_acq_twin_actually_carries_the_logger(self):
        """A twin silently replaced by a copy of the original would satisfy
        the motion test above, and record nothing at all."""
        text = _ACQ_PATH.read_text(encoding="utf-8")
        self.assertIn("thread data_logger():", text)
        self.assertIn("keep_logging", text)
        self.assertIn(str(params.ACQ_LOG_PORT), text)

    def test_original_carries_no_logger(self):
        """The output policy of the plan: etalement.script stays the
        untouched program, and the simulator keeps consuming it."""
        text = _BASE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("thread data_logger():", text)


if __name__ == "__main__":
    unittest.main()
