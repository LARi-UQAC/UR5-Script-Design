"""
tests/test_acq_export.py - The three guarantees of plan_acq_datalogger.md sec3
(task T2), plus the three extra checks of the T2 assignment.

The acquisition twin (etalement_acq.script) must never change the robot's
motion: it only wraps the untouched output of _build_urscript_lines() with a
50 Hz logger thread (sec4) inserted at three stable anchors. Six guarantees,
one class each:

  1. RegressionTests          - the base URScript is unaffected by the acq
                                 wrapping, and a headless export of it still
                                 matches tests/fixtures/golden_headless.script
                                 byte for byte (NOT the committed
                                 etalement.script - see F10 in
                                 docs/superpower/plans/erreur_hors_datalogger.md
                                 and the note in tests/test_export_settings.py).
  2. MotionEquivalenceTests    - ur5_sim.parsing.parse_poses() returns the
                                 exact same 4-tuples for both scripts.
  3. AcqThreadContentTests     - the thread body has the required elements
                                 (def, keep-logging flag, ACQ_MAX_SAMPLES
                                 guard, socket-before-motion, shutdown/STOP
                                 after the final retreat, counting sentinel)
                                 and none of the CB3-forbidden constructs
                                 (movel/stopl/list slice) between
                                 "thread data_logger():" and its closing "end".
  4. MemoryBudgetTests         - the PolyScope memory budget also holds for
                                 the acq file, with the percentage reported.
  5. WritePathTests            - generate_urscript_acq() defaults to
                                 params.ACQ_SCRIPT_PATH and honours an
                                 explicit filename override.
  6. OverwriteGuardTests       - the acq export goes through the same
                                 hand-edit guard as the original.

Every test writes into a fresh tempfile.mkdtemp(), never into the repo root,
and the module-level EXPORT_STATE_PATH digest file (design/export.py) is
patched to a path inside that temp directory for every test: generate_
urscript_acq()/_write_export() record a digest there unconditionally, keyed
by the output file's basename, and the real .etalement_export_state.json
(gitignored, but real) must not be corrupted by a test-generated digest for
the same basename the operator's own exports use ("etalement_acq.script").
"""

from __future__ import annotations

import inspect
import re
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import design.params as params
from design.export import (
    _build_acq_lines,
    _build_urscript_lines,
    _validate_script_memory,
    generate_urscript,
    generate_urscript_acq,
)
from design.settings import Settings, get_settings, set_settings
from design.trajectory import build_full_trajectory
from ur5_sim.parsing import parse_poses

GOLDEN = Path(__file__).resolve().parent / "fixtures" / "golden_headless.script"

_SLICE_RE = re.compile(r"\[\s*\d*\s*:\s*\d*\s*\]")


def _def_block(lines: list[str], header: str) -> list[str]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Extract a top-level "header ... end" block by scanning for the first
        unindented closing "end" after the header line, instead of a
        hardcoded line count or offset. Mirrors the anchor logic
        design.export._build_acq_lines() itself relies on: every def/thread
        emitted by this exporter is unindented, its body is indented by at
        least one space, so the first bare "end" line after the header is
        always the block's own closer, never a nested if/while "end".

    Inputs:
        lines (list[str]): full URScript line list to search.
        header (str): exact top-level line to start from, e.g.
            "thread data_logger():" or "def etalement():".

    Outputs:
        block (list[str]): lines from header to its closing "end", inclusive.
    --------------------------------------------------------------------------
    """
    start = lines.index(header)
    for i in range(start + 1, len(lines)):
        if lines[i] == "end":
            return lines[start:i + 1]
    raise AssertionError(f"closing 'end' of {header!r} not found")


def _forbidden_tokens(line: str) -> list[str]:
    """CB3 constructs illegal inside a secondary thread (ARCHITECTURE.md sec4
    / plan sec1 issue 9): movel/stopl calls and any list slice such as
    [0:3]."""
    found = []
    if "movel(" in line:
        found.append("movel(")
    if "stopl(" in line:
        found.append("stopl(")
    if _SLICE_RE.search(line):
        found.append("list slice")
    return found


class _AcqExportTestBase(unittest.TestCase):
    """Shared fixture: a fresh temp directory, default Settings, real cycles
    from build_full_trajectory(), and an EXPORT_STATE_PATH patched inside
    that temp directory so no test can corrupt the repo's real digest
    file."""

    def setUp(self) -> None:
        tmp_str = tempfile.mkdtemp()
        self.addCleanup(lambda: shutil.rmtree(tmp_str, ignore_errors=True))
        self.tmp = Path(tmp_str)

        set_settings(Settings())
        self.addCleanup(set_settings, Settings())

        state_patch = patch(
            "design.export.EXPORT_STATE_PATH", self.tmp / ".test_export_state.json"
        )
        state_patch.start()
        self.addCleanup(state_patch.stop)

        self.cycles = build_full_trajectory()


class RegressionTests(_AcqExportTestBase):
    """Guarantee 1 (plan sec3): the original generation path is untouched."""

    def test_build_urscript_lines_unaffected_by_acq_wrapping(self):
        base_lines = _build_urscript_lines(self.cycles)
        reference = list(base_lines)
        _build_acq_lines(base_lines)  # must not mutate its argument in place
        self.assertEqual(base_lines, reference)

    def test_build_urscript_lines_same_with_or_without_acq_afterwards(self):
        lines_before = _build_urscript_lines(self.cycles)
        _build_acq_lines(lines_before)
        lines_after = _build_urscript_lines(self.cycles)
        self.assertEqual(lines_before, lines_after)

    def test_headless_export_is_byte_identical_to_the_golden_fixture(self):
        out = self.tmp / "etalement.script"
        ok = generate_urscript(self.cycles, filename=out, force=True)
        self.assertTrue(ok)
        produced = out.read_text(encoding="utf-8")
        expected = GOLDEN.read_text(encoding="utf-8")
        self.assertEqual(produced, expected)


class MotionEquivalenceTests(_AcqExportTestBase):
    """Guarantee 2 (plan sec3): parse_poses() returns exactly the same
    poses, cycle indices and in_contact flags for the acq twin as for the
    base script - proof the logger thread changes no motion.

    Line numbers are deliberately excluded from the comparison: the acq
    thread definition is inserted earlier in the file (right before
    "def etalement():"), which shifts everything textually after that
    anchor. Every pose inside a def cycle_N() body sits before the anchor
    and keeps its exact line number; only the single final retreat pose
    (inside etalement()'s own body, after the insertion point) has a
    different line number between the two files - confirmed by inspection
    (346 poses in each file, the sole line-number difference is the last
    one, 500 -> 551, with identical pose/cycle/in_contact on every entry).
    """

    def test_parse_poses_identical_between_base_and_acq_scripts(self):
        base_path = self.tmp / "etalement.script"
        acq_path = self.tmp / "etalement_acq.script"
        self.assertTrue(
            generate_urscript(self.cycles, filename=base_path, force=True))
        self.assertTrue(
            generate_urscript_acq(self.cycles, filename=acq_path, force=True))

        base_poses = parse_poses(base_path)
        acq_poses = parse_poses(acq_path)

        self.assertGreater(len(base_poses), 100)
        self.assertEqual(len(base_poses), len(acq_poses))

        # (pose, cycle_idx, in_contact) - drop the line number (field 0).
        base_motion = [entry[1:] for entry in base_poses]
        acq_motion = [entry[1:] for entry in acq_poses]
        self.assertEqual(base_motion, acq_motion)


class AcqThreadContentTests(_AcqExportTestBase):
    """Guarantee 3 (plan sec3/sec4): the acq block carries the required
    elements and none of the CB3-forbidden constructs inside the thread
    body."""

    def setUp(self) -> None:
        super().setUp()
        base_lines = _build_urscript_lines(self.cycles)
        self.acq_lines = _build_acq_lines(base_lines)

    def test_thread_definition_flag_and_bounds_guard_present(self):
        block_text = "\n".join(_def_block(self.acq_lines, "thread data_logger():"))
        self.assertIn("thread data_logger():", block_text)
        self.assertIn("acq_keep_logging", block_text)
        self.assertIn("acq_index < ACQ_MAX_SAMPLES", block_text)

    def test_thread_body_has_no_forbidden_cb3_construct(self):
        block = _def_block(self.acq_lines, "thread data_logger():")
        for lineno, line in enumerate(block):
            tokens = _forbidden_tokens(line)
            self.assertEqual(
                tokens, [],
                f"forbidden token(s) {tokens} in thread body line {lineno}: "
                f"{line!r}")

    def test_socket_opens_before_any_motion(self):
        body = _def_block(self.acq_lines, "def etalement():")
        socket_idx = next(
            i for i, ln in enumerate(body) if "socket_open(" in ln)
        first_movel_idx = next(
            i for i, ln in enumerate(body) if ln.strip().startswith("movel("))
        self.assertLess(socket_idx, first_movel_idx)

    def test_shutdown_and_stop_handshake_after_final_retreat(self):
        body = _def_block(self.acq_lines, "def etalement():")
        last_movel_idx = max(
            i for i, ln in enumerate(body) if ln.strip().startswith("movel("))
        stop_flag_idx = next(
            i for i, ln in enumerate(body) if "acq_keep_logging = False" in ln)
        stop_line_idx = next(
            i for i, ln in enumerate(body)
            if 'socket_send_line("STOP", "acq")' in ln)
        self.assertGreater(stop_flag_idx, last_movel_idx)
        self.assertGreater(stop_line_idx, stop_flag_idx)

    def test_counting_sentinel_present(self):
        text = "\n".join(self.acq_lines)
        self.assertIn("acq_sample[0] = -1.0", text)
        self.assertIn("acq_sample[1] = acq_index", text)


class MemoryBudgetTests(_AcqExportTestBase):
    """Guarantee 4 (extra check 4): PolyScope memory budget also asserted on
    the acq file, with margin."""

    def test_acq_script_is_within_the_polyscope_memory_budget(self):
        out = self.tmp / "etalement_acq.script"
        self.assertTrue(
            generate_urscript_acq(self.cycles, filename=out, force=True))
        ok = _validate_script_memory(out, "URScript ACQ (test)")
        self.assertTrue(ok)

        size_bytes = out.stat().st_size
        max_bytes = get_settings().urscript_max_bytes
        pct = 100.0 * size_bytes / max_bytes
        print(
            f"[test_acq_export] memoire URScript ACQ : {size_bytes} / "
            f"{max_bytes} octets ({pct:.1f}% du budget PolyScope)")
        self.assertLessEqual(size_bytes, max_bytes)


class WritePathTests(_AcqExportTestBase):
    """Guarantee 5 (extra check 5): generate_urscript_acq() defaults to
    params.ACQ_SCRIPT_PATH and honours an explicit filename override."""

    def test_default_filename_parameter_is_the_acq_script_path(self):
        default = inspect.signature(
            generate_urscript_acq).parameters["filename"].default
        self.assertEqual(default, params.ACQ_SCRIPT_PATH)

    def test_filename_argument_redirects_the_write(self):
        out = self.tmp / "custom_acq_name.script"
        self.assertFalse(out.exists())
        ok = generate_urscript_acq(self.cycles, filename=out, force=True)
        self.assertTrue(ok)
        self.assertTrue(out.is_file())
        # The real repo-root default target must stay untouched by this test.
        self.assertNotEqual(out.resolve(), params.ACQ_SCRIPT_PATH.resolve())


class OverwriteGuardTests(_AcqExportTestBase):
    """Guarantee 6 (extra check 6): the acq export goes through the same
    hand-edit guard (check_overwrite / _write_export) as the original."""

    def test_hand_modified_acq_file_refused_without_force_overwritten_with_it(self):
        out = self.tmp / "etalement_acq.script"
        self.assertTrue(
            generate_urscript_acq(self.cycles, filename=out, force=True))

        handmade = ("# hand-edited by the operator for a robot trial\n"
                    + out.read_text(encoding="utf-8"))
        out.write_text(handmade, encoding="utf-8")

        refused = generate_urscript_acq(self.cycles, filename=out, force=False)
        self.assertFalse(refused)
        self.assertEqual(out.read_text(encoding="utf-8"), handmade)

        accepted = generate_urscript_acq(self.cycles, filename=out, force=True)
        self.assertTrue(accepted)
        self.assertNotEqual(out.read_text(encoding="utf-8"), handmade)


if __name__ == "__main__":
    unittest.main()
