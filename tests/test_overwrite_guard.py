"""
tests/test_overwrite_guard.py - F3 (docs/superpower/plans/erreur_hors_datalogger.md):
the hand-edit guard (design.export.check_overwrite / _write_export) must fail
CLOSED when it cannot read the existing output file, and must say so out loud
when its own bookkeeping (the export-state digest file) is missing or
corrupt, instead of overwriting silently in either case.

Seven classes, one per bullet of F3's "Potential tests" list:

  1. UnreadableTrackedFileTests.test_refuses_without_force
                                 - an existing, TRACKED output file that
                                   raises OSError on read (locked by another
                                   program, permission denied) refuses the
                                   export and leaves the file untouched.
  2. UnreadableTrackedFileTests.test_overwrites_with_force_exactly_one_warning
                                 - same case with force=True: the export
                                   proceeds, with exactly one warning line.
  3. MissingOrCorruptStateTests.test_state_file_deleted_...
                                 - the export-state file is gone while the
                                   output file exists and was hand-modified:
                                   chosen behavior is UNCHANGED (still
                                   allowed - refusing a first/untracked
                                   export would be a nuisance with no
                                   upside, per check_overwrite()'s own
                                   docstring), but it is now AUDIBLE: exactly
                                   one warning names the situation.
  4. MissingOrCorruptStateTests.test_state_file_corrupt_json_...
                                 - same assertion, corrupt JSON instead of a
                                   missing file, and no traceback.
  5. HandModifiedRefusalMessageTests
                                 - a genuinely hand-modified, tracked file:
                                   the refusal message names the file and
                                   both digests (known vs current).
  6. UntouchedFileTests          - a file untouched since its last export:
                                   no hand-edit-guard warning, export
                                   proceeds.
  7. UrpSpecificGuardTests       - etalement.urp specifically goes through
                                   the same guard as the .script: refused
                                   without force, overwritten only when
                                   force=True is the explicit ask.

To simulate an unreadable file PORTABLY on Windows, tests 1 and 2 patch
pathlib.Path.read_text to raise OSError for the one target path, rather than
stripping ACLs: NTFS permission changes are unreliable for the file's own
owner (who typically retains override rights via SeTakeOwnershipPrivilege /
the Administrators group), and even when they take effect they are slow and
leave stray ACL state behind if a test aborts. A patched read is immediate,
deterministic, and self-cleaning.

Every test uses tempfile.mkdtemp() (never the repo root) and patches both
design.settings.SETTINGS_PATH and design.export.EXPORT_STATE_PATH into that
directory, exactly as tests/test_acq_export.py already does, so a run cannot
corrupt the operator's real settings file or export-state digest. Settings()
defaults are passed explicitly to generate_urscript()/generate_urp() so F1's
separate validity gate (tests/test_settings_validation.py) never interferes
with what this file is testing.
"""

from __future__ import annotations

import contextlib
import io
import pathlib
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from design.export import _digest, check_overwrite, generate_urp, generate_urscript
from design.settings import Settings, set_settings
from design.trajectory import build_full_trajectory


@contextlib.contextmanager
def _unreadable(target: Path):
    """Makes reading `target` raise OSError, leaving every other path's
    read_text() (the export-state file, the golden fixture, ...) working
    exactly as before. See the module docstring for why this is preferred
    over an actual permission change on Windows."""
    original_read_text = pathlib.Path.read_text

    def _patched(self, *args, **kwargs):
        if str(self) == str(target):
            raise OSError(13, "simulated: locked by another program")
        return original_read_text(self, *args, **kwargs)

    with patch.object(pathlib.Path, "read_text", _patched):
        yield


class _OverwriteGuardTestBase(unittest.TestCase):
    """Fresh temp dir; SETTINGS_PATH and EXPORT_STATE_PATH both redirected
    into it (mirrors tests/test_acq_export.py's fixture), real cycles from
    build_full_trajectory(), default Settings() throughout."""

    def setUp(self) -> None:
        tmp_str = tempfile.mkdtemp()
        self.addCleanup(lambda: shutil.rmtree(tmp_str, ignore_errors=True))
        self.tmp = Path(tmp_str)

        settings_patch = patch(
            "design.settings.SETTINGS_PATH", self.tmp / ".test_settings.json")
        settings_patch.start()
        self.addCleanup(settings_patch.stop)

        self.state_path = self.tmp / ".test_export_state.json"
        state_patch = patch("design.export.EXPORT_STATE_PATH", self.state_path)
        state_patch.start()
        self.addCleanup(state_patch.stop)

        set_settings(Settings())
        self.addCleanup(set_settings, Settings())

        self.cycles = build_full_trajectory()

    def _export(self, out: Path, force: bool) -> tuple[bool, str]:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            ok = generate_urscript(
                self.cycles, filename=out, settings=Settings(), force=force)
        return ok, buf.getvalue()


class UnreadableTrackedFileTests(_OverwriteGuardTestBase):
    """Potential tests 1 and 2 - F3's core fix: fail CLOSED on OSError."""

    def test_refuses_without_force_and_leaves_the_file_untouched(self):
        out = self.tmp / "etalement.script"
        self.assertTrue(self._export(out, force=True)[0])  # baseline, tracked
        original_bytes = out.read_bytes()
        original_mtime = out.stat().st_mtime_ns

        with _unreadable(out):
            ok, output = self._export(out, force=False)

        self.assertFalse(ok)
        self.assertIn("n'a pas pu etre lu", output)
        self.assertEqual(out.read_bytes(), original_bytes)
        self.assertEqual(out.stat().st_mtime_ns, original_mtime)

    def test_overwrites_with_force_and_prints_exactly_one_warning(self):
        out = self.tmp / "etalement.script"
        self.assertTrue(self._export(out, force=True)[0])  # baseline, tracked

        with _unreadable(out):
            ok, output = self._export(out, force=True)

        self.assertTrue(ok)
        warn_lines = [
            ln for ln in output.splitlines() if "n'a pas pu etre lu" in ln]
        self.assertEqual(
            len(warn_lines), 1,
            f"expected exactly one read-failure warning, got {warn_lines!r}")


class MissingOrCorruptStateTests(_OverwriteGuardTestBase):
    """Potential tests 3 and 4 - the allow decision is unchanged, it is now
    audible instead of silent."""

    def _hand_modify(self, out: Path) -> str:
        original = out.read_text(encoding="utf-8")
        handmade = "# hand-tuned for a robot trial\n" + original
        out.write_text(handmade, encoding="utf-8")
        return handmade

    def test_state_file_deleted_warns_once_and_still_overwrites(self):
        out = self.tmp / "etalement.script"
        self.assertTrue(self._export(out, force=True)[0])
        handmade = self._hand_modify(out)

        self.assertTrue(self.state_path.is_file())
        self.state_path.unlink()

        ok, output = self._export(out, force=False)

        self.assertTrue(
            ok, "chosen behavior (F3): export stays ALLOWED when the state "
                "file is gone, only made audible")
        warn_lines = [
            ln for ln in output.splitlines() if "absent ou illisible" in ln]
        self.assertEqual(len(warn_lines), 1)
        self.assertIn(out.name, warn_lines[0])
        self.assertNotEqual(
            out.read_text(encoding="utf-8"), handmade,
            "the export must have actually overwritten the hand-edit")

    def test_state_file_corrupt_json_warns_once_no_traceback(self):
        out = self.tmp / "etalement.script"
        self.assertTrue(self._export(out, force=True)[0])
        handmade = self._hand_modify(out)

        self.state_path.write_text("{ not valid json", encoding="utf-8")

        # Must not raise: json.JSONDecodeError is caught inside
        # _load_export_state(), which reports ok=False instead.
        ok, output = self._export(out, force=False)

        self.assertTrue(ok)
        warn_lines = [
            ln for ln in output.splitlines() if "absent ou illisible" in ln]
        self.assertEqual(len(warn_lines), 1)
        self.assertNotEqual(out.read_text(encoding="utf-8"), handmade)


class HandModifiedRefusalMessageTests(_OverwriteGuardTestBase):
    """Potential test 5."""

    def test_message_names_the_file_and_both_digests(self):
        out = self.tmp / "etalement.script"
        self.assertTrue(self._export(out, force=True)[0])
        known_digest = _digest(out.read_text(encoding="utf-8"))

        handmade = "# hand-tuned for a robot trial\n" + out.read_text(
            encoding="utf-8")
        out.write_text(handmade, encoding="utf-8")
        current_digest = _digest(handmade)

        message = check_overwrite(out)
        self.assertIsNotNone(message)
        self.assertIn(out.name, message)
        self.assertIn(known_digest, message)
        self.assertIn(current_digest, message)


class UntouchedFileTests(_OverwriteGuardTestBase):
    """Potential test 6."""

    def test_untouched_file_no_hand_edit_warning_export_proceeds(self):
        out = self.tmp / "etalement.script"
        self.assertTrue(self._export(out, force=True)[0])

        ok, output = self._export(out, force=False)

        self.assertTrue(ok)
        # Deliberately narrow: a default export legitimately prints an
        # UNRELATED clamp warning (URSCRIPT_TRANSIT_V exceeds the PolyScope
        # cap by design, see design/settings_spec.py), so this asserts the
        # absence of the hand-edit-guard's own wording, not of every WARN.
        self.assertNotIn("a ete modifie depuis le dernier export", output)
        self.assertNotIn("n'a pas pu etre lu", output)
        self.assertNotIn("absent ou illisible", output)


class UrpSpecificGuardTests(_OverwriteGuardTestBase):
    """Potential test 7 - etalement.urp specifically: refused without
    force, overwritten only when force=True is the explicit ask (guards the
    operator memory that this file is hand-tuned on the pendant)."""

    def test_hand_edited_urp_refused_without_force_overwritten_with_it(self):
        out = self.tmp / "etalement.urp"
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            self.assertTrue(generate_urp(
                self.cycles, filename=out, settings=Settings(), force=True))

        handmade = out.read_text(encoding="utf-8").replace(
            "<program", "<!-- hand-tuned for a robot trial -->\n<program", 1)
        out.write_text(handmade, encoding="utf-8")

        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            refused = generate_urp(
                self.cycles, filename=out, settings=Settings(), force=False)
        self.assertFalse(refused)
        self.assertEqual(out.read_text(encoding="utf-8"), handmade)

        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            accepted = generate_urp(
                self.cycles, filename=out, settings=Settings(), force=True)
        self.assertTrue(accepted)
        self.assertNotEqual(out.read_text(encoding="utf-8"), handmade)


if __name__ == "__main__":
    unittest.main()
