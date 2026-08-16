"""
tests/test_export_bytes.py - F2 and F13
(docs/superpower/plans/erreur_hors_datalogger.md).

F2: _write_export() now writes with newline='\n', and _validate_script_memory()
measures len(content.encode('utf-8')) instead of Path.stat().st_size, so the
PolyScope memory gate is byte-driven rather than platform-driven (Windows text
mode used to add one CRLF per line, bytes the controller never sees).

F13: _reject_invalid_settings() is now called by all four generators
(generate_urscript, generate_urp, generate_urscript_acq, generate_urp_acq)
before anything is built or opened, and force=True does not override it -
force concerns only the F3 hand-edit guard, never settings validity.

Four classes:
  1. NoCarriageReturnTests      - F2, test 1: none of the four generators
                                   emit a \r byte, .script and .urp alike.
  2. ByteLengthMatchesGateTests - F2, test 2: the byte count
                                   _validate_script_memory() measures for the
                                   written content equals the file's size on
                                   disk. _validate_script_memory() is spied on
                                   rather than re-derived by hand, so this does
                                   not depend on reconstructing the
                                   URScript/XML assembly a second time.
  3. GateIsByteDrivenTests      - F2, test 3: the verdict flips exactly at the
                                   content-length boundary, driven through
                                   Settings.urscript_max_bytes (the settings
                                   layer), not by editing
                                   design.params.URSCRIPT_MAX_BYTES - see the
                                   class docstring for why the module constant
                                   would not even work.
  4. ValidityGateTests          - F13, tests 5-7: an invalid Settings object
                                   refuses each of the four generators and
                                   leaves no file at the target path,
                                   force=True does not change that, and a
                                   valid Settings object still exports
                                   normally through all four.

Fixture style follows tests/test_acq_export.py: every output goes to a fresh
tempfile.mkdtemp(), and design.export.EXPORT_STATE_PATH /
design.settings.SETTINGS_PATH are both patched into that directory so a run
cannot touch the operator's real digest file or settings override file.
"""

from __future__ import annotations

import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import design.export as export_module
from design.export import (
    _build_urscript_lines,
    _validate_script_memory,
    generate_urp,
    generate_urp_acq,
    generate_urscript,
    generate_urscript_acq,
)
from design.settings import Settings, set_settings
from design.trajectory import build_full_trajectory

# The four generators under test, each as (function, output-file name, the
# label it prints). Same order the module itself declares them in.
_GENERATORS = (
    (generate_urscript, "etalement.script", "URScript"),
    (generate_urp, "etalement.urp", "URP"),
    (generate_urscript_acq, "etalement_acq.script", "URScript ACQ"),
    (generate_urp_acq, "etalement_acq.urp", "URP ACQ"),
)


class _ExportBytesTestBase(unittest.TestCase):
    """Fresh temp dir; EXPORT_STATE_PATH and SETTINGS_PATH both redirected
    into it, mirroring tests/test_acq_export.py's fixture."""

    def setUp(self) -> None:
        tmp_str = tempfile.mkdtemp()
        self.addCleanup(lambda: shutil.rmtree(tmp_str, ignore_errors=True))
        self.tmp = Path(tmp_str)

        state_patch = patch(
            "design.export.EXPORT_STATE_PATH",
            self.tmp / ".test_export_state.json")
        state_patch.start()
        self.addCleanup(state_patch.stop)

        settings_patch = patch(
            "design.settings.SETTINGS_PATH", self.tmp / ".test_settings.json")
        settings_patch.start()
        self.addCleanup(settings_patch.stop)

        set_settings(Settings())
        self.addCleanup(set_settings, Settings())

        self.cycles = build_full_trajectory()


class NoCarriageReturnTests(_ExportBytesTestBase):
    """F2, test 1: no \r byte anywhere in any of the four generated files."""

    def test_no_carriage_return_in_any_generator_output(self):
        for generator, filename, label in _GENERATORS:
            with self.subTest(label=label):
                out = self.tmp / filename
                ok = generator(self.cycles, filename=out, force=True)
                self.assertTrue(ok, f"{label} export unexpectedly refused")
                raw = out.read_bytes()
                self.assertNotIn(
                    b"\r", raw, f"{label} output contains a CR byte")


class ByteLengthMatchesGateTests(_ExportBytesTestBase):
    """F2, test 2: disk and gate agree. The byte count
    _validate_script_memory() is handed for the written content is exactly
    the file's size on disk. The real function is spied on (call recorded,
    then delegated) rather than re-derived by hand, so the assertion does not
    depend on reconstructing the URScript/XML assembly a second time."""

    def _generate_and_capture_measured_content(self, generator, out):
        captured: dict[str, str | None] = {"content": None}
        real = export_module._validate_script_memory

        def _spy(filename, label, content=None):
            captured["content"] = content
            return real(filename, label, content)

        with patch("design.export._validate_script_memory", side_effect=_spy):
            ok = generator(self.cycles, filename=out, force=True)
        return ok, captured["content"]

    def test_disk_size_equals_the_measured_content_length(self):
        for generator, filename, label in _GENERATORS:
            with self.subTest(label=label):
                out = self.tmp / filename
                ok, content = self._generate_and_capture_measured_content(
                    generator, out)
                self.assertTrue(ok, f"{label} export unexpectedly refused")
                self.assertIsNotNone(
                    content, f"{label} did not pass its content to the gate")
                self.assertEqual(
                    out.stat().st_size, len(content.encode("utf-8")),
                    f"{label}: disk size and gate measurement disagree")


class GateIsByteDrivenTests(_ExportBytesTestBase):
    """F2, test 3: the verdict flips exactly at the content-length boundary,
    driven through Settings.urscript_max_bytes (the settings layer), never
    through design.params.URSCRIPT_MAX_BYTES.

    _validate_script_memory() is exercised directly rather than through a
    full generate_*() call. Settings.urscript_max_bytes is editable=False
    (design/settings_spec.py): Settings.validate() rejects ANY Settings
    object whose urscript_max_bytes differs from the params default, so a
    Settings instance built to carry a different budget is itself refused by
    _reject_invalid_settings() (F13) before a full generate_*() call would
    ever reach the memory check. See the final report for this as a
    defect noted outside F2/F13's own scope. Routing the threshold through
    design.params.URSCRIPT_MAX_BYTES instead would not work either:
    Settings.urscript_max_bytes is a dataclass field default evaluated once
    at class-definition time, so reassigning the module constant after
    import changes no Settings() instance, past or future - which is
    exactly why the settings layer, not params, is the only way to drive
    this threshold at all.
    """

    def test_gate_fails_one_byte_under_and_passes_one_byte_over(self):
        content = "\n".join(_build_urscript_lines(self.cycles))
        content_len = len(content.encode("utf-8"))
        target = self.tmp / "unused.script"

        low = Settings()
        low.urscript_max_bytes = content_len - 1
        set_settings(low)
        self.assertFalse(
            _validate_script_memory(target, "URScript", content),
            "one byte under budget must fail")

        high = Settings()
        high.urscript_max_bytes = content_len + 1
        set_settings(high)
        self.assertTrue(
            _validate_script_memory(target, "URScript", content),
            "one byte over budget must pass")


class ValidityGateTests(_ExportBytesTestBase):
    """F13, tests 5-7: _reject_invalid_settings() runs before anything is
    built or opened, in all four generators, and force=True does not
    override it - force only ever concerns the F3 hand-edit guard."""

    @staticmethod
    def _invalid_settings() -> Settings:
        s = Settings()
        s.force_z_target = 200.0  # SPECS bound is [2.0, 20.0].
        return s

    def test_invalid_settings_refuses_each_generator_and_writes_no_file(self):
        bad = self._invalid_settings()
        for generator, filename, label in _GENERATORS:
            with self.subTest(label=label):
                out = self.tmp / filename
                self.assertFalse(out.exists())
                ok = generator(self.cycles, filename=out, settings=bad)
                self.assertFalse(ok, f"{label} accepted invalid settings")
                self.assertFalse(
                    out.exists(),
                    f"{label} wrote a file despite invalid settings")

    def test_force_true_does_not_bypass_the_validity_gate(self):
        bad = self._invalid_settings()
        for generator, filename, label in _GENERATORS:
            with self.subTest(label=label):
                out = self.tmp / filename
                ok = generator(
                    self.cycles, filename=out, settings=bad, force=True)
                self.assertFalse(
                    ok, f"{label}: force=True bypassed the validity gate")
                self.assertFalse(
                    out.exists(),
                    f"{label}: force=True let an invalid export write a "
                    f"file")

    def test_force_true_still_bypasses_the_handedit_guard(self):
        good = Settings()
        for generator, filename, label in _GENERATORS:
            with self.subTest(label=label):
                out = self.tmp / filename
                self.assertTrue(
                    generator(self.cycles, filename=out, settings=good,
                              force=True),
                    f"{label}: initial export with valid settings refused")

                handmade = ("# hand-edited by the operator for a robot "
                            "trial\n" + out.read_text(encoding="utf-8"))
                out.write_text(handmade, encoding="utf-8")

                refused = generator(
                    self.cycles, filename=out, settings=good, force=False)
                self.assertFalse(
                    refused, f"{label}: hand-edited file was overwritten "
                             f"without force=True")
                self.assertEqual(out.read_text(encoding="utf-8"), handmade)

                accepted = generator(
                    self.cycles, filename=out, settings=good, force=True)
                self.assertTrue(
                    accepted,
                    f"{label}: force=True failed to bypass the hand-edit "
                    f"guard")
                self.assertNotEqual(
                    out.read_text(encoding="utf-8"), handmade)

    def test_valid_settings_still_exports_normally_through_all_four(self):
        good = Settings()
        for generator, filename, label in _GENERATORS:
            with self.subTest(label=label):
                out = self.tmp / filename
                ok = generator(
                    self.cycles, filename=out, settings=good, force=True)
                self.assertTrue(ok, f"{label} refused valid settings")
                self.assertTrue(out.is_file())


if __name__ == "__main__":
    unittest.main()
