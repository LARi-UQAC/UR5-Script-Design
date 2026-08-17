"""
tests/test_settings_validation.py - F1 (docs/superpower/plans/erreur_hors_datalogger.md):
Settings.from_file() bounds-checks its overrides instead of applying every
known key with a bare setattr, and generate_urscript()/generate_urp() refuse
to write anything when the settings they are handed are invalid.

Six classes, each covering one bullet of F1's "Potential tests" list:

  1. OutOfBoundsRefusalTests    - a numeric override outside its FieldSpec
                                   bounds (force_z_target = 200.0) is refused:
                                   from_file() returns PURE defaults (never a
                                   partial mix), and prints one WARN line
                                   naming the field, its value and its bounds,
                                   plus a summary line saying the file was
                                   refused.
  2. TypeRefusalTests           - a string where a float belongs
                                   (force_z_target = "six") is refused the
                                   same clean way: no TypeError, no
                                   traceback (the test itself would fail with
                                   an uncaught exception if from_file() let
                                   one through).
  3. ShapeRefusalTests          - p_ref of the wrong length, and p_ref set to
                                   a bare scalar, are both refused instead of
                                   raising - this is the exact branch that
                                   used to crash inside Settings.to_overrides()
                                   (`for v in current` on a non-iterable),
                                   because validate() used to `continue` on
                                   kind in ("points", "vector") without any
                                   check at all.
  4. NullValueRefusalTests      - a JSON `null` for ANY single field (looped
                                   over every FieldSpec in SPECS) is refused,
                                   never silently carried through as a
                                   Python None.
  5. ExportGateTests            - generate_urscript()/generate_urp() return
                                   False and write NOTHING (file bytes and
                                   mtime unchanged) when handed an invalid
                                   Settings object, even with force=True: the
                                   F1 gate runs before the F3 hand-edit guard
                                   and before any file is opened.
  6. BoundaryTests               - for every FieldSpec with numeric bounds,
                                   exactly lo and exactly hi are accepted
                                   through from_file(), and lo - eps / hi + eps
                                   are refused (pure defaults).
  7. NonRegressionTests          - a settings file with no real overrides
                                   still produces a headless export that is
                                   byte-identical to tests/fixtures/
                                   golden_headless.script (the gate adds a
                                   check, it does not move a value), and the
                                   versioned etalement_settings.example.json
                                   is not spuriously blocked.
  8. ReachabilityTests           - every FieldSpec in SPECS produces SOME
                                   validation error for a deliberately
                                   invalid override, regardless of its kind:
                                   a future spec added without bounds must
                                   fail this test, not pass silently (this is
                                   the regression guard for the "points"/
                                   "vector" kinds, which validate() used to
                                   skip outright before F1).

Every test uses tempfile.mkdtemp() (never the repo root) and patches both
design.settings.SETTINGS_PATH and design.export.EXPORT_STATE_PATH into that
directory, exactly as tests/test_acq_export.py already does, so a run cannot
corrupt the operator's real settings file or export-state digest.
"""

from __future__ import annotations

import contextlib
import io
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import design.params as params
from design.export import generate_urp, generate_urscript
from design.settings import Settings, set_settings
from design.settings_spec import SPECS
from design.trajectory import build_full_trajectory

GOLDEN = Path(__file__).resolve().parent / "fixtures" / "golden_headless.script"
EXAMPLE_SETTINGS_PATH = (
    Path(__file__).resolve().parents[1] / "etalement_settings.example.json")

# Epsilon for the boundary tests: small enough that float round-tripping
# through JSON never blurs a genuine lo/hi crossing (the tightest bound in
# SPECS, force_limit_xy, still spans 0.018 between lo and hi), large enough
# to stay well clear of float64 noise (~1e-16).
_EPS = 1e-6


class _SettingsValidationTestBase(unittest.TestCase):
    """Fresh temp dir; SETTINGS_PATH and EXPORT_STATE_PATH both redirected
    into it so a run cannot touch the operator's real settings file or
    export-state digest (mirrors tests/test_acq_export.py's fixture)."""

    def setUp(self) -> None:
        tmp_str = tempfile.mkdtemp()
        self.addCleanup(lambda: shutil.rmtree(tmp_str, ignore_errors=True))
        self.tmp = Path(tmp_str)

        settings_patch = patch(
            "design.settings.SETTINGS_PATH", self.tmp / ".test_settings.json")
        settings_patch.start()
        self.addCleanup(settings_patch.stop)

        state_patch = patch(
            "design.export.EXPORT_STATE_PATH",
            self.tmp / ".test_export_state.json")
        state_patch.start()
        self.addCleanup(state_patch.stop)

        set_settings(Settings())
        self.addCleanup(set_settings, Settings())

    def _write_json(self, name: str, overrides: dict) -> Path:
        path = self.tmp / name
        path.write_text(json.dumps({"overrides": overrides}), encoding="utf-8")
        return path

    def _from_file_quiet(self, path: Path) -> tuple[Settings, str]:
        """Loads path via Settings.from_file(), capturing everything it
        prints instead of letting it reach the test runner's console."""
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            loaded = Settings.from_file(path)
        return loaded, buf.getvalue()


class OutOfBoundsRefusalTests(_SettingsValidationTestBase):
    """Potential test 1."""

    def test_force_z_target_200_returns_defaults_and_names_field_and_bounds(self):
        path = self._write_json("bad.json", {"force_z_target": 200.0})
        loaded, output = self._from_file_quiet(path)

        self.assertEqual(loaded.force_z_target, params.FORCE_Z_TARGET)
        self.assertNotEqual(loaded.force_z_target, 200.0)

        lines = [ln for ln in output.splitlines() if ln.strip()]
        field_lines = [ln for ln in lines if "force_z_target" in ln]
        self.assertEqual(
            len(field_lines), 1,
            f"expected exactly one WARN line naming force_z_target, got "
            f"{field_lines!r}")
        self.assertIn("200", field_lines[0])
        self.assertIn("[2, 20]", field_lines[0])

        summary_lines = [ln for ln in lines if "refuse" in ln.lower()]
        self.assertEqual(
            len(summary_lines), 1,
            f"expected exactly one summary line saying the file was "
            f"refused, got {summary_lines!r}")


class TypeRefusalTests(_SettingsValidationTestBase):
    """Potential test 2."""

    def test_force_z_target_string_is_refused_no_traceback(self):
        path = self._write_json("bad.json", {"force_z_target": "six"})
        # Settings.from_file() must not raise - if it did, this call itself
        # would fail the test with an uncaught TypeError/ValueError.
        loaded, output = self._from_file_quiet(path)

        self.assertEqual(loaded.force_z_target, params.FORCE_Z_TARGET)
        self.assertIn("force_z_target", output)
        self.assertIn("refuse", output.lower())


class ShapeRefusalTests(_SettingsValidationTestBase):
    """Potential test 3 - the branch that used to raise inside
    Settings.to_overrides() (`for v in current` on a non-iterable)."""

    def test_p_ref_wrong_length_is_refused(self):
        path = self._write_json("bad.json", {"p_ref": [0.1, 0.2, 0.3]})
        loaded, output = self._from_file_quiet(path)
        self.assertEqual(loaded.p_ref, list(params.P_REF))
        self.assertIn("p_ref", output)

    def test_p_ref_scalar_is_refused_not_a_crash(self):
        path = self._write_json("bad.json", {"p_ref": 0.5})
        # Must not raise: pre-F1, validate() computed
        # `list(value) != list(default)` for this field with no prior shape
        # check, and list(0.5) itself raises TypeError.
        loaded, output = self._from_file_quiet(path)
        self.assertEqual(loaded.p_ref, list(params.P_REF))
        self.assertIn("p_ref", output)

    def test_probe_points_one_malformed_point_is_refused(self):
        # Correct length (3), but the middle point is a 3-tuple instead of
        # (x, y): exercises the per-point shape check, not the length check.
        bad_points = [[5.0, 5.0], [45.0, 5.0, 99.0], [25.0, 45.0]]
        path = self._write_json("bad.json", {"probe_points_plate_mm": bad_points})
        loaded, output = self._from_file_quiet(path)
        self.assertEqual(
            loaded.probe_points_plate_mm,
            [list(p) for p in params.PROBE_POINTS_PLATE_MM])
        self.assertIn("probe_points_plate_mm", output)


class NullValueRefusalTests(_SettingsValidationTestBase):
    """Potential test 4 - looped over every FieldSpec, not just one field."""

    def test_null_value_for_every_field_is_refused(self):
        defaults = Settings()
        for spec in SPECS:
            with self.subTest(field=spec.name):
                path = self._write_json(
                    f"null_{spec.name}.json", {spec.name: None})
                loaded, output = self._from_file_quiet(path)
                self.assertEqual(
                    getattr(loaded, spec.name), getattr(defaults, spec.name),
                    f"{spec.name} = null must be refused (pure defaults), "
                    f"never partially applied")
                self.assertIn(spec.name, output)


class ExportGateTests(_SettingsValidationTestBase):
    """Potential test 5 - the gate runs before ANY file is opened, and it is
    independent of force=True (which only overrides the F3 hand-edit
    guard, never the F1 validity gate)."""

    def setUp(self) -> None:
        super().setUp()
        self.cycles = build_full_trajectory()

    def test_generate_urscript_refuses_invalid_settings_writes_nothing(self):
        out = self.tmp / "would_be_invalid.script"
        out.write_text("PRE-EXISTING CONTENT\n", encoding="utf-8")
        before_mtime = out.stat().st_mtime_ns
        before_bytes = out.read_bytes()

        bad = Settings()
        bad.force_z_target = 200.0

        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            ok = generate_urscript(
                self.cycles, filename=out, settings=bad, force=True)
        self.assertFalse(ok)
        self.assertIn("force_z_target", buf.getvalue())

        self.assertEqual(before_mtime, out.stat().st_mtime_ns,
                          "file mtime changed: something was written")
        self.assertEqual(before_bytes, out.read_bytes(),
                          "file bytes changed: something was written")

    def test_generate_urp_refuses_invalid_settings_writes_nothing(self):
        out = self.tmp / "would_be_invalid.urp"
        out.write_text("<program/>\n", encoding="utf-8")
        before_mtime = out.stat().st_mtime_ns
        before_bytes = out.read_bytes()

        bad = Settings()
        bad.probe_force_thr = -5.0

        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            ok = generate_urp(
                self.cycles, filename=out, settings=bad, force=True)
        self.assertFalse(ok)
        self.assertIn("probe_force_thr", buf.getvalue())

        self.assertEqual(before_mtime, out.stat().st_mtime_ns)
        self.assertEqual(before_bytes, out.read_bytes())

    def test_generate_urscript_still_writes_for_valid_settings(self):
        out = self.tmp / "ok.script"
        ok = generate_urscript(
            self.cycles, filename=out, settings=Settings(), force=True)
        self.assertTrue(ok)
        self.assertTrue(out.is_file())


class BoundaryTests(_SettingsValidationTestBase):
    """Potential test 6."""

    def test_boundary_pairs_for_every_bounded_spec(self):
        defaults = Settings()
        for spec in SPECS:
            if spec.lo is None or spec.hi is None:
                continue
            with self.subTest(field=spec.name):
                cases = (
                    ("lo", spec.lo, True),
                    ("hi", spec.hi, True),
                    ("lo_minus_eps", spec.lo - _EPS, False),
                    ("hi_plus_eps", spec.hi + _EPS, False),
                )
                for label, value, should_pass in cases:
                    path = self._write_json(
                        f"{spec.name}_{label}.json", {spec.name: value})
                    loaded, _ = self._from_file_quiet(path)
                    actual = getattr(loaded, spec.name)
                    if should_pass:
                        self.assertAlmostEqual(
                            actual, value, places=6,
                            msg=f"{spec.name} = {label} ({value}) must be "
                                f"accepted exactly at the boundary")
                    else:
                        self.assertAlmostEqual(
                            actual, getattr(defaults, spec.name), places=9,
                            msg=f"{spec.name} = {label} ({value}) must be "
                                f"refused (pure defaults expected)")


class NonRegressionTests(_SettingsValidationTestBase):
    """Potential test 7 - the gate adds a check, it does not move a value."""

    def setUp(self) -> None:
        super().setUp()
        self.cycles = build_full_trajectory()

    def test_file_with_no_real_overrides_still_matches_the_golden_fixture(self):
        path = self._write_json("empty.json", {})
        loaded, _ = self._from_file_quiet(path)
        self.assertEqual(loaded.validate(), [])

        out = self.tmp / "etalement.script"
        ok = generate_urscript(
            self.cycles, filename=out, settings=loaded, force=True)
        self.assertTrue(ok)
        produced = out.read_text(encoding="utf-8")
        expected = GOLDEN.read_text(encoding="utf-8")
        self.assertEqual(produced, expected)

    def test_versioned_example_settings_file_is_not_blocked_by_the_gate(self):
        loaded, _ = self._from_file_quiet(EXAMPLE_SETTINGS_PATH)
        self.assertEqual(loaded.validate(), [])

        out = self.tmp / "etalement_example.script"
        ok = generate_urscript(
            self.cycles, filename=out, settings=loaded, force=True)
        self.assertTrue(
            ok, "a legitimate, in-bounds settings file must not be refused "
                "by the F1 gate")


class ReachabilityTests(_SettingsValidationTestBase):
    """Potential test 8 - regression guard for the "points"/"vector" kinds,
    which validate() used to skip outright (`continue`) with zero check
    before F1."""

    @staticmethod
    def _invalid_value_for(spec):
        default = getattr(Settings(), spec.name)
        if spec.kind == "choice":
            return "__not_a_documented_choice__"
        if spec.kind in ("points", "vector"):
            # Same type, wrong length: drop the last element.
            return list(default)[:-1]
        if spec.hi is not None:
            return spec.hi + 1
        if spec.lo is not None:
            return spec.lo - 1
        # No bounds at all (editable=False / computed field): any different
        # value must still be caught, via the read-only check.
        if isinstance(default, (int, float)):
            return default + 1
        return "__different__"

    def test_every_spec_is_reachable_by_validate(self):
        for spec in SPECS:
            with self.subTest(field=spec.name, kind=spec.kind):
                s = Settings()
                setattr(s, spec.name, self._invalid_value_for(spec))
                errors = s.validate()
                self.assertTrue(
                    any(spec.name in e for e in errors),
                    f"{spec.name} (kind={spec.kind}) produced no validation "
                    f"error for a deliberately invalid value - a future "
                    f"spec added without bounds must fail here, not pass "
                    f"silently.")


if __name__ == "__main__":
    unittest.main()
