"""
tests/test_settings.py - Couche de reglages (phase 1 du plan
docs/superpower/plans/plan_variables_UI.md).

Trois classes :
  - DefaultsTests   : chaque defaut du dataclass est egal a la constante
                      correspondante de design/params.py, et respecte ses
                      propres bornes.
  - RoundTripTests  : save() puis from_file() restitue les memes valeurs, et
                      le JSON ne porte QUE les surcharges.
  - ValidationTests : hors bornes refuse, champs non editables proteges,
                      clamps signales.
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import design.params as params
from design.settings import Settings, get_settings, reload_settings
from design.settings_spec import SPECS, spec_by_name


def _as_lists(seq):
    """Normalise une sequence imbriquee en listes, tuples compris."""
    return [_as_lists(v) if isinstance(v, (list, tuple)) else v for v in seq]


class DefaultsTests(unittest.TestCase):
    """Le dataclass ne doit jamais deriver de design/params.py."""

    def test_every_field_default_matches_params(self):
        s = Settings()
        for spec in SPECS:
            with self.subTest(field=spec.name):
                expected = getattr(params, spec.const)
                actual = getattr(s, spec.name)
                if isinstance(expected, (list, tuple)):
                    # Stockage en listes : c'est ce que rend un aller-retour
                    # JSON, donc ce que le dataclass doit porter.
                    self.assertEqual(_as_lists(actual), _as_lists(expected))
                elif isinstance(expected, float):
                    self.assertAlmostEqual(actual, expected, places=9)
                else:
                    self.assertEqual(actual, expected)

    def test_defaults_pass_their_own_bounds(self):
        # Piege classique : une borne haute posee sous la valeur par defaut
        # (URSCRIPT_TRANSIT_V = 0.3 alors que le clamp PolyScope est a 0.25)
        # rendrait les defauts invalides des l'ouverture de l'IHM.
        self.assertEqual(Settings().validate(), [])

    def test_every_spec_has_a_dataclass_field(self):
        names = {f for f in Settings().__dataclass_fields__}
        for spec in SPECS:
            with self.subTest(field=spec.name):
                self.assertIn(spec.name, names)

    def test_tcp_z_is_the_sum_of_its_components(self):
        s = Settings()
        self.assertAlmostEqual(
            s.tcp_z,
            s.tcp_ft300_z + s.tcp_coupling_z + s.tcp_gripper_z + s.tcp_finger_z,
            places=9,
        )


class RoundTripTests(unittest.TestCase):

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "etalement_settings.json"

    def tearDown(self):
        self._tmp.cleanup()

    def test_save_then_from_file_restores_values(self):
        s = Settings()
        s.force_z_target = 8.0
        s.probe_force_thr = 2.5
        s.save(self.path)
        back = Settings.from_file(self.path)
        self.assertAlmostEqual(back.force_z_target, 8.0)
        self.assertAlmostEqual(back.probe_force_thr, 2.5)

    def test_only_overrides_are_written(self):
        s = Settings()
        s.force_z_target = 8.0
        s.save(self.path)
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        self.assertEqual(set(payload["overrides"]), {"force_z_target"})

    def test_untouched_fields_keep_the_params_default(self):
        s = Settings()
        s.force_z_target = 8.0
        s.save(self.path)
        back = Settings.from_file(self.path)
        self.assertAlmostEqual(back.force_limit_xy, params.FORCE_LIMIT_XY)

    def test_missing_file_yields_pure_defaults(self):
        back = Settings.from_file(self.path / "absent.json")
        self.assertEqual(back.to_overrides(), {})

    def test_reset_returns_to_params(self):
        s = Settings()
        s.force_z_target = 8.0
        s.reset()
        self.assertAlmostEqual(s.force_z_target, params.FORCE_Z_TARGET)
        self.assertEqual(s.to_overrides(), {})

    def test_unknown_key_in_json_is_ignored(self):
        self.path.write_text(
            json.dumps({"overrides": {"champ_inexistant": 1.0}}), encoding="utf-8"
        )
        back = Settings.from_file(self.path)
        self.assertEqual(back.to_overrides(), {})

    def test_fingerprint_is_stable_and_tracks_overrides(self):
        a, b = Settings(), Settings()
        self.assertEqual(a.fingerprint(), b.fingerprint())
        b.force_z_target = 8.0
        self.assertNotEqual(a.fingerprint(), b.fingerprint())

    def test_get_settings_is_a_reloadable_singleton(self):
        first = get_settings()
        self.assertIs(first, get_settings())
        self.assertIsNot(first, reload_settings(self.path))


class ValidationTests(unittest.TestCase):

    def test_out_of_bounds_is_refused_and_names_field_and_bounds(self):
        s = Settings()
        s.force_z_target = 99.0
        errors = s.validate()
        self.assertEqual(len(errors), 1)
        self.assertIn("force_z_target", errors[0])
        self.assertIn("20", errors[0])

    def test_negative_force_is_refused(self):
        s = Settings()
        s.force_z_target = -1.0
        self.assertTrue(s.validate())

    def test_zero_probe_travel_is_refused(self):
        s = Settings()
        s.probe_max_travel = 0.0
        self.assertTrue(s.validate())

    def test_non_editable_field_cannot_deviate(self):
        s = Settings()
        s.urscript_max_tcp_speed = 0.9
        errors = s.validate()
        self.assertTrue(any("urscript_max_tcp_speed" in e for e in errors))

    def test_locked_calibration_field_cannot_deviate_while_locked(self):
        s = Settings()
        s.robot_x_origin = 0.5
        self.assertTrue(any("robot_x_origin" in e for e in s.validate()))
        s.calibration_unlocked = True
        self.assertEqual(s.validate(), [])

    def test_plane3_probe_mode_is_refused(self):
        s = Settings()
        s.probe_mode = "plane3"
        self.assertTrue(s.validate())

    def test_clamp_reports_transit_speed_above_polyscope_cap(self):
        s = Settings()
        clamps = s.clamps()
        self.assertTrue(any("urscript_transit_v" in c for c in clamps))
        self.assertAlmostEqual(
            s.clamped("urscript_transit_v"), params.URSCRIPT_MAX_TCP_SPEED
        )

    def test_clamp_leaves_a_compliant_speed_untouched(self):
        s = Settings()
        self.assertAlmostEqual(
            s.clamped("urscript_contact_v"), params.URSCRIPT_CONTACT_V
        )

    def test_cross_check_flags_contact_depth_against_recontact_speed(self):
        s = Settings()
        s.force_contact_depth = 0.015
        s.urscript_recontact_v = 0.002
        self.assertTrue(s.warnings())

    def test_every_spec_declares_a_label_and_a_group(self):
        for spec in SPECS:
            with self.subTest(field=spec.name):
                self.assertTrue(spec.label)
                self.assertIn(spec.group, {"force", "probe", "motion",
                                           "surface", "calibration"})

    def test_spec_lookup_by_name(self):
        self.assertEqual(spec_by_name("force_z_target").unit, "N")


if __name__ == "__main__":
    unittest.main()
