"""
tests/test_settings_persistence.py - Persistance des reglages (phase 7 du
plan docs/superpower/plans/plan_variables_UI.md).

Trois classes :
  - ExampleFileTests : etalement_settings.example.json existe, est un JSON
                       valide portant les trois cles attendues, ses
                       surcharges correspondent a des champs reels de SPECS
                       et passent validate().
  - StartupBannerTests : startup_banner() est vide aux defauts, liste les
                         ecarts saisis, et mentionne le plafonnement de
                         urscript_transit_v des qu'il y a un ecart.
  - GitignoreTests : etalement_settings.json est bien ignore par git.
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from design.settings import Settings, startup_banner
from design.settings_spec import SPECS

REPO_ROOT = Path(__file__).resolve().parents[1]
EXAMPLE_PATH = REPO_ROOT / "etalement_settings.example.json"
GITIGNORE_PATH = REPO_ROOT / ".gitignore"

_KNOWN_FIELDS = {spec.name for spec in SPECS}


class ExampleFileTests(unittest.TestCase):
    """etalement_settings.example.json sert de modele versionne."""

    def test_example_file_exists_and_is_valid_json(self):
        self.assertTrue(EXAMPLE_PATH.is_file())
        payload = json.loads(EXAMPLE_PATH.read_text(encoding="utf-8"))
        self.assertEqual(
            set(payload.keys()), {"_comment", "fingerprint", "overrides"}
        )

    def test_from_file_applies_overrides_and_leaves_the_rest_at_defaults(self):
        s = Settings.from_file(EXAMPLE_PATH)
        payload = json.loads(EXAMPLE_PATH.read_text(encoding="utf-8"))
        overrides = payload["overrides"]
        self.assertTrue(overrides, "le fichier d'exemple doit surcharger au "
                                    "moins un champ")
        for name, value in overrides.items():
            with self.subTest(field=name):
                self.assertEqual(getattr(s, name), value)
        # Un champ absent de overrides doit garder le defaut de params.py.
        defaults = Settings()
        for spec in SPECS:
            if spec.name not in overrides:
                with self.subTest(field=spec.name):
                    self.assertEqual(getattr(s, spec.name),
                                      getattr(defaults, spec.name))

    def test_every_override_key_names_a_real_spec_field(self):
        payload = json.loads(EXAMPLE_PATH.read_text(encoding="utf-8"))
        for name in payload["overrides"]:
            with self.subTest(field=name):
                self.assertIn(name, _KNOWN_FIELDS,
                               f"{name} ne correspond a aucun champ de "
                               f"SPECS : exemple documentant un champ "
                               f"inexistant.")

    def test_example_values_pass_validate(self):
        s = Settings.from_file(EXAMPLE_PATH)
        self.assertEqual(s.validate(), [])


class StartupBannerTests(unittest.TestCase):

    def test_empty_at_defaults(self):
        self.assertEqual(startup_banner(Settings()), [])

    def test_lists_an_override_with_field_name_old_and_new_value(self):
        s = Settings()
        s.force_z_target = 8.0
        lines = startup_banner(s)
        self.assertTrue(lines)
        joined = "\n".join(lines)
        self.assertIn("force_z_target", joined)
        self.assertIn("6", joined)
        self.assertIn("8", joined)
        self.assertIn("->", joined)

    def test_mentions_the_transit_speed_clamp_once_something_is_overridden(self):
        # URSCRIPT_TRANSIT_V par defaut (0.3 m/s) depasse deja le plafond
        # PolyScope (0.25 m/s) : clamps() n'est jamais vide, la bannière
        # doit donc le signaler des qu'un ecart declenche son affichage.
        s = Settings()
        s.probe_force_thr = 3.0
        lines = startup_banner(s)
        self.assertTrue(any("urscript_transit_v" in line for line in lines))
        self.assertTrue(any("[plafond]" in line for line in lines))

    def test_header_names_source_and_fingerprint(self):
        s = Settings()
        s.force_z_target = 8.0
        s.source = "essai_labo"
        lines = startup_banner(s)
        self.assertIn("essai_labo", lines[0])
        self.assertIn(s.fingerprint(), lines[0])

    def test_uses_the_active_settings_when_none_is_passed(self):
        from design.settings import set_settings
        s = Settings()
        s.force_z_target = 9.0
        set_settings(s)
        try:
            lines = startup_banner()
            self.assertTrue(any("force_z_target" in line for line in lines))
        finally:
            set_settings(Settings())


class GitignoreTests(unittest.TestCase):

    def test_gitignore_ignores_the_local_settings_file(self):
        text = GITIGNORE_PATH.read_text(encoding="utf-8")
        self.assertIn("etalement_settings.json", text)


class RoundTripTests(unittest.TestCase):
    """Ecriture puis relecture d'un fichier de surcharges temporaire."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name) / "etalement_settings.json"

    def tearDown(self):
        self._tmp.cleanup()

    def test_round_trip_banner_names_the_source(self):
        s = Settings()
        s.force_z_target = 7.5
        s.probe_force_thr = 2.5
        s.save(self.path)

        back = Settings.from_file(self.path)
        lines = startup_banner(back)

        self.assertTrue(lines)
        self.assertIn(str(self.path), lines[0])
        joined = "\n".join(lines)
        self.assertIn("force_z_target", joined)
        self.assertIn("probe_force_thr", joined)


if __name__ == "__main__":
    unittest.main()
