"""
tests/test_ui_settings.py - Fenetre de reglages, capture des valeurs
(phase 4 du plan docs/superpower/plans/plan_variables_UI.md).

Playwright a ete envisage puis ecarte le 15 aout 2026 (section 5.1 du plan) :
aucun pilote Tk n'existe. La validation se fait donc par unittest, en
pilotant SettingsWindow par son API (read_field, set_field, apply, reset,
set_unlocked) sur une racine Tk retiree (withdraw()), jamais affichee a
l'ecran. Le rendu visuel reste verifie a l'oeil par l'operateur, hors
perimetre de ce fichier.

Une seule classe, encadree par skipUnless sur la disponibilite de Tk.
"""

from __future__ import annotations

import sys
import tkinter
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    _probe = tkinter.Tk()
    _probe.destroy()
    _TK_OK = True
except Exception:
    _TK_OK = False

import design.params as params
from design.settings import Settings
from design.settings_spec import SPECS
from design.ui_settings import SettingsWindow, open_settings_window
from design.ui_widgets import default_bounds_text


@unittest.skipUnless(_TK_OK, "Tk indisponible")
class SettingsWindowTests(unittest.TestCase):
    """Capture des valeurs de la fenetre de reglages, sans affichage."""

    @classmethod
    def setUpClass(cls):
        cls.root = tkinter.Tk()
        cls.root.withdraw()

    @classmethod
    def tearDownClass(cls):
        cls.root.destroy()

    def setUp(self):
        self.settings = Settings()
        self.applied: list[Settings] = []
        self.exported: list[tuple[Settings, str]] = []
        self.window = SettingsWindow(
            self.root, self.settings,
            on_apply=self.applied.append,
            on_export=lambda s, stem: self.exported.append((s, stem)))
        self.window.top.withdraw()

    def tearDown(self):
        self.window.top.destroy()

    # 1. Une valeur valide saisie dans le widget se propage dans Settings
    #    apres apply().
    def test_valid_value_propagates_after_apply(self):
        self.window.set_field("force_z_target", 9.0)
        self.assertTrue(self.window.apply())
        self.assertAlmostEqual(self.settings.force_z_target, 9.0)
        self.assertEqual(self.applied, [self.settings])

    # 2. Hors bornes : refus, et application tout ou rien (aucun autre champ
    #    modifie n'est applique non plus).
    def test_out_of_bounds_value_is_rejected_atomically(self):
        default_xy = self.settings.force_limit_xy
        self.window.set_field("force_z_target", 999.0)
        self.window.set_field("force_limit_xy", 0.01)
        self.assertFalse(self.window.apply())
        self.assertAlmostEqual(self.settings.force_z_target, params.FORCE_Z_TARGET)
        self.assertAlmostEqual(self.settings.force_limit_xy, default_xy)
        self.assertIn("force_z_target", self.window.status_var.get())

    # 3. Un texte non numerique est refuse de la meme facon.
    def test_non_numeric_value_is_rejected(self):
        self.window.set_field("force_z_target", "abc")
        self.assertFalse(self.window.apply())
        self.assertAlmostEqual(self.settings.force_z_target, params.FORCE_Z_TARGET)

    # 4. Le defaut affiche a cote de chaque champ correspond a
    #    getattr(design.params, spec.const).
    def test_displayed_default_matches_params_for_every_field(self):
        for spec in SPECS:
            with self.subTest(field=spec.name):
                expected = getattr(params, spec.const)
                self.assertEqual(self.window.default_text(spec.name),
                                  default_bounds_text(spec, expected))

    # 5. Reinitialiser ramene tous les widgets aux defauts, to_overrides()
    #    redevient vide.
    def test_reset_restores_defaults_and_clears_overrides(self):
        self.window.set_field("force_z_target", 9.0)
        self.assertTrue(self.window.apply())
        self.assertNotEqual(self.settings.to_overrides(), {})
        self.window.reset()
        self.assertEqual(self.settings.to_overrides(), {})
        self.assertEqual(self.window.read_field("force_z_target"),
                          f"{params.FORCE_Z_TARGET:g}")

    # 6. editable=False (urscript_max_tcp_speed, urscript_max_bytes, tcp_z) :
    #    widgets desactives.
    def test_non_editable_fields_are_disabled(self):
        for spec in SPECS:
            if not spec.editable:
                with self.subTest(field=spec.name):
                    state = str(self.window.widgets[spec.name]["state"])
                    self.assertEqual(state, "disabled")

    # 7. enabled=False (champs du sondage plane3) : widgets desactives.
    def test_disabled_probe_fields_are_disabled(self):
        for spec in SPECS:
            if not spec.enabled:
                with self.subTest(field=spec.name):
                    state = str(self.window.widgets[spec.name]["state"])
                    self.assertEqual(state, "disabled")

    # 8. Un champ de calibration est desactive au depart, et devient
    #    editable apres set_unlocked(True) (et se reverrouille apres False).
    def test_calibration_field_unlocks_after_set_unlocked(self):
        widget = self.window.widgets["robot_x_origin"]
        self.assertEqual(str(widget["state"]), "disabled")
        self.window.set_unlocked(True)
        self.assertEqual(str(widget["state"]), "normal")
        self.window.set_unlocked(False)
        self.assertEqual(str(widget["state"]), "disabled")

    # 9. Le combobox probe_mode : le choix 'plane3' est refuse par apply()
    #    (settings.validate() le rejette explicitement, l'IHM ne filtre pas
    #    la liste du combobox).
    def test_probe_mode_plane3_is_rejected_by_apply(self):
        self.window.set_field("probe_mode", "plane3")
        self.assertFalse(self.window.apply())
        self.assertEqual(self.settings.probe_mode, params.PROBE_MODE)

    # 10. La ligne d'etat compte correctement les ecarts aux defauts.
    def test_status_line_counts_overrides(self):
        self.assertIn("Etat : 0 valeur(s)", self.window.status_var.get())
        self.window.set_field("force_z_target", 9.0)
        self.window.set_field("force_limit_xy", 0.01)
        self.assertTrue(self.window.apply())
        self.assertIn("Etat : 2 valeur(s)", self.window.status_var.get())
        self.window.reset()
        self.assertIn("Etat : 0 valeur(s)", self.window.status_var.get())

    # 11. Tous les champs de SPECS ont un widget dans self.widgets : aucun
    #     champ oublie par la generation des lignes.
    def test_every_spec_has_a_widget(self):
        for spec in SPECS:
            with self.subTest(field=spec.name):
                self.assertIn(spec.name, self.window.widgets)

    # Bonus : open_settings_window relie les rappels de bout en bout, c'est
    # le point d'entree que la phase 5 appellera.
    def test_open_settings_window_wires_callbacks(self):
        applied: list[Settings] = []
        exported: list[tuple[Settings, str]] = []
        window = open_settings_window(
            settings=Settings(), on_apply=applied.append,
            on_export=lambda s, stem: exported.append((s, stem)),
            master=self.root)
        try:
            window.top.withdraw()
            window.set_field("force_z_target", 7.0)
            window.export_stem_var.set("essai01")
            window.export()
            self.assertEqual(len(applied), 1)
            self.assertEqual(exported, [(window.settings, "essai01")])
        finally:
            window.top.destroy()


if __name__ == "__main__":
    unittest.main()
