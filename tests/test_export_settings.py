"""
tests/test_export_settings.py - L'exporteur lit les reglages a l'execution
(phases 2 et 6 du plan docs/superpower/plans/plan_variables_UI.md).

Deux classes :
  - IdentityTests    : a reglages par defaut, la sortie est identique octet
                       pour octet a tests/fixtures/golden_headless.script,
                       fige AVANT la bascule des imports par valeur.
  - UsesSettingsTests : regression sur l'obstacle de la section 2. Modifier
                       Settings doit reellement changer le script ou la
                       trajectoire ; aucun champ expose ne doit etre inerte.

Note sur le fichier de reference : le etalement.script du depot provient du
chemin d'export de l'interface (tous les points du trace), pas du chemin
headless (sous-echantillonne a URSCRIPT_N_WAYPOINTS_CIRCULAR). Les deux
divergent depuis longtemps, ce que la section 6.3 du plan appelle a unifier.
L'invariant d'identite porte donc sur la sortie headless, seule sortie
reproductible sans interface.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from design.export import (
    _build_urscript_lines,
    _recipe_header_lines,
    _settings_header_lines,
)
from design.settings import Settings, set_settings
from design.settings_spec import SPECS
from design.trajectory import build_full_trajectory

GOLDEN = Path(__file__).resolve().parent / "fixtures" / "golden_headless.script"

# Champs exposes qui n'influencent legitimement ni le script ni la trajectoire.
# Tout ajout ici doit porter sa raison : c'est la seule protection contre un
# champ qui donne a l'operateur une fausse impression de controle.
INERT_ON_PURPOSE: dict[str, str] = {
    "probe_mode": "une seule valeur utilisable, plane3 est parque",
    "circ_duration": "duree cible du protocole, affichage seulement",
    "lin_duration_odd": "duree cible du protocole, affichage seulement",
    "lin_duration_even": "duree cible du protocole, affichage seulement",
    "ui_discretization_points": "curseur de l'interface de trace, phase 5",
}


def _script_at(settings: Settings) -> str:
    set_settings(settings)
    return "\n".join(_build_urscript_lines(build_full_trajectory()))


def _trajectory_signature(settings: Settings) -> bytes:
    set_settings(settings)
    cycles = build_full_trajectory()
    return b"".join(np.round(c["pts"], 6).tobytes() for c in cycles)


def _perturb(settings: Settings, spec) -> bool:
    """Ecarte un champ de son defaut, en restant dans ses bornes. False si
    le champ ne se prete pas a une perturbation scalaire."""
    value = getattr(settings, spec.name)
    if spec.kind == "choice":
        other = [c for c in spec.choices if c != value]
        if not other:
            return False
        setattr(settings, spec.name, other[0])
        return True
    if spec.kind in ("points", "vector") or spec.lo is None:
        return False
    if spec.kind == "int":
        new = value + 1 if value + 1 <= spec.hi else value - 1
        if new < spec.lo or new == value:
            return False
        setattr(settings, spec.name, int(new))
        return True
    new = value * 1.5 if value else (spec.lo + spec.hi) / 2.0
    new = min(max(new, spec.lo), spec.hi)
    if abs(new - value) < 1e-12:
        new = (spec.lo + spec.hi) / 2.0
    if abs(new - value) < 1e-12:
        return False
    setattr(settings, spec.name, float(new))
    return True


class IdentityTests(unittest.TestCase):
    """A reglages par defaut, rien ne bouge."""

    def tearDown(self):
        set_settings(Settings())

    def test_default_export_is_byte_identical_to_the_golden(self):
        produced = _script_at(Settings())
        expected = GOLDEN.read_text(encoding="utf-8")
        self.assertEqual(produced, expected)

    def test_deviation_list_is_empty_when_nothing_is_overridden(self):
        # Seule la LISTE DES ECARTS est conditionnelle : un export aux defauts
        # n'a rien a lister. La recette, elle, est emise dans tous les cas
        # (F10) ; c'est ce que verifie le test suivant.
        self.assertEqual(_settings_header_lines(Settings()), [])

    def test_deviation_list_names_the_change(self):
        s = Settings()
        s.force_z_target = 8.0
        block = "\n".join(_settings_header_lines(s))
        self.assertIn("FORCE_Z_TARGET", block)
        self.assertIn("6.0 -> 8.0", block)

    def test_recipe_block_is_emitted_even_at_pure_defaults(self):
        # Le coeur de F10 : sans cette garantie, l'export le plus courant est
        # justement celui qui ne dit rien de ce qui l'a produit.
        script = _script_at(Settings())
        self.assertIn("=== RECETTE DE REPRODUCTION ===", script)
        self.assertIn("empreinte des reglages", script)

    def test_recipe_block_carries_no_date(self):
        # Une date rendrait tout export non reproductible, ce qui avait force
        # a rendre le bloc conditionnel. Elle ne doit pas revenir.
        block = "\n".join(_recipe_header_lines([], Settings()))
        self.assertNotIn("genere le", block)

    def test_recipe_block_states_the_cycle_configuration(self):
        s = Settings()
        cycles = build_full_trajectory()
        block = "\n".join(_recipe_header_lines(cycles, s))
        self.assertIn(f"cycles : {len(cycles)}", block)
        for cyc in cycles:
            self.assertIn(cyc["label"], block)
        self.assertIn(s.fingerprint(), block)


class UsesSettingsTests(unittest.TestCase):
    """Regression sur l'obstacle des imports par valeur (section 2)."""

    def tearDown(self):
        set_settings(Settings())

    def test_force_target_reaches_the_force_mode_wrench(self):
        # La force reste inlinee dans le wrench de force_mode. Son exposition
        # comme global URScript editable sur le pendant appartient a l'autre
        # plan (plan_optimisation_urscript.md, section 5).
        s = Settings()
        s.force_z_target = 12.0
        self.assertIn("[0, 0, -12.0, 0, 0, 0]", _script_at(s))

    def test_force_limits_reach_the_force_mode_call(self):
        s = Settings()
        s.force_limit_xy = 0.012
        self.assertIn("[0.012, 0.012,", _script_at(s))

    def test_probe_threshold_reaches_the_script(self):
        s = Settings()
        s.probe_force_thr = 2.5
        self.assertIn("global PROBE_FORCE_THR    = 2.5", _script_at(s))

    def test_robot_origin_moves_every_pose(self):
        s = Settings()
        s.calibration_unlocked = True
        s.robot_x_origin = 0.250
        self.assertNotEqual(_script_at(s), _script_at(Settings()))

    def test_waypoint_mode_all_emits_more_movel(self):
        base = _script_at(Settings()).count("movel(")
        s = Settings()
        s.circular_waypoint_mode = "all"
        self.assertGreater(_script_at(s).count("movel("), base)

    def test_surface_width_changes_the_trajectory(self):
        s = Settings()
        s.surface_w = 80.0
        self.assertNotEqual(_trajectory_signature(s),
                            _trajectory_signature(Settings()))

    def test_no_exposed_field_is_inert(self):
        """Chaque champ editable et actif change le script ou la trajectoire."""
        ref_script = _script_at(Settings())
        ref_traj = _trajectory_signature(Settings())
        for spec in SPECS:
            if not spec.editable or not spec.enabled:
                continue
            if spec.name in INERT_ON_PURPOSE:
                continue
            with self.subTest(field=spec.name):
                s = Settings()
                s.calibration_unlocked = True
                if not _perturb(s, spec):
                    self.skipTest("champ non perturbable par scalaire")
                changed = (_script_at(s) != ref_script
                           or _trajectory_signature(s) != ref_traj)
                self.assertTrue(
                    changed,
                    f"{spec.name} est expose dans l'IHM mais n'a aucun effet "
                    f"sur la sortie. L'ajouter a INERT_ON_PURPOSE avec sa "
                    f"raison, ou le brancher.")


if __name__ == "__main__":
    unittest.main()
