"""
tests/test_export_reproducibility.py - Verrouille F10 (docs/superpower/plans/
erreur_hors_datalogger.md) : ce que le depot montre doit pouvoir se
regenerer, et le bloc de recette qu'un export porte doit dire assez pour
verifier qu'il ne se trompe pas.

Deux references coexistent dans ce depot, et les confondre est exactement ce
que F10 reproche. tests/fixtures/golden_headless.script est la sortie du
chemin d'export headless aux reglages par defaut de design/params.py,
sous-echantillonnee sur les cycles circulaires. etalement.script est l'essai
de reference, le chemin d'export de l'interface (200 points par cycle
circulaire, cycles 4 a 6 triangules), regenerable depuis
etalement_trial.json. Les quatre tests exiges par F10 sont ici : identite
octet pour octet de l'export headless, reproduction de l'essai depuis sa
configuration enregistree, controle croise du bloc de recette (le nombre de
waypoints annonce doit correspondre au nombre reel de movel emis dans le
corps de chaque cycle), et un garde-fou qui nomme les deux fichiers quand
l'un est compare a la reference de l'autre. Un cinquieme test verifie que le
bloc de recette ne porte aucune date, la raison meme pour laquelle il est
redevenu inconditionnel.
"""

from __future__ import annotations

import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from design.export import _build_urscript_lines
from design.settings import Settings, get_settings, set_settings
from design.trajectory import build_full_trajectory
from design.trial_config import (
    TRIAL_CONFIG_PATH,
    build_trial_cycles,
    load_trial_config,
    trial_settings,
)

_REPO_ROOT = Path(__file__).resolve().parents[1]
GOLDEN = _REPO_ROOT / "tests" / "fixtures" / "golden_headless.script"
ETALEMENT_SCRIPT = _REPO_ROOT / "etalement.script"

# Trois movel de service par cycle, ni comptes ni annonces comme waypoints
# dans le bloc de recette : le transit d'entree, la descente de recontact
# (juste apres force_mode), et le transit de sortie. Voir la boucle sur
# `cycles` dans design/export.py::_build_urscript_lines.
_SERVICE_MOVELS_PER_CYCLE = 3

_HEADER_CYCLE_RE = re.compile(
    r"^#\s+cycle\s+(\d+)\s*:\s*type=(\S+)\s+waypoints=(\d+)\s+label=(.*)$")
_DEF_CYCLE_RE = re.compile(r"^def cycle_(\d+)\(\):$")
_DATE_RE = re.compile(r"\b20\d{2}-\d{2}-\d{2}\b")


def _headless_script() -> tuple[str, list[dict]]:
    """
    Construit le script du chemin d'export headless, aux reglages par
    defaut : c'est ce que produit `python ur5_etalementv6.py --export
    --no-show` sur un poste sans etalement_settings.json local.
    """
    set_settings(Settings())
    cycles = build_full_trajectory()
    lines = _build_urscript_lines(cycles, settings=Settings())
    return "\n".join(lines), cycles


def _trial_script(
    config_path: Path = TRIAL_CONFIG_PATH,
) -> tuple[str, list[dict], Settings]:
    """
    Rejoue le chemin d'export de l'interface a partir de la configuration
    d'essai enregistree : c'est ce que produit `python ur5_etalementv6.py
    --export-trial --no-show`.

    N'installe deliberement AUCUN reglage prealable. C'est build_trial_cycles
    qui installe ceux de l'essai, et le test doit passer par ce chemin-la, pas
    par un chemin propre a lui : une version anterieure de cette aide posait
    elle-meme les defauts purs, ce qui rendait le test vert alors que le CLI
    laissait la geometrie se calculer avec les reglages du poste.
    """
    cfg = load_trial_config(config_path)
    cycles = build_trial_cycles(cfg)
    s_trial = get_settings()
    lines = _build_urscript_lines(cycles, settings=s_trial)
    return "\n".join(lines), cycles, s_trial


def _header_waypoints_by_cycle(script: str) -> dict[int, int]:
    """
    Extrait, pour chaque cycle, le nombre de waypoints annonce par le bloc
    de recette (ligne '#   cycle N : type=... waypoints=... label=...').
    """
    out: dict[int, int] = {}
    for line in script.split("\n"):
        m = _HEADER_CYCLE_RE.match(line)
        if m:
            out[int(m.group(1))] = int(m.group(3))
    return out


def _body_movel_counts_by_cycle(script: str) -> dict[int, int]:
    """
    Compte les 'movel(' effectivement ecrits dans le corps de chaque
    'def cycle_N(): ... end' tel qu'emis dans le script, independamment de
    ce que le bloc de recette annonce plus haut dans le fichier.
    """
    out: dict[int, int] = {}
    current: int | None = None
    for line in script.split("\n"):
        m = _DEF_CYCLE_RE.match(line)
        if m:
            current = int(m.group(1))
            out[current] = 0
            continue
        if current is None:
            continue
        if line == "end":
            current = None
            continue
        if line.strip().startswith("movel("):
            out[current] += 1
    return out


def _reference_mismatch_message(
    export_label: str,
    wrong_reference: Path,
    wrong_export_path: str,
    correct_reference: Path,
    correct_export_path: str,
) -> str:
    """
    --------------------------------------------------------------------------
    Purpose:
        Message emis quand un export est compare a la mauvaise des deux
        references du depot. Nomme les deux fichiers ainsi que leur chemin
        d'export (interface ou headless), pour qu'une confusion future entre
        etalement.script (essai, chemin d'export interface) et
        golden_headless.script (chemin d'export headless, sous-echantillonne)
        ne puisse plus passer en silence.

    Inputs:
        export_label (str): designe l'export controle (par exemple
            'export headless').
        wrong_reference (Path): fichier auquel l'export a ete compare a tort.
        wrong_export_path (str): chemin d'export associe a wrong_reference.
        correct_reference (Path): fichier qui aurait du servir de reference.
        correct_export_path (str): chemin d'export associe a correct_reference.

    Outputs:
        message (str): explication nommant les deux fichiers et leurs deux
        chemins d'export.
    --------------------------------------------------------------------------
    """
    return (
        f"{export_label} a ete compare a {wrong_reference.name} "
        f"(chemin d'export {wrong_export_path}), qui n'est pas sa reference. "
        f"La bonne reference pour {export_label} est "
        f"{correct_reference.name} (chemin d'export {correct_export_path}).")


def _assert_matches_reference(
    testcase: unittest.TestCase,
    produced: str,
    export_label: str,
    reference: Path,
    reference_export_path: str,
    other_reference: Path,
    other_export_path: str,
) -> None:
    """
    --------------------------------------------------------------------------
    Purpose:
        Compare `produced` a `reference` octet pour octet. En cas d'ecart,
        echoue via _reference_mismatch_message plutot que par la diff
        generique d'assertEqual, pour que le message d'echec nomme les deux
        references du depot au lieu de seulement montrer ou les chaines
        divergent.

    Inputs:
        testcase (unittest.TestCase): cas de test appelant, pour testcase.fail.
        produced (str): contenu de l'export a verifier.
        export_label (str): designe l'export controle.
        reference (Path): reference attendue pour cet export.
        reference_export_path (str): chemin d'export associe a reference.
        other_reference (Path): l'AUTRE reference du depot, nommee dans le
            message d'echec pour qu'une confusion entre les deux se voie.
        other_export_path (str): chemin d'export associe a other_reference.

    Outputs:
        None. Leve AssertionError (via testcase.fail) en cas d'ecart.
    --------------------------------------------------------------------------
    """
    expected = reference.read_text(encoding="utf-8")
    if produced != expected:
        testcase.fail(_reference_mismatch_message(
            export_label, reference, reference_export_path,
            other_reference, other_export_path))


class HeadlessExportIdentityTests(unittest.TestCase):
    """Test 1 de F10 : la sortie headless aux defauts ne bouge pas."""

    def tearDown(self):
        set_settings(Settings())

    def test_headless_export_equals_golden_fixture_byte_for_byte(self):
        """
        Lecture en utf-8 sans conversion de fin de ligne, pour que la
        comparaison reste reellement octet pour octet : c'est ce fichier qui
        verifie que le generateur n'a pas change de comportement aux
        reglages par defaut de design/params.py.
        """
        produced, _ = _headless_script()
        expected = GOLDEN.read_text(encoding="utf-8")
        self.assertEqual(
            produced, expected,
            f"L'export headless aux defauts ne correspond plus a "
            f"{GOLDEN}. Regenerer la reference si l'ecart est voulu.")


class TrialReproductionTests(unittest.TestCase):
    """Test 2 de F10 : l'essai se regenere depuis sa configuration."""

    def tearDown(self):
        set_settings(Settings())

    def test_regenerating_the_trial_reproduces_the_committed_script(self):
        """
        Le message d'assertion nomme sa source de configuration
        (etalement_trial.json) : un echec doit dire d'ou venait la
        configuration qui n'a pas reproduit l'artefact commis, plutot que de
        laisser deviner si la cause est le fichier de configuration ou le
        generateur.
        """
        produced, _, _ = _trial_script()
        expected = ETALEMENT_SCRIPT.read_text(encoding="utf-8")
        self.assertEqual(
            produced, expected,
            f"La regeneration de l'essai depuis {TRIAL_CONFIG_PATH} ne "
            f"reproduit plus {ETALEMENT_SCRIPT} octet pour octet. "
            f"Configuration source : {TRIAL_CONFIG_PATH}.")

    def test_workstation_settings_do_not_leak_into_the_regenerated_trial(self):
        """
        Un essai doit se regenerer a l'identique quelle que soit la machine.
        Les generateurs et les conversions de repere lisent le singleton de
        reglages, donc un poste portant un etalement_settings.json pourrait
        teinter la geometrie de l'essai sans que rien ne le signale. On
        installe ici des reglages de poste franchement deviants avant de
        regenerer, et l'artefact doit rester le meme octet pour octet.
        """
        poste = Settings()
        poste.circ_r_circle = 3.1
        poste.circ_n_circles = 17
        poste.robot_x_origin = 0.42
        poste.calibration_unlocked = True
        set_settings(poste)

        produced, _, _ = _trial_script()
        expected = ETALEMENT_SCRIPT.read_text(encoding="utf-8")
        self.assertEqual(
            produced, expected,
            "Les reglages du poste ont fuite dans la regeneration de l'essai : "
            "la geometrie a ete calculee avec autre chose que "
            f"{TRIAL_CONFIG_PATH}.")


class RecipeHeaderCrossCheckTests(unittest.TestCase):
    """
    Test 3 de F10 : le bloc de recette dit assez pour se verifier lui-meme,
    sur les deux chemins d'export du depot.
    """

    def tearDown(self):
        set_settings(Settings())

    def test_recipe_header_waypoints_match_the_emitted_movels_on_both_paths(self):
        """
        Verifie, pour l'export headless et pour l'essai, que le bloc de
        recette nomme le nombre de cycles, le type et le nombre de
        waypoints de chacun, et l'empreinte des reglages ; puis, controle
        croise qui donne sa valeur au test, que le nombre de waypoints
        annonce par cycle correspond exactement au nombre de 'movel' emis
        dans le corps de ce cycle, une fois retires les trois movel de
        service (transit d'entree, descente de recontact, transit de
        sortie). Un en-tete qui ment vaut moins qu'une absence d'en-tete.
        """
        headless_script, headless_cycles = _headless_script()
        trial_script, trial_cycles, _ = _trial_script()
        cases = [
            ("headless", headless_script, headless_cycles),
            ("essai (interface)", trial_script, trial_cycles),
        ]

        for path_label, script, cycles in cases:
            with self.subTest(chemin=path_label):
                self.assertIn(
                    f"cycles : {len(cycles)}", script,
                    f"Le bloc de recette du chemin {path_label} ne nomme "
                    f"pas le nombre de cycles.")
                self.assertIn(
                    "empreinte des reglages", script,
                    f"Le bloc de recette du chemin {path_label} ne porte "
                    f"pas l'empreinte des reglages.")

                header_wp = _header_waypoints_by_cycle(script)
                body_movels = _body_movel_counts_by_cycle(script)
                self.assertEqual(
                    set(header_wp), set(range(1, len(cycles) + 1)),
                    f"Le bloc de recette du chemin {path_label} n'annonce "
                    f"pas exactement un cycle par cycle reellement emis.")

                for idx in range(1, len(cycles) + 1):
                    announced = header_wp[idx]
                    emitted = body_movels.get(idx)
                    self.assertIsNotNone(
                        emitted,
                        f"Chemin {path_label}, cycle {idx} : aucun corps "
                        f"'def cycle_{idx}():' trouve dans le script.")
                    reproduced = emitted - _SERVICE_MOVELS_PER_CYCLE
                    self.assertEqual(
                        announced, reproduced,
                        f"Chemin {path_label}, cycle {idx} : l'en-tete "
                        f"annonce {announced} waypoints, mais le corps emet "
                        f"{emitted} movel, soit {reproduced} une fois "
                        f"retires les {_SERVICE_MOVELS_PER_CYCLE} movel de "
                        f"service. L'en-tete et le corps du script ne "
                        f"s'accordent plus.")


class WrongReferenceGuardTests(unittest.TestCase):
    """
    Test 4 de F10 : comparer un export a la mauvaise des deux references ne
    doit pas passer en silence.
    """

    def tearDown(self):
        set_settings(Settings())

    def test_comparing_headless_against_the_trial_reference_fails_and_names_both(self):
        """
        L'export headless compare a etalement.script (la reference de
        l'essai interface, pas la sienne) doit echouer, et le message doit
        nommer les deux fichiers ainsi que leurs deux chemins d'export
        (interface contre headless), pour qu'une confusion future entre
        golden_headless.script et etalement.script ne puisse plus passer
        inapercue.
        """
        produced_headless, _ = _headless_script()

        with self.assertRaises(AssertionError) as ctx:
            _assert_matches_reference(
                self, produced_headless, export_label="export headless",
                reference=ETALEMENT_SCRIPT,
                reference_export_path="interface (essai)",
                other_reference=GOLDEN, other_export_path="headless")

        message = str(ctx.exception)
        self.assertIn(ETALEMENT_SCRIPT.name, message)
        self.assertIn(GOLDEN.name, message)
        self.assertIn("interface", message)
        self.assertIn("headless", message)

        # Le garde-fou est verifie dans l'autre sens aussi : comparer
        # l'export headless a SA propre reference reussit sans lever.
        _assert_matches_reference(
            self, produced_headless, export_label="export headless",
            reference=GOLDEN, reference_export_path="headless",
            other_reference=ETALEMENT_SCRIPT,
            other_export_path="interface (essai)")


class RecipeHeaderCarriesNoDateTests(unittest.TestCase):
    """
    Bonus : verrouille la raison meme pour laquelle le bloc de recette est
    redevenu inconditionnel (F10) plutot que garde conditionnel comme avant.
    """

    def tearDown(self):
        set_settings(Settings())

    def test_neither_tracked_reference_carries_a_date(self):
        """
        Une date rendrait le bloc non deterministe et casserait la
        comparaison octet pour octet d'une sortie nominale, ce qui avait
        force a ne l'emettre que rarement. Elle ne doit reapparaitre dans
        aucune des deux references tracees.
        """
        for reference in (GOLDEN, ETALEMENT_SCRIPT):
            with self.subTest(fichier=reference.name):
                text = reference.read_text(encoding="utf-8")
                self.assertIsNone(
                    _DATE_RE.search(text),
                    f"{reference.name} porte une date, ce qui le rendrait "
                    f"non reproductible d'un jour a l'autre.")


if __name__ == "__main__":
    unittest.main()
