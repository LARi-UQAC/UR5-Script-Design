"""
design/trial_config.py - Regeneration headless de l'essai de reference.

etalement.script est l'artefact d'essai : c'est le programme qui a tourne sur
le robot. Jusqu'au 2026-08-16 rien dans le depot ne disait ce qui l'avait
produit, donc personne ne pouvait le regenerer, ni verifier qu'il s'accordait
encore avec le code. C'est le defaut F10 de
docs/superpower/plans/erreur_hors_datalogger.md.

Sa configuration a ete recuperee par inversion de ses poses vers le repere
plaque, puis verifiee par reproduction octet pour octet. Elle vit maintenant
dans etalement_trial.json, versionne, et ce module la rejoue sans interface :
c'est le chemin d'export de design/app.py, mais pilote par un fichier au lieu
de curseurs.

Ne pas confondre les deux fichiers JSON du depot. etalement_settings.json porte
les reglages d'un poste et reste gitignore ; etalement_trial.json decrit un
essai et est versionne.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import design.params as params
from design.settings import Settings, set_settings
from design.trajectory import (
    circular_cycle,
    get_waypoint_indices,
    linear_cycle,
    resample_points,
    triangular_cycle,
)

TRIAL_CONFIG_PATH: Path = params.REPO_ROOT / "etalement_trial.json"


def load_trial_config(path: Path | str = TRIAL_CONFIG_PATH) -> dict[str, Any]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Lit la configuration de l'essai de reference.

    Inputs:
        path (Path | str): chemin du fichier de configuration d'essai.

    Outputs:
        config (dict): configuration brute.
    --------------------------------------------------------------------------
    """
    return json.loads(Path(path).read_text(encoding="utf-8"))


def trial_settings(config: dict[str, Any]) -> Settings:
    """
    --------------------------------------------------------------------------
    Purpose:
        Construit les reglages de l'essai : les defauts de design/params.py,
        surcharges par les seuls ecarts que la configuration declare. Volontai-
        rement independant de etalement_settings.json, sinon la regeneration
        d'un essai dependrait des reglages du poste qui la lance.

    Inputs:
        config (dict): configuration d'essai.

    Outputs:
        settings (Settings): reglages de l'essai.
    --------------------------------------------------------------------------
    """
    s = Settings()
    for name, value in config.get("settings_overrides", {}).items():
        setattr(s, name, value)
    s.source = str(TRIAL_CONFIG_PATH.name)
    return s


def activate_trial(config: dict[str, Any]) -> Settings:
    """
    --------------------------------------------------------------------------
    Purpose:
        Installe les reglages de l'essai comme reglages DE PROCESSUS, et les
        rend.

        C'est indispensable et non decoratif. Les generateurs de trajectoire et
        les conversions de repere (`circular_cycle`, `plate_to_robot`,
        `_abs_pose`) lisent le singleton `get_settings()`, pas un argument :
        passer les reglages d'essai au seul `_build_urscript_lines` laisserait
        la geometrie se calculer avec les reglages du POSTE. Un essai
        regenere sur une machine portant un `etalement_settings.json` ne serait
        alors pas celui qu'on croit, ce qui detruirait la reproductibilite que
        F10 vient d'etablir. Aujourd'hui invisible parce que
        `settings_overrides` est vide, mais c'est une bombe a retardement, pas
        une subtilite theorique.

        L'appelant qui poursuit apres l'export doit restaurer les reglages
        precedents ; `design/app.py` le fait.

    Inputs:
        config (dict): configuration d'essai.

    Outputs:
        settings (Settings): les reglages d'essai, desormais actifs.
    --------------------------------------------------------------------------
    """
    s = trial_settings(config)
    set_settings(s)
    return s


def build_trial_cycles(config: dict[str, Any]) -> list[dict]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Rejoue le chemin d'export de l'interface a partir de la configuration
        d'essai. Les cycles circulaires sont reechantillonnes puis emis en
        entier (c'est ce que fait l'interface, contrairement au sous-echantil-
        lonnage headless) ; les cycles rectilignes suivent la forme declaree.

        Installe d'abord les reglages de l'essai, voir activate_trial : sans
        cela les cycles seraient traces avec les reglages du poste.

    Inputs:
        config (dict): configuration d'essai.

    Outputs:
        cycles (list[dict]): cycles prets pour design.export.
    --------------------------------------------------------------------------
    """
    activate_trial(config)
    circ = config["circular"]
    lin = config["linear"]
    colors_circ = ['#1f77b4', '#ff7f0e', '#2ca02c']
    colors_lin = ['#d62728', '#9467bd', '#8c564b']
    cycles: list[dict] = []

    for i, rot in enumerate(circ["rotations_deg"]):
        pts = resample_points(circular_cycle(rotation_deg=rot),
                              circ["points_per_cycle"])
        cycles.append({
            'label': circ["label_template"].format(n=i + 1, rot=rot),
            'color': colors_circ[i % len(colors_circ)],
            'pts': pts,
            'type': 'circular',
            'waypoint_indices': list(range(len(pts))),
        })

    triangular = lin.get("shape") == "triangular"
    offset = len(cycles)
    for i, rot in enumerate(lin["rotations_deg"]):
        pts = (triangular_cycle(rotation_deg=rot) if triangular
               else linear_cycle(rotation_deg=rot))
        # Le trace triangule est deja reduit a ses sommets, l'interface les
        # emet donc tous ; un trace rectiligne passe par la selection des
        # coins. C'est exactement ce que fait design/app.py.
        wp = (list(range(len(pts))) if triangular
              else get_waypoint_indices(len(pts), 'linear'))
        cycles.append({
            'label': lin["label_template"].format(n=offset + i + 1, rot=rot),
            'color': colors_lin[i % len(colors_lin)],
            'pts': pts,
            'type': 'linear',
            'waypoint_indices': wp,
        })
    return cycles
