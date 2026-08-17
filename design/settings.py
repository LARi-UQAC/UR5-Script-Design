"""
design/settings.py - Reglages effectifs du protocole, lus a l'execution.

Trois niveaux coexistent : design/params.py porte les defauts codes en dur
(sous git, jamais ecrits par l'interface), ce module porte l'objet lu a
l'execution par l'exporteur, l'interface et le simulateur, et
etalement_settings.json porte les seules surcharges de l'operateur.

REGLE D'ECRITURE, valable pour tout le code appelant : ne JAMAIS faire
`from design.settings import force_z_target`. Toujours `s = get_settings()`
puis `s.force_z_target` au moment de l'usage. Un `from X import Y` copie la
valeur a l'import et rend le reglage sans effet, ce qui est precisement
l'obstacle que cette couche leve.

Metadonnees des champs : design/settings_spec.py.
Plan : docs/superpower/plans/plan_variables_UI.md, sections 2, 3 et 6.1.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import design.params as params
from design.settings_spec import SPECS, spec_by_name

SETTINGS_PATH: Path = params.REPO_ROOT / "etalement_settings.json"


def _default(const: str) -> Any:
    """Valeur codee en dur d'une constante de design/params.py."""
    return getattr(params, const)


@dataclass
class Settings:
    """
    Reglages effectifs. Chaque defaut est la constante de design/params.py de
    meme nom ; l'invariant est verifie par tests/test_settings.py.
    """

    # --- Force ---
    force_z_target: float = params.FORCE_Z_TARGET
    force_limit_xy: float = params.FORCE_LIMIT_XY
    force_limit_z: float = params.FORCE_LIMIT_Z
    force_limit_rot: float = params.FORCE_LIMIT_ROT
    force_contact_depth: float = params.FORCE_CONTACT_DEPTH

    # --- Sondage ---
    probe_mode: str = params.PROBE_MODE
    probe_force_thr: float = params.PROBE_FORCE_THR
    probe_descent_v: float = params.PROBE_DESCENT_V
    probe_accel: float = params.PROBE_ACCEL
    probe_max_travel: float = params.PROBE_MAX_TRAVEL
    probe_approach_mm: float = params.PROBE_APPROACH_MM
    probe_tilt_max_rad: float = params.PROBE_TILT_MAX_RAD
    probe_retry_max: int = params.PROBE_RETRY_MAX
    probe_points_plate_mm: list = field(
        default_factory=lambda: [list(p) for p in params.PROBE_POINTS_PLATE_MM])

    # --- Mouvement et URScript ---
    urscript_accel: float = params.URSCRIPT_ACCEL
    urscript_transit_v: float = params.URSCRIPT_TRANSIT_V
    urscript_contact_v: float = params.URSCRIPT_CONTACT_V
    urscript_recontact_v: float = params.URSCRIPT_RECONTACT_V
    urscript_blend: float = params.URSCRIPT_BLEND
    urscript_n_waypoints_circular: int = params.URSCRIPT_N_WAYPOINTS_CIRCULAR
    circular_waypoint_mode: str = params.CIRCULAR_WAYPOINT_MODE
    circ_speed: float = params.CIRC_SPEED
    lin_speed: float = params.LIN_SPEED
    urscript_max_tcp_speed: float = params.URSCRIPT_MAX_TCP_SPEED
    urscript_max_bytes: int = params.URSCRIPT_MAX_BYTES

    # --- Surface, hauteurs et forme du chemin ---
    surface_w: float = params.SURFACE_W
    surface_h: float = params.SURFACE_H
    margin: float = params.MARGIN
    z_transit: float = params.Z_TRANSIT
    z_retreat_end: float = params.Z_RETREAT_END
    circ_y_start: float = params.CIRC_Y_START
    circ_duration: float = params.CIRC_DURATION
    lin_duration_odd: float = params.LIN_DURATION_ODD
    lin_duration_even: float = params.LIN_DURATION_EVEN
    circ_r_circle: float = params.CIRC_R_CIRCLE
    n_circular_cycles: int = params.N_CIRCULAR_CYCLES
    circ_n_passes: int = params.CIRC_N_PASSES
    circ_n_circles: int = params.CIRC_N_CIRCLES
    ui_discretization_points: int = params.UI_DISCRETIZATION_POINTS

    # --- Calibration robot (onglet verrouille) ---
    robot_x_origin: float = params.ROBOT_X_ORIGIN
    robot_y_origin: float = params.ROBOT_Y_ORIGIN
    robot_z_surface: float = params.ROBOT_Z_SURFACE
    robot_rx: float = params.ROBOT_RX
    robot_ry: float = params.ROBOT_RY
    robot_rz: float = params.ROBOT_RZ
    robot_base_rotation_deg: float = params.ROBOT_BASE_ROTATION_DEG
    p_ref: list = field(default_factory=lambda: list(params.P_REF))
    tcp_ft300_z: float = params.TCP_FT300_Z
    tcp_coupling_z: float = params.TCP_COUPLING_Z
    tcp_gripper_z: float = params.TCP_GRIPPER_Z
    tcp_finger_z: float = params.TCP_FINGER_Z
    tcp_z: float = params.TCP_Z
    safe_approach_radius_m: float = params.SAFE_APPROACH_RADIUS_M
    q_safe_joints_rad: list = field(
        default_factory=lambda: list(params.Q_SAFE_JOINTS_RAD))

    # --- Etat d'interface, hors table de metadonnees ---
    # Deverrouillage explicite de l'onglet calibration. Jamais persiste : une
    # session repart toujours verrouillee.
    calibration_unlocked: bool = False
    source: str = "defauts"

    # -- Surcharges ---------------------------------------------------------

    def to_overrides(self) -> dict[str, Any]:
        """
        ----------------------------------------------------------------------
        Purpose:
            Liste les seuls champs qui different de leur defaut params.py.

        Outputs:
            overrides (dict[str, Any]): nom du champ -> valeur courante.
        ----------------------------------------------------------------------
        """
        out: dict[str, Any] = {}
        for spec in SPECS:
            current = getattr(self, spec.name)
            default = _default(spec.const)
            if isinstance(default, (list, tuple)):
                if [list(v) if isinstance(v, (list, tuple)) else v
                        for v in current] != [
                        list(v) if isinstance(v, (list, tuple)) else v
                        for v in default]:
                    out[spec.name] = current
            elif current != default:
                out[spec.name] = current
        return out

    def fingerprint(self) -> str:
        """Condense court des surcharges, pour rapprocher un .script d'un jeu."""
        blob = json.dumps(self.to_overrides(), sort_keys=True, default=list)
        return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:7]

    # -- Persistance --------------------------------------------------------

    @classmethod
    def from_file(cls, path: Path | str = SETTINGS_PATH) -> "Settings":
        """
        ----------------------------------------------------------------------
        Purpose:
            Construit des reglages a partir des defauts, puis applique les
            surcharges du JSON s'il existe, et valide le resultat (F1,
            docs/superpower/plans/erreur_hors_datalogger.md). Une cle
            inconnue est ignoree ; un fichier illisible OU dont au moins un
            champ ne passe pas validate() ramene aux DEFAUTS PURS, jamais un
            melange partiel : un fichier qui n'applique que la moitie de ses
            surcharges est pire que les deux extremes, car rien dans l'en-
            tete exporte ne dit laquelle a passe. Chaque champ fautif est
            imprime (WARN, nommant le champ, sa valeur et ses bornes), suivi
            d'une ligne resumant le refus.

            Deverrouillage transitoire de calibration pour la seule duree de
            cette validation : une surcharge de calibration ECRITE DANS LE
            FICHIER est deja l'action explicite que l'onglet verrouille
            demande normalement a l'IHM (un hand-edit ne peut pas venir d'un
            clic accidentel). L'objet retourne repart neanmoins verrouille
            (calibration_unlocked = False), conformement a la regle "une
            session repart toujours verrouillee" ; seules les bornes de
            chaque champ de calibration restent opposables.

        Inputs:
            path (Path | str): chemin du fichier de surcharges.

        Outputs:
            settings (Settings): reglages effectifs, toujours valides.
        ----------------------------------------------------------------------
        """
        s = cls()
        path = Path(path)
        if not path.is_file():
            return s
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            overrides = payload.get("overrides", {})
            if not isinstance(overrides, dict):
                raise TypeError("'overrides' doit etre un objet JSON")
        except (json.JSONDecodeError, OSError, AttributeError, TypeError):
            print(f"WARN: {path.name} illisible, retour aux defauts.")
            return s
        known = {spec.name for spec in SPECS}
        for name, value in overrides.items():
            if name in known:
                setattr(s, name, value)
        s.source = str(path)

        s.calibration_unlocked = True
        errors = s.validate()
        s.calibration_unlocked = False
        if errors:
            for err in errors:
                print(f"WARN: {path.name} : {err}")
            print(f"WARN: {path.name} refuse ({len(errors)} champ(s) hors "
                  f"norme) : retour aux defauts.")
            return cls()
        return s

    def save(self, path: Path | str = SETTINGS_PATH) -> None:
        """Ecrit les seules surcharges, avec l'empreinte, en JSON indente."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "_comment": "Surcharges des reglages du protocole d'etalement. "
                        "Les champs absents gardent le defaut de "
                        "design/params.py.",
            "fingerprint": self.fingerprint(),
            "overrides": self.to_overrides(),
        }
        path.write_text(
            json.dumps(payload, indent=2, sort_keys=True, default=list) + "\n",
            encoding="utf-8")

    def reset(self, group: str | None = None) -> None:
        """Ramene aux defauts params.py, un onglet ou tout."""
        for spec in SPECS:
            if group is None or spec.group == group:
                default = _default(spec.const)
                setattr(self, spec.name,
                        list(default) if isinstance(default, (list, tuple))
                        else default)
        if group is None:
            self.source = "defauts"

    # -- Validation ---------------------------------------------------------

    def validate(self) -> list[str]:
        """
        ----------------------------------------------------------------------
        Purpose:
            Controle dur. Un message par champ fautif, nommant le champ et ses
            bornes. Liste vide quand tout passe.

            Type et forme sont verifies AVANT tout calcul qui suppose la
            bonne forme (F1, docs/superpower/plans/erreur_hors_datalogger.md).
            Un champ de type liste (p_ref, q_safe_joints_rad,
            probe_points_plate_mm) recevant une valeur non iterable (null,
            scalaire) ou de la mauvaise longueur est refuse ici, avant le
            calcul de `changed` : `list(None)` ou `list(0.5)` leverait sinon
            un TypeError brut, exactement celui que to_overrides() leve plus
            loin dans la chaine (export, fingerprint, save) quand ce controle
            n'a pas encore eu lieu.

        Outputs:
            errors (list[str]): messages d'erreur destines a l'operateur.
        ----------------------------------------------------------------------
        """
        errors: list[str] = []
        for spec in SPECS:
            value = getattr(self, spec.name)
            default = _default(spec.const)
            is_sequence = isinstance(default, (list, tuple))

            if is_sequence:
                if not isinstance(value, (list, tuple)):
                    errors.append(
                        f"{spec.name} : attendu une liste de {len(default)} "
                        f"valeur(s) ({spec.label}), recu {value!r}.")
                    continue
                if len(value) != len(default):
                    errors.append(
                        f"{spec.name} : {len(value)} valeur(s) recue(s), "
                        f"{len(default)} attendue(s) ({spec.label}).")
                    continue

            changed = (list(value) != list(default) if is_sequence
                       else value != default)

            if not spec.editable and changed:
                errors.append(
                    f"{spec.name} : lecture seule ({spec.label}), valeur "
                    f"imposee {default}.")
                continue
            if spec.locked and changed and not self.calibration_unlocked:
                errors.append(
                    f"{spec.name} : onglet calibration verrouille, cocher "
                    f"le deverrouillage avant de modifier {spec.label}.")
                continue
            if spec.kind == "choice":
                if value not in spec.choices:
                    errors.append(
                        f"{spec.name} : valeur '{value}' hors des choix "
                        f"{list(spec.choices)}.")
                elif value in spec.disabled_choices:
                    errors.append(
                        f"{spec.name} : le choix '{value}' est documente mais "
                        f"indisponible. {spec.note}".strip())
                continue
            if spec.kind == "points":
                # Chaque element doit etre un couple (x, y) numerique. La
                # forme et la longueur globales sont deja assurees ci-dessus.
                for i, point in enumerate(value):
                    if not isinstance(point, (list, tuple)) or len(point) != 2:
                        errors.append(
                            f"{spec.name}[{i}] : attendu un couple (x, y), "
                            f"recu {point!r}.")
                        continue
                    for coord in point:
                        try:
                            float(coord)
                        except (TypeError, ValueError):
                            errors.append(
                                f"{spec.name}[{i}] : coordonnee {coord!r} "
                                f"n'est pas un nombre.")
                continue
            if spec.kind == "vector":
                for i, component in enumerate(value):
                    try:
                        float(component)
                    except (TypeError, ValueError):
                        errors.append(
                            f"{spec.name}[{i}] : {component!r} n'est pas un "
                            f"nombre.")
                continue
            if spec.lo is None or spec.hi is None:
                continue
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                errors.append(f"{spec.name} : '{value}' n'est pas un nombre.")
                continue
            if not (spec.lo <= numeric <= spec.hi):
                errors.append(
                    f"{spec.name} : {numeric:g} hors bornes "
                    f"[{spec.lo:g}, {spec.hi:g}] ({spec.label}, {spec.unit}).")
        return errors

    def clamped(self, name: str) -> float:
        """Valeur d'un champ apres le plafond TCP du controleur."""
        value = float(getattr(self, name))
        if spec_by_name(name).clamp_tcp:
            return min(value, self.urscript_max_tcp_speed)
        return value

    def clamps(self) -> list[str]:
        """Champs que le plafond PolyScope corrige, avec la valeur appliquee."""
        out: list[str] = []
        for spec in SPECS:
            if not spec.clamp_tcp:
                continue
            value = float(getattr(self, spec.name))
            if value > self.urscript_max_tcp_speed:
                out.append(
                    f"{spec.name} : {value:g} m/s ramene a "
                    f"{self.urscript_max_tcp_speed:g} m/s (plafond PolyScope).")
        return out

    def warnings(self) -> list[str]:
        """
        Controles croises, non bloquants. Ils visent les combinaisons
        physiquement douteuses que les bornes champ par champ laissent passer.
        """
        out: list[str] = []
        # Une descente de recontact lente sur une profondeur importante tient
        # le doigt en charge bien plus longtemps que le protocole ne prevoit.
        if self.urscript_recontact_v > 0:
            duree = self.force_contact_depth / self.urscript_recontact_v
            if duree > 2.0:
                out.append(
                    f"Descente de recontact longue : {duree:.1f} s pour "
                    f"{self.force_contact_depth * 1000:.1f} mm a "
                    f"{self.urscript_recontact_v:g} m/s.")
        if self.force_z_target > 10.0 and self.force_contact_depth < 0.003:
            out.append(
                "Force elevee sur faible profondeur de contact : la "
                "regulation peut depasser avant de trouver l'appui.")
        if self.force_limit_z > self.urscript_contact_v:
            out.append(
                "FORCE_LIMIT_Z (vitesse de compliance) depasse la vitesse de "
                "contact commandee.")
        return out


# --- Singleton rechargeable -------------------------------------------------

_ACTIVE: Settings | None = None


def get_settings() -> Settings:
    """Reglages actifs du processus, charges au premier appel."""
    global _ACTIVE
    if _ACTIVE is None:
        _ACTIVE = Settings.from_file()
    return _ACTIVE


def reload_settings(path: Path | str = SETTINGS_PATH) -> Settings:
    """Relit le fichier de surcharges et remplace les reglages actifs."""
    global _ACTIVE
    _ACTIVE = Settings.from_file(path)
    return _ACTIVE


def set_settings(settings: Settings) -> Settings:
    """Installe un objet de reglages deja construit (interface, tests)."""
    global _ACTIVE
    _ACTIVE = settings
    return _ACTIVE


# --- Bannière de démarrage ---------------------------------------------------

def startup_banner(settings: Settings | None = None) -> list[str]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Lignes a afficher au demarrage : d'ou viennent les reglages actifs et
        en quoi ils s'ecartent des defauts. Liste VIDE quand rien n'est
        surcharge, pour ne pas polluer un demarrage nominal. L'appel se pose
        dans design/app.py (phase 5) ; cette fonction ne fait que le calcul.

    Inputs:
        settings (Settings | None): reglages a rapporter ; get_settings()
            au premier appel si omis.

    Outputs:
        lines (list[str]): entete nommant la source et l'empreinte, une
            ligne par ecart saisi (nom_du_champ : defaut -> valeur unite),
            puis les plafonnements (clamps) et avertissements (warnings)
            eventuels, prefixes pour ne pas etre confondus avec un ecart
            saisi par l'operateur.
    --------------------------------------------------------------------------
    """
    s = settings if settings is not None else get_settings()
    overrides = s.to_overrides()
    if not overrides:
        return []
    lines: list[str] = [
        f"Reglages actifs : {s.source} (empreinte {s.fingerprint()})"
    ]
    for name in sorted(overrides):
        spec = spec_by_name(name)
        default = _default(spec.const)
        value = overrides[name]
        unit = f" {spec.unit}" if spec.unit else ""
        lines.append(f"  {name} : {default} -> {value}{unit}")
    for clamp in s.clamps():
        lines.append(f"  [plafond] {clamp}")
    for warning in s.warnings():
        lines.append(f"  [avertissement] {warning}")
    return lines
