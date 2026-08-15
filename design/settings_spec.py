"""
design/settings_spec.py - Table des metadonnees des reglages du protocole.

Une entree FieldSpec par champ expose dans l'interface : libelle, unite,
bornes, groupe d'onglet, et les trois drapeaux qui decident de son
editabilite. L'IHM (design/ui_settings.py) engendre ses lignes a partir de
cette table plutot que de repeter du code par champ ; design/settings.py y lit
ses defauts et ses regles de validation.

Trois drapeaux, a ne pas confondre :
  - editable=False : jamais modifiable (limite du controleur, valeur calculee).
  - locked=True    : modifiable seulement apres deverrouillage explicite
                     (onglet calibration, cf. section 4.5 du plan).
  - enabled=False  : affiche mais grise (champs propres au sondage plane3,
                     qui reste indisponible ; cf. section 4.2 du plan).

Voir docs/superpower/plans/plan_variables_UI.md, sections 4 et 6.1.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FieldSpec:
    """Metadonnees d'un champ de reglage."""

    name: str                      # attribut du dataclass Settings
    const: str                     # constante correspondante dans design.params
    label: str                     # libelle affiche dans l'IHM
    unit: str                      # unite affichee, chaine vide si sans unite
    group: str                     # force | probe | motion | surface | calibration
    kind: str = "float"            # float | int | choice | points | vector
    lo: float | None = None        # borne basse dure, None si sans objet
    hi: float | None = None        # borne haute dure
    choices: tuple[str, ...] = ()  # valeurs admises quand kind == "choice"
    # Choix documentes mais indisponibles : ils restent visibles dans le
    # libelle pour dire ce qui existera, sans etre selectionnables. Un menu
    # qui propose une option cassee est pire que pas de menu (plan, section 8).
    disabled_choices: tuple[str, ...] = ()
    editable: bool = True
    locked: bool = False
    enabled: bool = True
    clamp_tcp: bool = False        # plafonne par URSCRIPT_MAX_TCP_SPEED
    note: str = ""


# --- Onglet 1 : Force -------------------------------------------------------
_FORCE: tuple[FieldSpec, ...] = (
    FieldSpec("force_z_target", "FORCE_Z_TARGET", "Force cible Z", "N",
              "force", lo=2.0, hi=20.0,
              note="Cible protocole 6.0 +/- 0.5 N. Sous 2 N la regulation "
                   "n'est pas stable avec le FT-300 (bruit 0.1 N, seuil "
                   "recommande 1 N monte sur robot)."),
    FieldSpec("force_limit_xy", "FORCE_LIMIT_XY", "Deviation max XY", "m",
              "force", lo=0.002, hi=0.020,
              note="Axe NON compliant : distance, pas une vitesse. Au-dela, "
                   "arret de protection 'Maximum position deviation exceeded'. "
                   "Ne pas revenir a 0.002, qui a deja fait fauter un essai."),
    FieldSpec("force_limit_z", "FORCE_LIMIT_Z", "Vitesse max Z", "m/s",
              "force", lo=0.005, hi=0.100,
              note="Axe COMPLIANT : c'est une VITESSE, contrairement a ses "
                   "voisines du meme vecteur limits."),
    FieldSpec("force_limit_rot", "FORCE_LIMIT_ROT", "Deviation max rotation",
              "rad", "force", lo=0.05, hi=0.60, note="0.35 rad = 20 deg."),
    FieldSpec("force_contact_depth", "FORCE_CONTACT_DEPTH",
              "Profondeur de contact", "m", "force", lo=0.001, hi=0.015,
              note="Profondeur visee sous le plan nominal a la descente de "
                   "recontact."),
)

# --- Onglet 2 : Sondage -----------------------------------------------------
_PROBE: tuple[FieldSpec, ...] = (
    FieldSpec("probe_mode", "PROBE_MODE", "Mode de sondage", "", "probe",
              kind="choice", choices=("z1", "plane3"),
              disabled_choices=("plane3",),
              note="plane3 indisponible : le sondage 3 points est parque, il "
                   "est fige en Z. Voir plan_optimisation_urscript.md "
                   "section 6."),
    FieldSpec("probe_force_thr", "PROBE_FORCE_THR", "Seuil de contact", "N",
              "probe", lo=1.5, hi=10.0,
              note="Norme des trois composantes de force."),
    FieldSpec("probe_descent_v", "PROBE_DESCENT_V", "Vitesse de descente",
              "m/s", "probe", lo=0.001, hi=0.020, clamp_tcp=True),
    FieldSpec("probe_accel", "PROBE_ACCEL", "Acceleration de sondage",
              "m/s^2", "probe", lo=0.01, hi=0.50),
    FieldSpec("probe_max_travel", "PROBE_MAX_TRAVEL", "Course max", "m",
              "probe", lo=0.02, hi=0.30,
              note="Securite anti-collision : echec du sondage au-dela."),
    FieldSpec("probe_approach_mm", "PROBE_APPROACH_MM", "Hauteur d'approche",
              "mm", "probe", lo=5.0, hi=100.0, enabled=False,
              note="Mode plane3 seulement."),
    FieldSpec("probe_tilt_max_rad", "PROBE_TILT_MAX_RAD", "Inclinaison max",
              "rad", "probe", lo=0.01, hi=0.30, enabled=False,
              note="Mode plane3 seulement."),
    FieldSpec("probe_retry_max", "PROBE_RETRY_MAX", "Nombre de reprises", "",
              "probe", kind="int", lo=0, hi=3, enabled=False,
              note="Mode plane3 seulement."),
    FieldSpec("probe_points_plate_mm", "PROBE_POINTS_PLATE_MM",
              "Points de sondage", "mm", "probe", kind="points",
              enabled=False, note="Mode plane3 seulement, trois points dans "
                                  "la plaque."),
)

# --- Onglet 3 : Mouvement et URScript ---------------------------------------
_MOTION: tuple[FieldSpec, ...] = (
    FieldSpec("urscript_accel", "URSCRIPT_ACCEL", "Acceleration", "m/s^2",
              "motion", lo=0.1, hi=2.0),
    FieldSpec("urscript_transit_v", "URSCRIPT_TRANSIT_V", "Vitesse de transit",
              "m/s", "motion", lo=0.02, hi=2.0, clamp_tcp=True,
              note="Plafonnee a URSCRIPT_MAX_TCP_SPEED par le controleur. Le "
                   "defaut 0.3 est deja au-dessus du plafond, donc clampe."),
    FieldSpec("urscript_contact_v", "URSCRIPT_CONTACT_V",
              "Vitesse au contact", "m/s", "motion", lo=0.005, hi=0.25,
              clamp_tcp=True),
    FieldSpec("urscript_recontact_v", "URSCRIPT_RECONTACT_V",
              "Vitesse de recontact", "m/s", "motion", lo=0.002, hi=0.05,
              clamp_tcp=True, note="Descente au contact."),
    FieldSpec("urscript_blend", "URSCRIPT_BLEND", "Blend hors contact", "m",
              "motion", lo=0.0, hi=0.010),
    FieldSpec("urscript_n_waypoints_circular", "URSCRIPT_N_WAYPOINTS_CIRCULAR",
              "Waypoints par cycle circulaire", "", "motion", kind="int",
              lo=20, hi=2000,
              note="Utilise quand la densite est en mode sous-echantillonne."),
    FieldSpec("circular_waypoint_mode", "CIRCULAR_WAYPOINT_MODE",
              "Densite des waypoints circulaires", "", "motion",
              kind="choice", choices=("subsample", "all"),
              note="subsample : sous-echantillonner au nombre ci-dessus, ce "
                   "que fait l'export headless. all : tous les points du "
                   "trace, ce que faisait l'export depuis l'IHM."),
    FieldSpec("circ_speed", "CIRC_SPEED", "Vitesse cycles circulaires",
              "mm/s", "motion", lo=5.0, hi=250.0),
    FieldSpec("lin_speed", "LIN_SPEED", "Vitesse cycles rectilignes", "mm/s",
              "motion", lo=5.0, hi=250.0),
    FieldSpec("urscript_max_tcp_speed", "URSCRIPT_MAX_TCP_SPEED",
              "Vitesse TCP max", "m/s", "motion", editable=False,
              note="Limite PolyScope, pas une preference."),
    FieldSpec("urscript_max_bytes", "URSCRIPT_MAX_BYTES", "Budget memoire",
              "octets", "motion", kind="int", editable=False,
              note="Budget memoire du controleur."),
)

# --- Onglet 4 : Surface, hauteurs et forme du chemin ------------------------
_SURFACE: tuple[FieldSpec, ...] = (
    FieldSpec("surface_w", "SURFACE_W", "Largeur de la plaque", "mm",
              "surface", lo=10.0, hi=200.0),
    FieldSpec("surface_h", "SURFACE_H", "Hauteur de la plaque", "mm",
              "surface", lo=10.0, hi=200.0),
    FieldSpec("margin", "MARGIN", "Marge depuis le bord", "mm", "surface",
              lo=0.0, hi=20.0),
    FieldSpec("z_transit", "Z_TRANSIT", "Hauteur de degagement", "mm",
              "surface", lo=2.0, hi=50.0),
    FieldSpec("z_retreat_end", "Z_RETREAT_END", "Retrait final", "mm",
              "surface", lo=5.0, hi=100.0),
    FieldSpec("circ_y_start", "CIRC_Y_START", "Y de depart circulaire", "mm",
              "surface", lo=0.0, hi=25.0),
    FieldSpec("circ_duration", "CIRC_DURATION", "Duree cycle circulaire", "s",
              "surface", lo=1.0, hi=60.0, note="Cible du protocole."),
    FieldSpec("lin_duration_odd", "LIN_DURATION_ODD",
              "Duree cycles 4 et 6", "s", "surface", lo=1.0, hi=60.0),
    FieldSpec("lin_duration_even", "LIN_DURATION_EVEN", "Duree cycle 5", "s",
              "surface", lo=1.0, hi=60.0),
    # Les cinq champs ci-dessous sont les curseurs deja presents dans
    # design/app.py. Ils deviennent des vues sur Settings a la phase 5, pour
    # qu'il n'existe qu'une seule source de verite (plan, section 6.3).
    FieldSpec("circ_r_circle", "CIRC_R_CIRCLE", "Rayon des petits cercles",
              "mm", "surface", lo=0.0, hi=10.0),
    FieldSpec("n_circular_cycles", "N_CIRCULAR_CYCLES",
              "Nombre de cycles circulaires", "", "surface", kind="int",
              lo=0, hi=10),
    FieldSpec("circ_n_passes", "CIRC_N_PASSES", "Passes par cycle circulaire",
              "", "surface", kind="int", lo=0, hi=10),
    FieldSpec("circ_n_circles", "CIRC_N_CIRCLES", "Cercles par passe", "",
              "surface", kind="int", lo=1, hi=60),
    FieldSpec("ui_discretization_points", "UI_DISCRETIZATION_POINTS",
              "Points de discretisation", "", "surface", kind="int",
              lo=0, hi=2000,
              note="Curseur de l'interface de trace, cycles 1 a 3. "
                   "0 = automatique, densite naturelle du trace."),
)

# --- Onglet 5 : Calibration robot, verrouille -------------------------------
_CALIBRATION: tuple[FieldSpec, ...] = (
    FieldSpec("robot_x_origin", "ROBOT_X_ORIGIN", "Origine X", "m",
              "calibration", lo=-2.0, hi=2.0, locked=True),
    FieldSpec("robot_y_origin", "ROBOT_Y_ORIGIN", "Origine Y", "m",
              "calibration", lo=-2.0, hi=2.0, locked=True),
    FieldSpec("robot_z_surface", "ROBOT_Z_SURFACE", "Z de la surface", "m",
              "calibration", lo=-2.0, hi=2.0, locked=True),
    FieldSpec("robot_rx", "ROBOT_RX", "Rotation RX", "rad", "calibration",
              lo=-6.2832, hi=6.2832, locked=True),
    FieldSpec("robot_ry", "ROBOT_RY", "Rotation RY", "rad", "calibration",
              lo=-6.2832, hi=6.2832, locked=True),
    FieldSpec("robot_rz", "ROBOT_RZ", "Rotation RZ", "rad", "calibration",
              lo=-6.2832, hi=6.2832, locked=True),
    FieldSpec("robot_base_rotation_deg", "ROBOT_BASE_ROTATION_DEG",
              "Rotation de la base", "deg", "calibration", lo=-360.0,
              hi=360.0, locked=True),
    FieldSpec("p_ref", "P_REF", "Ancre monde P_REF", "m, rad", "calibration",
              kind="vector", locked=True),
    FieldSpec("tcp_ft300_z", "TCP_FT300_Z", "Longueur FT-300", "mm",
              "calibration", lo=0.0, hi=500.0, locked=True),
    FieldSpec("tcp_coupling_z", "TCP_COUPLING_Z", "Longueur coupling", "mm",
              "calibration", lo=0.0, hi=500.0, locked=True),
    FieldSpec("tcp_gripper_z", "TCP_GRIPPER_Z", "Longueur pince 2F-85", "mm",
              "calibration", lo=0.0, hi=500.0, locked=True),
    FieldSpec("tcp_finger_z", "TCP_FINGER_Z", "Longueur doigt silicone", "mm",
              "calibration", lo=0.0, hi=500.0, locked=True),
    FieldSpec("tcp_z", "TCP_Z", "Offset TCP total", "mm", "calibration",
              editable=False,
              note="Somme des quatre longueurs ci-dessus. Jamais saisissable "
                   "independamment de ses composantes."),
    FieldSpec("safe_approach_radius_m", "SAFE_APPROACH_RADIUS_M",
              "Rayon d'approche sure", "m", "calibration", lo=0.1, hi=2.0,
              locked=True),
    FieldSpec("q_safe_joints_rad", "Q_SAFE_JOINTS_RAD",
              "Pose articulaire sure", "rad", "calibration", kind="vector",
              locked=True),
)

SPECS: tuple[FieldSpec, ...] = (
    _FORCE + _PROBE + _MOTION + _SURFACE + _CALIBRATION
)

GROUP_LABELS: dict[str, str] = {
    "force": "Force",
    "probe": "Sondage",
    "motion": "Mouvement",
    "surface": "Surface",
    "calibration": "Calibration",
}

GROUP_ORDER: tuple[str, ...] = ("force", "probe", "motion", "surface",
                                "calibration")

_BY_NAME: dict[str, FieldSpec] = {spec.name: spec for spec in SPECS}


def spec_by_name(name: str) -> FieldSpec:
    """
    --------------------------------------------------------------------------
    Purpose:
        Retrouve la metadonnee d'un champ par son nom d'attribut.

    Inputs:
        name (str): nom de l'attribut du dataclass Settings.

    Outputs:
        spec (FieldSpec): la metadonnee correspondante.
    --------------------------------------------------------------------------
    """
    return _BY_NAME[name]


def specs_for_group(group: str) -> tuple[FieldSpec, ...]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Liste les champs d'un onglet, dans l'ordre de declaration.

    Inputs:
        group (str): force | probe | motion | surface | calibration.

    Outputs:
        specs (tuple[FieldSpec, ...]): les champs de cet onglet.
    --------------------------------------------------------------------------
    """
    return tuple(spec for spec in SPECS if spec.group == group)
