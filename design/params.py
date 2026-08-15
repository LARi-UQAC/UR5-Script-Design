"""
design/params.py — Source unique de vérité pour tous les paramètres du protocole.

Toutes les constantes géométriques, robot, force et URScript sont définies ici.
- ur5_etalementv6 (design UI) importe directement ce module.
- ur5_sim/config.py importe les constantes partagées depuis ce module.
"""

from __future__ import annotations

from pathlib import Path

# --- Chemins ---
REPO_ROOT: Path = Path(__file__).resolve().parents[1]
SCRIPT_PATH: Path = REPO_ROOT / "etalement.script"
URP_PATH: Path = REPO_ROOT / "etalement.urp"

# =============================================================================
# SURFACE
# =============================================================================
SURFACE_W: float = 50.0   # mm — largeur de la plaque
SURFACE_H: float = 50.0   # mm — hauteur de la plaque
MARGIN: float = 4.0        # mm — marge depuis le bord

# =============================================================================
# HAUTEURS ROBOT
# =============================================================================
Z_CONTACT: float = 0.0    # mm — plan de la surface (z=0 dans le repère plaque)
Z_TRANSIT: float = 10.0   # mm — hauteur de remontée entre les cycles
Z_RETREAT_END: float = 30.0  # mm — remontée finale en Z (monde) après tous les
                             # cycles, pour dégager la plaque et laisser
                             # l'opérateur la retirer. Mouvement de retrait pur,
                             # hors force_mode, point d'arrêt du programme.

# =============================================================================
# CYCLES CIRCULAIRES (boustrophedon + épicycloïde)
# =============================================================================
N_CIRCULAR_CYCLES: int = 3
CIRC_N_PASSES: int = 4
CIRC_N_CIRCLES: int = 20
CIRC_R_CIRCLE: float = 5.0    # mm — rayon de chaque petit cercle
CIRC_Y_START: float = 5.0     # mm — Y du point de départ/arrivée
CIRC_SPEED: float = 36.0      # mm/s — vitesse robot
CIRC_DURATION: float = 11.0   # sec — durée cible par cycle

# =============================================================================
# CYCLES RECTILIGNES
# =============================================================================
N_LINEAR_CYCLES: int = 3
LIN_N_PASSES: int = 13
LIN_N_POINTS_PER_SEGMENT: int = 50
LIN_SPEED: float = 80.0       # mm/s — vitesse robot
LIN_DURATION_ODD: float = 7.5  # sec — durée cycles 4 et 6
LIN_DURATION_EVEN: float = 6.0 # sec — durée cycle 5

# =============================================================================
# TCP (Tool Center Point)
# =============================================================================
TCP_FT300_Z: float = 34.8    # mm — hauteur du capteur Robotiq FT-300
TCP_COUPLING_Z: float = 13.9  # mm — coupling Robotiq FT-300 -> 2F-85
# Longueur physique de la 2F-85 seulement : entre dans l'offset set_tcp (TCP_Z).
# La pince n'est JAMAIS actionnée par le script généré (support passif du doigt
# silicone). Aucun rq_*/activation/payload émis. Voir design/export.py.
TCP_GRIPPER_Z: float = 145.0  # mm — pince 2F-85
TCP_FINGER_Z: float = 72.0   # mm — doigt silicone hémisphérique
TCP_X: float = 0.0
TCP_Y: float = 0.0
TCP_Z: float = TCP_FT300_Z + TCP_COUPLING_Z + TCP_GRIPPER_Z + TCP_FINGER_Z  # ~265.7 mm

# =============================================================================
# COORDONNÉES ROBOT (à calibrer sur site)
# =============================================================================
ROBOT_X_ORIGIN: float = 0.200    # m
ROBOT_Y_ORIGIN: float = -0.300   # m
ROBOT_Z_SURFACE: float = 0.050   # m
ROBOT_RX: float = 3.14159        # rad — outil pointant vers le bas
ROBOT_RY: float = 0.0
ROBOT_RZ: float = 0.0
ROBOT_BASE_ROTATION_DEG: float = 225.0  # deg

# =============================================================================
# ANCRE CIBLE (pose de référence monde)
# =============================================================================
P_REF: list[float] = [-0.011, 0.6, 0.05, 3.14159, 0.0, 0.0]

# =============================================================================
# CONTRÔLE DE FORCE EN Z
# =============================================================================
FORCE_Z_TARGET: float = 6.0      # N
# force_mode limits : axe COMPLIANT (Z) = vitesse max ; axes NON COMPLIANTS
# (X, Y, rotations) = deviation max toleree par rapport a la trajectoire avant
# arret de protection. FORCE_LIMIT_XY trop faible (2 mm) declenchait
# "Force mode: Maximum position deviation exceeded" pendant l'etalement (le
# doigt silicone traine lateralement et ecarte le TCP du chemin commande).
FORCE_LIMIT_XY: float = 0.008    # m — deviation XY max en force_mode (non compliant)
FORCE_LIMIT_Z: float = 0.040     # m/s — vitesse Z max en force_mode (compliant)
FORCE_LIMIT_ROT: float = 0.35    # rad — deviation rotation max en force_mode (non compliant)
FORCE_CONTACT_DEPTH: float = 0.005  # m

# =============================================================================
# PARAMÈTRES URSCRIPT
# =============================================================================
URSCRIPT_ACCEL: float = 0.8       # m/s²
URSCRIPT_BLEND: float = 0.002     # m
URSCRIPT_TRANSIT_V: float = 0.3   # m/s
URSCRIPT_CONTACT_V: float = 0.05  # m/s
URSCRIPT_RECONTACT_V: float = 0.01 # m/s
URSCRIPT_N_WAYPOINTS_CIRCULAR: int = 80
URSCRIPT_MAX_TCP_SPEED: float = 0.250  # m/s — hard cap PolyScope
URSCRIPT_MAX_BYTES: int = 200_000      # octets — budget mémoire PolyScope

# =============================================================================
# SONDAGE DE SURFACE (probe Z runtime)
# =============================================================================
PROBE_APPROACH_MM: float = 30.0
PROBE_DESCENT_V: float = 0.004    # m/s
PROBE_ACCEL: float = 0.05         # m/s²
PROBE_FORCE_THR: float = 4.0      # N
PROBE_RETRY_MAX: int = 1
PROBE_TILT_MAX_RAD: float = 0.0873  # rad (~5 deg)
PROBE_POINTS_PLATE_MM: list[tuple[float, float]] = [
    (5.0, 5.0), (45.0, 5.0), (25.0, 45.0)
]
PROBE_FLOOR_PLATE_MM: float = -10.0  # mm
PROBE_MAX_TRAVEL: float = 0.15   # m — course max de descente du sondage Z (securite anti-collision)
# Mode de sondage expose dans l'interface de reglage. Seul 'z1' est utilisable :
# 'plane3' (sondage 3 points) est parque, fige en Z, et reste grise dans l'IHM.
PROBE_MODE: str = 'z1'

# =============================================================================
# DENSITE DES WAYPOINTS ET INTERFACE
# =============================================================================
# 'subsample' sous-echantillonne chaque cycle circulaire a
# URSCRIPT_N_WAYPOINTS_CIRCULAR points, ce que fait l'export headless ; 'all'
# emet tous les points du trace, ce que faisait l'export depuis l'interface.
# Deux chemins d'export produisaient jusqu'ici des densites differentes.
CIRCULAR_WAYPOINT_MODE: str = 'subsample'
# Points de discretisation du curseur de l'interface de trace, cycles 1 a 3.
# 0 = automatique, c'est-a-dire la densite naturelle du trace genere.
UI_DISCRETIZATION_POINTS: int = 0

# =============================================================================
# SÉCURITÉ APPROCHE INITIALE
# =============================================================================
SAFE_APPROACH_RADIUS_M: float = 0.8  # m
Q_SAFE_JOINTS_RAD: tuple[float, ...] = (
    0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0
)
