"""Project-wide constants and paths.

Single source of truth for the geometric anchors, file locations, and tuning
parameters that the validation pipeline reads. Modules import from here rather
than embedding magic numbers locally.
"""

from __future__ import annotations

import math
from pathlib import Path

from design.params import (
    P_REF as _P_REF_LIST,
    TCP_Z as _TCP_Z_MM,
    FORCE_Z_TARGET as _FORCE_Z_TARGET,
    FORCE_CONTACT_DEPTH as _FORCE_CONTACT_DEPTH,
    URSCRIPT_MAX_TCP_SPEED as _MAX_TCP_SPEED,
    PROBE_TILT_MAX_RAD as _PROBE_TILT_MAX_RAD,
)

REPO_ROOT: Path = Path(__file__).resolve().parents[1]
SCRIPT_PATH: Path = REPO_ROOT / "etalement.script"
ROBOTIQ_MESH_DIR: Path = REPO_ROOT / "meshes" / "robotiq"
SUPPORT_TOOL_MESH_PATH: Path = REPO_ROOT / "meshes" / "Support doigt.stl"

# URScript poses : (x, y, z, rx, ry, rz) — synchronised with design.params.P_REF.
P_REF_RAW: tuple[float, ...] = tuple(_P_REF_LIST)
P_ANCHOR_OLD_RAW: tuple[float, ...] = P_REF_RAW
# Additional simulation-only frame rotation around Y (rad).
# Kept at 0 now that P_REF has identity orientation: the trajectory
# already lies in the world XY plane after the URScript transform chain.
SIM_TRAJ_ROT_Y_RAD: float = 0.0

# --- Tool offset (tool0 -> TCP) -----------------------------------------
# rtb's UR5 URDF exposes ``tool0`` as the flange frame, already including
# the canonical 82.3 mm Y offset from ``wrist_3_link``. The IK targets the
# ``tool0`` link directly, and the TCP (silicone finger tip) sits
# ``TCP_TOOL_Z_M`` further along tool0 Z. The on-robot ``set_tcp(...)``
# call wraps the same offset, so the simulator and the real controller
# stay in agreement.
TCP_TOOL_Z_M: float = _TCP_Z_MM / 1000.0           # m — synchronisé avec design.params.TCP_Z
END_LINK: str = "tool0"                         # IK + fkine target link

# Optional custom tool mounted between gripper fingers (local to gripper base).
# Tune XYZ/RPY if the STL needs fine positioning in the finger extrusion.
SUPPORT_TOOL_LOCAL_XYZ: tuple[float, float, float] = (0.0, 0.0, 0.213)
SUPPORT_TOOL_LOCAL_RPY: tuple[float, float, float] = (math.pi, 0.0, 0.0)

# Playback - time step between successive movel/movej targets in the buffer.
DT: float = 0.05
SIM_SPEED: float = 1.0

# Mesh decimation targets (triangles per sub-mesh).
TARGET_FACES_LINK: int = 50
TARGET_FACES_FT300: int = 200
TARGET_FACES_GRIPPER_BASE: int = 300
TARGET_FACES_FINGER: int = 80
TARGET_FACES_DEFAULT: int = 120

# --- Test surface (50x50 mm plate) + force_mode kinematic surrogate ---
# Source de verite geometrique : ur5_etalementv6.SURFACE_W/H/Z_CONTACT,
# plate_to_robot et ROBOT_*. Importes paressement par
# ur5_sim/visualization/surface.py pour eviter une dependance matplotlib
# au niveau de ce module de configuration.
SURFACE_THICKNESS_M: float = 0.005             # 5 mm - plaque assez epaisse pour rester visible dans Swift
SURFACE_COLOR_RGBA: tuple[float, float, float, float] = (
    # Bleu pastel opaque : la valeur alpha precedente (0.45) etait trop
    # transparente dans le rendu three.js de Swift et rendait la plaque
    # quasi invisible a cote du robot mat.
    0.30, 0.55, 0.90, 0.95,
)
SURFACE_CLEARANCE_M: float = 0.0               # marge transit au-dessus du plan
SURFACE_ENABLE_CLAMP: bool = True              # False -> uniquement visualisation
FORCE_Z_TARGET_N: float = _FORCE_Z_TARGET           # synchronisé avec design.params
CONTACT_SNAP_TOL_M: float = 1e-6               # seuil de consignation des ecarts pre-snap
# Profondeur cible de la descente de recontact emise par
# ur5_etalementv6._build_urscript_lines (pose_contact_deep = ROBOT_Z_SURFACE
# - FORCE_CONTACT_DEPTH = 5 mm sous le plan nominal). Le robot reel s'arrete
# au contact via force_mode ; le simulateur cinematique sans physique voit
# cette cible "profonde" comme une deviation. Le cli filtre les events
# SURFACE_DEVIATION dont la profondeur tombe a +/- TOL autour de cette cible
# pour eviter le spam de 6 events / run (un par cycle).
SURFACE_FORCE_TARGET_DEPTH_M: float = _FORCE_CONTACT_DEPTH    # synchronisé avec design.params
SURFACE_FORCE_TARGET_TOL_M: float = 1e-4       # m — fenetre +/- 100 um

# --- Limite TCP PolyScope ---
# Miroir de ur5_etalementv6.URSCRIPT_MAX_TCP_SPEED. Le simulateur lit les
# vitesses TCP declarees comme `global <NAME> = <value>` au preambule du
# .script et flag tout depassement (le controleur reel declenche un safety
# stop).
URSCRIPT_MAX_TCP_SPEED_MPS: float = _MAX_TCP_SPEED     # synchronisé avec design.params

# --- Sondage 3 points (L4 : ur5_sim simule la phase probe_surface_plane) ---
# Pendant l'execution reelle, ``probe_surface_plane()`` (URScript) descend en
# 3 points jusqu'a detecter un contact a 4 N, reconstruit le plan mesure et
# en derive ``MEAS_FRAME``. Le simulateur n'a pas de modele de force ; on
# fournit a la place un plan virtuel parametrable. Les parametres ci-dessous
# decrivent l'ecart attendu entre plaque reelle et plaque nominale ; le sim
# rejoue les 3 descentes par intersection geometrique avec ce plan, puis
# valide que la reconstruction Rodrigues + ``apply_correction`` ramene les
# waypoints exactement sur le plan virtuel.
# DESACTIVE - A REVOIR (rework futur). Le sondage 3 points est INCORRECT (fixe
# en Z, ne gere ni rotation ni hauteur de plaque inconnue). L'export URScript a
# ete bascule sur un sondage Z 1 point (probe_surface_z) ; le simulateur 3 points
# (ur5_sim/probe.py + _run_probe_simulation) et ses tests sont parques. Mettre
# False desactive proprement le rejeu 3 points dans --check / --visualize.
# Remettre True (et reactiver tests/test_probe_sim.py) lors du rework.
SIM_PROBE_ENABLE: bool = False
SIM_PROBE_PLATE_DZ_M: float = 0.0       # m — translation plaque reelle le long
                                        # de la normale nominale (positif = plus haut)
SIM_PROBE_PLATE_TILT_X_RAD: float = 0.0 # rad — basculement autour de l'axe X monde
SIM_PROBE_PLATE_TILT_Y_RAD: float = 0.0 # rad — basculement autour de l'axe Y monde
# Miroir de PROBE_TILT_MAX_RAD cote ur5_etalementv6. Si le tilt mesure
# reconstruit depasse ce seuil, le sim flag la condition d'echec (popup +
# halt cote robot reel) sans interrompre le run --check.
SIM_PROBE_TILT_MAX_RAD: float = _PROBE_TILT_MAX_RAD    # synchronisé avec design.params
# Tolerance sur le residu post-correction (distance signee waypoint
# corrige au plan virtuel). Si depasse, la reconstruction MEAS_FRAME ne
# ramene pas le waypoint sur la plaque reelle - bug ou parametre incompatible.
SIM_PROBE_RESIDUAL_TOL_M: float = 1e-5
