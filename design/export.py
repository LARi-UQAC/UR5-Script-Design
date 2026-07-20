"""
design/export.py — Génération des fichiers URScript (.script) et PolyScope (.urp).

Le programme généré n'actionne JAMAIS la pince Robotiq 2F-85 : c'est un support
passif pour le doigt silicone, seule sa longueur (TCP_GRIPPER_Z) entre dans
set_tcp(). Aucun rq_*, set_payload, set_tool_voltage ni RS485 outil n'est émis.
Une erreur PolyScope « wait for activation completed » au chargement provient du
URCap Robotiq Grippers de l'installation du robot (auto-activation BeforeStart),
pas de ce fichier — la corriger côté robot (voir le bloc d'en-tête émis dans le
script).

Fonctions exportées :
  - generate_urscript()       : écrit etalement.script
  - generate_urp()            : écrit etalement.urp
  - _build_urscript_lines()   : construit la liste des lignes URScript
  - _validate_script_memory() : vérifie le budget mémoire PolyScope
  - _clamp_tcp_speed()        : plafonne à URSCRIPT_MAX_TCP_SPEED
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from design.geometry import (
    _abs_pose,
    _fmt_pose,
    _fmt_raw_pose,
    mm_to_m,
    plate_to_robot,
)
from design.params import (
    FORCE_CONTACT_DEPTH,
    FORCE_LIMIT_ROT,
    FORCE_LIMIT_XY,
    FORCE_LIMIT_Z,
    FORCE_Z_TARGET,
    PROBE_ACCEL,
    PROBE_APPROACH_MM,
    PROBE_DESCENT_V,
    PROBE_FLOOR_PLATE_MM,
    PROBE_FORCE_THR,
    PROBE_MAX_TRAVEL,
    PROBE_POINTS_PLATE_MM,
    PROBE_RETRY_MAX,
    PROBE_TILT_MAX_RAD,
    P_REF,
    ROBOT_RX, ROBOT_RY, ROBOT_RZ,
    ROBOT_X_ORIGIN, ROBOT_Y_ORIGIN, ROBOT_Z_SURFACE,
    SCRIPT_PATH,
    TCP_X, TCP_Y, TCP_Z,
    URP_PATH,
    URSCRIPT_ACCEL,
    URSCRIPT_BLEND,
    URSCRIPT_CONTACT_V,
    URSCRIPT_MAX_BYTES,
    URSCRIPT_MAX_TCP_SPEED,
    URSCRIPT_N_WAYPOINTS_CIRCULAR,
    URSCRIPT_RECONTACT_V,
    URSCRIPT_TRANSIT_V,
    Z_TRANSIT,
    Z_RETREAT_END,
    CIRC_SPEED, LIN_SPEED,
)
from design.trajectory import get_waypoint_indices


def _clamp_tcp_speed(name: str, v_mps: float) -> float:
    """
    Plafonne une vitesse TCP à URSCRIPT_MAX_TCP_SPEED (limite PolyScope).
    Imprime un avertissement quand le clamp s'active.
    """
    if v_mps > URSCRIPT_MAX_TCP_SPEED:
        print(f"WARN: {name} = {v_mps:.3f} m/s > limite PolyScope "
              f"{URSCRIPT_MAX_TCP_SPEED:.3f} m/s, clamp.")
        return URSCRIPT_MAX_TCP_SPEED
    return v_mps


def _validate_script_memory(filename: Path, label: str) -> bool:
    """
    Vérifie que le fichier généré reste dans le budget mémoire PolyScope.
    Retourne False si dépassé, True sinon.
    """
    size_bytes = filename.stat().st_size
    pct = 100.0 * size_bytes / URSCRIPT_MAX_BYTES
    print(f"Mémoire {label}: {size_bytes} octets / {URSCRIPT_MAX_BYTES} "
          f"({pct:.1f}% du budget PolyScope)")
    if size_bytes > URSCRIPT_MAX_BYTES:
        print(f"ECHEC EXPORT {label}: budget mémoire URSCRIPT_MAX_BYTES = "
              f"{URSCRIPT_MAX_BYTES} octets dépassé de "
              f"{size_bytes - URSCRIPT_MAX_BYTES} octets. "
              f"Réduire URSCRIPT_N_WAYPOINTS_CIRCULAR / LIN_N_POINTS_PER_SEGMENT.")
        return False
    return True


def _build_urscript_lines(cycles: list[dict]) -> list[str]:
    """
    Construit la liste des lignes URScript (partagée entre generate_urscript
    et generate_urp). Inclut le sondage de surface 3 points, l'ajustement de
    plan, et la correction par waypoint via apply_correction().

    Émet un bloc d'en-tête « PINCE 2F-85 : AUCUN ACTIONNEMENT » : aucune commande
    de pince n'est générée ici (la 2F-85 est un support passif ; voir docstring
    du module). Ce bloc apparaît donc dans etalement.script ET dans le <script>
    du .urp.
    """
    z_transit_m = ROBOT_Z_SURFACE + mm_to_m(Z_TRANSIT)
    speed_var_map = {'circular': 'V_CIRC', 'linear': 'V_RECT'}

    # Le plafond URSCRIPT_MAX_TCP_SPEED (0.25 m/s) est applique par PolyScope
    # sur le vrai robot ; on ne l'emet plus dans le .script. La vitesse de
    # transit (post-clamp) est inlinee directement sur chaque movel de
    # transit au lieu d'etre exposee comme global URSCRIPT_TRANSIT_V.
    transit_v_mps = _clamp_tcp_speed("URSCRIPT_TRANSIT_V", URSCRIPT_TRANSIT_V)

    probe_pts_robot = [
        plate_to_robot(x_mm, y_mm) for (x_mm, y_mm) in PROBE_POINTS_PLATE_MM
    ]
    probe_nominal_contact = [
        _abs_pose([px_r, py_r, ROBOT_Z_SURFACE, ROBOT_RX, ROBOT_RY, ROBOT_RZ])
        for (px_r, py_r) in probe_pts_robot
    ]
    # nominal_frame_pose : ancre du repere nominal (1er point de contact nominal).
    # Conserve : NOMINAL_FRAME en est derive et les cycles l'utilisent toujours.
    nominal_frame_pose = probe_nominal_contact[0]

    # -------------------------------------------------------------------------
    # SONDAGE 3 POINTS DESACTIVE (a revoir / rework futur).
    # Les constructions ci-dessous (_nhat, probe_blocks) n'alimentaient que le
    # probe plan 3 points, lui-meme desactive plus bas. On les commente pour ne
    # plus rien emettre s'y rapportant. Voir le bloc commente des defs
    # probe_one/probe_surface_plane pour le detail de l'issue.
    # _p0 = _abs_pose([ROBOT_X_ORIGIN, ROBOT_Y_ORIGIN, ROBOT_Z_SURFACE,
    #                  ROBOT_RX, ROBOT_RY, ROBOT_RZ])
    # _p1 = _abs_pose([ROBOT_X_ORIGIN, ROBOT_Y_ORIGIN, ROBOT_Z_SURFACE + 0.001,
    #                  ROBOT_RX, ROBOT_RY, ROBOT_RZ])
    # _n = np.array(_p1[:3]) - np.array(_p0[:3])
    # _nhat = _n / np.linalg.norm(_n)
    #
    # probe_blocks = []
    # for (px_r, py_r) in probe_pts_robot:
    #     z_app = ROBOT_Z_SURFACE + mm_to_m(PROBE_APPROACH_MM)
    #     z_flo = ROBOT_Z_SURFACE + mm_to_m(PROBE_FLOOR_PLATE_MM)
    #     pose_app = _fmt_pose([px_r, py_r, z_app, ROBOT_RX, ROBOT_RY, ROBOT_RZ])
    #     pose_floor = _fmt_pose([px_r, py_r, z_flo, ROBOT_RX, ROBOT_RY, ROBOT_RZ])
    #     probe_blocks.append((pose_app, pose_floor))
    # -------------------------------------------------------------------------

    p_ref_str = 'p[' + ', '.join(f'{v}' for v in P_REF) + ']'
    nominal_frame_str = _fmt_raw_pose(nominal_frame_pose)
    lines = [
        '# UR5 - Protocole etalement cosmetique',
        '# Genere automatiquement par ur5_etalement.py',
        '# IMPORTANT : calibrer ROBOT_X_ORIGIN, ROBOT_Y_ORIGIN, ROBOT_Z_SURFACE avant execution',
        '#',
        '# === PINCE 2F-85 : AUCUN ACTIONNEMENT ===',
        '# Ce programme n a pas de commande de pince. La 2F-85 est un support',
        '# passif pour le doigt silicone ; seule sa longueur (TCP_GRIPPER_Z=145 mm)',
        '# entre dans set_tcp(). Aucun rq_activate / rq_* / RS485 outil / set_payload.',
        '# Erreur PolyScope "wait for activation completed" au chargement = URCap',
        '# Robotiq Grippers de l INSTALLATION du robot (auto-activation BeforeStart),',
        '# pas ce fichier. Corriger sur le robot : Installation > URCaps/Gripper >',
        '# desactiver l auto-activation, ou retirer le URCap, ou charger sur une',
        '# installation sans pince.',
        '#',
        '',
        '# --- Ancre de la zone d\'etalement ---',
        f'global P_REF = {p_ref_str}',
        '',
        '# --- Vitesses (m/s) par phase ---',
        '# Note : URSCRIPT_MAX_TCP_SPEED (limite PolyScope, 0.25 m/s) n\'est pas',
        '# emis ici ; il est applique par le controleur. La vitesse de transit',
        '# est inlinee directement sur les movel de transit.',
        f'global URSCRIPT_CONTACT_V = {_clamp_tcp_speed("URSCRIPT_CONTACT_V", URSCRIPT_CONTACT_V):.4f}  #sym:URSCRIPT_CONTACT_V',
        f'global URSCRIPT_RECONTACT_V = {_clamp_tcp_speed("URSCRIPT_RECONTACT_V", URSCRIPT_RECONTACT_V):.4f}  #sym:URSCRIPT_RECONTACT_V',
        f'global V_CIRC = {_clamp_tcp_speed("V_CIRC", mm_to_m(CIRC_SPEED)):.4f}  #sym:CIRC_SPEED',
        f'global V_RECT = {_clamp_tcp_speed("V_RECT", mm_to_m(LIN_SPEED)):.4f}  #sym:LIN_SPEED',
        '',
        '# --- Accelerations (m/s^2) par phase ---',
        f'global URSCRIPT_ACCEL = {URSCRIPT_ACCEL}  #sym:URSCRIPT_ACCEL',
        'global A_INIT = 1.2  #sym:A_INIT',
        '',
        '# --- Facteurs multiplicateurs globaux ---',
        'global SPEED_FACTOR = 1.0',
        'global ACCEL_FACTOR = 1.0',
        f'global URSCRIPT_BLEND = {URSCRIPT_BLEND}  #sym:URSCRIPT_BLEND',
        '',
        '# --- Sondage de surface : 1 point en Z (sondage 3 points DESACTIVE, voir bloc commente) ---',
        f'global PROBE_FORCE_THR    = {PROBE_FORCE_THR}',
        f'global PROBE_DESCENT_V    = {_clamp_tcp_speed("PROBE_DESCENT_V", PROBE_DESCENT_V):.4f}',
        f'global PROBE_ACCEL        = {PROBE_ACCEL}',
        f'global PROBE_MAX_TRAVEL   = {PROBE_MAX_TRAVEL}',
        # PROBE_TILT_MAX_RAD / NHAT_* / NOMINAL_P{i} : globals du sondage 3 points
        # (DESACTIVE). Non emis. NOMINAL_FRAME et MEAS_FRAME restent requis par les
        # cycles (apply_correction + force_mode) ; le sondage Z 1 point ecrit la
        # composante Z de MEAS_FRAME a l'execution.
        # f'global PROBE_TILT_MAX_RAD = {PROBE_TILT_MAX_RAD}',
        # f'global NHAT_X = {_nhat[0]:.9f}',
        # f'global NHAT_Y = {_nhat[1]:.9f}',
        # f'global NHAT_Z = {_nhat[2]:.9f}',
        f'global NOMINAL_FRAME = {nominal_frame_str}',
        f'global MEAS_FRAME = {nominal_frame_str}',
    ]
    # NOMINAL_P{i} : points nominaux du sondage 3 points (DESACTIVE). Non emis.
    # for i, pose in enumerate(probe_nominal_contact, start=1):
    #     lines.append(f'global NOMINAL_P{i}_X = {pose[0]:.6f}')
    #     lines.append(f'global NOMINAL_P{i}_Y = {pose[1]:.6f}')
    #     lines.append(f'global NOMINAL_P{i}_Z = {pose[2]:.6f}')
    lines += [
        'global contact_found = False',
        'global contact_pose = p[0,0,0,0,0,0]',
        '',
        # =========================================================================
        # SONDAGE 3 POINTS (plan + tilt) : DESACTIVE - A REVOIR (rework futur).
        #
        # ISSUE (demande Pr. Otis) :
        #   - Le sondage doit utiliser le capteur d'effort pour TOUCHER la surface
        #     et ajuster le Z du robot d'apres la mesure du capteur.
        #   - Le sondage 3 points actuel est FIXE en Z (poses "floor" a un Z code
        #     en dur) : il n'autorise ni rotation de la plaque d'essai ni hauteur
        #     differente.
        #   - Le Z exact de la plaque n'est PAS connu : il depend de la
        #     manipulation de l'operateur (jog du robot, pose de la plaque).
        #   - A retravailler plus tard (sondage multi-points force-adapte, gestion
        #     de l'inclinaison).
        #
        # Rappel CB3 (fix deja en place, reutilise par le sondage Z 1 point) :
        #   stopl/movel sont interdits dans un thread secondaire sur PolyScope 3.x
        #   ("Error position ... stopl(2.0)"). La descente surveillee se fait donc
        #   dans le thread principal : speedl (non bloquant) + lecture de force +
        #   stopl. Pas de slice get_tcp_force()[0:3] (absent du parser CB3).
        #
        # Remplacement actuel : probe_surface_z() ci-dessous (1 point, en -Z base),
        # qui ecrit la composante Z de MEAS_FRAME. Les cycles 1 a 6 sont inchanges
        # (apply_correction decale les waypoints du meme dz ; force_mode regule 6 N).
        # =========================================================================
        '# --- Sondage Z (1 point) : trouve le plan de la plaque par contact force ---',
        '# Depart = pose courante (operateur jogge le doigt juste au-dessus du',
        '# plateau). Descente en -Z (base) jusqu au contact ; le Z touche devient',
        '# la reference de TOUS les cycles 1 a 6 via MEAS_FRAME.',
        'def probe_surface_z():',
        '  textmsg(">>> Sondage Z : 1 point (recherche du plateau)")',
        '  start_pose = get_actual_tcp_pose()',
        '  zero_ftsensor()',
        '  sleep(0.2)',
        '  # CB3 (PolyScope 3.x) : descente surveillee dans le thread principal',
        '  # (speedl non bloquant + lecture force + stopl). Pas de thread (stopl/',
        '  # movel interdits dans un thread). get_tcp_force() lu en entier, norme',
        '  # calculee a la main (pas de slice [0:3], absent du parser CB3).',
        '  f_norm = 0.0',
        '  travel = 0.0',
        '  while f_norm <= PROBE_FORCE_THR and travel < PROBE_MAX_TRAVEL:',
        '    speedl([0, 0, -PROBE_DESCENT_V, 0, 0, 0], PROBE_ACCEL, 0.05)',
        '    F_tcp = get_tcp_force()',
        '    f_norm = sqrt(F_tcp[0]*F_tcp[0] + F_tcp[1]*F_tcp[1] + F_tcp[2]*F_tcp[2])',
        '    cur = get_actual_tcp_pose()',
        '    travel = start_pose[2] - cur[2]',
        '  end',
        '  stopl(2.0)',
        '  if f_norm <= PROBE_FORCE_THR:',
        '    popup("Sondage Z echoue - aucun contact", "Rapprocher le doigt du plateau puis relancer", error=True)',
        '    halt',
        '  end',
        '  touch = get_actual_tcp_pose()',
        '  # Reference Z des cycles : repere nominal translate en Z jusqu au plan',
        '  # touche. Orientation/XY restent nominaux (1 point ne mesure pas',
        '  # l inclinaison -> rework futur). apply_correction decale chaque',
        '  # waypoint du meme dz ; force_mode(MEAS_FRAME) regule les 6 N en Z.',
        '  MEAS_FRAME = p[NOMINAL_FRAME[0], NOMINAL_FRAME[1], touch[2], NOMINAL_FRAME[3], NOMINAL_FRAME[4], NOMINAL_FRAME[5]]',
        '  textmsg("Plateau detecte, Z (m) = ", touch[2])',
        'end',
        '',
    ]
    # =========================================================================
    # SONDAGE 3 POINTS : code URScript preserve pour rework futur, NON emis.
    # Chaine litterale inerte (no-op). Les poses etaient injectees par f-string
    # (placeholders <...> ci-dessous). Voir le header de probe_surface_z pour
    # l'issue et la raison de la desactivation.
    # =========================================================================
    r'''
def probe_one(approach_pose, floor_pose, label):
  attempt = 0
  while attempt <= PROBE_RETRY_MAX:
    contact_found = False
    movel(approach_pose, a=A_INIT*ACCEL_FACTOR, v=URSCRIPT_TRANSIT_V*SPEED_FACTOR)
    sleep(0.4)
    zero_ftsensor()
    sleep(0.2)
    # descente surveillee thread principal : speedl + force + stopl (CB3-safe)
    ddx = floor_pose[0] - approach_pose[0]
    ddy = floor_pose[1] - approach_pose[1]
    ddz = floor_pose[2] - approach_pose[2]
    dnrm = sqrt(ddx*ddx + ddy*ddy + ddz*ddz)
    ux = ddx / dnrm
    uy = ddy / dnrm
    uz = ddz / dnrm
    f_norm = 0.0
    proj = 0.0
    while f_norm <= PROBE_FORCE_THR and proj < dnrm:
      speedl([ux*PROBE_DESCENT_V, uy*PROBE_DESCENT_V, uz*PROBE_DESCENT_V, 0, 0, 0], PROBE_ACCEL, 0.05)
      F_tcp = get_tcp_force()
      f_norm = sqrt(F_tcp[0]*F_tcp[0] + F_tcp[1]*F_tcp[1] + F_tcp[2]*F_tcp[2])
      cur = get_actual_tcp_pose()
      proj = (cur[0]-approach_pose[0])*ux + (cur[1]-approach_pose[1])*uy + (cur[2]-approach_pose[2])*uz
    end
    stopl(2.0)
    if f_norm > PROBE_FORCE_THR:
      contact_pose = get_actual_tcp_pose()
      contact_found = True
    end
    if contact_found:
      textmsg(label, " contact OK")
      return contact_pose
    end
    textmsg(label, " contact NOK, retry attempt=", attempt+1)
    attempt = attempt + 1
  end
  popup("Probe failed - aucun contact detecte", "Verifier echantillon/calibration", error=True)
  halt
end

def probe_surface_plane():
  textmsg(">>> Sondage de la surface : 3 points")
  cp1 = probe_one(<approach_P1>, <floor_P1>, "P1")
  cp2 = probe_one(<approach_P2>, <floor_P2>, "P2")
  cp3 = probe_one(<approach_P3>, <floor_P3>, "P3")
  v12x = cp2[0] - cp1[0]
  v12y = cp2[1] - cp1[1]
  v12z = cp2[2] - cp1[2]
  v13x = cp3[0] - cp1[0]
  v13y = cp3[1] - cp1[1]
  v13z = cp3[2] - cp1[2]
  nx = v12y*v13z - v12z*v13y
  ny = v12z*v13x - v12x*v13z
  nz = v12x*v13y - v12y*v13x
  nrm = sqrt(nx*nx + ny*ny + nz*nz)
  nx = nx / nrm
  ny = ny / nrm
  nz = nz / nrm
  dot_nom = NHAT_X*nx + NHAT_Y*ny + NHAT_Z*nz
  if dot_nom < 0:
    nx = -nx
    ny = -ny
    nz = -nz
    dot_nom = -dot_nom
  end
  ax = NHAT_Y*nz - NHAT_Z*ny
  ay = NHAT_Z*nx - NHAT_X*nz
  az = NHAT_X*ny - NHAT_Y*nx
  anrm = sqrt(ax*ax + ay*ay + az*az)
  ang = atan2(anrm, dot_nom)
  if ang > PROBE_TILT_MAX_RAD:
    textmsg("Tilt mesure (rad) = ", ang)
    popup("Plan mesure trop incline", "ang > PROBE_TILT_MAX_RAD - verifier installation/calibration", error=True)
    halt
  end
  if anrm > 0.000000001:
    ax = ax / anrm
    ay = ay / anrm
    az = az / anrm
  else:
    ax = 0.0
    ay = 0.0
    az = 0.0
  end
  pose_rot   = p[cp1[0], cp1[1], cp1[2], ax*ang, ay*ang, az*ang]
  pose_orient = p[0, 0, 0, NOMINAL_FRAME[3], NOMINAL_FRAME[4], NOMINAL_FRAME[5]]
  MEAS_FRAME = pose_trans(pose_rot, pose_orient)
  textmsg("Surface tilt (rad) = ", ang)
  textmsg("MEAS_FRAME origin Z (m) = ", cp1[2])
end
'''
    lines += [
        'def apply_correction(p_world):',
        '  return pose_trans(MEAS_FRAME, pose_trans(pose_inv(NOMINAL_FRAME), p_world))',
        'end',
        '',
    ]

    for idx, cyc in enumerate(cycles, start=1):
        pts = cyc['pts']
        spd_var = speed_var_map[cyc['type']]
        lines.append(f'def cycle_{idx}():')
        lines.append(f'  # --- {cyc["label"]} ---')

        px0, py0 = plate_to_robot(pts[0, 0], pts[0, 1])
        pose_transit_in = _fmt_pose([px0, py0, z_transit_m, ROBOT_RX, ROBOT_RY, ROBOT_RZ])
        pose_contact_deep = _fmt_pose([px0, py0, ROBOT_Z_SURFACE - FORCE_CONTACT_DEPTH,
                                       ROBOT_RX, ROBOT_RY, ROBOT_RZ])

        lines.append(f'  movel(apply_correction({pose_transit_in}), '
                     f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v={transit_v_mps:.4f}*SPEED_FACTOR)')
        lines.append('  sleep(0.2)')
        lines.append('  zero_ftsensor()')
        lines.append('  sleep(0.2)')
        # limits force_mode : axe compliant Z = vitesse max (FORCE_LIMIT_Z) ;
        # axes non compliants X/Y/rot = deviation max toleree vs trajectoire
        # avant arret ("Force mode: Maximum position deviation exceeded").
        # FORCE_LIMIT_XY trop faible faisait fauter l'etalement (traine du doigt).
        lines.append(f'  force_mode(MEAS_FRAME, [0, 0, 1, 0, 0, 0], '
                     f'[0, 0, {-FORCE_Z_TARGET:.1f}, 0, 0, 0], 2, '
                     f'[{FORCE_LIMIT_XY}, {FORCE_LIMIT_XY}, {FORCE_LIMIT_Z}, '
                     f'{FORCE_LIMIT_ROT}, {FORCE_LIMIT_ROT}, {FORCE_LIMIT_ROT}])')
        lines.append(f'  movel(apply_correction({pose_contact_deep}), '
                     f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v=URSCRIPT_RECONTACT_V*SPEED_FACTOR)')

        waypoint_indices = cyc.get('waypoint_indices')
        if waypoint_indices is None:
            waypoint_indices = get_waypoint_indices(len(pts), cyc['type'])
        for i in waypoint_indices:
            px, py = plate_to_robot(pts[i, 0], pts[i, 1])
            pose_wp = _fmt_pose([px, py, ROBOT_Z_SURFACE, ROBOT_RX, ROBOT_RY, ROBOT_RZ])
            lines.append(f'  movel(apply_correction({pose_wp}), '
                         f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v={spd_var}*SPEED_FACTOR, r=URSCRIPT_BLEND)')

        px_last, py_last = plate_to_robot(pts[-1, 0], pts[-1, 1])
        pose_transit_out = _fmt_pose([px_last, py_last, z_transit_m, ROBOT_RX, ROBOT_RY, ROBOT_RZ])
        lines.append('  end_force_mode()')
        lines.append(f'  movel(apply_correction({pose_transit_out}), '
                     f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v={transit_v_mps:.4f}*SPEED_FACTOR)')
        lines.append('end')
        lines.append('')

    lines += [
        'def etalement():',
        # set_tcp intègre TCP_GRIPPER_Z (longueur de la 2F-85) dans l'offset TCP_Z.
        # C'est le SEUL endroit où la pince intervient : sa longueur, pas son
        # actionnement. Aucune activation / ouverture / fermeture n'est commandée.
        f'  set_tcp(p[{mm_to_m(TCP_X)}, {mm_to_m(TCP_Y)}, {mm_to_m(TCP_Z)}, 0, 0, 0])',
        '  # Aucune pose home absolue : la position de depart est la pose courante',
        '  # du robot (mesuree a l\'execution). L\'operateur jogge le robot ou il',
        '  # veut, clique Start, et le sondage de surface part de la.',
        '  home = get_actual_tcp_pose()',
        '  probe_surface_z()',
        '',
    ]
    for idx in range(1, len(cycles) + 1):
        lines.append(f'  cycle_{idx}()')

    # --- Retrait final : remontee de Z_RETREAT_END au-dessus de la surface ---
    # Apres le dernier cycle, le TCP est a Z_TRANSIT au-dessus de la surface, au
    # XY du dernier waypoint. On leve l'effecteur tout droit le long de la
    # normale de la surface (meme XY, Z = surface + Z_RETREAT_END) puis le
    # programme s'arrete la, degageant la plaque pour que l'operateur la retire.
    # Pose absolue literale passee par apply_correction (comme tous les
    # waypoints) : tilt-aware sur le plan mesure, deterministe, et parsee/
    # validee par ur5_sim (contrairement a un get_actual_tcp_pose() runtime).
    # px_last / py_last conservent les coords robot du dernier waypoint du
    # dernier cycle (portee de la boucle ci-dessus).
    z_retreat_end_m = ROBOT_Z_SURFACE + mm_to_m(Z_RETREAT_END)
    pose_retreat_end = _fmt_pose([px_last, py_last, z_retreat_end_m,
                                  ROBOT_RX, ROBOT_RY, ROBOT_RZ])
    lines.append(f'  movel(apply_correction({pose_retreat_end}), '
                 f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v={transit_v_mps:.4f}*SPEED_FACTOR)')
    lines += ['end', '', 'etalement()']
    return lines


def generate_urscript(cycles: list[dict], filename: Path = SCRIPT_PATH) -> bool:
    """
    Génère un fichier URScript exécutable sur le contrôleur UR5.
    Retourne False si le budget mémoire PolyScope est dépassé.
    """
    lines = _build_urscript_lines(cycles)
    filename = Path(filename)
    filename.parent.mkdir(parents=True, exist_ok=True)
    with open(filename, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
    print(f'URScript exporté -> {filename}  ({len(lines)} lignes)')
    return _validate_script_memory(filename, "URScript")


def generate_urp(cycles: list[dict], filename: Path = URP_PATH) -> bool:
    """
    Génère un fichier .urp (XML PolyScope) exécutable sur UR3/UR5/UR10.
    """
    import xml.etree.ElementTree as ET
    from xml.dom import minidom

    script_content = '\n'.join(_build_urscript_lines(cycles))

    root = ET.Element('program')
    root.set('version', '6.0')
    robot = ET.SubElement(root, 'robot')
    robot.set('speed', '100')
    robot.set('acceleration', '100')
    script_node = ET.SubElement(robot, 'script')
    script_node.text = script_content

    xml_str = minidom.parseString(ET.tostring(root, encoding='unicode')) \
                     .toprettyxml(indent='  ', encoding=None)
    xml_lines = xml_str.split('\n')
    if xml_lines[0].startswith('<?xml'):
        xml_lines = xml_lines[1:]
    xml_out = '\n'.join(xml_lines)

    filename = Path(filename)
    filename.parent.mkdir(parents=True, exist_ok=True)
    with open(filename, 'w', encoding='utf-8') as f:
        f.write(xml_out)
    print(f'URP exporté -> {filename}')
    return _validate_script_memory(filename, "URP")
