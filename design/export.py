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
  - generate_urscript_acq()   : écrit etalement_acq.script (jumeau acquisition)
  - generate_urp_acq()        : écrit etalement_acq.urp (jumeau acquisition)
  - _build_urscript_lines()   : construit la liste des lignes URScript
  - _build_acq_lines()        : enveloppe ces lignes du processus d'acquisition
  - _validate_script_memory() : vérifie le budget mémoire PolyScope
  - _clamp_tcp_speed()        : plafonne à URSCRIPT_MAX_TCP_SPEED
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import design.params as params
from design.geometry import (
    _abs_pose,
    _fmt_pose,
    _fmt_raw_pose,
    mm_to_m,
    plate_to_robot,
)
from design.params import SCRIPT_PATH, URP_PATH
from design.settings import Settings, get_settings
from design.settings_spec import SPECS
from design.trajectory import get_waypoint_indices


def _clamp_tcp_speed(name: str, v_mps: float, cap: float | None = None) -> float:
    """
    Plafonne une vitesse TCP à URSCRIPT_MAX_TCP_SPEED (limite PolyScope).
    Imprime un avertissement quand le clamp s'active.
    """
    if cap is None:
        cap = get_settings().urscript_max_tcp_speed
    if v_mps > cap:
        print(f"WARN: {name} = {v_mps:.3f} m/s > limite PolyScope "
              f"{cap:.3f} m/s, clamp.")
        return cap
    return v_mps


def _reject_invalid_settings(settings: Settings, label: str) -> bool:
    """
    --------------------------------------------------------------------------
    Purpose:
        Barriere de validite commune aux quatre generateurs (F1,
        docs/superpower/plans/erreur_hors_datalogger.md). Un export est ce qui
        atteint le robot : c'est le dernier endroit ou des reglages hors
        bornes peuvent encore etre arretes. Rien n'est ouvert ni ecrit quand
        elle se declenche, et force= ne l'outrepasse pas : force ne concerne
        que le garde-fou de retouche a la main, jamais la validite.

    Inputs:
        settings (Settings): reglages effectifs de l'export.
        label (str): etiquette du generateur, pour le message.

    Outputs:
        rejected (bool): True si l'export doit etre refuse (messages deja
        imprimes), False si les reglages passent.
    --------------------------------------------------------------------------
    """
    errors = settings.validate()
    if not errors:
        return False
    print(f"ECHEC EXPORT {label}: reglages invalides, export refuse "
          f"(rien n'est ecrit).")
    for err in errors:
        print(f"  WARN: {err}")
    return True


def _validate_script_memory(filename: Path, label: str,
                            content: str | None = None) -> bool:
    """
    Vérifie que le fichier généré reste dans le budget mémoire PolyScope.
    Retourne False si dépassé, True sinon.

    Mesure les octets que le contrôleur verra, pas ceux que le disque local
    porte (F2). `Path.stat().st_size` compte les CRLF que le mode texte de
    Windows ajoute : la référence en portait 817 de plus que sa propre chaîne,
    donc le même export passait ou échouait selon le système d'exploitation
    du poste. `content` est la chaîne écrite ; en son absence le fichier est
    relu en binaire, ce qui donne le même compte depuis que `_write_export`
    écrit en LF.
    """
    max_bytes = get_settings().urscript_max_bytes
    if content is not None:
        size_bytes = len(content.encode('utf-8'))
    else:
        size_bytes = len(filename.read_bytes())
    pct = 100.0 * size_bytes / max_bytes
    print(f"Mémoire {label}: {size_bytes} octets / {max_bytes} "
          f"({pct:.1f}% du budget PolyScope)")
    if size_bytes > max_bytes:
        print(f"ECHEC EXPORT {label}: budget mémoire URSCRIPT_MAX_BYTES = "
              f"{max_bytes} octets dépassé de "
              f"{size_bytes - max_bytes} octets. "
              f"Réduire URSCRIPT_N_WAYPOINTS_CIRCULAR / LIN_N_POINTS_PER_SEGMENT.")
        return False
    return True


def _settings_header_lines(settings: Settings) -> list[str]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Partie « ecarts aux defauts » de l'en-tete de recette. Conservee
        separement de _recipe_header_lines parce qu'elle est la seule partie
        conditionnelle : un export aux defauts n'a rien a lister.

    Inputs:
        settings (Settings): reglages effectifs de l'export.

    Outputs:
        lines (list[str]): lignes de commentaire URScript, vide si aucun ecart.
    --------------------------------------------------------------------------
    """
    overrides = settings.to_overrides()
    if not overrides:
        return []
    lines = ['# reglages differents des defauts de design/params.py :']
    by_name = {spec.name: spec for spec in SPECS}
    for name in sorted(overrides):
        spec = by_name[name]
        default = getattr(params, spec.const)
        lines.append(
            f'#   {spec.const:<28} {default} -> {overrides[name]} '
            f'{spec.unit}'.rstrip())
    return lines


def _recipe_header_lines(cycles: list[dict], settings: Settings) -> list[str]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Recette de reproduction, emise dans TOUT export (F10,
        docs/superpower/plans/erreur_hors_datalogger.md). Un fichier doit dire
        ce qui l'a produit, sinon personne ne peut regenerer le programme qui a
        mene un essai, ni verifier que l'artefact et le code s'accordent encore.

        Le bloc etait auparavant conditionnel aux ecarts aux defauts, et
        portait un horodatage. C'etait l'horodatage le probleme, pas le
        contenu : non deterministe, il cassait la comparaison octet pour octet
        d'une sortie nominale, ce qui a impose de n'emettre le bloc que
        rarement, donc de ne rien tracer dans le cas le plus courant. La date
        est donc retiree, git et le CSV d'acquisition la portent deja, et le
        bloc devient deterministe, donc emis toujours et verifiable par un
        test. Un fichier qui dit ce qui l'a produit vaut mieux qu'un fichier
        qui dit quand il l'a ete.

    Inputs:
        cycles (list[dict]): cycles tels que passes au generateur.
        settings (Settings): reglages effectifs de l'export.

    Outputs:
        lines (list[str]): lignes de commentaire URScript, jamais vide.
    --------------------------------------------------------------------------
    """
    lines = [
        '# === RECETTE DE REPRODUCTION ===',
        '# Ce bloc enonce ce qui a produit ce fichier. Il ne porte pas de date :',
        '# une date rendrait l export non reproductible, et git la porte deja.',
        f'# cycles : {len(cycles)}',
    ]
    for idx, cyc in enumerate(cycles, start=1):
        n_wp = len(cyc.get('waypoint_indices')
                   if cyc.get('waypoint_indices') is not None
                   else get_waypoint_indices(len(cyc['pts']), cyc['type']))
        lines.append(
            f'#   cycle {idx} : type={cyc["type"]:<8} waypoints={n_wp:<5} '
            f'label={cyc["label"]}')
    lines += _settings_header_lines(settings)
    lines.append(f'# empreinte des reglages : {settings.fingerprint()}')
    lines.append('#')
    return lines


def _build_urscript_lines(cycles: list[dict],
                          settings: Settings | None = None) -> list[str]:
    """
    Construit la liste des lignes URScript (partagée entre generate_urscript
    et generate_urp). Inclut le sondage de surface 3 points, l'ajustement de
    plan, et la correction par waypoint via apply_correction().

    Émet un bloc d'en-tête « PINCE 2F-85 : AUCUN ACTIONNEMENT » : aucune commande
    de pince n'est générée ici (la 2F-85 est un support passif ; voir docstring
    du module). Ce bloc apparaît donc dans etalement.script ET dans le <script>
    du .urp.
    """
    # Reglages lus a l'appel, jamais importes par valeur (plan, section 2).
    s = settings or get_settings()
    cap = s.urscript_max_tcp_speed
    z_transit_m = s.robot_z_surface + mm_to_m(s.z_transit)
    speed_var_map = {'circular': 'V_CIRC', 'linear': 'V_RECT'}

    # Le plafond URSCRIPT_MAX_TCP_SPEED (0.25 m/s) est applique par PolyScope
    # sur le vrai robot ; on ne l'emet plus dans le .script. La vitesse de
    # transit (post-clamp) est inlinee directement sur chaque movel de
    # transit au lieu d'etre exposee comme global URSCRIPT_TRANSIT_V.
    transit_v_mps = _clamp_tcp_speed("URSCRIPT_TRANSIT_V",
                                     s.urscript_transit_v, cap)

    probe_pts_robot = [
        plate_to_robot(x_mm, y_mm) for (x_mm, y_mm) in s.probe_points_plate_mm
    ]
    probe_nominal_contact = [
        _abs_pose([px_r, py_r, s.robot_z_surface,
                   s.robot_rx, s.robot_ry, s.robot_rz])
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

    p_ref_str = 'p[' + ', '.join(f'{v}' for v in s.p_ref) + ']'
    nominal_frame_str = _fmt_raw_pose(nominal_frame_pose)
    lines = [
        '# UR5 - Protocole etalement cosmetique',
        '# Genere automatiquement par ur5_etalement.py',
        '# IMPORTANT : calibrer ROBOT_X_ORIGIN, ROBOT_Y_ORIGIN, ROBOT_Z_SURFACE avant execution',
        '#',
        *_recipe_header_lines(cycles, s),
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
        f'global URSCRIPT_CONTACT_V = {_clamp_tcp_speed("URSCRIPT_CONTACT_V", s.urscript_contact_v, cap):.4f}  #sym:URSCRIPT_CONTACT_V',
        f'global URSCRIPT_RECONTACT_V = {_clamp_tcp_speed("URSCRIPT_RECONTACT_V", s.urscript_recontact_v, cap):.4f}  #sym:URSCRIPT_RECONTACT_V',
        f'global V_CIRC = {_clamp_tcp_speed("V_CIRC", mm_to_m(s.circ_speed), cap):.4f}  #sym:CIRC_SPEED',
        f'global V_RECT = {_clamp_tcp_speed("V_RECT", mm_to_m(s.lin_speed), cap):.4f}  #sym:LIN_SPEED',
        '',
        '# --- Accelerations (m/s^2) par phase ---',
        f'global URSCRIPT_ACCEL = {s.urscript_accel}  #sym:URSCRIPT_ACCEL',
        'global A_INIT = 1.2  #sym:A_INIT',
        '',
        '# --- Facteurs multiplicateurs globaux ---',
        'global SPEED_FACTOR = 1.0',
        'global ACCEL_FACTOR = 1.0',
        f'global URSCRIPT_BLEND = {s.urscript_blend}  #sym:URSCRIPT_BLEND',
        '',
        '# --- Sondage de surface : 1 point en Z (sondage 3 points DESACTIVE, voir bloc commente) ---',
        f'global PROBE_FORCE_THR    = {s.probe_force_thr}',
        f'global PROBE_DESCENT_V    = {_clamp_tcp_speed("PROBE_DESCENT_V", s.probe_descent_v, cap):.4f}',
        f'global PROBE_ACCEL        = {s.probe_accel}',
        f'global PROBE_MAX_TRAVEL   = {s.probe_max_travel}',
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
        pose_transit_in = _fmt_pose([px0, py0, z_transit_m,
                                     s.robot_rx, s.robot_ry, s.robot_rz])
        pose_contact_deep = _fmt_pose([px0, py0,
                                       s.robot_z_surface - s.force_contact_depth,
                                       s.robot_rx, s.robot_ry, s.robot_rz])

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
                     f'[0, 0, {-s.force_z_target:.1f}, 0, 0, 0], 2, '
                     f'[{s.force_limit_xy}, {s.force_limit_xy}, {s.force_limit_z}, '
                     f'{s.force_limit_rot}, {s.force_limit_rot}, {s.force_limit_rot}])')
        lines.append(f'  movel(apply_correction({pose_contact_deep}), '
                     f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v=URSCRIPT_RECONTACT_V*SPEED_FACTOR)')

        waypoint_indices = cyc.get('waypoint_indices')
        if waypoint_indices is None:
            waypoint_indices = get_waypoint_indices(len(pts), cyc['type'])
        for i in waypoint_indices:
            px, py = plate_to_robot(pts[i, 0], pts[i, 1])
            pose_wp = _fmt_pose([px, py, s.robot_z_surface,
                                 s.robot_rx, s.robot_ry, s.robot_rz])
            lines.append(f'  movel(apply_correction({pose_wp}), '
                         f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v={spd_var}*SPEED_FACTOR, r=URSCRIPT_BLEND)')

        px_last, py_last = plate_to_robot(pts[-1, 0], pts[-1, 1])
        pose_transit_out = _fmt_pose([px_last, py_last, z_transit_m,
                                      s.robot_rx, s.robot_ry, s.robot_rz])
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
        f'  set_tcp(p[{mm_to_m(params.TCP_X)}, {mm_to_m(params.TCP_Y)}, '
        f'{mm_to_m(s.tcp_z)}, 0, 0, 0])',
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
    z_retreat_end_m = s.robot_z_surface + mm_to_m(s.z_retreat_end)
    pose_retreat_end = _fmt_pose([px_last, py_last, z_retreat_end_m,
                                  s.robot_rx, s.robot_ry, s.robot_rz])
    lines.append(f'  movel(apply_correction({pose_retreat_end}), '
                 f'a=URSCRIPT_ACCEL*ACCEL_FACTOR, v={transit_v_mps:.4f}*SPEED_FACTOR)')
    lines += ['end', '', 'etalement()']
    return lines


# Ancres d'insertion du bloc d'acquisition. Ce sont des lignes que
# _build_urscript_lines() emet toujours ; si l'une disparait, l'export acq
# echoue franchement (ValueError) au lieu de produire un programme ampute.
_ACQ_ANCHOR_THREAD = 'def etalement():'
_ACQ_ANCHOR_SET_TCP = '  set_tcp(p['
_ACQ_ANCHOR_TAIL = 'etalement()'


def _acq_thread_lines() -> list[str]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Bloc URScript du thread d'acquisition 50 Hz, insere avant
        `def etalement():`. Contraintes CB3 (PolyScope 3.x) respectees ici :
        aucun movel/stopl dans un thread, aucun slice de liste, aucune
        allocation dans la boucle, aucune construction de chaine.

        Cadence : le tick de `sync()` vaut 8 ms sur CB3, donc 20 ms n'est pas
        atteignable. La boucle alterne 2 et 3 ticks (16 et 24 ms) pour une
        moyenne exacte de 20.000 ms, et chaque echantillon porte son temps de
        tick reel, ce qui rend l'analyse exacte malgre la gigue de +/-4 ms.

    Outputs:
        lines (list[str]): lignes URScript du bloc thread.
    --------------------------------------------------------------------------
    """
    return [
        '# === ACQUISITION 50 Hz ===',
        '# Ajout au programme d\'etalement : aucun waypoint, aucune vitesse et',
        '# aucun parametre de force n\'est modifie par ce bloc. Le thread lit la',
        '# pose et l\'envoie au daemon (cle USB) sur la boucle locale ; le fichier',
        '# CSV est ecrit par le daemon a la fin, jamais pendant le mouvement.',
        f'global ACQ_LOG_PORT = {params.ACQ_LOG_PORT}',
        f'global ACQ_MAX_SAMPLES = {params.ACQ_MAX_SAMPLES}',
        '# Bascule de repli : True fait porter par le script les efforts internes',
        '# get_tcp_force() (estimation par courants moteur, erreur de plusieurs N)',
        '# au lieu de laisser le daemon fusionner le flux FT-300 calibre.',
        'global ACQ_USE_INTERNAL_FORCE = False',
        'global acq_keep_logging = True',
        'global acq_index = 0',
        'global acq_ticks = 0',
        '# Liste reutilisee a chaque echantillon : affectation indexee seulement,',
        '# aucune allocation dans la boucle (list_append n\'existe pas sur CB3).',
        'global acq_sample = [0.0, 0.0, 0.0, 0.0]',
        '',
        'thread data_logger():',
        '  # acq_long alterne 2 et 3 ticks de 8 ms : 16, 24, 16, 24 ms.',
        '  acq_long = False',
        '  while acq_keep_logging and acq_index < ACQ_MAX_SAMPLES:',
        '    sync()',
        '    sync()',
        '    acq_ticks = acq_ticks + 2',
        '    if acq_long:',
        '      sync()',
        '      acq_ticks = acq_ticks + 1',
        '    end',
        '    acq_long = not acq_long',
        '    acq_pose = get_actual_tcp_pose()',
        '    acq_sample[0] = acq_ticks * 0.008',
        '    acq_sample[1] = acq_pose[0]',
        '    acq_sample[2] = acq_pose[1]',
        '    acq_sample[3] = acq_pose[2]',
        '    # socket_send_line serialise la liste ([t,x,y,z]) : aucune chaine',
        '    # n\'est construite ici, to_str / str_cat n\'existent pas sur CB3.',
        '    socket_send_line(acq_sample, "acq")',
        '    acq_index = acq_index + 1',
        '  end',
        '  # Sortie de boucle sur l\'un ou l\'autre test : l\'arret au plafond du',
        '  # tampon est automatique, sans debordement possible.',
        'end',
        '',
    ]


def _acq_open_lines() -> list[str]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Ouverture du socket et demarrage du thread, insere juste apres
        set_tcp(). Place avant tout mouvement, sondage Z compris : un daemon
        absent arrete le programme par popup avant que le robot ne bouge,
        plutot que de faire un essai complet sans enregistrement.

    Outputs:
        lines (list[str]): lignes URScript a inserer dans etalement().
    --------------------------------------------------------------------------
    """
    return [
        '  # --- Acquisition : ouverture avant tout mouvement ---',
        '  acq_ouvert = socket_open("127.0.0.1", ACQ_LOG_PORT, "acq")',
        '  if not acq_ouvert:',
        '    popup("Daemon d\'acquisition injoignable sur 127.0.0.1", '
        '"Acquisition", error=True)',
        '    halt',
        '  end',
        '  acq_logger = run data_logger()',
    ]


def _acq_stop_lines() -> list[str]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Arret du thread et poignee de main d'export, insere apres le retrait
        final et avant la fin de etalement(). L'ordre compte : le drapeau
        arrete la boucle, le sleep laisse passer un dernier cycle de 24 ms, le
        kill garantit qu'aucun echantillon n'est ecrit apres, et seulement
        ensuite le daemon recoit l'ordre d'ecrire le fichier.

        Le compte d'echantillons voyage dans une liste sentinelle de premier
        champ negatif, pas dans une chaine "STOP <n>" : CB3 3.x n'a ni to_str
        ni str_cat, donc "STOP " + acq_index est impossible a construire. Le
        litteral "STOP" qui suit, lui, est une constante, donc legal.

    Outputs:
        lines (list[str]): lignes URScript a inserer dans etalement().
    --------------------------------------------------------------------------
    """
    return [
        '',
        '  # --- Acquisition : arret du thread puis export ---',
        '  acq_keep_logging = False',
        '  sleep(0.1)',
        '  kill acq_logger',
        '  # Sentinelle de comptage : premier champ negatif, compte en second.',
        '  acq_sample[0] = -1.0',
        '  acq_sample[1] = acq_index',
        '  acq_sample[2] = 0.0',
        '  acq_sample[3] = 0.0',
        '  socket_send_line(acq_sample, "acq")',
        '  socket_send_line("STOP", "acq")',
        '  acq_reponse = socket_read_string("acq")',
        '  textmsg("Acquisition : ", acq_reponse)',
        '  popup(acq_reponse, "Acquisition terminee")',
        '  socket_close("acq")',
    ]


def _build_acq_lines(base_lines: list[str]) -> list[str]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Enveloppe la sortie intacte de _build_urscript_lines() du processus
        d'acquisition. Le mouvement n'est pas touche : aucune ligne existante
        n'est modifiee ni supprimee, seules des lignes sont inserees a trois
        ancres. C'est ce qui permet a tests/test_acq_export.py de prouver que
        parse_poses() rend exactement les memes 4-uplets pour les deux
        fichiers.

        Une ancre absente est une erreur d'export (ValueError), jamais un saut
        silencieux : un programme ampute de son thread partirait sur le robot
        en donnant l'illusion d'enregistrer.

    Inputs:
        base_lines (list[str]): lignes de etalement.script, non modifiees.

    Outputs:
        lines (list[str]): lignes de etalement_acq.script.
    --------------------------------------------------------------------------
    """
    lines = list(base_lines)

    try:
        i_thread = lines.index(_ACQ_ANCHOR_THREAD)
    except ValueError:
        raise ValueError(
            f"Export acq impossible : ancre '{_ACQ_ANCHOR_THREAD}' absente du "
            f"script de base. _build_urscript_lines() a change de forme ; "
            f"corriger _build_acq_lines() avant d'exporter.")

    i_set_tcp = next(
        (i for i, ln in enumerate(lines)
         if ln.startswith(_ACQ_ANCHOR_SET_TCP) and i > i_thread), None)
    if i_set_tcp is None:
        raise ValueError(
            "Export acq impossible : ancre set_tcp absente de etalement(). "
            "Le socket doit s'ouvrir avant le premier mouvement.")

    i_tail = next(
        (i for i in range(len(lines) - 1, -1, -1)
         if lines[i] == _ACQ_ANCHOR_TAIL), None)
    if i_tail is None:
        raise ValueError(
            "Export acq impossible : appel final 'etalement()' absent.")
    i_end = next(
        (i for i in range(i_tail - 1, i_set_tcp, -1) if lines[i] == 'end'),
        None)
    if i_end is None:
        raise ValueError(
            "Export acq impossible : fin de etalement() introuvable avant "
            "l'appel final.")

    # Insertions de la fin vers le debut : les index calcules ci-dessus
    # restent valides tant qu'on n'a pas insere avant eux.
    lines[i_end:i_end] = _acq_stop_lines()
    lines[i_set_tcp + 1:i_set_tcp + 1] = _acq_open_lines()
    lines[i_thread:i_thread] = _acq_thread_lines()
    return lines


# État du dernier export, pour détecter un fichier retouché à la main. Le
# `.urp` de référence a été ajusté manuellement pour des essais robot, et
# `generate_urp` l'écrasait jusqu'ici sans le moindre avertissement.
EXPORT_STATE_PATH: Path = params.REPO_ROOT / ".etalement_export_state.json"


def _digest(text: str) -> str:
    return hashlib.sha256(text.encode('utf-8')).hexdigest()[:16]


def _load_export_state() -> tuple[dict[str, str], bool]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Lit l'etat d'export. Le second element (`ok`) distingue un etat
        VALIDE mais qui ne connait simplement pas ce fichier (premier export
        legitime d'un poste neuf : `ok=True`, dict incomplet) d'un etat ABSENT
        ou CORROMPU (`ok=False`, dict toujours vide) : sans cette distinction,
        `check_overwrite` ne peut pas dire pourquoi il ne sait rien du
        fichier vise, et le cas F3 (garde-fou disarme par la disparition du
        fichier d'etat) resterait silencieux.

    Outputs:
        state (dict[str, str]): nom de fichier -> empreinte, {} si absent ou
            illisible.
        ok (bool): False si le fichier d'etat est absent ou n'a pas pu etre
            decode ; True s'il a ete lu et parse normalement (meme vide).
    --------------------------------------------------------------------------
    """
    if not EXPORT_STATE_PATH.is_file():
        return {}, False
    try:
        return json.loads(EXPORT_STATE_PATH.read_text(encoding='utf-8')), True
    except (json.JSONDecodeError, OSError):
        return {}, False


def _record_export(filename: Path, content: str) -> None:
    state, _ = _load_export_state()
    state[filename.name] = _digest(content)
    try:
        EXPORT_STATE_PATH.write_text(
            json.dumps(state, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
    except OSError as exc:
        print(f"WARN: etat d'export non enregistre ({exc}).")


def check_overwrite(filename: Path) -> str | None:
    """
    --------------------------------------------------------------------------
    Purpose:
        Dit si le fichier de sortie a ete retouche depuis le dernier export,
        auquel cas l'ecraser detruirait un reglage saisi a la main.

        Un fichier inconnu d'un etat d'export par ailleurs VALIDE ne
        declenche rien : on ne sait pas d'ou il vient, et refuser le premier
        export de chaque poste serait une nuisance sans contrepartie (F3,
        docs/superpower/plans/erreur_hors_datalogger.md ; decision inchangee,
        seulement rendue audible quand la cause est un etat perdu plutot
        qu'un premier export legitime : voir le WARN ci-dessous).

        Deux echecs de LECTURE, traites differemment (F3) :
          - etat d'export absent ou corrompu (`_load_export_state` renvoie
            `ok=False`) alors que le fichier de sortie existe deja : la
            decision reste d'autoriser l'export (un poste neuf ou un fichier
            d'etat perdu ne doit pas bloquer indefiniment), mais un WARN
            nomme la situation au lieu de se taire.
          - fichier de sortie present et TRACE par l'etat mais illisible
            (verrou d'un autre programme, permission) : c'est exactement le
            cas que ce garde-fou existe pour couvrir (etalement.urp ajuste
            a la main entre essais robot), donc on echoue FERME : message de
            refus, sauf force=True qui outrepasse quand meme (avec avertisse-
            ment, cote appelant).

    Inputs:
        filename (Path): fichier de sortie vise.

    Outputs:
        message (str | None): avertissement a montrer, None si l'ecrasement
        est sans risque.
    --------------------------------------------------------------------------
    """
    filename = Path(filename)
    if not filename.is_file():
        return None
    state, state_ok = _load_export_state()
    if not state_ok:
        print(f"WARN: {EXPORT_STATE_PATH.name} absent ou illisible : "
              f"impossible de savoir si {filename.name} a deja ete retouche "
              f"a la main. Export autorise sans verification de retouche "
              f"pour ce fichier (poste neuf, ou etat d'export perdu).")
        return None
    known = state.get(filename.name)
    if known is None:
        return None
    try:
        current = _digest(filename.read_text(encoding='utf-8'))
    except OSError as exc:
        return (f"{filename.name} n'a pas pu etre lu ({exc}) : impossible de "
                f"verifier s'il a ete retouche a la main depuis le dernier "
                f"export (peut-etre ouvert dans un autre programme). Refus "
                f"par prudence pour ne pas ecraser a l'aveugle. Relancer "
                f"avec force=True pour passer outre.")
    if current == known:
        return None
    return (f"{filename.name} a ete modifie depuis le dernier export "
            f"(empreinte {current} au lieu de {known}). L'ecraser perdrait "
            f"ces retouches. Relancer avec force=True pour passer outre, ou "
            f"exporter sous un autre nom.")


def _write_export(filename: Path, content: str, label: str,
                  force: bool) -> bool:
    """Ecrit un fichier de sortie apres controle d'ecrasement.

    Ecrit en LF explicite (F2). Le fichier part sur un controleur Linux, et
    le mode texte de Windows y glissait un CRLF par ligne : 817 octets que la
    chaine generee ne contient pas, invisibles a la relecture puisque
    `read_text` les retraduit. Un poste Windows et un poste Linux produisaient
    donc deux fichiers differents pour un meme export.
    """
    filename = Path(filename)
    warning = check_overwrite(filename)
    if warning and not force:
        print(f"ECHEC EXPORT {label}: {warning}")
        return False
    if warning:
        print(f"WARN: {warning}")
    filename.parent.mkdir(parents=True, exist_ok=True)
    filename.write_text(content, encoding='utf-8', newline='\n')
    _record_export(filename, content)
    return True


def generate_urscript(cycles: list[dict], filename: Path = SCRIPT_PATH,
                      settings: Settings | None = None,
                      force: bool = False) -> bool:
    """
    Génère un fichier URScript exécutable sur le contrôleur UR5.
    Retourne False si les réglages sont invalides (F1,
    docs/superpower/plans/erreur_hors_datalogger.md : rien n'est écrit dans
    ce cas, le contrôle a lieu avant la moindre ouverture de fichier), si le
    fichier a été retouché à la main (sauf force=True), ou si le budget
    mémoire PolyScope est dépassé.
    """
    s = settings or get_settings()
    if _reject_invalid_settings(s, "URScript"):
        return False
    lines = _build_urscript_lines(cycles, s)
    content = '\n'.join(lines)
    filename = Path(filename)
    if not _write_export(filename, content, "URScript", force):
        return False
    print(f'URScript exporté -> {filename}  ({len(lines)} lignes)')
    return _validate_script_memory(filename, "URScript", content)


def generate_urp(cycles: list[dict], filename: Path = URP_PATH,
                 settings: Settings | None = None,
                 force: bool = False) -> bool:
    """
    Génère un fichier .urp (XML PolyScope) exécutable sur UR3/UR5/UR10.

    Refuse d'écraser un `.urp` retouché à la main tant que force=True n'est pas
    passé : le fichier de référence porte des réglages d'essai robot saisis
    directement sur le pendant. Refuse aussi, avant toute écriture, des
    réglages invalides (F1, docs/superpower/plans/erreur_hors_datalogger.md).
    """
    import xml.etree.ElementTree as ET
    from xml.dom import minidom

    s = settings or get_settings()
    if _reject_invalid_settings(s, "URP"):
        return False

    script_content = '\n'.join(_build_urscript_lines(cycles, s))

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
    if not _write_export(filename, xml_out, "URP", force):
        return False
    print(f'URP exporté -> {filename}')
    return _validate_script_memory(filename, "URP", xml_out)


def generate_urscript_acq(cycles: list[dict],
                          filename: Path = params.ACQ_SCRIPT_PATH,
                          settings: Settings | None = None,
                          force: bool = False) -> bool:
    """
    --------------------------------------------------------------------------
    Purpose:
        Genere le jumeau acquisition de generate_urscript() : les memes lignes
        de mouvement, construites par _build_urscript_lines() sans la moindre
        modification, puis enveloppees du processus d'acquisition 50 Hz par
        _build_acq_lines(). Ecrit par le meme _write_export() que l'original :
        le garde-fou de retouche a la main et l'empreinte d'etat d'export
        s'appliquent donc identiquement au fichier _acq.

    Inputs:
        cycles (list[dict]): cycles de trajectoire (memes que generate_urscript).
        filename (Path): fichier de sortie, ACQ_SCRIPT_PATH par defaut.
        settings (Settings | None): reglages effectifs ; None -> get_settings().
        force (bool): outrepasse le garde-fou de retouche a la main.

    Outputs:
        ok (bool): False si retouche a la main (sauf force=True) ou si le
        budget memoire PolyScope est depasse ; True sinon.
    --------------------------------------------------------------------------
    """
    s = settings or get_settings()
    if _reject_invalid_settings(s, "URScript ACQ"):
        return False
    base_lines = _build_urscript_lines(cycles, s)
    lines = _build_acq_lines(base_lines)
    content = '\n'.join(lines)
    filename = Path(filename)
    if not _write_export(filename, content, "URScript ACQ", force):
        return False
    print(f'URScript ACQ exporté -> {filename}  ({len(lines)} lignes)')
    return _validate_script_memory(filename, "URScript ACQ", content)


def generate_urp_acq(cycles: list[dict],
                     filename: Path = params.ACQ_URP_PATH,
                     settings: Settings | None = None,
                     force: bool = False) -> bool:
    """
    --------------------------------------------------------------------------
    Purpose:
        Genere le jumeau acquisition de generate_urp() : meme assemblage XML
        PolyScope que generate_urp() (non touche, voir sa docstring pour le
        garde-fou d'ecrasement), mais avec le contenu <script> produit par
        _build_acq_lines(_build_urscript_lines(...)) au lieu de
        _build_urscript_lines() seul.

    Inputs:
        cycles (list[dict]): cycles de trajectoire (memes que generate_urp).
        filename (Path): fichier de sortie, ACQ_URP_PATH par defaut.
        settings (Settings | None): reglages effectifs ; None -> get_settings().
        force (bool): outrepasse le garde-fou de retouche a la main.

    Outputs:
        ok (bool): False si retouche a la main (sauf force=True) ou si le
        budget memoire PolyScope est depasse ; True sinon.
    --------------------------------------------------------------------------
    """
    import xml.etree.ElementTree as ET
    from xml.dom import minidom

    s = settings or get_settings()
    if _reject_invalid_settings(s, "URP ACQ"):
        return False
    base_lines = _build_urscript_lines(cycles, s)
    script_content = '\n'.join(_build_acq_lines(base_lines))

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
    if not _write_export(filename, xml_out, "URP ACQ", force):
        return False
    print(f'URP ACQ exporté -> {filename}')
    return _validate_script_memory(filename, "URP ACQ", xml_out)
