"""Surface-probe simulation (L4 audit closure).

DESACTIVE - A REVOIR (rework futur).
    Ce module simule le sondage 3 points (``probe_surface_plane``), qui s'est
    revele INCORRECT : il est fixe en Z et ne gere ni la rotation de la plaque
    ni une hauteur de plaque inconnue (dependante de la manipulation de
    l'operateur). L'export URScript a ete bascule sur un sondage Z 1 point
    (``probe_surface_z``). Ce module et son rejeu (``run_probe_simulation``,
    plus bas dans ce fichier, garde par ``SIM_PROBE_ENABLE = False``) sont donc
    inertes ;
    ils sont conserves, non supprimes, pour le rework. Les tests associes sont
    commentes dans ``tests/test_probe_sim.py``.

Reproduces ``probe_surface_plane()`` from ``ur5_etalementv6.py`` in Python so
the ``ur5_sim`` validation pipeline covers the 3-point probing phase. Three
responsibilities:

* **Virtual plate** : :func:`build_virtual_plate` parametrizes the "real"
  plate as nominal pose offset along the nominal normal plus two small tilt
  angles around world X / Y. Lives in :mod:`ur5_sim.config`.
* **Geometric descent** : :func:`simulate_probe_descent` intersects the
  ``approach -> floor`` segment emitted by ``ur5_etalementv6._build_urscript_lines``
  with the virtual plate plane. The result mirrors ``contact_pose = get_actual_tcp_pose()``
  inside the ``probe_watcher`` thread on the real controller.
* **Frame reconstruction** : :func:`compute_meas_frame` implements the exact
  Rodrigues sequence URScript runs on the controller (``v12 = cp2 - cp1``,
  ``v13 = cp3 - cp1``, ``n_meas = v12 x v13``, flip toward NHAT, ``axis =
  NHAT x n_meas``, ``angle = atan2(|axis|, dot)``, ``MEAS_FRAME = pose_trans(
  pose_rot, pose_orient)``).

The simulator does *not* model the FT-300 force loop. The virtual plate is
the surrogate; if ``SIM_PROBE_PLATE_DZ_M = SIM_PROBE_PLATE_TILT_*_RAD = 0`` the
plate equals the nominal plane and the reconstruction returns the identity
transform (``MEAS_FRAME == NOMINAL_FRAME``), matching the design-time
assumption used everywhere else in :mod:`ur5_sim`.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from spatialmath import SE3, SO3

from ur5_sim.config import (
    SCRIPT_PATH,
    SIM_PROBE_PLATE_DZ_M,
    SIM_PROBE_PLATE_TILT_X_RAD,
    SIM_PROBE_PLATE_TILT_Y_RAD,
    SIM_PROBE_RESIDUAL_TOL_M,
    SIM_PROBE_TILT_MAX_RAD,
)
from ur5_sim.parsing.urscript import (
    parse_nhat,
    parse_nominal_frame,
    parse_probe_blocks,
    urscript_pose,
)


def _pose6_to_se3(pose6: tuple) -> SE3:
    return urscript_pose(*pose6)


def _se3_to_pose6(pose: SE3) -> tuple[float, ...]:
    t = pose.t
    rv = SO3(pose.R).eulervec()
    return (float(t[0]), float(t[1]), float(t[2]),
            float(rv[0]), float(rv[1]), float(rv[2]))


def build_virtual_plate(
    nominal_frame_pose6: tuple,
    nhat_world: tuple,
    dz_m: float,
    tilt_x_rad: float,
    tilt_y_rad: float,
) -> dict:
    """Return the virtual plate plane in world coordinates.

    Parameters
    ----------
    nominal_frame_pose6 : tuple
        ``(x, y, z, rx, ry, rz)`` absolute world pose of P1 nominal contact.
        Origin of the virtual plate = ``nominal_frame_pose6[:3]`` shifted by
        ``dz_m * nhat_world``.
    nhat_world : tuple
        ``(nx, ny, nz)`` nominal plate normal (world frame).
    dz_m : float
        Real plate vertical offset along ``nhat_world`` (positive = higher
        than nominal).
    tilt_x_rad, tilt_y_rad : float
        Small rotations around world X / Y applied to ``nhat_world`` to
        produce the tilted real normal.

    Returns
    -------
    dict
        ``{"origin": (3,), "normal": (3,)}``. Normal is unit-length.
    """
    origin_nom = np.asarray(nominal_frame_pose6[:3], dtype=float)
    nhat = np.asarray(nhat_world, dtype=float)
    nhat = nhat / max(float(np.linalg.norm(nhat)), 1e-12)

    cx, sx = np.cos(tilt_x_rad), np.sin(tilt_x_rad)
    cy, sy = np.cos(tilt_y_rad), np.sin(tilt_y_rad)
    R_x = np.array([[1.0, 0.0, 0.0],
                    [0.0, cx, -sx],
                    [0.0, sx,  cx]])
    R_y = np.array([[ cy, 0.0, sy],
                    [0.0, 1.0, 0.0],
                    [-sy, 0.0, cy]])
    normal = R_x @ R_y @ nhat
    normal /= max(float(np.linalg.norm(normal)), 1e-12)
    origin = origin_nom + dz_m * nhat
    return {"origin": origin, "normal": normal}


def simulate_probe_descent(
    approach_pose6: tuple,
    floor_pose6: tuple,
    virtual_plate: dict,
) -> Optional[tuple]:
    """Intersect descent segment ``approach -> floor`` with virtual plate.

    Returns the 6-tuple contact pose (orientation kept from ``approach``)
    or ``None`` if the descent does not cross the plane inside the segment
    (mirrors ``contact_found = False`` on the real controller : no contact
    detected within ``PROBE_FLOOR_PLATE_MM`` of travel).
    """
    a = np.asarray(approach_pose6[:3], dtype=float)
    f = np.asarray(floor_pose6[:3], dtype=float)
    direction = f - a
    denom = float(np.dot(direction, virtual_plate["normal"]))
    if abs(denom) < 1e-12:
        return None
    t = float(np.dot(virtual_plate["origin"] - a, virtual_plate["normal"]) / denom)
    if t < 0.0 or t > 1.0:
        return None
    contact = a + t * direction
    return (float(contact[0]), float(contact[1]), float(contact[2]),
            float(approach_pose6[3]), float(approach_pose6[4]),
            float(approach_pose6[5]))


def compute_meas_frame(
    cp1: tuple, cp2: tuple, cp3: tuple,
    nhat_world: tuple,
    nominal_frame_pose6: tuple,
) -> tuple[SE3, float]:
    """Reproduce the URScript Rodrigues reconstruction of ``MEAS_FRAME``.

    Mirrors lines 1234-1281 of ``ur5_etalementv6._build_urscript_lines``.

    Returns
    -------
    (meas_frame_se3, tilt_rad)
        ``meas_frame_se3`` : SE3 pose ready to feed
        :func:`apply_correction`. ``tilt_rad`` : angle between nominal
        normal and measured normal ; caller compares against
        ``SIM_PROBE_TILT_MAX_RAD``.
    """
    cp1a = np.asarray(cp1[:3], dtype=float)
    cp2a = np.asarray(cp2[:3], dtype=float)
    cp3a = np.asarray(cp3[:3], dtype=float)
    v12 = cp2a - cp1a
    v13 = cp3a - cp1a
    n_meas = np.cross(v12, v13)
    nrm = float(np.linalg.norm(n_meas))
    if nrm < 1e-12:
        raise ValueError("Probe points colinear : cannot reconstruct plane")
    n_meas = n_meas / nrm

    nhat = np.asarray(nhat_world, dtype=float)
    nhat = nhat / max(float(np.linalg.norm(nhat)), 1e-12)
    dot_nom = float(np.dot(nhat, n_meas))
    if dot_nom < 0.0:
        n_meas = -n_meas
        dot_nom = -dot_nom

    axis = np.cross(nhat, n_meas)
    anrm = float(np.linalg.norm(axis))
    angle = float(np.arctan2(anrm, dot_nom))
    if anrm > 1e-9:
        axis = axis / anrm
    else:
        axis = np.zeros(3)

    rotvec = axis * angle
    # pose_rot = p[cp1, rotvec] -> SE3 with translation cp1 and rotation Rodrigues
    pose_rot = SE3.Rt(SO3.EulerVec(rotvec), cp1a)
    # pose_orient = p[0, NOMINAL_FRAME[3:]] -> rotation-only SE3 carrying nominal
    # tool orientation. URScript builds MEAS_FRAME = pose_trans(pose_rot, pose_orient).
    nom_rotvec = np.asarray(nominal_frame_pose6[3:], dtype=float)
    pose_orient = SE3.Rt(SO3.EulerVec(nom_rotvec), [0.0, 0.0, 0.0])
    meas_frame = pose_rot * pose_orient
    return meas_frame, angle


def apply_correction(
    waypoint_world_pose6: tuple,
    meas_frame: SE3,
    nominal_frame_pose6: tuple,
) -> SE3:
    """Python equivalent of the URScript ``apply_correction`` helper.

    ``apply_correction(p) = MEAS_FRAME * inv(NOMINAL_FRAME) * p``. When
    ``MEAS_FRAME == NOMINAL_FRAME``, returns the input untouched.
    """
    nominal_se3 = _pose6_to_se3(nominal_frame_pose6)
    p_se3 = _pose6_to_se3(waypoint_world_pose6)
    return meas_frame * nominal_se3.inv() * p_se3


def signed_distance_to_plane(point_xyz: np.ndarray, virtual_plate: dict) -> float:
    """Signed distance from a 3D point to the virtual plate plane (m).

    Positive when the point sits on the same side as the plate normal.
    Used to validate that ``apply_correction`` brings a contact-z waypoint
    onto the real plane (residual <= ``SIM_PROBE_RESIDUAL_TOL_M``).
    """
    p = np.asarray(point_xyz, dtype=float)
    return float(np.dot(p - virtual_plate["origin"], virtual_plate["normal"]))


def run_probe_simulation(
    segments_for_residual: list,
) -> list[tuple[int, str, object]]:
    """Simulate ``probe_surface_plane()`` against a virtual plate.

    Returns a list of ``(lineno, kind, detail)`` events consumed by
    :func:`ur5_sim.reporting.text_report.report`. Skips silently with a
    ``PROBE_SKIPPED`` event if the script predates the 3-point probe block.
    """
    blocks = parse_probe_blocks(SCRIPT_PATH)
    nominal = parse_nominal_frame(SCRIPT_PATH)
    nhat = parse_nhat(SCRIPT_PATH)
    if not blocks or nominal is None or nhat is None:
        return [(0, "PROBE_SKIPPED",
                 "script lacks probe_surface_plane() block - sim coverage off")]
    if len(blocks) != 3:
        return [(blocks[0][1], "PROBE_SKIPPED",
                 f"expected 3 probe points, got {len(blocks)}")]

    virtual_plate = build_virtual_plate(
        nominal_frame_pose6=nominal,
        nhat_world=nhat,
        dz_m=SIM_PROBE_PLATE_DZ_M,
        tilt_x_rad=SIM_PROBE_PLATE_TILT_X_RAD,
        tilt_y_rad=SIM_PROBE_PLATE_TILT_Y_RAD,
    )

    events: list[tuple[int, str, object]] = []
    contacts: list[tuple[float, ...]] = []
    for idx, lineno, approach, floor in blocks:
        cp = simulate_probe_descent(approach, floor, virtual_plate)
        if cp is None:
            events.append((lineno, "PROBE_NO_CONTACT",
                           f"P{idx}: descente n'intersecte pas le plan virtuel "
                           f"(dz={SIM_PROBE_PLATE_DZ_M*1000:+.1f} mm, "
                           f"tilt_x={SIM_PROBE_PLATE_TILT_X_RAD:+.4f} rad, "
                           f"tilt_y={SIM_PROBE_PLATE_TILT_Y_RAD:+.4f} rad)"))
            return events
        contacts.append(cp)
        events.append((lineno, "PROBE_OK",
                       f"P{idx} contact a z = {cp[2]*1000:+.3f} mm "
                       f"(approche z = {approach[2]*1000:+.3f} mm)"))

    try:
        meas_frame, tilt_rad = compute_meas_frame(
            contacts[0], contacts[1], contacts[2], nhat, nominal,
        )
    except ValueError as exc:
        events.append((blocks[0][1], "PROBE_NO_CONTACT", str(exc)))
        return events

    if tilt_rad > SIM_PROBE_TILT_MAX_RAD:
        events.append((blocks[0][1], "PROBE_TILT_EXCEEDED",
                       f"tilt mesure = {tilt_rad:.4f} rad > "
                       f"SIM_PROBE_TILT_MAX_RAD = {SIM_PROBE_TILT_MAX_RAD:.4f} rad"))
        return events

    # Validation : prendre le premier waypoint in_contact du cycle 1 qui est
    # sur le plan nominal (ROBOT_Z_SURFACE), lui appliquer apply_correction
    # Python et verifier qu'il atterrit sur le plan virtuel a
    # SIM_PROBE_RESIDUAL_TOL_M pres. Le tout premier in_contact emis par le
    # generateur est la "descente profonde" (z = ROBOT_Z_SURFACE -
    # FORCE_CONTACT_DEPTH = ~5 mm sous le plan) : le controleur de force
    # arrete reellement le robot au contact, mais le sim sans physique
    # verrait un faux residu. On filtre donc les poses sous le plan nominal
    # (le sim ne corrige que les vrais waypoints de trajectoire).
    sample_lineno = None
    sample_pose = None
    n = np.asarray((nhat[0], nhat[1], nhat[2]), dtype=float)
    o = np.asarray(nominal[:3], dtype=float)
    # Iterate over the ORIGINAL parsed segments (not the densified buffer):
    # SE3-slerp substeps between transit_in and contact_deep would otherwise
    # appear at z = +Z_TRANSIT above the plane and corrupt the residual.
    for seg in segments_for_residual:
        if seg.cycle_idx != 1 or not seg.in_contact:
            continue
        p_xyz = np.asarray(seg.pose[:3], dtype=float)
        signed = float(np.dot(p_xyz - o, n))
        if signed < -1e-4:  # >0.1 mm sous le plan nominal = cible force, skip
            continue
        sample_lineno = seg.lineno
        sample_pose = seg.pose
        break
    if sample_pose is None:
        events.append((blocks[0][1], "PROBE_SKIPPED",
                       "no in_contact waypoint in cycle 1 - residual not checked"))
        return events

    corrected = apply_correction(sample_pose, meas_frame, nominal)
    residual_m = signed_distance_to_plane(corrected.t, virtual_plate)
    if abs(residual_m) > SIM_PROBE_RESIDUAL_TOL_M:
        events.append((sample_lineno, "PROBE_RESIDUAL",
                       f"residu = {residual_m*1000:+.4f} mm > tol = "
                       f"{SIM_PROBE_RESIDUAL_TOL_M*1000:.4f} mm "
                       f"(tilt reconstruit = {tilt_rad:.4f} rad)"))
    else:
        events.append((sample_lineno, "PROBE_OK",
                       f"residu post-correction = {residual_m*1000:+.4f} mm "
                       f"(tilt reconstruit = {tilt_rad:.4f} rad)"))
    return events
