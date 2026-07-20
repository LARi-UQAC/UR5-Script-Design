"""Test surface (50x50 mm plate) - geometry + kinematic force surrogate.

Two responsibilities :

* **Geometry** : compute the world-frame frame of the test plate (center,
  unit normal, four corners) by replaying the exact transform chain that
  ``ur5_sim.cli`` applies to every URScript pose (``plate_to_robot`` ->
  ``transform`` -> ``rotate_translation_y``). Returning the plate in the
  same world frame as the trajectory guarantees that the visual cuboid and
  the TCP trajectory stay co-located, whatever ``P_REF`` becomes.
* **Force regulation surrogate** : in ``etalement.script`` the 6 N
  ``force_mode`` regulates Z so the tool stays glued to the plate during
  every X-Y stroke. The simulator has no physics layer ; this module
  emulates the behaviour kinematically with two projection helpers :
  - :func:`snap_pose_onto_surface` (bidirectional, used during contact),
  - :func:`clamp_pose_above_surface` (unilateral, used during transit).
  :func:`apply_surface_constraint` dispatches between the two based on the
  ``in_contact`` flag emitted by :func:`ur5_sim.parsing.urscript.parse_poses`.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
from spatialmath import SE3

from design.geometry import _abs_pose, plate_to_robot
from design.params import (
    PROBE_POINTS_PLATE_MM,
    ROBOT_RX, ROBOT_RY, ROBOT_RZ,
    ROBOT_Z_SURFACE,
    SURFACE_W, SURFACE_H,
    Z_CONTACT, Z_TRANSIT,
)
from ur5_sim.config import (
    SIM_TRAJ_ROT_Y_RAD,
    SURFACE_COLOR_RGBA,
    SURFACE_THICKNESS_M,
)
from ur5_sim.kinematics.transforms import rotate_translation_y
from ur5_sim.parsing.urscript import transform, urscript_pose


def _plate_corner_world(
    corner_mm: Tuple[float, float],
    z_mm: float,
    p_anchor_old: SE3,
    p_ref: SE3,
) -> np.ndarray:
    """Take a plate-frame (x_mm, y_mm, z_mm) point through the full pipeline.

    Reproduces exactly the chain that a URScript pose follows between the
    Python generator and the simulator forward kinematics :

    1. ``plate_to_robot`` maps the plate-frame point (mm) into the robot
       base frame (m) using ``ROBOT_BASE_ROTATION_DEG`` and
       ``ROBOT_X/Y_ORIGIN``. This is the ``p_orig`` of the URScript.
    2. ``_abs_pose`` pre-bakes the ``pose_trans(P_REF, pose_inv(P_ANCHOR_OLD))``
       transformation so the .script lines carry absolute poses (the form
       emitted by ``generate_urscript`` since the absolute-pose refactor).
    3. ``cli.py`` reads those absolute poses and unconditionally applies
       :func:`ur5_sim.parsing.urscript.transform` (``p_ref * p_anchor_old.inv()``)
       on top of them. To stay co-located with the rendered trajectory, the
       surface corners must absorb the same extra multiplication.
    4. :func:`rotate_translation_y` finally remaps the translation around
       world Y so the playback aligns with the design-UI subplots.
    """
    px_m, py_m = plate_to_robot(corner_mm[0], corner_mm[1])
    pz_m = ROBOT_Z_SURFACE + z_mm / 1000.0
    pose_raw = [
        px_m, py_m, pz_m,
        ROBOT_RX, ROBOT_RY, ROBOT_RZ,
    ]
    # Etape 2 : pre-bake URScript (sortie en repere absolu).
    abs_pose = _abs_pose(pose_raw)
    pose_se3 = urscript_pose(*abs_pose)
    # Etape 3 : meme transform que cli.py applique sur chaque pose script.
    pose_tf = transform(pose_se3, p_anchor_old, p_ref)
    # Etape 4 : rotation Y de remapping pour matcher la vue 2D.
    pose_tf = rotate_translation_y(pose_tf, SIM_TRAJ_ROT_Y_RAD)
    return np.asarray(pose_tf.t, dtype=float)


def compute_surface_frame(p_anchor_old: SE3, p_ref: SE3) -> dict:
    """Build the world-frame description of the test plate.

    Returns
    -------
    dict
        ``{"center": (3,), "normal": (3,), "R_world": (3, 3),
        "corners_world": (4, 3), "w_m": float, "h_m": float}``.
        ``normal`` is oriented so that the transit poses
        (``Z_TRANSIT`` above the contact plane in plate frame) sit on
        the positive side ; the kinematic clamp and snap rely on this
        convention.
    """
    w_mm = float(SURFACE_W)
    h_mm = float(SURFACE_H)

    # Coins en repere plaque (z = Z_CONTACT = 0 mm) :
    # ordre c0=(0,0), c1=(W,0), c2=(W,H), c3=(0,H).
    corners_mm = [(0.0, 0.0), (w_mm, 0.0), (w_mm, h_mm), (0.0, h_mm)]
    corners_world = np.stack([
        _plate_corner_world(c, Z_CONTACT, p_anchor_old, p_ref)
        for c in corners_mm
    ])

    center = corners_world.mean(axis=0)

    edge_x = corners_world[1] - corners_world[0]
    edge_y = corners_world[3] - corners_world[0]
    w_m = float(np.linalg.norm(edge_x))
    h_m = float(np.linalg.norm(edge_y))
    ex = edge_x / max(w_m, 1e-12)
    ey = edge_y / max(h_m, 1e-12)
    normal = np.cross(ex, ey)
    normal /= max(float(np.linalg.norm(normal)), 1e-12)

    # Oriente la normale vers le cote des transit. Un point a
    # Z_TRANSIT mm au-dessus du plan dans le repere plaque doit avoir
    # un produit scalaire positif avec ``normal``.
    transit_world = _plate_corner_world(
        (w_mm / 2.0, h_mm / 2.0), Z_TRANSIT,
        p_anchor_old, p_ref,
    )
    if float(np.dot(normal, transit_world - center)) < 0.0:
        normal = -normal
        ey = -ey  # garde R_world droitier apres flip

    # ``R_world`` : colonnes = axes plaque exprimes en monde
    # (ex = +X plaque, ey = +Y plaque, normal = +Z plaque).
    R_world = np.column_stack([ex, ey, normal])

    return {
        "center": center,
        "normal": normal,
        "R_world": R_world,
        "corners_world": corners_world,
        "w_m": w_m,
        "h_m": h_m,
    }


def _signed_distance(t: np.ndarray, frame: dict) -> float:
    return float(np.dot(frame["normal"], t - frame["center"]))


def clamp_pose_above_surface(
    pose: SE3, frame: dict, clearance: float = 0.0,
) -> Tuple[SE3, float]:
    """Project the pose translation onto the half-space ``n.(t - O) >= clearance``.

    Returns the new SE3 and the penetration depth (positive when the
    incoming pose was below the clearance plane, 0 otherwise).
    """
    t = np.asarray(pose.t, dtype=float)
    d = _signed_distance(t, frame) - clearance
    if d >= 0.0:
        return pose, 0.0
    t_new = t - d * frame["normal"]  # d < 0 -> push up by -d along normal
    return SE3.Rt(pose.R, t_new.tolist()), float(-d)


def snap_pose_onto_surface(pose: SE3, frame: dict) -> Tuple[SE3, float]:
    """Project the pose translation orthogonally onto the surface plane.

    Returns the new SE3 and the signed offset of the incoming pose along
    the surface normal (positive = above, negative = below). The caller
    can use the magnitude to flag a pre-snap deviation that the script
    should not have produced inside a ``force_mode`` block.
    """
    t = np.asarray(pose.t, dtype=float)
    d = _signed_distance(t, frame)
    t_new = t - d * frame["normal"]
    return SE3.Rt(pose.R, t_new.tolist()), float(d)


def apply_surface_constraint(
    pose: SE3, frame: dict, in_contact: bool, clearance: float = 0.0,
) -> Tuple[SE3, str, float]:
    """Dispatch to ``snap`` (contact) or ``clamp`` (transit).

    Returns ``(pose_out, kind, depth)`` where ``kind`` is
    ``"SURFACE_DEVIATION"`` during contact (depth is signed) or
    ``"SURFACE_CLAMP"`` during transit (depth is positive penetration).
    A depth of 0 means the incoming pose was already on the right side
    of the plane (and on the plane itself in the contact case).
    """
    if in_contact:
        pose_out, signed = snap_pose_onto_surface(pose, frame)
        return pose_out, "SURFACE_DEVIATION", signed
    pose_out, depth = clamp_pose_above_surface(pose, frame, clearance)
    return pose_out, "SURFACE_CLAMP", depth


def attach_surface_to_swift(env, frame: dict):
    """Add the test plate as a thin Cuboid to the Swift scene.

    Returns the ``sg.Cuboid`` handle, or ``None`` if Swift /
    ``spatialgeometry`` is unavailable.
    """
    if env is None or frame is None:
        return None
    try:
        import spatialgeometry as sg
    except Exception as exc:  # pragma: no cover
        print(f"[surface] spatialgeometry unavailable ({exc!r}) - no surface mesh.")
        return None

    thickness = float(SURFACE_THICKNESS_M)
    # Centre cuboid demi-epaisseur sous le plan : la face superieure
    # du Cuboid coincide alors avec le plan de la surface.
    center = np.asarray(frame["center"], dtype=float)
    normal = np.asarray(frame["normal"], dtype=float)
    T = np.eye(4)
    T[:3, :3] = frame["R_world"]
    T[:3, 3] = center - 0.5 * thickness * normal

    try:
        shape = sg.Cuboid(
            scale=[frame["w_m"], frame["h_m"], thickness],
            color=SURFACE_COLOR_RGBA,
        )
    except TypeError:
        # Versions plus anciennes de spatialgeometry n'acceptent pas
        # ``color`` au constructeur ; on tente l'assignation a posteriori.
        shape = sg.Cuboid(scale=[frame["w_m"], frame["h_m"], thickness])
        if hasattr(shape, "color"):
            try:
                shape.color = SURFACE_COLOR_RGBA
            except Exception:  # pragma: no cover
                pass
    shape.T = T
    try:
        env.add(shape, collision_alpha=1.0)
    except TypeError:
        env.add(shape)
    return shape


def make_corners_xy_polygon(frame: dict) -> Optional[np.ndarray]:
    """Return the (4, 2) array of plate corners projected on world XY.

    Convenience helper for the matplotlib ``ax_xy`` overlay. The
    polygon is closed implicitly by the consumer.
    """
    if frame is None:
        return None
    corners = np.asarray(frame.get("corners_world"))
    if corners.ndim != 2 or corners.shape[1] < 2:
        return None
    return corners[:, :2].copy()


# --- 9-point test pattern (matches meshes/TestMeasure.PNG) -----------------
# Geometry extracted from meshes/TestMeasure.PNG (259x259 px, plate bbox
# 10..247 x 12..248 -> ~4.74 px/mm). The 8 outer marks lie on a circle of
# radius ~15 mm centred at (25, 25) mm in the plate frame; the centre
# mark is point #1. Each mark has diameter ~9 mm. Numbering follows the
# original figure (CW from the right): #7 at 0 deg, then #6 (45 deg),
# #5 (90 deg), #4 (135 deg), #3 (180 deg), #2 (225 deg), #9 (270 deg),
# #8 (315 deg). Angle is measured in the plate frame with +y pointing
# "down" in the original image (i.e. the same axis the design UI uses).
TEST_POINT_DIAMETER_MM = 9.0
TEST_POINT_RING_R_MM = 15.0
TEST_POINT_CENTER_MM = (25.0, 25.0)
_TEST_POINT_ANGLES_DEG = {
    1: None,    # centre
    7: 0.0,
    6: 45.0,
    5: 90.0,
    4: 135.0,
    3: 180.0,
    2: 225.0,
    9: 270.0,
    8: 315.0,
}


def test_points_plate_mm() -> list[tuple[int, float, float]]:
    """Return ``(label, x_mm, y_mm)`` for the 9 measurement marks in plate frame."""
    cx, cy = TEST_POINT_CENTER_MM
    r = TEST_POINT_RING_R_MM
    pts: list[tuple[int, float, float]] = []
    for label, deg in _TEST_POINT_ANGLES_DEG.items():
        if deg is None:
            pts.append((label, cx, cy))
        else:
            a = float(np.deg2rad(deg))
            pts.append((label, cx + r * float(np.cos(a)), cy + r * float(np.sin(a))))
    return pts


def compute_test_points_world(
    p_anchor_old: SE3, p_ref: SE3,
) -> list[tuple[int, np.ndarray]]:
    """Map the 9 marks through the plate->world chain used by the trajectory.

    Returns ``[(label, world_xyz), ...]``. ``world_xyz`` lives in the same
    frame as ``compute_surface_frame`` corners, so the marks plot exactly
    on the plate polygon drawn by ``make_corners_xy_polygon``.
    """
    out: list[tuple[int, np.ndarray]] = []
    for label, x_mm, y_mm in test_points_plate_mm():
        xyz = _plate_corner_world(
            (x_mm, y_mm), Z_CONTACT, p_anchor_old, p_ref,
        )
        out.append((label, xyz))
    return out


def compute_probe_points_world(
    p_anchor_old: SE3, p_ref: SE3,
) -> list[np.ndarray]:
    """Map the 3 surface-probe points through the plate->world chain.

    Returns ``[world_xyz, ...]`` in the same frame as
    :func:`compute_test_points_world`, so the probe markers plot exactly on the
    plate polygon drawn by :func:`make_corners_xy_polygon`. The points are
    ``design.params.PROBE_POINTS_PLATE_MM`` — the 3 poses the on-robot
    ``probe_surface_plane()`` touches to measure the plate plane and tilt.
    """
    return [
        _plate_corner_world((x_mm, y_mm), Z_CONTACT, p_anchor_old, p_ref)
        for (x_mm, y_mm) in PROBE_POINTS_PLATE_MM
    ]


def test_point_radius_world_m(frame: dict) -> float:
    """Diameter of a single mark (mm) converted to metres in world frame.

    Rotations and translations preserve length, so the 9 mm plate-frame
    diameter is the same in world frame; the helper just centralises the
    constant for the matplotlib overlay.
    """
    _ = frame  # signature kept for symmetry with make_corners_xy_polygon
    return 0.5 * TEST_POINT_DIAMETER_MM / 1000.0
