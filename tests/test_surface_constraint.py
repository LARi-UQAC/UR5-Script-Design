"""Tests for the test-surface frame + kinematic force surrogate.

Exercises :
* :func:`compute_surface_frame` returns a frame whose normal is oriented
  toward the transit half-space and whose extents match
  ``ur5_etalementv6.SURFACE_W``/``SURFACE_H``.
* :func:`snap_pose_onto_surface` projects onto the plane and reports the
  signed deviation.
* :func:`clamp_pose_above_surface` only pushes upward and reports the
  penetration depth.
* :func:`apply_surface_constraint` dispatches between snap (contact) and
  clamp (transit) and is idempotent on poses already on/above the plane.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np
from spatialmath import SE3

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ur5_sim.config import P_ANCHOR_OLD_RAW, P_REF_RAW
from ur5_sim.parsing.urscript import urscript_pose
from ur5_sim.visualization.surface import (
    apply_surface_constraint,
    clamp_pose_above_surface,
    compute_surface_frame,
    snap_pose_onto_surface,
)


class SurfaceFrameTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.anchor = urscript_pose(*P_ANCHOR_OLD_RAW)
        cls.ref = urscript_pose(*P_REF_RAW)
        cls.frame = compute_surface_frame(cls.anchor, cls.ref)

    def test_frame_shape_and_extents(self):
        from design.params import SURFACE_W, SURFACE_H
        self.assertAlmostEqual(self.frame["w_m"], SURFACE_W / 1000.0, places=6)
        self.assertAlmostEqual(self.frame["h_m"], SURFACE_H / 1000.0, places=6)
        self.assertEqual(self.frame["corners_world"].shape, (4, 3))
        self.assertAlmostEqual(float(np.linalg.norm(self.frame["normal"])), 1.0, places=9)

    def test_normal_points_toward_transit(self):
        # Un point a Z_TRANSIT mm au-dessus du plan plaque doit avoir un
        # produit scalaire positif avec la normale du repere surface.
        from design.geometry import _abs_pose, plate_to_robot
        from design.params import (
            ROBOT_RX, ROBOT_RY, ROBOT_RZ, ROBOT_Z_SURFACE,
            SURFACE_W, SURFACE_H, Z_TRANSIT,
        )
        from ur5_sim.config import SIM_TRAJ_ROT_Y_RAD
        from ur5_sim.kinematics.transforms import rotate_translation_y
        from ur5_sim.parsing.urscript import transform

        cx_m, cy_m = plate_to_robot(SURFACE_W / 2.0, SURFACE_H / 2.0)
        transit_raw = [
            cx_m, cy_m, ROBOT_Z_SURFACE + Z_TRANSIT / 1000.0,
            ROBOT_RX, ROBOT_RY, ROBOT_RZ,
        ]
        transit_abs = _abs_pose(transit_raw)
        transit_tf = rotate_translation_y(
            transform(urscript_pose(*transit_abs), self.anchor, self.ref),
            SIM_TRAJ_ROT_Y_RAD,
        )
        offset = np.asarray(transit_tf.t) - self.frame["center"]
        self.assertGreater(float(np.dot(self.frame["normal"], offset)), 0.0)

    def test_snap_projects_pose_onto_plane(self):
        # Pose synthetique 5 mm au-dessus du centre du plan.
        above = SE3.Rt(np.eye(3), self.frame["center"] + 0.005 * self.frame["normal"])
        snapped, signed = snap_pose_onto_surface(above, self.frame)
        self.assertAlmostEqual(signed, 0.005, places=6)
        residual = float(np.dot(
            self.frame["normal"],
            np.asarray(snapped.t) - self.frame["center"],
        ))
        self.assertAlmostEqual(residual, 0.0, places=9)

    def test_snap_handles_negative_offset(self):
        below = SE3.Rt(np.eye(3), self.frame["center"] - 0.002 * self.frame["normal"])
        snapped, signed = snap_pose_onto_surface(below, self.frame)
        self.assertAlmostEqual(signed, -0.002, places=6)
        residual = float(np.dot(
            self.frame["normal"],
            np.asarray(snapped.t) - self.frame["center"],
        ))
        self.assertAlmostEqual(residual, 0.0, places=9)

    def test_clamp_only_pushes_up(self):
        below = SE3.Rt(np.eye(3), self.frame["center"] - 0.001 * self.frame["normal"])
        clamped, depth = clamp_pose_above_surface(below, self.frame, 0.0)
        self.assertAlmostEqual(depth, 0.001, places=6)
        residual = float(np.dot(
            self.frame["normal"],
            np.asarray(clamped.t) - self.frame["center"],
        ))
        self.assertAlmostEqual(residual, 0.0, places=9)

    def test_clamp_no_change_when_above(self):
        above = SE3.Rt(np.eye(3), self.frame["center"] + 0.010 * self.frame["normal"])
        clamped, depth = clamp_pose_above_surface(above, self.frame, 0.0)
        self.assertEqual(depth, 0.0)
        self.assertTrue(np.allclose(np.asarray(clamped.t), np.asarray(above.t)))

    def test_apply_surface_constraint_dispatch(self):
        # Contact pose 3 mm au-dessus du plan -> snap + SURFACE_DEVIATION.
        contact_pose = SE3.Rt(
            np.eye(3), self.frame["center"] + 0.003 * self.frame["normal"],
        )
        _out, kind, depth = apply_surface_constraint(
            contact_pose, self.frame, in_contact=True, clearance=0.0,
        )
        self.assertEqual(kind, "SURFACE_DEVIATION")
        self.assertAlmostEqual(depth, 0.003, places=6)

        # Transit pose 4 mm sous le plan -> clamp + SURFACE_CLAMP.
        transit_pose = SE3.Rt(
            np.eye(3), self.frame["center"] - 0.004 * self.frame["normal"],
        )
        _out2, kind2, depth2 = apply_surface_constraint(
            transit_pose, self.frame, in_contact=False, clearance=0.0,
        )
        self.assertEqual(kind2, "SURFACE_CLAMP")
        self.assertAlmostEqual(depth2, 0.004, places=6)


if __name__ == "__main__":
    unittest.main()
