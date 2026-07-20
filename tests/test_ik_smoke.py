"""End-to-end smoke test : parse a handful of poses, run IK, assert no failure."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np
import roboticstoolbox as rtb

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ur5_sim.config import P_ANCHOR_OLD_RAW, P_REF_RAW, SCRIPT_PATH
from ur5_sim.kinematics.ik import run_ik
from ur5_sim.parsing.urscript import parse_poses, transform, urscript_pose


def _first_n_poses(n: int):
    anchor = urscript_pose(*P_ANCHOR_OLD_RAW)
    ref = urscript_pose(*P_REF_RAW)
    parsed = parse_poses(SCRIPT_PATH)[:n]
    poses_xform = [
        (lineno, transform(urscript_pose(*pose), anchor, ref))
        for lineno, pose, _cycle, _in_contact in parsed
    ]
    return poses_xform


class IkSmokeTests(unittest.TestCase):
    def test_first_ten_poses_solve_under_target_anchor(self):
        poses_xform = _first_n_poses(10)
        self.assertEqual(len(poses_xform), 10)
        robot = rtb.models.UR5()
        trajectory, failures = run_ik(robot, poses_xform, robot.qr)
        self.assertEqual(len(trajectory), 10)
        self.assertEqual(failures, [])

    def test_progress_callback_fires_once_per_pose(self):
        # The viewer's background recompute relies on run_ik calling progress()
        # exactly once per pose with a monotone (done, total) and on it not
        # perturbing the solved trajectory.
        poses_xform = _first_n_poses(10)
        robot = rtb.models.UR5()
        calls: list[tuple[int, int]] = []
        traj_p, _ = run_ik(
            robot, poses_xform, robot.qr,
            progress=lambda done, total: calls.append((done, total)),
        )
        self.assertEqual(len(calls), len(poses_xform))
        self.assertEqual([d for d, _ in calls], list(range(1, len(poses_xform) + 1)))
        self.assertTrue(all(t == len(poses_xform) for _, t in calls))

        traj_n, _ = run_ik(robot, poses_xform, robot.qr)
        self.assertEqual(len(traj_p), len(traj_n))
        for qp, qn in zip(traj_p, traj_n):
            self.assertTrue(np.allclose(qp, qn))


if __name__ == "__main__":
    unittest.main()
