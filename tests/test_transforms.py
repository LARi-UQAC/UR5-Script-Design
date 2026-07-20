"""Sanity checks on the numpy SE(3) helpers."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ur5_sim.kinematics.transforms import rpy_to_R, se3


class TransformsTests(unittest.TestCase):
    def test_identity_rotation(self):
        R = rpy_to_R((0.0, 0.0, 0.0))
        self.assertTrue(np.allclose(R, np.eye(3)))

    def test_pi_about_x_flips_y_and_z(self):
        R = rpy_to_R((math.pi, 0.0, 0.0))
        v = np.array([1.0, 2.0, 3.0])
        out = R @ v
        self.assertTrue(np.allclose(out, [1.0, -2.0, -3.0], atol=1e-9))

    def test_se3_translation_only(self):
        T = se3([0.1, 0.2, 0.3])
        self.assertTrue(np.allclose(T[:3, 3], [0.1, 0.2, 0.3]))
        self.assertTrue(np.allclose(T[:3, :3], np.eye(3)))


if __name__ == "__main__":
    unittest.main()
