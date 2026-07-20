"""Tests for the force-target depth filter in ``ur5_sim.cli``.

The URScript generator emits a deliberate ``pose_contact_deep`` waypoint
``FORCE_CONTACT_DEPTH = 5 mm`` below the nominal plate so the on-robot
force regulator has authority to stop the descent. The kinematic surrogate
has no force model and would log every recontact as a SURFACE_DEVIATION
of -5 mm. The cli filter recognises this expected depth and drops it.

A genuine bug (different depth or wrong sign) must still be reported.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ur5_sim.cli import _is_force_target_depth
from ur5_sim.config import (
    SURFACE_FORCE_TARGET_DEPTH_M,
    SURFACE_FORCE_TARGET_TOL_M,
)


class ForceTargetFilterTests(unittest.TestCase):
    def test_recontact_descent_is_filtered(self) -> None:
        # Pose exactly at -FORCE_CONTACT_DEPTH (m) is the force regulator
        # contract and must be silenced.
        self.assertTrue(
            _is_force_target_depth("SURFACE_DEVIATION",
                                   -SURFACE_FORCE_TARGET_DEPTH_M),
        )

    def test_within_tolerance_is_filtered(self) -> None:
        # +/- 50 um around the nominal force depth still considered the
        # deliberate target (rounding noise from the .script).
        self.assertTrue(
            _is_force_target_depth(
                "SURFACE_DEVIATION",
                -SURFACE_FORCE_TARGET_DEPTH_M + 0.5 * SURFACE_FORCE_TARGET_TOL_M,
            ),
        )
        self.assertTrue(
            _is_force_target_depth(
                "SURFACE_DEVIATION",
                -SURFACE_FORCE_TARGET_DEPTH_M - 0.5 * SURFACE_FORCE_TARGET_TOL_M,
            ),
        )

    def test_zero_deviation_is_not_filtered(self) -> None:
        # A pose exactly on the plane is not a force target ; the filter
        # only swallows the deliberate ``-FORCE_CONTACT_DEPTH`` depth.
        self.assertFalse(_is_force_target_depth("SURFACE_DEVIATION", 0.0))

    def test_genuine_deeper_bug_is_not_filtered(self) -> None:
        # Pose 10 mm below the plane (2x the contract) is a real bug.
        self.assertFalse(
            _is_force_target_depth("SURFACE_DEVIATION",
                                   -2.0 * SURFACE_FORCE_TARGET_DEPTH_M),
        )

    def test_above_plate_is_not_filtered(self) -> None:
        # Positive depth (waypoint above plate during contact) is a real
        # bug: contact stroke must lie on the plane, not above it.
        self.assertFalse(
            _is_force_target_depth("SURFACE_DEVIATION",
                                   +SURFACE_FORCE_TARGET_DEPTH_M),
        )

    def test_surface_clamp_kind_passes_through(self) -> None:
        # Transit clamp events are independent of the force-mode contract.
        self.assertFalse(
            _is_force_target_depth("SURFACE_CLAMP",
                                   -SURFACE_FORCE_TARGET_DEPTH_M),
        )


if __name__ == "__main__":
    unittest.main()
