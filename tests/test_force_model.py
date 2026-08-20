"""
Tests for the FT-300 force surrogate.

The model is a named surrogate driven by the simulator's own penetration
depth, NOT a physics simulation and NOT measured data. These tests pin its
stated behavior, not the realism of its parameters.
"""

import unittest

from ur5_sim.config import FORCE_Z_TARGET_N
from ur5_sim.force_model import ForceModel


def _model(noise: float = 0.0) -> ForceModel:
    """A noiseless model by default, so behavior is asserted without slack."""
    return ForceModel(
        stiffness_n_per_m=4000.0,
        tau_s=0.05,
        friction_mu=0.8,
        noise_n=noise,
        seed=20260814,
        target_n=FORCE_Z_TARGET_N,
    )


class ForceModelTests(unittest.TestCase):

    DT = 1.0 / 125.0

    def _settle(self, model: ForceModel, seconds: float, **kwargs) -> tuple:
        out = (0.0, 0.0, 0.0)
        for _ in range(int(seconds / self.DT)):
            out = model.step(dt=self.DT, **kwargs)
        return out

    def test_transit_force_is_zero(self) -> None:
        m = _model()
        fx, fy, fz = self._settle(
            m, 1.0, in_contact=False, penetration_m=0.0, vx=0.05, vy=0.0
        )
        self.assertAlmostEqual(fz, 0.0, places=3)
        self.assertAlmostEqual(fx, 0.0, places=3)
        self.assertAlmostEqual(fy, 0.0, places=3)

    def test_contact_converges_to_the_force_target(self) -> None:
        m = _model()
        _, _, fz = self._settle(
            m, 1.0, in_contact=True, penetration_m=0.0, vx=0.0, vy=0.0
        )
        # Negative while pressing into the plate, matching the monitor's own
        # sample row (ForceZ = -6.012345).
        self.assertAlmostEqual(fz, -FORCE_Z_TARGET_N, places=3)

    def test_penetration_adds_a_stiffness_transient(self) -> None:
        m = _model()
        _, _, fz = self._settle(
            m, 1.0, in_contact=True, penetration_m=0.001, vx=0.0, vy=0.0
        )
        # 1 mm deeper at 4000 N/m is 4 N on top of the 6 N target.
        self.assertAlmostEqual(fz, -(FORCE_Z_TARGET_N + 4.0), places=3)

    def test_response_is_gradual_not_instant(self) -> None:
        m = _model()
        _, _, after_one_step = m.step(
            dt=self.DT, in_contact=True, penetration_m=0.0, vx=0.0, vy=0.0
        )
        self.assertLess(abs(after_one_step), FORCE_Z_TARGET_N)
        self.assertGreater(abs(after_one_step), 0.0)

    def test_friction_opposes_travel(self) -> None:
        m = _model()
        fx, fy, fz = self._settle(
            m, 1.0, in_contact=True, penetration_m=0.0, vx=0.05, vy=0.0
        )
        self.assertLess(fx, 0.0)                     # opposes +x travel
        self.assertAlmostEqual(fy, 0.0, places=6)
        self.assertAlmostEqual(fx, -0.8 * abs(fz), places=3)

    def test_friction_follows_the_travel_direction(self) -> None:
        m = _model()
        fx, fy, _ = self._settle(
            m, 1.0, in_contact=True, penetration_m=0.0, vx=0.0, vy=-0.05
        )
        self.assertAlmostEqual(fx, 0.0, places=6)
        self.assertGreater(fy, 0.0)                  # opposes -y travel

    def test_no_friction_at_rest(self) -> None:
        m = _model()
        fx, fy, _ = self._settle(
            m, 1.0, in_contact=True, penetration_m=0.0, vx=0.0, vy=0.0
        )
        self.assertAlmostEqual(fx, 0.0, places=9)
        self.assertAlmostEqual(fy, 0.0, places=9)

    def test_noise_is_present_and_bounded(self) -> None:
        m = _model(noise=0.05)
        samples = [
            m.step(dt=self.DT, in_contact=False, penetration_m=0.0, vx=0.0, vy=0.0)[2]
            for _ in range(500)
        ]
        self.assertGreater(len(set(samples)), 400)   # actually varying
        self.assertLess(max(abs(s) for s in samples), 0.5)

    def test_same_seed_gives_the_same_sequence(self) -> None:
        a = _model(noise=0.05)
        b = _model(noise=0.05)
        for _ in range(50):
            self.assertEqual(
                a.step(dt=self.DT, in_contact=True, penetration_m=0.0, vx=0.01, vy=0.0),
                b.step(dt=self.DT, in_contact=True, penetration_m=0.0, vx=0.01, vy=0.0),
            )


if __name__ == "__main__":
    unittest.main()
