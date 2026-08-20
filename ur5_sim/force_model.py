"""
FT-300 force surrogate for the RTDE emulator.

The simulator has no physics layer (ARCHITECTURE.md section 5). This module
does not add one. It synthesises a force signal from state the simulator
already computes - contact flag, penetration depth, TCP velocity - so the
result responds to real trajectory events, including the deliberate 5 mm
recontact overshoot, rather than replaying a canned waveform.

WARNING: the parameters are plausible, not measured. Output resembles FT-300
data; it is not FT-300 data. See ur5_sim/config.py FORCE_MODEL_*.
"""

from __future__ import annotations

import math
import random


class ForceModel:
    """
    Contact regulation plus Coulomb friction plus sensor noise.

    In contact, Fz relaxes toward the regulated target with a first-order
    response, offset by a stiffness term proportional to how far the tool
    actually is below the plane. In transit it relaxes toward zero. The
    tangential components oppose the direction of travel, scaled by the normal
    force. Sign convention: Fz is negative while pressing into the plate.
    """

    def __init__(
        self,
        stiffness_n_per_m: float,
        tau_s: float,
        friction_mu: float,
        noise_n: float,
        seed: int,
        target_n: float,
    ) -> None:
        self._stiffness = float(stiffness_n_per_m)
        self._tau = max(float(tau_s), 1e-9)
        self._mu = float(friction_mu)
        self._noise = float(noise_n)
        self._target = float(target_n)
        self._rng = random.Random(seed)
        self._fz = 0.0

    def step(
        self,
        dt: float,
        in_contact: bool,
        penetration_m: float,
        vx: float,
        vy: float,
    ) -> tuple[float, float, float]:
        """
        ----------------------------------------------------------------------
        Purpose:
            Advance the model by one emitter tick and return the synthesised
            force triple.

        Inputs:
            dt (float): tick length, seconds.
            in_contact (bool): True between force_mode and end_force_mode.
            penetration_m (float): depth below the plane, positive downward.
            vx (float): TCP velocity along world X, m/s.
            vy (float): TCP velocity along world Y, m/s.

        Outputs:
            force (tuple[float, float, float]): (Fx, Fy, Fz) in newtons.
        ----------------------------------------------------------------------
        """
        if in_contact:
            target = -(self._target + self._stiffness * penetration_m)
        else:
            target = 0.0

        # Exponential approach, exact for the step response of a first-order
        # lag, so the result does not depend on the tick length.
        alpha = 1.0 - math.exp(-dt / self._tau)
        self._fz += (target - self._fz) * alpha

        speed = math.hypot(vx, vy)
        if speed > 1e-9:
            tangential = -self._mu * abs(self._fz)
            fx = tangential * (vx / speed)
            fy = tangential * (vy / speed)
        else:
            fx = 0.0
            fy = 0.0

        if self._noise > 0.0:
            fx += self._rng.gauss(0.0, self._noise)
            fy += self._rng.gauss(0.0, self._noise)
            return (fx, fy, self._fz + self._rng.gauss(0.0, self._noise))
        return (fx, fy, self._fz)
