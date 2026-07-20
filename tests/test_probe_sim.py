"""Tests for the surface-probe simulation — DESACTIVES (rework futur).

DESACTIVE - A REVOIR. Le sondage 3 points (``probe_surface_plane``) s'est revele
INCORRECT : fixe en Z, il ne gere ni la rotation de la plaque ni une hauteur de
plaque inconnue (dependante de la manipulation de l'operateur). L'export URScript
a ete bascule sur un sondage Z 1 point (``probe_surface_z``).

Le simulateur 3 points (``ur5_sim/probe.py``), ses helpers de parsing
(``parse_probe_blocks`` / ``parse_nhat``) et les tests ci-dessous se rapportent
tous a ce processus incorrect et sont PARQUES pour un rework futur. Le code de
test est commente (preserve, non supprime) dans la chaine inerte ci-dessous.

Lors du rework du sondage : reactiver ces tests (sortir le code de la chaine
inerte), remettre ``SIM_PROBE_ENABLE = True`` dans ``ur5_sim/config.py``, et
adapter ``ProbeParserTests`` au nouveau format de sondage.
"""

# =============================================================================
# 3-POINT PROBE TESTS : DESACTIVES - A REVOIR (rework futur).
# Le sondage 3 points est incorrect (voir docstring + ur5_sim/probe.py).
# Code de test conserve ci-dessous en chaine litterale inerte (non execute) :
# aucun TestCase n'est collecte par unittest tant que ce bloc reste une chaine.
# =============================================================================
r'''
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ur5_sim.config import SCRIPT_PATH, SIM_PROBE_RESIDUAL_TOL_M
from ur5_sim.parsing.urscript import (
    parse_nhat,
    parse_nominal_frame,
    parse_probe_blocks,
)
from ur5_sim.probe import (
    apply_correction,
    build_virtual_plate,
    compute_meas_frame,
    signed_distance_to_plane,
    simulate_probe_descent,
)


class ProbeParserTests(unittest.TestCase):
    # Le sondage 3 points est DESACTIVE (remplace par probe_surface_z, sondage Z
    # 1 point ; voir design/export.py). Le script genere n'emet donc plus de blocs
    # probe 3 points ni de globals NHAT_*. Ces tests verifient cet etat ; le
    # modele simulateur 3 points (classes ci-dessous) reste valide et teste.
    def test_no_three_point_blocks_when_disabled(self) -> None:
        # Sondage 3 points desactive -> aucun bloc probe dans le script genere.
        blocks = parse_probe_blocks(SCRIPT_PATH)
        self.assertEqual(blocks, [])

    def test_nominal_frame_present_nhat_absent(self) -> None:
        nominal = parse_nominal_frame(SCRIPT_PATH)
        nhat = parse_nhat(SCRIPT_PATH)
        # NOMINAL_FRAME reste requis par les cycles (apply_correction + force_mode).
        self.assertIsNotNone(nominal)
        # NHAT_* retire avec le sondage 3 points.
        self.assertIsNone(nhat)


class VirtualPlateTests(unittest.TestCase):
    def setUp(self) -> None:
        self.nominal = (0.0, 0.0, 0.0, math.pi, 0.0, 0.0)
        self.nhat = (0.0, 0.0, 1.0)

    def test_zero_offset_zero_tilt_matches_nominal(self) -> None:
        plate = build_virtual_plate(self.nominal, self.nhat, 0.0, 0.0, 0.0)
        np.testing.assert_allclose(plate["origin"], (0.0, 0.0, 0.0), atol=1e-12)
        np.testing.assert_allclose(plate["normal"], (0.0, 0.0, 1.0), atol=1e-12)

    def test_dz_shifts_origin_along_nhat(self) -> None:
        plate = build_virtual_plate(self.nominal, self.nhat, 0.005, 0.0, 0.0)
        np.testing.assert_allclose(plate["origin"], (0.0, 0.0, 0.005), atol=1e-12)

    def test_tilt_y_rotates_normal(self) -> None:
        plate = build_virtual_plate(self.nominal, self.nhat, 0.0, 0.0, 0.1)
        self.assertAlmostEqual(plate["normal"][0], math.sin(0.1), places=6)
        self.assertAlmostEqual(plate["normal"][2], math.cos(0.1), places=6)


class ProbeDescentTests(unittest.TestCase):
    def test_descent_intersects_plane(self) -> None:
        plate = {"origin": np.array([0.0, 0.0, 0.0]),
                 "normal": np.array([0.0, 0.0, 1.0])}
        approach = (0.005, 0.005, 0.030, math.pi, 0.0, 0.0)
        floor = (0.005, 0.005, -0.010, math.pi, 0.0, 0.0)
        cp = simulate_probe_descent(approach, floor, plate)
        self.assertIsNotNone(cp)
        np.testing.assert_allclose(cp[:3], (0.005, 0.005, 0.0), atol=1e-12)
        self.assertAlmostEqual(cp[3], math.pi, places=6)

    def test_descent_misses_returns_none(self) -> None:
        # Plate well below the floor reach.
        plate = {"origin": np.array([0.0, 0.0, -0.5]),
                 "normal": np.array([0.0, 0.0, 1.0])}
        approach = (0.0, 0.0, 0.030, math.pi, 0.0, 0.0)
        floor = (0.0, 0.0, -0.010, math.pi, 0.0, 0.0)
        self.assertIsNone(simulate_probe_descent(approach, floor, plate))


class MeasFrameTests(unittest.TestCase):
    def setUp(self) -> None:
        self.nominal = (0.0, 0.0, 0.0, math.pi, 0.0, 0.0)
        self.nhat = (0.0, 0.0, 1.0)

    def test_identity_when_plate_matches_nominal(self) -> None:
        plate = build_virtual_plate(self.nominal, self.nhat, 0.0, 0.0, 0.0)
        approach_floor = [
            ((0.005, 0.005, 0.030, math.pi, 0.0, 0.0),
             (0.005, 0.005, -0.010, math.pi, 0.0, 0.0)),
            ((0.045, 0.005, 0.030, math.pi, 0.0, 0.0),
             (0.045, 0.005, -0.010, math.pi, 0.0, 0.0)),
            ((0.025, 0.045, 0.030, math.pi, 0.0, 0.0),
             (0.025, 0.045, -0.010, math.pi, 0.0, 0.0)),
        ]
        cps = [simulate_probe_descent(a, f, plate) for a, f in approach_floor]
        meas, tilt = compute_meas_frame(cps[0], cps[1], cps[2], self.nhat, self.nominal)
        self.assertAlmostEqual(tilt, 0.0, places=9)
        # MEAS_FRAME equals NOMINAL_FRAME (translation = origin, rotation = pi/X).
        np.testing.assert_allclose(meas.t, (0.005, 0.005, 0.0), atol=1e-12)

    def test_tilt_reconstructed_within_machine_precision(self) -> None:
        # Real plate tilted 2 deg around world Y, no dz.
        tilt_target = math.radians(2.0)
        plate = build_virtual_plate(self.nominal, self.nhat, 0.0, 0.0, tilt_target)
        approach_floor = [
            ((0.005, 0.005, 0.030, math.pi, 0.0, 0.0),
             (0.005, 0.005, -0.010, math.pi, 0.0, 0.0)),
            ((0.045, 0.005, 0.030, math.pi, 0.0, 0.0),
             (0.045, 0.005, -0.010, math.pi, 0.0, 0.0)),
            ((0.025, 0.045, 0.030, math.pi, 0.0, 0.0),
             (0.025, 0.045, -0.010, math.pi, 0.0, 0.0)),
        ]
        cps = [simulate_probe_descent(a, f, plate) for a, f in approach_floor]
        for cp in cps:
            self.assertIsNotNone(cp)
        _meas, tilt_reco = compute_meas_frame(
            cps[0], cps[1], cps[2], self.nhat, self.nominal,
        )
        self.assertAlmostEqual(tilt_reco, tilt_target, places=6)


class ApplyCorrectionTests(unittest.TestCase):
    def test_correction_brings_contact_waypoint_onto_tilted_plate(self) -> None:
        nominal = (0.0, 0.0, 0.0, math.pi, 0.0, 0.0)
        nhat = (0.0, 0.0, 1.0)
        tilt_y = math.radians(1.5)
        dz = 0.003  # 3 mm plate higher than nominal
        plate = build_virtual_plate(nominal, nhat, dz, 0.0, tilt_y)

        approach_floor = [
            ((0.005, 0.005, 0.030, math.pi, 0.0, 0.0),
             (0.005, 0.005, -0.010, math.pi, 0.0, 0.0)),
            ((0.045, 0.005, 0.030, math.pi, 0.0, 0.0),
             (0.045, 0.005, -0.010, math.pi, 0.0, 0.0)),
            ((0.025, 0.045, 0.030, math.pi, 0.0, 0.0),
             (0.025, 0.045, -0.010, math.pi, 0.0, 0.0)),
        ]
        cps = [simulate_probe_descent(a, f, plate) for a, f in approach_floor]
        meas, _tilt = compute_meas_frame(cps[0], cps[1], cps[2], nhat, nominal)

        # Waypoint a Z = 0 (plate-frame contact) en monde, en plein milieu.
        waypoint = (0.025, 0.025, 0.0, math.pi, 0.0, 0.0)
        corrected = apply_correction(waypoint, meas, nominal)
        residual = signed_distance_to_plane(corrected.t, plate)
        self.assertLess(abs(residual), SIM_PROBE_RESIDUAL_TOL_M)


if __name__ == "__main__":
    unittest.main()
'''
