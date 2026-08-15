"""
tests/test_sim_reads_settings.py - Le simulateur lit les memes reglages que
l'exporteur (phase 3 du plan docs/superpower/plans/plan_variables_UI.md).

Sans cela, l'operateur change une valeur dans l'interface, exporte, puis
`python -m ur5_sim --check` valide encore avec les defauts : les deux
processus doivent lire la meme source.
"""

from __future__ import annotations

import importlib
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import design.params as params
import ur5_sim.config as sim_config
from design.settings import Settings, set_settings


class SimConfigTests(unittest.TestCase):

    def tearDown(self):
        set_settings(Settings())
        importlib.reload(sim_config)

    def _reload_with(self, settings: Settings):
        set_settings(settings)
        return importlib.reload(sim_config)

    def test_defaults_match_params_when_nothing_is_overridden(self):
        cfg = self._reload_with(Settings())
        self.assertAlmostEqual(cfg.FORCE_Z_TARGET_N, params.FORCE_Z_TARGET)
        self.assertAlmostEqual(cfg.SURFACE_FORCE_TARGET_DEPTH_M,
                               params.FORCE_CONTACT_DEPTH)
        self.assertAlmostEqual(cfg.URSCRIPT_MAX_TCP_SPEED_MPS,
                               params.URSCRIPT_MAX_TCP_SPEED)
        self.assertAlmostEqual(cfg.TCP_TOOL_Z_M, params.TCP_Z / 1000.0)

    def test_force_target_override_reaches_the_simulator(self):
        s = Settings()
        s.force_z_target = 9.0
        self.assertAlmostEqual(self._reload_with(s).FORCE_Z_TARGET_N, 9.0)

    def test_contact_depth_override_reaches_the_deviation_filter(self):
        # Le filtre des ecarts de recontact se cale sur cette profondeur ;
        # s'il gardait le defaut, chaque cycle produirait un faux
        # SURFACE_DEVIATION.
        s = Settings()
        s.force_contact_depth = 0.009
        self.assertAlmostEqual(
            self._reload_with(s).SURFACE_FORCE_TARGET_DEPTH_M, 0.009)

    def test_calibration_override_reaches_the_anchor(self):
        s = Settings()
        s.calibration_unlocked = True
        s.p_ref = [-0.02, 0.61, 0.05, 3.14159, 0.0, 0.0]
        self.assertAlmostEqual(self._reload_with(s).P_REF_RAW[0], -0.02)

    def test_tcp_length_override_reaches_the_tool_offset(self):
        s = Settings()
        s.calibration_unlocked = True
        s.tcp_z = 300.0
        self.assertAlmostEqual(self._reload_with(s).TCP_TOOL_Z_M, 0.300)

    def test_settings_summary_names_the_source_and_the_deviations(self):
        s = Settings()
        s.force_z_target = 8.0
        cfg = self._reload_with(s)
        summary = cfg.settings_summary()
        self.assertIn("defauts", summary)
        self.assertIn("force_z_target", summary)
        self.assertIn("8", summary)

    def test_settings_summary_is_short_when_nothing_deviates(self):
        cfg = self._reload_with(Settings())
        self.assertIn("aucun ecart", cfg.settings_summary())


if __name__ == "__main__":
    unittest.main()
