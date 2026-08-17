"""
tests/test_sim_reads_settings.py - Le simulateur lit les memes reglages que
l'exporteur (phase 3 du plan docs/superpower/plans/plan_variables_UI.md).

Sans cela, l'operateur change une valeur dans l'interface, exporte, puis
`python -m ur5_sim --check` valide encore avec les defauts : les deux
processus doivent lire la meme source.
"""

from __future__ import annotations

import importlib
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import design.params as params
import design.settings as settings_module
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

    # -- F4 (docs/superpower/plans/erreur_hors_datalogger.md): the freeze at
    # import is intentional and documented at ur5_sim/config.py:16-21 (one
    # process, one read, _READ_AT published by settings_summary()). These
    # three tests pin that behavior instead of assuming it, so a future
    # change that quietly makes config.py reactive again is caught here
    # rather than discovered on a robot that validated against stale bounds.

    def test_reload_settings_does_not_move_already_bound_constants(self):
        """design.settings.reload_settings() replaces the *active* Settings
        object for future get_settings() callers. It must have zero effect
        on ur5_sim.config's already-derived constants, since nothing
        re-executes that module's top level - only importlib.reload(...)
        does, which is exactly what the sibling tests above exercise."""
        cfg = self._reload_with(Settings())
        frozen = cfg.FORCE_Z_TARGET_N
        with tempfile.TemporaryDirectory() as tmp:
            other_path = Path(tmp) / "other_settings.json"
            other_path.write_text(json.dumps({
                "overrides": {"force_z_target": frozen + 11.0},
            }), encoding="utf-8")
            settings_module.reload_settings(other_path)
            self.assertAlmostEqual(
                cfg.FORCE_Z_TARGET_N, frozen,
                msg=(
                    "ur5_sim.config.FORCE_Z_TARGET_N moved after "
                    "design.settings.reload_settings() ran against a "
                    "different file with no importlib.reload(ur5_sim.config) "
                    "in between. config.py reads get_settings() exactly once "
                    "at import (see its own module comment); this constant "
                    "moving means the one-read-per-process design silently "
                    "stopped holding, and every sibling constant derived the "
                    "same way (SURFACE_FORCE_TARGET_DEPTH_M, "
                    "URSCRIPT_MAX_TCP_SPEED_MPS, TCP_TOOL_Z_M, ...) is now "
                    "suspect too."
                ),
            )

    def test_settings_summary_reports_read_timestamp_and_source_path(self):
        with tempfile.TemporaryDirectory() as tmp:
            settings_path = Path(tmp) / "etalement_settings.json"
            settings_path.write_text(json.dumps({
                "overrides": {"force_z_target": 7.5},
            }), encoding="utf-8")
            cfg = self._reload_with(Settings.from_file(settings_path))
            summary = cfg.settings_summary()
            self.assertIn(
                str(settings_path), summary,
                "settings_summary() must name the source file actually "
                "used (Settings.source), not just 'defauts' or a bare "
                "field name - a --check report is worthless if it cannot "
                "say which file it validated against")
            self.assertRegex(
                summary, r"lus a \d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}",
                "settings_summary() must publish the read timestamp "
                "(_READ_AT) so the --check report can state when the "
                "values it validated against were actually read, since the "
                "operator may have edited the settings file since")

    def test_settings_written_before_reload_is_reflected_after_is_not(self):
        """The two halves of F4's third pinned case, side by side: a
        settings source active BEFORE ur5_sim.config's module body runs
        (the reload below stands in for that first import) is reflected;
        one that becomes active AFTERWARD, with no further reload, is not."""
        with tempfile.TemporaryDirectory() as tmp:
            before_path = Path(tmp) / "before.json"
            before_path.write_text(json.dumps({
                "overrides": {"force_z_target": 12.0},
            }), encoding="utf-8")
            cfg = self._reload_with(Settings.from_file(before_path))
            self.assertAlmostEqual(
                cfg.FORCE_Z_TARGET_N, 12.0,
                msg="a settings source active before ur5_sim.config's "
                    "module body ran must be reflected in its constants")

            after_path = Path(tmp) / "after.json"
            after_path.write_text(json.dumps({
                # In-bounds (design/settings_spec.py: force_z_target in
                # [2, 20]) so this exercises the freeze itself, not F1's
                # separate out-of-bounds refusal path.
                "overrides": {"force_z_target": 18.0},
            }), encoding="utf-8")
            settings_module.reload_settings(after_path)  # no reload of sim_config follows
            self.assertAlmostEqual(
                cfg.FORCE_Z_TARGET_N, 12.0,
                msg="a settings file written after ur5_sim.config already "
                    "imported must NOT retroactively change its constants; "
                    "only importlib.reload(ur5_sim.config) can do that")


if __name__ == "__main__":
    unittest.main()
