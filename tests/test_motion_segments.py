"""Tests for the velocity-faithful motion-segment pipeline.

Covers:
* :func:`ur5_sim.parsing.urscript.parse_speed_symbol_table` reads the script
  preamble globals.
* :func:`ur5_sim.parsing.urscript._eval_speed_expr` resolves both single-token
  and product-of-two-tokens v= expressions.
* :func:`ur5_sim.parsing.urscript.parse_motion_segments` returns one
  :class:`MotionSegment` per movel/movej, with v_value resolved against the
  symbol table.
* :func:`ur5_sim.kinematics.motion.densify_segments` subdivides segments by
  ``distance / min(v, cap)`` and emits SEGMENT_VELOCITY_EXCEEDED events when
  the declared velocity exceeds the PolyScope cap.
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ur5_sim.config import DT, SCRIPT_PATH, URSCRIPT_MAX_TCP_SPEED_MPS  # noqa: E402
from ur5_sim.kinematics.motion import (  # noqa: E402
    MOVEJ_NOMINAL_DURATION_S,
    densify_segments,
)
from ur5_sim.parsing.urscript import (  # noqa: E402
    MotionSegment,
    _eval_speed_expr,
    parse_motion_segments,
    parse_speed_symbol_table,
)


_MINI_SCRIPT = """
# preamble
global SPEED_FACTOR = 1.0
global ACCEL_FACTOR = 1.0
global URSCRIPT_ACCEL = 0.8
global URSCRIPT_RECONTACT_V = 0.0100
global V_CIRC = 0.0360
global V_RECT = 0.0800
global V_INIT = 0.5

def cycle_1():
  movel(apply_correction(p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), a=URSCRIPT_ACCEL*ACCEL_FACTOR, v=0.2500*SPEED_FACTOR)
  force_mode(MEAS_FRAME, [0, 0, 1, 0, 0, 0], [0, 0, -6.0, 0, 0, 0], 2, [0.002, 0.002, 0.04, 0.35, 0.35, 0.35])
  movel(apply_correction(p[0.01, 0.0, 0.0, 0.0, 0.0, 0.0]), a=URSCRIPT_ACCEL*ACCEL_FACTOR, v=URSCRIPT_RECONTACT_V*SPEED_FACTOR)
  movel(apply_correction(p[0.02, 0.0, 0.0, 0.0, 0.0, 0.0]), a=URSCRIPT_ACCEL*ACCEL_FACTOR, v=V_CIRC*SPEED_FACTOR, r=0.002)
  end_force_mode()
  movel(apply_correction(p[0.02, 0.0, 0.01, 0.0, 0.0, 0.0]), a=URSCRIPT_ACCEL*ACCEL_FACTOR, v=0.2500*SPEED_FACTOR)
end

def etalement():
  movej(p[0.0, 0.0, 0.05, 0.0, 0.0, 0.0], a=1.2*ACCEL_FACTOR, v=V_INIT*SPEED_FACTOR)
  cycle_1()
end
"""


def _write_temp_script(content: str) -> Path:
    tmp = tempfile.NamedTemporaryFile("w", delete=False, suffix=".script", encoding="utf-8")
    tmp.write(content)
    tmp.close()
    return Path(tmp.name)


class SymbolTableTests(unittest.TestCase):
    def test_extracts_numeric_globals_from_preamble(self) -> None:
        script = _write_temp_script(_MINI_SCRIPT)
        try:
            symtab = parse_speed_symbol_table(script)
            self.assertEqual(symtab["SPEED_FACTOR"], 1.0)
            self.assertEqual(symtab["URSCRIPT_RECONTACT_V"], 0.01)
            self.assertEqual(symtab["V_CIRC"], 0.036)
            self.assertEqual(symtab["V_INIT"], 0.5)
            # Pose globals (p[...]) must not appear.
            self.assertNotIn("MEAS_FRAME", symtab)
        finally:
            script.unlink(missing_ok=True)


class EvalSpeedExprTests(unittest.TestCase):
    def setUp(self) -> None:
        self.symtab = {
            "SPEED_FACTOR": 1.0,
            "URSCRIPT_RECONTACT_V": 0.01,
            "V_CIRC": 0.036,
        }

    def test_literal_alone(self) -> None:
        self.assertEqual(_eval_speed_expr("0.2500", None, self.symtab), 0.25)

    def test_literal_times_factor(self) -> None:
        self.assertEqual(
            _eval_speed_expr("0.2500", "SPEED_FACTOR", self.symtab), 0.25
        )

    def test_name_times_factor(self) -> None:
        self.assertAlmostEqual(
            _eval_speed_expr("V_CIRC", "SPEED_FACTOR", self.symtab),
            0.036,
        )

    def test_unknown_name_returns_none(self) -> None:
        self.assertIsNone(_eval_speed_expr("UNKNOWN_V", "SPEED_FACTOR", self.symtab))


class ParseMotionSegmentsTests(unittest.TestCase):
    def test_resolves_v_per_movel_in_mini_script(self) -> None:
        script = _write_temp_script(_MINI_SCRIPT)
        try:
            segments = parse_motion_segments(script)
        finally:
            script.unlink(missing_ok=True)
        # 5 motion calls: 4 movel + 1 movej.
        self.assertEqual(len(segments), 5)
        kinds = [s.kind for s in segments]
        self.assertEqual(kinds.count("movel"), 4)
        self.assertEqual(kinds.count("movej"), 1)
        v_values = [s.v_value for s in segments]
        # transit movel inlined literal 0.25
        self.assertAlmostEqual(v_values[0], 0.25, places=6)
        # recontact at URSCRIPT_RECONTACT_V * SPEED_FACTOR = 0.01
        self.assertAlmostEqual(v_values[1], 0.01, places=6)
        # cycle waypoint at V_CIRC
        self.assertAlmostEqual(v_values[2], 0.036, places=6)
        # transit out inlined literal 0.25
        self.assertAlmostEqual(v_values[3], 0.25, places=6)
        # movej home: V_INIT in rad/s
        self.assertEqual(segments[4].kind, "movej")
        self.assertEqual(segments[4].v_unit, "rad/s")
        self.assertAlmostEqual(v_values[4], 0.5, places=6)

    def test_in_contact_flag_tracks_force_mode(self) -> None:
        script = _write_temp_script(_MINI_SCRIPT)
        try:
            segments = parse_motion_segments(script)
        finally:
            script.unlink(missing_ok=True)
        contact = [s.in_contact for s in segments]
        # Order in mini script: transit_in (False), recontact (True),
        # waypoint (True), transit_out (False), movej home (False).
        self.assertEqual(contact, [False, True, True, False, False])


class DensifySegmentsTests(unittest.TestCase):
    def _seg(
        self,
        x: float,
        v: float,
        kind: str = "movel",
        in_contact: bool = False,
    ) -> MotionSegment:
        return MotionSegment(
            lineno=10,
            pose=(x, 0.0, 0.0, 0.0, 0.0, 0.0),
            cycle_idx=1,
            in_contact=in_contact,
            kind=kind,
            v_value=v,
            v_unit="m/s" if kind == "movel" else "rad/s",
        )

    def test_total_time_matches_distance_over_velocity(self) -> None:
        # Two-pose segment: 0 -> 0.30 m at 0.25 m/s => 1.20 s.
        segs = [
            self._seg(0.0, 0.25),
            self._seg(0.30, 0.25),
        ]
        densified, events = densify_segments(segs, DT, URSCRIPT_MAX_TCP_SPEED_MPS)
        self.assertEqual(events, [])
        # First pose held as a single frame + n_sub frames for the segment.
        n_sub = round(1.20 / DT)
        self.assertEqual(len(densified), 1 + n_sub)

    def test_clamps_above_cap_and_reports(self) -> None:
        # Declared 0.5 m/s exceeds the 0.25 cap. Duration must use 0.25 m/s.
        segs = [
            self._seg(0.0, 0.5),
            self._seg(0.10, 0.5),
        ]
        densified, events = densify_segments(segs, DT, URSCRIPT_MAX_TCP_SPEED_MPS)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0][1], "SEGMENT_VELOCITY_EXCEEDED")
        # 0.10 m / 0.25 m/s = 0.40 s => n_sub = 8 substeps at DT=0.05.
        self.assertEqual(len(densified), 1 + round(0.40 / DT))

    def test_unknown_v_falls_back_to_cap(self) -> None:
        segs = [
            self._seg(0.0, 0.25),
            MotionSegment(
                lineno=11,
                pose=(0.10, 0.0, 0.0, 0.0, 0.0, 0.0),
                cycle_idx=1,
                in_contact=False,
                kind="movel",
                v_value=None,
                v_unit="m/s",
            ),
        ]
        _, events = densify_segments(segs, DT, URSCRIPT_MAX_TCP_SPEED_MPS)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0][1], "SEGMENT_VELOCITY_UNKNOWN")

    def test_movej_uses_nominal_duration(self) -> None:
        segs = [
            self._seg(0.0, 0.25),
            self._seg(0.10, 0.5, kind="movej"),
        ]
        densified, _ = densify_segments(segs, DT, URSCRIPT_MAX_TCP_SPEED_MPS)
        expected_sub = round(MOVEJ_NOMINAL_DURATION_S / DT)
        self.assertEqual(len(densified), 1 + expected_sub)


class RealScriptIntegrationTests(unittest.TestCase):
    def test_real_etalement_script_resolves_every_v(self) -> None:
        segments = parse_motion_segments(SCRIPT_PATH)
        self.assertGreater(len(segments), 0)
        unresolved = [s for s in segments if s.v_value is None]
        self.assertEqual(
            unresolved, [],
            msg=(
                "Every movel/movej in the exported script must declare a "
                "v= expression resolvable against the preamble globals. "
                f"Unresolved: {[(s.lineno, s.kind) for s in unresolved]}"
            ),
        )
        for s in segments:
            if s.kind == "movel" and s.v_value is not None:
                self.assertLessEqual(
                    s.v_value, URSCRIPT_MAX_TCP_SPEED_MPS + 1e-9,
                    msg=(
                        f"movel at line {s.lineno} declares v={s.v_value} "
                        f"> cap {URSCRIPT_MAX_TCP_SPEED_MPS}"
                    ),
                )

    def test_densify_real_script_produces_longer_buffer(self) -> None:
        segments = parse_motion_segments(SCRIPT_PATH)
        densified, _events = densify_segments(
            segments, DT, URSCRIPT_MAX_TCP_SPEED_MPS,
        )
        # The densified buffer must be at least as long as the raw segment
        # list (every segment contributes >= 1 frame, recontact / probe
        # segments contribute many more).
        self.assertGreaterEqual(len(densified), len(segments))


if __name__ == "__main__":
    unittest.main()
