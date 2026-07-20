"""Tests for PolyScope limits enforcement (250 mm/s TCP + script memory).

Covers:
* :func:`design.export._clamp_tcp_speed` plafonne au-dessus de la limite
  et passe les valeurs deja conformes.
* :func:`design.export._validate_script_memory` retourne ``True`` / ``False``
  selon que le fichier reste sous ``URSCRIPT_MAX_BYTES``.
* :func:`ur5_sim.parsing.urscript.parse_tcp_speed_globals` extrait toutes les
  vitesses TCP declarees au preambule.
* Toutes les vitesses TCP emises par ``design.export._build_urscript_lines``
  respectent la limite ``URSCRIPT_MAX_TCP_SPEED`` apres clamp.
"""

from __future__ import annotations

import io
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from design.export import _clamp_tcp_speed, _validate_script_memory  # noqa: E402
from design.params import URSCRIPT_MAX_BYTES, URSCRIPT_MAX_TCP_SPEED  # noqa: E402
from ur5_sim.config import SCRIPT_PATH, URSCRIPT_MAX_TCP_SPEED_MPS  # noqa: E402
from ur5_sim.parsing.urscript import (  # noqa: E402
    TCP_SPEED_GLOBAL_NAMES,
    parse_tcp_speed_globals,
)


class ClampSpeedTests(unittest.TestCase):
    def test_passes_through_when_under_limit(self) -> None:
        buf = io.StringIO()
        with redirect_stdout(buf):
            v = _clamp_tcp_speed("TEST_V", 0.100)
        self.assertEqual(v, 0.100)
        self.assertEqual(buf.getvalue(), "")

    def test_clamps_when_over_limit(self) -> None:
        buf = io.StringIO()
        with redirect_stdout(buf):
            v = _clamp_tcp_speed("TEST_V", 0.500)
        self.assertEqual(v, URSCRIPT_MAX_TCP_SPEED)
        self.assertIn("WARN", buf.getvalue())
        self.assertIn("TEST_V", buf.getvalue())

    def test_passes_through_when_at_limit(self) -> None:
        buf = io.StringIO()
        with redirect_stdout(buf):
            v = _clamp_tcp_speed("TEST_V", URSCRIPT_MAX_TCP_SPEED)
        self.assertEqual(v, URSCRIPT_MAX_TCP_SPEED)
        self.assertEqual(buf.getvalue(), "")


class MemoryBudgetTests(unittest.TestCase):
    def test_returns_true_when_under_budget(self) -> None:
        with tempfile.NamedTemporaryFile("w", delete=False, suffix=".script") as tmp:
            tmp.write("A" * 100)
            tmp_path = Path(tmp.name)
        try:
            buf = io.StringIO()
            with redirect_stdout(buf):
                ok = _validate_script_memory(tmp_path, "URScript")
            self.assertTrue(ok)
            self.assertIn("Mémoire", buf.getvalue())
        finally:
            tmp_path.unlink(missing_ok=True)

    def test_returns_false_when_over_budget(self) -> None:
        with tempfile.NamedTemporaryFile("w", delete=False, suffix=".script") as tmp:
            tmp.write("A" * (URSCRIPT_MAX_BYTES + 1))
            tmp_path = Path(tmp.name)
        try:
            buf = io.StringIO()
            with redirect_stdout(buf):
                ok = _validate_script_memory(tmp_path, "URScript")
            self.assertFalse(ok)
            self.assertIn("ECHEC EXPORT", buf.getvalue())
        finally:
            tmp_path.unlink(missing_ok=True)


class SpeedGlobalsParserTests(unittest.TestCase):
    def test_extracts_known_speeds_from_real_script(self) -> None:
        speeds = parse_tcp_speed_globals(SCRIPT_PATH)
        # Toutes les vitesses TCP nommees doivent etre presentes (le
        # generateur les emet inconditionnellement).
        for name in TCP_SPEED_GLOBAL_NAMES:
            self.assertIn(name, speeds, msg=f"{name} missing")
        for name, value in speeds.items():
            self.assertLessEqual(
                value, URSCRIPT_MAX_TCP_SPEED_MPS,
                msg=f"{name} = {value} m/s > {URSCRIPT_MAX_TCP_SPEED_MPS} m/s",
            )

    def test_ignores_non_speed_globals(self) -> None:
        speeds = parse_tcp_speed_globals(SCRIPT_PATH)
        # NHAT_X est un global du preambule mais n'est pas une vitesse TCP.
        self.assertNotIn("NHAT_X", speeds)
        self.assertNotIn("PROBE_FORCE_THR", speeds)


if __name__ == "__main__":
    unittest.main()
