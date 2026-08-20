"""
Tests for the CLI side of the RTDE emulator (ur5_sim/emulate.py).

No socket, no GUI, no robot model: these pin the three decisions that are
easy to get silently wrong - when the emulator is opened at all, that no flag
can widen the bind address, and which sign convention the force surrogate is
fed.
"""

import contextlib
import io
import unittest

from spatialmath import SE3, SO3

from ur5_sim import emulate
from ur5_sim.cli import build_parser
from ur5_sim.config import RTDE_EMU_HOST


class WantsRtdeTests(unittest.TestCase):

    def _args(self, argv: list[str]):
        return build_parser().parse_args(argv)

    def test_check_never_opens_the_socket(self) -> None:
        """--check has no real-time pacing; a 151 s protocol would stream in
        seconds and stamp a CSV that resembles no trial."""
        self.assertFalse(emulate.wants_rtde(self._args(["--check"])))

    def test_visualize_opens_it_by_default(self) -> None:
        self.assertTrue(emulate.wants_rtde(self._args(["--visualize"])))

    def test_no_rtde_serve_overrides_the_visualize_default(self) -> None:
        args = self._args(["--visualize", "--no-rtde-serve"])
        self.assertFalse(emulate.wants_rtde(args))

    def test_rtde_serve_alone_is_refused_and_says_so(self) -> None:
        """--check has no path that drives the run state, so an opened socket
        would sit at STOPPED for ever. Refusing loudly beats a dead socket."""
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            wanted = emulate.wants_rtde(self._args(["--check", "--rtde-serve"]))
        self.assertFalse(wanted)
        self.assertIn("--rtde-serve ignored", out.getvalue())

    def test_emulate_implies_it(self) -> None:
        self.assertTrue(emulate.wants_rtde(self._args(["--emulate"])))

    def test_no_flag_can_widen_the_bind_address(self) -> None:
        """It must stay unreachable from the lab VLAN, never mistaken for a
        robot, so the host is a constant and not a command-line option."""
        self.assertEqual(RTDE_EMU_HOST, "127.0.0.1")
        options = build_parser()._option_string_actions
        self.assertNotIn("--rtde-host", options)
        self.assertNotIn("--host", options)


class BufferConversionTests(unittest.TestCase):

    def test_penetration_is_positive_downward_in_contact(self) -> None:
        # In contact the reported depth is signed along the surface normal:
        # negative is below the plane, which is the recontact overshoot.
        self.assertAlmostEqual(
            emulate.penetration_from_depth(True, -0.005), 0.005)
        self.assertAlmostEqual(emulate.penetration_from_depth(True, 0.004), 0.0)

    def test_penetration_in_transit_is_already_positive(self) -> None:
        self.assertAlmostEqual(
            emulate.penetration_from_depth(False, 0.003), 0.003)
        self.assertAlmostEqual(emulate.penetration_from_depth(False, 0.0), 0.0)

    def test_pose_flattening_inverts_the_script_pose_reader(self) -> None:
        """URScript orientation is an axis-angle rotation vector; SO3.EulerVec
        reads it and SE3.eulervec() must write the same six numbers back."""
        wanted = (-0.011, 0.6, 0.05, 0.1, -3.0, 0.2)
        pose = SE3.Rt(SO3.EulerVec(wanted[3:]), list(wanted[:3]))
        got = emulate.poses_to_xyzrpy([(42, pose)])
        self.assertEqual(len(got), 1)
        for a, b in zip(got[0], wanted):
            self.assertAlmostEqual(a, b, places=9)


if __name__ == "__main__":
    unittest.main()
