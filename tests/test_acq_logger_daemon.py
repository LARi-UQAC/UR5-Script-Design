"""Tests for the on-robot acquisition daemon, onrobot/acq_logger_daemon.py.

The daemon ships alone on a USB key (no package, no __init__.py), so it is
loaded the same way its own docstring says a caller must load it:
importlib.util.spec_from_file_location. No sys.path trick, no import of
"onrobot" as a package.

Full contract: docs/superpower/plans/plan_acq_datalogger.md, sections
3-bis (daemon contract), 4 (acquisition block / sentinel handshake) and 5
(CSV format).

Everything here runs offline: a tempfile.mkdtemp() directory stands in for
the USB key, a lambda returning a fixed epoch stands in for the wall
clock, and real TCP sockets bound on port 0 (OS-chosen) stand in for both
the robot client and the FT-300. Every socket opened here is closed and
every thread started here is joined, in tearDown if not sooner, so a
failing assertion cannot leave the suite hanging.
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import os
import socket
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

_REPO_ROOT = Path(__file__).resolve().parents[1]
_DAEMON_PATH = _REPO_ROOT / "onrobot" / "acq_logger_daemon.py"


def _load_daemon_module():
    """Load acq_logger_daemon.py by path, exactly as its own docstring
    prescribes: no package, no relative import, so the file keeps working
    when copied alone onto the USB key."""
    spec = importlib.util.spec_from_file_location("acq_logger_daemon", _DAEMON_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


daemon = _load_daemon_module()


def _send_line(sock, text):
    sock.sendall((text + "\n").encode("ascii"))


def _recv_line(sock_file):
    raw = sock_file.readline()
    return raw.decode("ascii").strip()


def _read_data_rows(path):
    """Read a written CSV back and split off the 7 metadata lines and the
    header, returning only the data rows as tuples of float."""
    with open(path, "rb") as f:
        text = f.read().decode("ascii")
    lines = text.splitlines()
    data_lines = lines[8:]  # 7 "# " metadata lines + 1 header line
    return [tuple(float(v) for v in ln.split(",")) for ln in data_lines]


class _FakeFtReader(object):
    """Duck-typed stand-in for FTReader. LogServer only ever calls
    .latest() and reads .ok (see the LogServer class docstring: "ft_reader
    (FTReader-like)"), so most LogServer tests need no real FT-300 socket
    at all."""

    def __init__(self, triple=(0.0, 0.0, 0.0), ok=False):
        self._triple = triple
        self.ok = ok

    def latest(self):
        return self._triple


# --------------------------------------------------------------------------
# Pure functions: parse_ft_line, parse_sample_line, find_usb_mount,
# format_csv, next_free_name.
# --------------------------------------------------------------------------

class ParseFtLineTests(unittest.TestCase):
    def test_six_field_with_parens_and_irregular_whitespace(self):
        triple = daemon.parse_ft_line("(  0.12 ,-3.40,  5.60 , 0.00,0.00 , 0.00)")
        self.assertEqual(triple, (0.12, -3.40, 5.60))

    def test_six_field_without_parens(self):
        triple = daemon.parse_ft_line("0.12,-3.40,5.60,0.00,0.00,0.00")
        self.assertEqual(triple, (0.12, -3.40, 5.60))

    def test_three_field_variant(self):
        triple = daemon.parse_ft_line("0.12,-3.40,5.60")
        self.assertEqual(triple, (0.12, -3.40, 5.60))

    def test_garbage_returns_none_without_raising(self):
        self.assertIsNone(daemon.parse_ft_line("not,a,valid,line,at,all,extra"))
        self.assertIsNone(daemon.parse_ft_line("abc,def,ghi"))

    def test_empty_line_returns_none(self):
        self.assertIsNone(daemon.parse_ft_line(""))
        self.assertIsNone(daemon.parse_ft_line("   \n"))


class ParseSampleLineTests(unittest.TestCase):
    def test_four_field_with_brackets(self):
        self.assertEqual(
            daemon.parse_sample_line("[0.016,0.412,-0.298,0.101]"),
            (0.016, 0.412, -0.298, 0.101),
        )

    def test_four_field_without_brackets(self):
        self.assertEqual(
            daemon.parse_sample_line("0.016,0.412,-0.298,0.101"),
            (0.016, 0.412, -0.298, 0.101),
        )

    def test_seven_field_with_brackets(self):
        parsed = daemon.parse_sample_line("[0.016,0.412,-0.298,0.101,1.0,2.0,3.0]")
        self.assertEqual(parsed, (0.016, 0.412, -0.298, 0.101, 1.0, 2.0, 3.0))

    def test_seven_field_without_brackets(self):
        parsed = daemon.parse_sample_line("0.016,0.412,-0.298,0.101,1.0,2.0,3.0")
        self.assertEqual(parsed, (0.016, 0.412, -0.298, 0.101, 1.0, 2.0, 3.0))

    def test_garbage_returns_none(self):
        self.assertIsNone(daemon.parse_sample_line("hello world"))
        self.assertIsNone(daemon.parse_sample_line("[1.0,2.0]"))

    def test_stop_line_is_never_mistaken_for_a_sample(self):
        self.assertIsNone(daemon.parse_sample_line("STOP 42"))
        self.assertIsNone(daemon.parse_sample_line("STOP"))

    def test_retry_line_is_never_mistaken_for_a_sample(self):
        self.assertIsNone(daemon.parse_sample_line("RETRY"))


class FindUsbMountTests(unittest.TestCase):
    def test_vfat_under_media_is_found(self):
        text = (
            "proc /proc proc rw,relatime 0 0\n"
            "/dev/sda1 / ext4 rw,relatime 0 0\n"
            "/dev/sdb1 /media/usb0 vfat rw,relatime,uid=1000,gid=1000 0 0\n"
            "tmpfs /run tmpfs rw,nosuid 0 0\n"
        )
        self.assertEqual(daemon.find_usb_mount(text), "/media/usb0")

    def test_exfat_under_programs_is_found(self):
        text = (
            "proc /proc proc rw,relatime 0 0\n"
            "/dev/sdc1 /programs/key exfat rw,relatime,uid=1000 0 0\n"
        )
        self.assertEqual(daemon.find_usb_mount(text), "/programs/key")

    def test_no_usb_filesystem_returns_none(self):
        text = (
            "proc /proc proc rw,relatime 0 0\n"
            "/dev/sda1 / ext4 rw,relatime 0 0\n"
            "tmpfs /run tmpfs rw,nosuid 0 0\n"
        )
        self.assertIsNone(daemon.find_usb_mount(text))

    def test_non_usb_filesystem_under_media_is_not_picked(self):
        # A network share or an internal ext4 partition mounted under
        # /media must not be mistaken for the USB key just because the
        # mount path looks right; fstype is what actually matters.
        text = (
            "//nas/share /media/nas cifs rw,relatime 0 0\n"
            "/dev/sda2 /media/data ext4 rw,relatime 0 0\n"
        )
        self.assertIsNone(daemon.find_usb_mount(text))


class FormatCsvTests(unittest.TestCase):
    def test_metadata_header_and_exact_row_formatting(self):
        meta = {
            "robot_model": "UR5 CB3",
            "polyscope_version": "3.11.0.82155 (20 August 2019)",
            "date": "2026-08-16",
            "time": "10:00:00",
            "target_hz": 50,
            "max_buffer": 11700,
            "n_samples": 2,
        }
        rows = [
            (0.02, -0.123456, 0.234567, -6.012345, 0.412345, -0.298765, 0.101234),
            (0.04, 1.0, -2.0, 3.0, 4.0, -5.0, 6.0),
        ]
        data = daemon.format_csv(meta, rows)
        self.assertIsInstance(data, bytes)
        text = data.decode("ascii")
        self.assertNotIn("\r", text)
        lines = text.split("\n")
        self.assertEqual(lines[-1], "")  # single trailing newline, no blank tail
        body = lines[:-1]
        self.assertEqual(len(body), 7 + 1 + 2)
        self.assertEqual(body[0], "# Robot Model: UR5 CB3")
        self.assertEqual(body[1], "# PolyScope Version: 3.11.0.82155 (20 August 2019)")
        self.assertEqual(body[2], "# File Creation Date: 2026-08-16")
        self.assertEqual(body[3], "# File Creation Time: 10:00:00")
        self.assertEqual(body[4], "# Target Acquisition Frequency: 50 Hz")
        self.assertEqual(body[5], "# Maximum Buffer Size: 11700")
        self.assertEqual(body[6], "# Actual Number of Collected Samples: 2")
        self.assertEqual(body[7], daemon.HEADER)
        self.assertEqual(body[7], "Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ")
        self.assertEqual(
            body[8],
            "0.020,-0.123456,0.234567,-6.012345,0.412345,-0.298765,0.101234",
        )
        self.assertEqual(
            body[9], "0.040,1.000000,-2.000000,3.000000,4.000000,-5.000000,6.000000"
        )

    def test_defaults_are_used_when_meta_is_empty(self):
        data = daemon.format_csv({}, [])
        lines = data.decode("ascii").split("\n")
        self.assertTrue(lines[0].startswith("# Robot Model: UR5 CB3"))
        self.assertIn(daemon.HEADER, lines)


class NextFreeNameTests(unittest.TestCase):
    def test_no_collision_returns_bare_stem(self):
        self.assertEqual(
            daemon.next_free_name([], "ACQ_log_20260816_100000"),
            "ACQ_log_20260816_100000.csv",
        )

    def test_one_collision_returns_suffix_1(self):
        existing = ["ACQ_log_20260816_100000.csv"]
        self.assertEqual(
            daemon.next_free_name(existing, "ACQ_log_20260816_100000"),
            "ACQ_log_20260816_100000_1.csv",
        )

    def test_several_collisions_return_first_free_suffix(self):
        existing = [
            "ACQ_log_20260816_100000.csv",
            "ACQ_log_20260816_100000_1.csv",
            "ACQ_log_20260816_100000_2.csv",
        ]
        self.assertEqual(
            daemon.next_free_name(existing, "ACQ_log_20260816_100000"),
            "ACQ_log_20260816_100000_3.csv",
        )


# --------------------------------------------------------------------------
# FTReader against a genuine loopback fake FT-300 (no mock): connect,
# parse, .latest()/.ok update, clean stop().
# --------------------------------------------------------------------------

class FtReaderRealSocketTests(unittest.TestCase):
    def setUp(self):
        self._fake_ft = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._fake_ft.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._fake_ft.bind(("127.0.0.1", 0))
        self._fake_ft.listen(1)
        self.port = self._fake_ft.getsockname()[1]
        self._conn = None

    def tearDown(self):
        if self._conn is not None:
            self._conn.close()
        self._fake_ft.close()

    def test_latest_and_ok_update_from_a_real_connection(self):
        reader = daemon.FTReader("127.0.0.1", self.port)
        reader.start()
        try:
            self._conn, _addr = self._fake_ft.accept()
            self._conn.sendall(b"(1.5,-2.5,6.25,0.0,0.0,0.0)\n")
            deadline = time.time() + 2.0
            while not reader.ok and time.time() < deadline:
                time.sleep(0.01)
            self.assertTrue(reader.ok)
            self.assertEqual(reader.latest(), (1.5, -2.5, 6.25))
        finally:
            # Close both ends BEFORE stop(): the reader's socket read has
            # no timeout once connected, so unless the peer disappears
            # first, stop()'s 2 s join would race a blocking readline()
            # that has nothing to wake it. Closing the listening socket
            # too means a reconnect attempt fails fast (connection
            # refused) instead of quietly succeeding into a new socket
            # that nobody will ever close.
            if self._conn is not None:
                self._conn.close()
                self._conn = None
            self._fake_ft.close()
            reader.stop()
        self.assertFalse(reader._thread.is_alive())


# --------------------------------------------------------------------------
# LogServer over a real loopback TCP session: one background thread plays
# the daemon (serve_one_session blocks on accept-then-readline), the main
# thread plays the robot client. Shared base class handles the socket and
# thread bookkeeping so no test can leave either dangling.
# --------------------------------------------------------------------------

_RAISE_SENTINEL = object()


class _BaseSessionTestCase(unittest.TestCase):
    FIXED_EPOCH = 1734000000.0  # arbitrary; never compared to a hardcoded string

    def setUp(self):
        self.usb_dir = tempfile.mkdtemp(prefix="acq_usb_")
        self._out_dir = [self.usb_dir]
        self.clock = lambda: self.FIXED_EPOCH
        self.ft_reader = _FakeFtReader()
        self.server = daemon.LogServer(
            0, self.ft_reader, self._resolve_out_dir, self.clock
        )
        self.server.start()
        self.port = self.server._sock.getsockname()[1]
        self._threads = []
        self._client_socks = []

    def _resolve_out_dir(self):
        value = self._out_dir[0]
        if value is _RAISE_SENTINEL:
            # A pure-ASCII stand-in for "the resolver itself failed"
            # (e.g. the USB key vanished mid-trial). Deliberately not a
            # colliding-path os.makedirs() failure here: that raises a
            # real, OS-generated OSError whose text is localized (see
            # KnownDaemonBugTests below for what that does on this
            # host), which would make this required-coverage test
            # depend on the host's locale instead of on the daemon's
            # documented ERR/RETRY contract.
            raise OSError("simulated: output directory not available")
        return value

    def tearDown(self):
        for thread in self._threads:
            thread.join(timeout=5.0)
        for sock, sock_file in self._client_socks:
            try:
                sock_file.close()
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        self.server.stop()
        _rmtree_ignore_errors(self.usb_dir)

    def _connect(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(("127.0.0.1", self.port))
        sock_file = sock.makefile("rb")
        self._client_socks.append((sock, sock_file))
        return sock, sock_file

    def _serve_in_background(self):
        box = {}

        def _target():
            box["result"] = self.server.serve_one_session()

        thread = threading.Thread(target=_target)
        thread.start()
        self._threads.append(thread)
        return box, thread

    def _expected_filename(self):
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.FIXED_EPOCH))
        return "ACQ_log_%s.csv" % stamp


def _rmtree_ignore_errors(path):
    import shutil

    shutil.rmtree(path, ignore_errors=True)


class LogServerFullSessionTests(_BaseSessionTestCase):
    def test_full_session_reply_and_row_accounting(self):
        sock, sock_file = self._connect()
        box, thread = self._serve_in_background()
        _send_line(sock, "[0.020,0.100,0.200,0.300]")
        _send_line(sock, "[0.040,0.101,0.201,0.301]")
        _send_line(sock, "this is not a sample")  # -> n_bad, no reply
        _send_line(sock, "[0.060,0.102,0.202,0.302]")
        _send_line(sock, "STOP")
        reply = _recv_line(sock_file)
        thread.join(timeout=5.0)
        self.assertFalse(thread.is_alive())
        result = box["result"]

        self.assertEqual(reply, "OK %s 3" % self._expected_filename())
        self.assertEqual(result.n_rows, 3)
        self.assertEqual(result.n_bad, 1)
        self.assertFalse(result.truncated)

        rows = _read_data_rows(result.path)
        self.assertEqual(len(rows), 3)  # only collected rows, no preallocated tail


class LogServerSentinelTests(_BaseSessionTestCase):
    def test_sentinel_row_is_excluded_from_csv(self):
        sock, sock_file = self._connect()
        box, thread = self._serve_in_background()
        _send_line(sock, "[0.020,0.1,0.2,0.3]")
        _send_line(sock, "[0.040,0.11,0.21,0.31]")
        _send_line(sock, "[-1.0,2,0.0,0.0]")  # sentinel, matches the 2 samples above
        _send_line(sock, "STOP")
        _recv_line(sock_file)
        thread.join(timeout=5.0)
        result = box["result"]

        self.assertEqual(result.n_rows, 2)
        text = Path(result.path).read_bytes().decode("ascii")
        self.assertNotIn("-1.000", text)

    def test_sentinel_mismatch_is_reported_without_altering_rows(self):
        sock, sock_file = self._connect()
        box, thread = self._serve_in_background()
        _send_line(sock, "[0.020,0.1,0.2,0.3]")
        _send_line(sock, "[0.040,0.11,0.21,0.31]")
        _send_line(sock, "[0.060,0.12,0.22,0.32]")
        _send_line(sock, "[-1.0,5,0.0,0.0]")  # robot claims 5, only 3 actually sent
        stdout = io.StringIO()
        with contextlib.redirect_stdout(stdout):
            _send_line(sock, "STOP")
            _recv_line(sock_file)
            thread.join(timeout=5.0)
        result = box["result"]

        # Rows kept are exactly what was received: never padded or
        # truncated to match the robot's own (higher) claim.
        self.assertEqual(result.n_rows, 3)
        self.assertIn("robot sent 5 samples, 3 received", stdout.getvalue())

    def test_sentinel_before_cap_is_honoured_and_session_truncated(self):
        sock, sock_file = self._connect()
        with mock.patch.object(daemon, "MAX_SAMPLES", 4):
            box, thread = self._serve_in_background()
            for i in range(6):
                t = 0.02 * (i + 1)
                _send_line(sock, "[%.3f,%.3f,%.3f,%.3f]" % (t, i * 0.01, i * 0.02, i * 0.03))
            _send_line(sock, "[-1.0,6,0.0,0.0]")  # sentinel arrives after the cap
            _send_line(sock, "STOP")
            _recv_line(sock_file)
            thread.join(timeout=5.0)
            # If the daemon had blocked once MAX_SAMPLES was hit, STOP
            # would never be read and this thread would still be alive
            # (or the earlier _recv_line would have hung until the test
            # runner's own timeout).
            self.assertFalse(thread.is_alive())
        result = box["result"]

        self.assertTrue(result.truncated)
        self.assertEqual(result.n_rows, 4)
        rows = _read_data_rows(result.path)
        self.assertEqual(len(rows), 4)


class LogServerColumnOrderTests(_BaseSessionTestCase):
    def test_four_field_sample_merges_ft_reader_latest_triple(self):
        self.ft_reader._triple = (1.5, -2.5, 6.25)
        self.ft_reader.ok = True
        sock, sock_file = self._connect()
        box, thread = self._serve_in_background()
        _send_line(sock, "[0.020,0.111,0.222,0.333]")
        _send_line(sock, "STOP")
        _recv_line(sock_file)
        thread.join(timeout=5.0)
        result = box["result"]

        row = _read_data_rows(result.path)[0]
        # Column order per HEADER: Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ
        self.assertAlmostEqual(row[0], 0.020, places=3)
        self.assertAlmostEqual(row[1], 1.5, places=6)
        self.assertAlmostEqual(row[2], -2.5, places=6)
        self.assertAlmostEqual(row[3], 6.25, places=6)
        self.assertAlmostEqual(row[4], 0.111, places=6)
        self.assertAlmostEqual(row[5], 0.222, places=6)
        self.assertAlmostEqual(row[6], 0.333, places=6)

    def test_seven_field_sample_uses_its_own_force_not_ft_reader(self):
        self.ft_reader._triple = (9.0, 9.0, 9.0)  # must be ignored for this row
        self.ft_reader.ok = True
        sock, sock_file = self._connect()
        box, thread = self._serve_in_background()
        _send_line(sock, "[0.040,0.444,0.555,0.666,1.1,2.2,3.3]")
        _send_line(sock, "STOP")
        _recv_line(sock_file)
        thread.join(timeout=5.0)
        result = box["result"]

        row = _read_data_rows(result.path)[0]
        self.assertAlmostEqual(row[1], 1.1, places=6)
        self.assertAlmostEqual(row[2], 2.2, places=6)
        self.assertAlmostEqual(row[3], 3.3, places=6)
        self.assertAlmostEqual(row[4], 0.444, places=6)
        self.assertAlmostEqual(row[5], 0.555, places=6)
        self.assertAlmostEqual(row[6], 0.666, places=6)


class LogServerFailureRetryTests(_BaseSessionTestCase):
    def test_unwritable_dir_replies_err_then_retry_after_fix_succeeds(self):
        self._out_dir[0] = _RAISE_SENTINEL

        sock, sock_file = self._connect()
        box, thread = self._serve_in_background()
        _send_line(sock, "[0.020,0.1,0.2,0.3]")
        _send_line(sock, "[0.040,0.11,0.21,0.31]")
        _send_line(sock, "STOP")
        err_reply = _recv_line(sock_file)
        self.assertTrue(err_reply.startswith("ERR "), err_reply)

        # Fix the resolver, then RETRY on the SAME connection: the two
        # rows buffered before the failed STOP must still be there. (A
        # dropped connection is a documented, untested limitation - see
        # the daemon's own commit message.)
        self._out_dir[0] = self.usb_dir
        _send_line(sock, "RETRY")
        ok_reply = _recv_line(sock_file)
        thread.join(timeout=5.0)
        result = box["result"]

        self.assertTrue(ok_reply.startswith("OK "), ok_reply)
        self.assertEqual(result.n_rows, 2)
        rows = _read_data_rows(result.path)
        self.assertEqual(len(rows), 2)


class KnownDaemonBugTests(_BaseSessionTestCase):
    """Not required coverage: a defect this suite happened to surface
    while building the item above. Left failing on purpose, per the
    task instructions to report a daemon-side failure rather than
    "fix" acq_logger_daemon.py to make it pass."""

    def test_localized_os_error_text_breaks_the_err_reply(self):
        """_finalize's except clause builds "ERR %s" % exc straight
        from the caught OSError, and _reply() hard-encodes to ASCII
        (acq_logger_daemon.py, _reply()). A real filesystem OSError's
        text is whatever the OS localizes it to - on this Windows host
        (French locale) os.makedirs() colliding with an existing file
        raises FileExistsError with an accented message, so _reply()
        itself raises UnicodeEncodeError. That exception is not caught
        anywhere in _finalize or serve_one_session, so it propagates out
        of the background thread: no "ERR ..." line is ever sent, and
        serve_one_session's own finally block closes the socket, which
        also destroys the connection RETRY would have needed. This is
        the real, portable "unwritable directory" scenario (a colliding
        file at the resolved path, not a synthetic OSError), and it
        fails the documented contract (section 3-bis: a write failure
        replies ERR and keeps the connection open for RETRY).
        """
        bad_path = os.path.join(self.usb_dir, "not_a_directory")
        with open(bad_path, "w"):
            pass
        self._out_dir[0] = bad_path

        sock, sock_file = self._connect()
        box, thread = self._serve_in_background()
        _send_line(sock, "[0.020,0.1,0.2,0.3]")
        _send_line(sock, "STOP")
        err_reply = _recv_line(sock_file)
        thread.join(timeout=5.0)

        self.assertTrue(
            err_reply.startswith("ERR "),
            "expected 'ERR ...' per the daemon contract (section 3-bis); "
            "got %r instead - _reply()'s ascii-only encode of a localized "
            "OSError message crashes the reply path, see this test's "
            "docstring" % (err_reply,),
        )


if __name__ == "__main__":
    unittest.main()
