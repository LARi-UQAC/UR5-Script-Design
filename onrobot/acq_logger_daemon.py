"""
acq_logger_daemon.py - on-robot acquisition daemon for the ISO/COLIPA
cosmetic-spread trials (UR5 CB3 + Robotiq FT-300).

Runs stand-alone from a USB key in the robot controller (CB3 ships Python
2.7; this module stays valid under Python 3 for the test suite). Plain
module, no package, no relative import, so it loads via
importlib.util.spec_from_file_location and still runs copied alone.

FTReader keeps the latest (Fx, Fy, Fz) from the FT-300 text stream
(127.0.0.1:63351). LogServer is the loopback TCP server (127.0.0.1:50100)
the URScript data_logger thread streams into; on "STOP <n>" it writes the
trial as one CSV, fsyncs, and replies. Zero USB I/O during motion. Full
contract: docs/superpower/plans/plan_acq_datalogger.md, sec. 2, 3-bis, 5.
"""

from __future__ import print_function

import os
import socket
import sys
import threading
import time

DEFAULT_LOG_PORT = 50100
DEFAULT_FT_PORT = 63351
MAX_SAMPLES = 11700
HEADER = "Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ"

def parse_ft_line(line):
    """
    --------------------------------------------------------------------------
    Purpose:
        Parse one FT-300 line: "( 0.12 , -3.40 , 5.60 , 0.00 , 0.00 , 0.00 )",
        parens optional, whitespace irrelevant, comma separated, 6 fields
        (Fx,Fy,Fz,Mx,My,Mz) or a bare 3-field (Fx,Fy,Fz) variant. Never
        raises; anything else is just reported back.
    Inputs:
        line (str): one raw line from the FT-300 socket.
    Outputs:
        triple (tuple or None): (fx, fy, fz) floats, or None.
    --------------------------------------------------------------------------
    """
    text = line.strip()
    if not text:
        return None
    text = text.replace("(", "").replace(")", "")
    parts = [p.strip() for p in text.split(",")]
    if len(parts) not in (3, 6):
        return None
    try:
        values = [float(p) for p in parts]
    except ValueError:
        return None
    return (values[0], values[1], values[2])

def parse_sample_line(line):
    """
    --------------------------------------------------------------------------
    Purpose:
        Parse one line from socket_send_line(sample_buf, ...): bracketed or
        bare list of 4 fields (t,x,y,z) - normal path, force read from the
        FT-300 - or 7 fields (t,x,y,z,fx,fy,fz), the USE_INTERNAL_FORCE
        fallback. Never raises; this also safely absorbs "STOP <n>" /
        "RETRY" control lines, which never split into 4 or 7 fields.
    Inputs:
        line (str): one raw line from the log-server socket.
    Outputs:
        sample (tuple or None): (t,x,y,z) or (t,x,y,z,fx,fy,fz), or None.
    --------------------------------------------------------------------------
    """
    text = line.strip()
    if not text:
        return None
    text = text.replace("[", "").replace("]", "")
    parts = [p.strip() for p in text.split(",")]
    if len(parts) not in (4, 7):
        return None
    try:
        values = [float(p) for p in parts]
    except ValueError:
        return None
    return tuple(values)

def find_usb_mount(proc_mounts_text):
    """
    --------------------------------------------------------------------------
    Purpose:
        Find the USB key mount point in /proc/mounts text: the first
        vfat/exfat filesystem mounted under /media or /programs. Pure over
        the text so the daemon's real /proc/mounts is never touched by a
        test.
    Inputs:
        proc_mounts_text (str): full text of /proc/mounts (or a fixture).
    Outputs:
        mount_point (str or None): matched mount path, or None. The caller
        falls back to /tmp and records that in the CSV metadata.
    --------------------------------------------------------------------------
    """
    for line in proc_mounts_text.splitlines():
        fields = line.split()
        if len(fields) < 3:
            continue
        mount_point, fstype = fields[1], fields[2]
        if fstype in ("vfat", "exfat") and (
            mount_point.startswith("/media") or mount_point.startswith("/programs")
        ):
            return mount_point
    return None

def format_csv(meta, rows):
    """
    --------------------------------------------------------------------------
    Purpose:
        Render one trial as the CSV bytes of plan section 5: a 7-line "# "
        metadata block, the fixed HEADER, then one row per sample. Pure -
        no socket, no clock, no disk - the caller opens 'wb', writes these
        bytes, flush()es, os.fsync()s and closes, in that order.
    Inputs:
        meta (dict): optional fields, defaulted - "robot_model",
            "polyscope_version", "date" (YYYY-MM-DD), "time" (HH:MM:SS),
            "target_hz", "max_buffer", "n_samples".
        rows (list): 7-tuples already in column order (t,fx,fy,fz,x,y,z).
    Outputs:
        data (bytes): ASCII bytes for the CSV file, Time at 3 decimals,
        the six force/pose columns at 6.
    --------------------------------------------------------------------------
    """
    polyscope_default = "3.11.0.82155 (20 August 2019)"
    lines = [
        "# Robot Model: %s" % meta.get("robot_model", "UR5 CB3"),
        "# PolyScope Version: %s" % meta.get("polyscope_version", polyscope_default),
        "# File Creation Date: %s" % meta.get("date", ""),
        "# File Creation Time: %s" % meta.get("time", ""),
        "# Target Acquisition Frequency: %s Hz" % meta.get("target_hz", 50),
        "# Maximum Buffer Size: %s" % meta.get("max_buffer", MAX_SAMPLES),
        "# Actual Number of Collected Samples: %s" % meta.get("n_samples", len(rows)),
        HEADER,
    ]
    row_fmt = "{0:.3f},{1:.6f},{2:.6f},{3:.6f},{4:.6f},{5:.6f},{6:.6f}"
    lines.extend([row_fmt.format(*row) for row in rows])
    text = "\n".join(lines) + "\n"
    return text.encode("ascii")

def next_free_name(dir_listing, stem):
    """
    --------------------------------------------------------------------------
    Purpose:
        Turn a stem into a collision-free "<stem>.csv" name, then
        "<stem>_1.csv", "<stem>_2.csv", ... against a directory listing.
        Overwriting is impossible by construction.
    Inputs:
        dir_listing (iterable of str): existing filenames (os.listdir()).
        stem (str): filename stem, normally "ACQ_log_YYYYMMDD_HHMMSS".
    Outputs:
        filename (str): "<stem>.csv" or the first free numbered variant.
    --------------------------------------------------------------------------
    """
    existing = set(dir_listing)
    candidate = "%s.csv" % stem
    if candidate not in existing:
        return candidate
    i = 1
    while True:
        candidate = "%s_%d.csv" % (stem, i)
        if candidate not in existing:
            return candidate
        i += 1

class SessionResult(object):
    """
    --------------------------------------------------------------------------
    Purpose:
        Outcome of one session, returned by LogServer.serve_one_session().
        Plain data holder, no behaviour.
    Inputs:
        path (str or None), n_rows (int), n_bad (int), ft_ok (bool):
        CSV path (None if never written), rows appended, unparsable lines,
        whether FTReader had synced a frame by session end.
        truncated (bool): True once MAX_SAMPLES was reached (later samples
            drained but never appended).
    Outputs:
        none (stores the five inputs above as attributes).
    --------------------------------------------------------------------------
    """
    def __init__(self, path, n_rows, n_bad, ft_ok, truncated):
        self.path = path
        self.n_rows = n_rows
        self.n_bad = n_bad
        self.ft_ok = ft_ok
        self.truncated = truncated

def _safe_close(closable):
    """Close a socket or socket-file, ignoring any error (shutdown path)."""
    try:
        closable.close()
    except socket.error:
        pass

class FTReader(object):
    """
    --------------------------------------------------------------------------
    Purpose:
        Background reader for the FT-300 text stream: keeps only the
        latest (Fx, Fy, Fz) behind a lock, reconnects with a 1 s backoff,
        logs only on a connect/disconnect state change, and prints exactly
        one RAW sample the first time it ever connects, before parsing it -
        the wire format for this URCap build is unconfirmed, and a
        mismatch must be seen in the log, not silently miscolumned into a
        CSV (plan section 8).
    Inputs:
        host (str): FT-300 host, "127.0.0.1" on the robot controller.
        port (int): FT-300 port, DEFAULT_FT_PORT unless overridden.
    Outputs:
        none directly; state is read through .latest() and .ok.
    --------------------------------------------------------------------------
    """
    def __init__(self, host, port):
        self._host = host
        self._port = port
        self._lock = threading.Lock()
        self._latest = (0.0, 0.0, 0.0)
        self.ok = False
        self._stop_flag = threading.Event()
        self._thread = None
        self._raw_logged = False

    def start(self):
        """Start the background reconnect-and-parse thread (daemon thread)."""
        self._stop_flag.clear()
        self._thread = threading.Thread(target=self._run)
        self._thread.setDaemon(True)
        self._thread.start()

    def stop(self):
        """Signal the background thread to exit and wait briefly for it."""
        self._stop_flag.set()
        if self._thread is not None:
            self._thread.join(2.0)

    def latest(self):
        """Return the last parsed (fx, fy, fz), or (0.0, 0.0, 0.0) before it."""
        with self._lock:
            return self._latest

    def _run(self):
        """Reconnect loop: connect, drain lines until dropped, back off 1 s."""
        was_connected = False
        while not self._stop_flag.is_set():
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2.0)
                sock.connect((self._host, self._port))
                sock.settimeout(None)
                if not was_connected:
                    print("[ACQ] FT-300 connected at %s:%d" % (self._host, self._port))
                    was_connected = True
                self._read_loop(sock)
            except socket.error as exc:
                if was_connected:
                    print("[ACQ] FT-300 disconnected: %s" % exc)
                    was_connected = False
            finally:
                if sock is not None:
                    _safe_close(sock)
            if not self._stop_flag.is_set():
                self._stop_flag.wait(1.0)

    def _read_loop(self, sock):
        """Read lines from one live connection until it drops or stop()."""
        sock_file = sock.makefile("rb")
        try:
            while not self._stop_flag.is_set():
                raw = sock_file.readline()
                if not raw:
                    raise socket.error("FT-300 stream closed")
                line = raw.decode("ascii", "replace")
                if not self._raw_logged:
                    print("[ACQ] FT-300 raw sample: %r" % line.strip())
                    self._raw_logged = True
                parsed = parse_ft_line(line)
                if parsed is not None:
                    with self._lock:
                        self._latest = parsed
                    self.ok = True
        finally:
            _safe_close(sock_file)

class LogServer(object):
    """
    --------------------------------------------------------------------------
    Purpose:
        Loopback TCP server the URScript data_logger thread streams into.
        ASCII line protocol: a sample line appends to the RAM list (no
        reply); an unparsable line counts into n_bad (no reply); "STOP
        <n>" writes the CSV, fsyncs, replies "OK <file> <n>"; a write
        failure replies "ERR <reason>" and keeps the buffer for this
        connection intact, so a later "RETRY" rewrites and replies "OK
        ..." or "ERR ...". At MAX_SAMPLES the server stops appending but
        keeps draining so the robot thread never blocks on a full TCP
        buffer, and marks the session truncated.
    Inputs:
        port (int): TCP port to bind, DEFAULT_LOG_PORT unless overridden.
        ft_reader (FTReader-like): supplies .latest() and .ok per row.
        out_dir_resolver (callable): zero-arg -> CSV output directory
            (real: detected USB mount or /tmp; test: temp dir).
        clock (callable): zero-arg -> seconds since the epoch (real:
            time.time; test: a fixed fake value).
    Outputs:
        none directly; see .serve_one_session().
    --------------------------------------------------------------------------
    """
    def __init__(self, port, ft_reader, out_dir_resolver, clock):
        self._port = port
        self._ft_reader = ft_reader
        self._out_dir_resolver = out_dir_resolver
        self._clock = clock
        self._sock = None

    def start(self):
        """Bind and listen on 127.0.0.1:port (one connection at a time)."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", self._port))
        self._sock.listen(1)
        print("[ACQ] log server listening on 127.0.0.1:%d" % self._port)

    def stop(self):
        """Close the listening socket, if open."""
        if self._sock is not None:
            _safe_close(self._sock)
            self._sock = None

    def serve_one_session(self):
        """Accept one connection, serve it per the class-docstring protocol,
        and return a SessionResult (result.path is None if the connection
        dropped before a write ever succeeded)."""
        conn, addr = self._sock.accept()
        print("[ACQ] connection from %s" % (addr,))
        rows = []
        n_bad = 0
        truncated = False
        robot_count = None
        sock_file = conn.makefile("rb")
        try:
            while True:
                raw = sock_file.readline()
                if not raw:
                    print("[ACQ] connection closed before STOP")
                    return SessionResult(None, len(rows), n_bad, self._ft_reader.ok, truncated)
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                if line == "RETRY" or line.startswith("STOP"):
                    result = self._finalize(conn, rows, n_bad, truncated,
                                            robot_count)
                    if result is not None:
                        return result
                    continue
                parsed = parse_sample_line(line)
                if parsed is None:
                    n_bad += 1
                    continue
                # Sentinelle de comptage : le script CB3 ne peut pas construire
                # "STOP <n>" (ni to_str ni str_cat sur PolyScope 3.x), donc le
                # compte arrive comme une liste a premier champ negatif, juste
                # avant le litteral "STOP". Elle se parse comme un echantillon
                # valide : sans ce test elle serait ecrite en derniere ligne de
                # donnees, avec Time = -1.000 et PoseX = le compte.
                # Testee avant le plafond du tampon, sinon une session tronquee
                # la perdrait, et c'est justement la que le compte compte.
                if parsed[0] < 0:
                    robot_count = int(parsed[1])
                    print("[ACQ] robot reports %d samples sent" % robot_count)
                    continue
                if len(rows) >= MAX_SAMPLES:
                    truncated = True
                    continue
                rows.append(self._to_csv_row(parsed))
        finally:
            _safe_close(sock_file)
            _safe_close(conn)

    def _to_csv_row(self, parsed):
        """Reorder one parsed sample to CSV column order (t,fx,fy,fz,x,y,z)."""
        if len(parsed) == 7:
            t, x, y, z, fx, fy, fz = parsed
        else:
            t, x, y, z = parsed
            fx, fy, fz = self._ft_reader.latest()
        return (t, fx, fy, fz, x, y, z)

    def _finalize(self, conn, rows, n_bad, truncated, robot_count=None):
        """Write the CSV to the resolved directory; a failure keeps rows intact.

        robot_count is what the robot says it sent. A disagreement with the
        number of rows received is reported, never corrected: it is the only
        evidence available that lines were dropped on the way.
        """
        if robot_count is not None and robot_count != len(rows):
            print("[ACQ] WARNING: robot sent %d samples, %d received (%d lost)"
                  % (robot_count, len(rows), robot_count - len(rows)))
        try:
            out_dir = self._out_dir_resolver()
            if not os.path.isdir(out_dir):
                os.makedirs(out_dir)
            listing = os.listdir(out_dir)
            now = time.localtime(self._clock())
            stamp = time.strftime("%Y%m%d_%H%M%S", now)
            filename = next_free_name(listing, "ACQ_log_%s" % stamp)
            path = os.path.join(out_dir, filename)
            meta = {"n_samples": len(rows), "max_buffer": MAX_SAMPLES,
                     "date": time.strftime("%Y-%m-%d", now),
                     "time": time.strftime("%H:%M:%S", now),
                     "robot_count": robot_count}
            data = format_csv(meta, rows)
            out_file = open(path, "wb")
            try:
                out_file.write(data)
                out_file.flush()
                os.fsync(out_file.fileno())
            finally:
                out_file.close()
        except (IOError, OSError) as exc:
            self._reply(conn, "ERR %s" % exc)
            return None
        self._reply(conn, "OK %s %d" % (filename, len(rows)))
        return SessionResult(path, len(rows), n_bad, self._ft_reader.ok, truncated)

    def _reply(self, conn, text):
        """Send one ASCII reply line, newline-terminated.

        Encoded with "replace", never strict. An ERR line carries the operating
        system's own error text, which is localized: on a French Windows the
        message for an existing directory is "Impossible de creer un fichier
        deja existant". Strict ASCII raises UnicodeEncodeError here, inside the
        server thread, so the one path whose entire purpose is to tell the robot
        the write failed would itself fail, silently, and the robot would wait
        for a reply that never comes. A mangled character in a diagnostic is
        always better than no diagnostic.
        """
        conn.sendall((text + "\n").encode("ascii", "replace"))

def _default_out_dir_resolver():
    """Real USB-mount resolver: /proc/mounts, else /tmp with a warning."""
    try:
        mounts_file = open("/proc/mounts", "r")
        text = mounts_file.read()
        mounts_file.close()
    except (IOError, OSError):
        text = ""
    mount = find_usb_mount(text)
    if mount is None:
        print("[ACQ] WARNING: no USB mount under /media or /programs, using /tmp")
        return "/tmp"
    return mount

def main(argv):
    """Wire the real FTReader/LogServer and serve sessions until killed."""
    log_port = int(argv[0]) if len(argv) > 0 else DEFAULT_LOG_PORT
    ft_port = int(argv[1]) if len(argv) > 1 else DEFAULT_FT_PORT
    ft_reader = FTReader("127.0.0.1", ft_port)
    ft_reader.start()
    server = LogServer(log_port, ft_reader, _default_out_dir_resolver, time.time)
    server.start()
    print("[ACQ] daemon ready")
    try:
        while True:
            r = server.serve_one_session()
            print("[ACQ] session done: path=%s n_rows=%d n_bad=%d ft_ok=%s "
                  "truncated=%s" % (r.path, r.n_rows, r.n_bad, r.ft_ok, r.truncated))
    except KeyboardInterrupt:
        print("[ACQ] daemon stopping")
    finally:
        server.stop()
        ft_reader.stop()

if __name__ == "__main__":
    main(sys.argv[1:])
