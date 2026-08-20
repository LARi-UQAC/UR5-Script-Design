"""
RTDE emulator - server side of the Real-Time Data Exchange protocol.

Lets ur5_sim present itself to datalogger/rtde_fallback_monitor.exe exactly as
a UR5 CB3 controller does, so the monitor can be exercised end to end with no
robot present. Output packages only: there is no input path, no register write
and no motion interface, so nothing that speaks to this server can command it.

Design: ../docs/superpower/specs/spec_rtde_emulator.md
Counterpart: ../datalogger/rtde_fallback_monitor.c

Every constant below is duplicated in that C file by necessity. tests/
test_rtde_server.py pins them from this side so a drift fails a test rather
than filling a lab CSV with plausible wrong numbers.
"""

from __future__ import annotations

import bisect
import socket
import struct
import threading
import time
from typing import Sequence, TYPE_CHECKING

if TYPE_CHECKING:  # typing only, so the runtime import graph stays flat
    from ur5_sim.force_model import ForceModel

# --- RTDE package types (one byte, after the 2-byte big-endian size) ---
RTDE_REQUEST_PROTOCOL_VERSION: int = 86       # 'V'
RTDE_TEXT_MESSAGE: int = 77                   # 'M'
RTDE_DATA_PACKAGE: int = 85                   # 'U'
RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS: int = 79  # 'O'
RTDE_CONTROL_PACKAGE_START: int = 83          # 'S'

RTDE_HEADER_SIZE: int = 3

# --- Output recipe served to the client ---
RTDE_OUTPUT_RECIPE: str = (
    "timestamp,actual_TCP_pose,actual_TCP_force,runtime_state"
)
RTDE_OUTPUT_TYPES: str = "DOUBLE,VECTOR6D,VECTOR6D,UINT32"

# --- Byte layout of one data payload, given that recipe ---
FIELD_OFF_TIMESTAMP: int = 0
FIELD_OFF_TCP_POSE: int = 8       # VECTOR6D
FIELD_OFF_TCP_FORCE: int = 56     # VECTOR6D
FIELD_OFF_RUNTIME_STATE: int = 104  # UINT32
RTDE_PAYLOAD_SIZE: int = 108

# --- runtime_state enumeration ---
RT_STOPPING: int = 0
RT_STOPPED: int = 1
RT_PLAYING: int = 2
RT_PAUSING: int = 3
RT_PAUSED: int = 4
RT_RESUMING: int = 5

_HEADER_STRUCT = struct.Struct(">HB")
_PAYLOAD_STRUCT = struct.Struct(">d6d6dI")


def encode_packet(pkg_type: int, payload: bytes) -> bytes:
    """
    --------------------------------------------------------------------------
    Purpose:
        Frame one RTDE package: 2-byte big-endian total size (header included),
        then the 1-byte type, then the body.

    Inputs:
        pkg_type (int): RTDE package type code.
        payload (bytes): body, possibly empty.

    Outputs:
        packet (bytes): the framed package, ready to send.
    --------------------------------------------------------------------------
    """
    return _HEADER_STRUCT.pack(RTDE_HEADER_SIZE + len(payload), pkg_type) + payload


def encode_data_payload(
    timestamp: float,
    pose6: Sequence[float],
    force6: Sequence[float],
    runtime_state: int,
) -> bytes:
    """
    --------------------------------------------------------------------------
    Purpose:
        Encode one RTDE_DATA_PACKAGE body for the served recipe. The single
        struct format is what guarantees the field offsets the C monitor
        decodes at; it is never assembled field by field.

    Inputs:
        timestamp (float): controller clock, seconds.
        pose6 (Sequence[float]): actual_TCP_pose, 6 values (m and rad).
        force6 (Sequence[float]): actual_TCP_force, 6 values (N and Nm).
        runtime_state (int): program execution state, RT_* above.

    Outputs:
        payload (bytes): exactly RTDE_PAYLOAD_SIZE bytes, big-endian.
    --------------------------------------------------------------------------
    """
    return _PAYLOAD_STRUCT.pack(timestamp, *pose6, *force6, runtime_state)


class RunStateMachine:
    """
    Program-execution state as the controller reports it.

    Transitions are not instantaneous on a real CB3: a pause is PAUSING then
    PAUSED, a resume is RESUMING then PLAYING, a stop is STOPPING then STOPPED.
    Only a start is immediate. Reproducing that is what makes the monitor's
    file-boundary logic face the real enum sequence rather than a simplified
    two-state one.
    """

    def __init__(self, transition_packets: int = 2) -> None:
        self._state: int = RT_STOPPED
        self._pending: int | None = None
        self._left: int = 0
        self._transition_packets: int = max(1, int(transition_packets))

    def request(self, target: int) -> None:
        """
        ----------------------------------------------------------------------
        Purpose:
            Ask for a stable state. Ignored when it is already current or
            already pending, so a double-click cannot restart a transition.

        Inputs:
            target (int): RT_PLAYING, RT_PAUSED or RT_STOPPED.

        Outputs:
            None.
        ----------------------------------------------------------------------
        """
        if target == self._state or target == self._pending:
            return
        if target == RT_PLAYING and self._state == RT_STOPPED:
            self._state = RT_PLAYING
            self._pending = None
            self._left = 0
        elif target == RT_PLAYING and self._state == RT_PAUSED:
            self._begin(RT_RESUMING, RT_PLAYING)
        elif target == RT_PAUSED and self._state == RT_PLAYING:
            self._begin(RT_PAUSING, RT_PAUSED)
        elif target == RT_STOPPED and self._state != RT_STOPPED:
            self._begin(RT_STOPPING, RT_STOPPED)

    def _begin(self, transient: int, final: int) -> None:
        self._state = transient
        self._pending = final
        self._left = self._transition_packets

    def next_state(self) -> int:
        """
        ----------------------------------------------------------------------
        Purpose:
            Return the state for the packet about to be sent, and advance any
            transition. Call exactly once per emitted packet.

        Inputs:
            None.

        Outputs:
            state (int): the RT_* value to put in this packet.
        ----------------------------------------------------------------------
        """
        state = self._state
        if self._pending is not None:
            self._left -= 1
            if self._left <= 0:
                self._state = self._pending
                self._pending = None
        return state


def interpolate_pose(
    poses: Sequence[Sequence[float]],
    times: Sequence[float],
    t: float,
) -> tuple[float, ...]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Sample the trajectory at an arbitrary time. The trajectory exists only
        every DT (0.05 s, 20 Hz) but the emulator emits at 125 Hz, so the
        intermediate points have to be built here - exactly what a controller
        does between waypoints.

        Translation is linearly interpolated. Orientation is taken from the
        nearer frame rather than interpolated: the exported trajectory holds
        orientation constant within a cycle, so interpolating it buys nothing
        and would pull in a rotation library this module deliberately avoids.

    Inputs:
        poses (Sequence[Sequence[float]]): per-frame (x, y, z, rx, ry, rz).
        times (Sequence[float]): ascending frame times, same length as poses.
        t (float): wanted time, seconds. Clamped to the trajectory span.

    Outputs:
        pose (tuple[float, ...]): 6 values at time t.
    --------------------------------------------------------------------------
    """
    if len(poses) == 1 or t <= times[0]:
        return tuple(poses[0])
    if t >= times[-1]:
        return tuple(poses[-1])

    i = bisect.bisect_right(times, t) - 1
    i = min(max(i, 0), len(poses) - 2)
    span = times[i + 1] - times[i]
    alpha = 0.0 if span <= 0.0 else (t - times[i]) / span

    a, b = poses[i], poses[i + 1]
    nearer = a if alpha < 0.5 else b
    return (
        a[0] + (b[0] - a[0]) * alpha,
        a[1] + (b[1] - a[1]) * alpha,
        a[2] + (b[2] - a[2]) * alpha,
        nearer[3], nearer[4], nearer[5],
    )


def _sample_index(times: Sequence[float], count: int, t: float) -> int:
    """
    --------------------------------------------------------------------------
    Purpose:
        Index of the frame in effect at time t, for the per-frame flags that
        are not interpolated (contact, penetration). Takes the sequences as
        arguments rather than reading them off the server, so the caller can
        resolve an index against the same snapshot it will index into.

    Inputs:
        times (Sequence[float]): ascending frame times.
        count (int): number of usable frames in the list about to be indexed.
        t (float): wanted time, seconds.

    Outputs:
        index (int): clamped to [0, count - 1]; 0 when count is 0, which the
            caller must treat as "no per-frame data" rather than as frame 0.
    --------------------------------------------------------------------------
    """
    if count <= 0:
        return 0
    i = bisect.bisect_right(times, t) - 1
    return min(max(i, 0), count - 1)


class RtdeServer:
    """
    Serves one RTDE client on loopback at the controller rate.

    The server owns the run: load_run() hands it the trajectory once, then
    set_run_state() only reports what the viewer is doing. Interpolation,
    force synthesis, encoding and pacing all happen in the emitter thread, so
    the stream never inherits the render loop's timer jitter and the server can
    be driven by a test with no GUI at all.

    One client at a time. Real RTDE accepts several; this is a deliberate
    simplification for a test rig.
    """

    def __init__(
        self,
        host: str,
        port: int,
        rate_hz: float,
        force_model: "ForceModel | None" = None,
        transition_packets: int = 2,
    ) -> None:
        self.host = host
        self._requested_port = int(port)
        self._port = int(port)
        self._period = 1.0 / float(rate_hz)
        self._force_model = force_model
        self._machine = RunStateMachine(transition_packets=transition_packets)
        self._recipe_id = 1

        self._listener: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        self._poses: list[tuple[float, ...]] = [(0.0,) * 6]
        self._times: list[float] = [0.0]
        self._contact: list[bool] = [False]
        self._penetration: list[float] = [0.0]

        self._sim_time = 0.0
        self._t0 = time.perf_counter()
        self._prev_xy: tuple[float, float] | None = None
        self._dropped = 0

    # -- lifecycle ----------------------------------------------------

    def start(self) -> bool:
        """
        ----------------------------------------------------------------------
        Purpose:
            Bind, listen and start the emitter thread. A busy port is reported,
            never raised: the visualizer must run with or without the emulator.

        Inputs:
            None.

        Outputs:
            ok (bool): True when serving, False when the port was unavailable.
        ----------------------------------------------------------------------
        """
        listener: socket.socket | None = None
        try:
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Explicitly OFF. On Windows SO_REUSEADDR lets a second socket bind
            # a port already in use and silently steal connections, so leaving
            # it on would hide a duplicate emulator instead of reporting it.
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
            listener.bind((self.host, self._requested_port))
            listener.listen(1)
            listener.settimeout(0.2)
        except OSError as exc:
            # Closed here: a socket that failed to bind still holds a handle,
            # and leaking one per attempt turns a degraded path into a leak.
            if listener is not None:
                try:
                    listener.close()
                except OSError:
                    pass
            print(f"[rtde-emu] port {self._requested_port} unavailable ({exc}); "
                  f"continuing without RTDE")
            return False

        self._listener = listener
        self._port = listener.getsockname()[1]
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._serve_forever, name="rtde-emu", daemon=True)
        self._thread.start()
        print(f"[rtde-emu] serving RTDE on {self.host}:{self._port} "
              f"(simulated source, not a robot)")
        return True

    def stop(self) -> None:
        """Idempotent shutdown; safe from a finally block."""
        self._stop_event.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)
        listener, self._listener = self._listener, None
        if listener is not None:
            try:
                listener.close()
            except OSError:
                pass
            dropped = self.dropped_packets
            if dropped:
                print(f"[rtde-emu] {dropped} packet(s) dropped to a slow client")

    @property
    def port(self) -> int:
        return self._port

    @property
    def dropped_packets(self) -> int:
        with self._lock:
            return self._dropped

    # -- inputs from the simulator ------------------------------------

    def load_run(
        self,
        poses: Sequence[Sequence[float]],
        times: Sequence[float],
        in_contact: Sequence[bool],
        penetration_m: Sequence[float],
    ) -> None:
        """Hand the server the trajectory it will replay. Plain floats only."""
        with self._lock:
            self._poses = [tuple(float(v) for v in p) for p in poses]
            self._times = [float(t) for t in times]
            self._contact = [bool(c) for c in in_contact]
            self._penetration = [float(d) for d in penetration_m]

    def set_run_state(self, running: bool, sim_time: float, finished: bool) -> None:
        """Report what the viewer is doing. Called from the render tick."""
        with self._lock:
            self._sim_time = float(sim_time)
            if finished or (not running and sim_time <= 0.0):
                self._machine.request(RT_STOPPED)
            elif running:
                self._machine.request(RT_PLAYING)
            else:
                self._machine.request(RT_PAUSED)

    # -- emitter ------------------------------------------------------

    def _build_payload(self) -> bytes:
        with self._lock:
            sim_t = self._sim_time
            poses, times = self._poses, self._times
            contact, penetration = self._contact, self._penetration
            state = self._machine.next_state()

        pose = interpolate_pose(poses, times, sim_t)
        # Resolved against the four lists just snapshotted, never by reading
        # self._* a second time: load_run() rebinds all four at once, so a
        # second read could pair an index taken from one trajectory with a
        # contact flag taken from the next, or raise IndexError in a thread
        # whose death would look to the client like a disconnection.
        count = min(len(contact), len(penetration))
        idx = _sample_index(times, count, sim_t)
        contact_now = bool(contact[idx]) if count else False
        penetration_now = float(penetration[idx]) if count else 0.0

        vx = vy = 0.0
        if self._prev_xy is not None:
            vx = (pose[0] - self._prev_xy[0]) / self._period
            vy = (pose[1] - self._prev_xy[1]) / self._period
        self._prev_xy = (pose[0], pose[1])

        moving = state in (RT_PLAYING, RT_PAUSING, RT_RESUMING)
        if self._force_model is not None:
            fx, fy, fz = self._force_model.step(
                dt=self._period,
                in_contact=contact_now and moving,
                penetration_m=penetration_now,
                vx=vx if moving else 0.0,
                vy=vy if moving else 0.0,
            )
        else:
            fx = fy = fz = 0.0

        timestamp = time.perf_counter() - self._t0
        return encode_data_payload(timestamp, pose, (fx, fy, fz, 0.0, 0.0, 0.0), state)

    def _serve_forever(self) -> None:
        while not self._stop_event.is_set():
            listener = self._listener
            if listener is None:
                return
            try:
                client, _ = listener.accept()
            except (socket.timeout, TimeoutError):
                continue
            except OSError:
                return
            try:
                self._session(client)
            except (OSError, ConnectionError, struct.error) as exc:
                print(f"[rtde-emu] client session ended: {exc!r}")
            finally:
                try:
                    client.close()
                except OSError:
                    pass

    def _session(self, client: socket.socket) -> None:
        # A deadline on the handshake, not just on the stream. The accept loop
        # serves one client at a time, so a socket that connects and then says
        # nothing - a port scan, or the monitor killed between connect and
        # send - would otherwise hold this thread in recv() for ever and lock
        # every later client out. 10 s matches the deadline the C monitor
        # applies to the same exchange from its side.
        client.settimeout(10.0)
        version = self._handshake(client)
        if version is None:
            return
        # Blocking with a send deadline, NOT non-blocking. A non-blocking
        # sendall can deliver a partial packet and desync the stream framing
        # for good, which is far worse than the stall it avoids - and the
        # stall it avoids is confined to this thread anyway, so it can never
        # reach the render loop. A client too slow to drain within the
        # deadline is treated as gone; the monitor reconnects on its own.
        client.settimeout(max(1.0, self._period * 50))
        # Velocity is a difference between consecutive emitted frames, so a
        # value left over from the previous client would make the first force
        # sample of this one a spike out of nowhere.
        self._prev_xy = None
        next_send = time.perf_counter()
        while not self._stop_event.is_set():
            now = time.perf_counter()
            if now < next_send:
                time.sleep(min(self._period, next_send - now))
                continue
            next_send += self._period
            if next_send < now:                       # fell behind, re-anchor
                next_send = now + self._period

            body = self._build_payload()
            if version >= 2:
                body = bytes([self._recipe_id]) + body
            try:
                client.sendall(encode_packet(RTDE_DATA_PACKAGE, body))
            except (socket.timeout, TimeoutError):
                # A sendall that timed out may already have written part of
                # the packet, and there is no way to learn how much. The only
                # safe move is to drop the connection: resuming on it would
                # desync the framing for as long as the client stays up.
                with self._lock:
                    self._dropped += 1
                print("[rtde-emu] client did not drain in time; dropping it")
                return
            except OSError:
                return

    def _recv_packet(self, client: socket.socket) -> tuple[int, bytes]:
        head = b""
        while len(head) < RTDE_HEADER_SIZE:
            chunk = client.recv(RTDE_HEADER_SIZE - len(head))
            if not chunk:
                raise ConnectionError("client closed during handshake")
            head += chunk
        size, pkg_type = _HEADER_STRUCT.unpack(head)
        body = b""
        while len(body) < size - RTDE_HEADER_SIZE:
            chunk = client.recv(size - RTDE_HEADER_SIZE - len(body))
            if not chunk:
                raise ConnectionError("client closed during handshake")
            body += chunk
        return pkg_type, body

    def _handshake(self, client: socket.socket) -> int | None:
        """
        ----------------------------------------------------------------------
        Purpose:
            Answer the three setup packages a CB3 controller answers, in the
            order datalogger/rtde_fallback_monitor.c sends them: protocol
            version, output recipe, start. Version 2 prefixes the recipe reply
            and every data package with the recipe id, version 1 has no such
            byte, and the C client decodes at an offset chosen from the version
            negotiated here, so a wrong reply shifts every field it reads.

        Inputs:
            client (socket.socket): the accepted connection.

        Outputs:
            version (int | None): 1 or 2 once the stream is started, None when
                the client sent something else and the session is abandoned.
        ----------------------------------------------------------------------
        """
        version = 2
        while True:
            pkg_type, body = self._recv_packet(client)
            if pkg_type != RTDE_REQUEST_PROTOCOL_VERSION:
                return None
            want = struct.unpack(">H", body[:2])[0]
            accepted = 1 if want in (1, 2) else 0
            client.sendall(encode_packet(
                RTDE_REQUEST_PROTOCOL_VERSION, bytes([accepted])))
            if accepted:
                version = want
                break

        # The output frequency the version-2 body carries is accepted and
        # ignored: this server emits at the rate it was constructed with, and
        # the monitor takes its cadence from the packet timestamps anyway.
        pkg_type, _ = self._recv_packet(client)
        if pkg_type != RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
            return None
        reply = RTDE_OUTPUT_TYPES.encode("ascii")
        if version >= 2:
            reply = bytes([self._recipe_id]) + reply
        client.sendall(encode_packet(RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, reply))

        pkg_type, _ = self._recv_packet(client)
        if pkg_type != RTDE_CONTROL_PACKAGE_START:
            return None
        client.sendall(encode_packet(RTDE_CONTROL_PACKAGE_START, bytes([1])))
        return version
