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

import socket
import struct
import threading
import time
from typing import Sequence, TYPE_CHECKING

if TYPE_CHECKING:  # typing only, so the runtime import graph stays flat
    from ur5_sim.force_model import ForceModel

# The wire layer (package framing, the output recipe, the runtime-state
# machine, trajectory sampling) moved to ur5_sim/rtde_wire.py to respect the
# project's per-file token ceiling. This re-export is deliberate: every name
# importable from ur5_sim.rtde_server before the split stays importable from
# it after, so tests/test_rtde_server.py needed no changes.
from ur5_sim.rtde_wire import (
    FIELD_OFF_RUNTIME_STATE,
    FIELD_OFF_TCP_FORCE,
    FIELD_OFF_TCP_POSE,
    FIELD_OFF_TIMESTAMP,
    RT_PAUSED,
    RT_PAUSING,
    RT_PLAYING,
    RT_RESUMING,
    RT_STOPPED,
    RT_STOPPING,
    RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS,
    RTDE_CONTROL_PACKAGE_START,
    RTDE_DATA_PACKAGE,
    RTDE_HEADER_SIZE,
    RTDE_OUTPUT_RECIPE,
    RTDE_OUTPUT_TYPES,
    RTDE_PAYLOAD_SIZE,
    RTDE_REQUEST_PROTOCOL_VERSION,
    RTDE_TEXT_MESSAGE,
    RunStateMachine,
    _HEADER_STRUCT,
    _PAYLOAD_STRUCT,
    _sample_index,
    encode_data_payload,
    encode_packet,
    interpolate_pose,
)
# Same reason, same shape: the headless driver lives in ur5_sim/rtde_headless.py
# and is re-exported here, so ur5_sim.rtde_server.run_headless stays the public
# spelling whatever the file layout underneath.
from ur5_sim.rtde_headless import run_headless  # noqa: F401


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
