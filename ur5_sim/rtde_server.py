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
import struct
from typing import Sequence

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
