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
