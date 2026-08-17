# RTDE Emulator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. This is decided, not a preference — see "Execution model" below; do not fall back to superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax for tracking.

> **Starting cold?** Read, in this order: this header, "Execution model", "Model routing",
> "Defects: two regimes", and "Global Constraints". Then dispatch Task 1. You do not need to
> read the whole file, and no subagent should be given it.

**Goal:** Make `ur5_sim` serve a UR5 CB3 RTDE stream on loopback so the already-built `rtde_fallback_monitor.exe` can be tested end to end with no robot present.

**Architecture:** A new `ur5_sim/rtde_server.py` owns a TCP server and a 125 Hz emitter thread. The viewer hands it the trajectory once, then only reports run state; interpolation, force synthesis, encoding and pacing all happen inside the thread, so the stream never inherits matplotlib's timer jitter and the server tests without a GUI. A companion `ur5_sim/force_model.py` synthesises a plausible FT-300 signal from the simulator's own penetration depth. The existing UDP telemetry to the design UI is untouched.

**Tech Stack:** Python 3.13, stdlib only for the two new modules (`socket`, `struct`, `threading`, `random`, `math`, `bisect`). Tests are stdlib `unittest`. The counterpart is C (MinGW-w64) in `datalogger/`.

**Spec:** [`../specs/spec_rtde_emulator.md`](../specs/spec_rtde_emulator.md)

**Branch:** none. Work directly on `main` and commit there, task by task. This repo has a
single developer and no feature-branch workflow; a branch here only fragments the plan
across refs, which has already happened once.

## Execution model - subagents, one per task

**Decided 2026-08-16: this plan is executed by subagents, one per task, not inline.**

The reason is cost, and the gap is roughly an order of magnitude rather than a few percent.
A session that has been designing this work carries the monitor's C source, its test
harness, the spec and this plan's 2556 lines in its context, and re-processes all of it at
every step. A subagent receives only its own task section, about 2 000 tokens, plus the
Global Constraints below. Seven of the ten tasks then run on Sonnet, which costs roughly a
fifth of Opus per token.

Consequences for whoever drives this:

- **Give each subagent its task section and the Global Constraints, not the whole file.**
  Each task carries its own test code, implementation code, verification command and commit,
  precisely so a cold agent can execute it without reading the rest.
- **Honor the Model line on each task.** It is the whole point of the split; ignoring it
  puts the cheap work on the expensive model and wastes the arrangement.
- **The driver reviews between tasks and runs the full suite.** No agent grades its own
  work - the same rule the defect register applies to its own fixes.
- **Do not run two subagents on tasks that touch the same file.** Tasks 6 and 7 both edit
  `viewer.py`; they are sequential, never parallel.

## Model routing

Each task carries a **Model:** line. Use it — it is the difference between a
cheap run and an expensive one, and seven of the ten tasks do not need the
expensive model.

| Model | When | Tasks |
|---|---|---|
| **Sonnet** | The task's code and tests are given in full below. The work is transcription plus running the verification. No judgment about existing code is required. | 1, 2, 3, 4, 8, 9, 10 |
| **Opus** | The task edits existing files whose structure must be read and understood first, or carries a correctness trap that a plausible-looking implementation would fall into. | 5, 6, 7 |

Two rules that keep the routing honest:

- **A Sonnet task that turns out to need judgment is a plan defect, not a
  model failure.** Stop, say which step was underspecified, and escalate that
  task rather than guessing.
- **Never downgrade tasks 5, 6 or 7.** Their reasons are stated in their own
  Model lines and are specific, not generic caution.

## Defects: two regimes, and which one you are in

This work has two halves, and they treat a newly found bug in **opposite** ways. Know which
half you are in before touching anything.

### Regime A - implementing Tasks 1 to 10 (feature work)

You will find faults in existing code that this plan does not cover. **Do not fix them
here.** Log them in [`erreur_hors_datalogger.md`](erreur_hors_datalogger.md) and carry on
with the task.

The reason is reviewability: a diff that both adds the emulator and repairs unrelated code
cannot be judged on either count, and the repair is the half that gets waved through.

If a task in this plan turns out to be underspecified, that is a **plan** defect: report it
against this file rather than logging it as a code fault in the register.

### Regime B - clearing the remaining register entries (repair work)

The same session also finishes the open items of
[`erreur_hors_datalogger.md`](erreur_hors_datalogger.md), following the procedure that file
already sets out: its "Writing protocol" for concurrent writes, and its own model column,
which is decided there and is **not** re-argued here.

**In this regime, a bug found while making a correction is fixed immediately**, not logged
for later. The reviewability argument of Regime A does not apply: the diff is already a
repair diff, so a second repair belongs in it, and deferring means re-deriving the same
context later at full cost.

Two things still hold while fixing immediately:

- **Record it anyway.** Since 2026-08-16 that file is a record as well as a backlog, with
  `FIXED` markers stating what was done and how it was verified. A fix that leaves no entry
  removes the only trace that the fault ever existed.
- **Keep it a separate commit** from the correction that uncovered it, so each remains
  reviewable on its own. Immediate means "in this session", not "in the same commit".

The one exception: if the newly found fault is High severity, or changes behavior the
operator depends on (an export that reaches the robot, the overwrite guard, a settings
bound), log it and stop for a decision instead of fixing it in passing. Those are the
entries the register itself routes to Opus for a decision, not to a fixer.

### Ownership right now, 2026-08-16

Four entries are open. **Another Claude Code session is working F10 live**, so files may
change under you mid-task.

| Entry | Owner |
|---|---|
| **F10** | Another session, in progress. **Do not touch it**, and expect `design/export.py`, the emitted header and `tests/fixtures/golden_headless.script` to move without warning. |
| **F15** | This session. Fully specified in its entry, Sonnet writes it, Opus reviews the diff and runs the C harness. |
| **F6** | This session, folded into Task 6: `viewer.py` is 916 lines and the register requires the split **before** PAUSE is added to it, not after. |
| **F7** | This session, and it *is* Task 6. Do not open a second correction for it. |

Because another session is writing concurrently, re-read any shared file immediately before
editing it - the register, `design/export.py`, the golden fixture - and never from a copy
read at session start.

## Global Constraints

- `ur5_sim/rtde_server.py` and `ur5_sim/force_model.py` import **stdlib only**. No matplotlib, no Swift, no spatialmath, no numpy.
- The emulator binds `127.0.0.1` only, never `0.0.0.0`. It must be unreachable from the lab VLAN.
- New constants go in `ur5_sim/config.py`, **not** `design/params.py`: they are simulation-only and never reach the exported script. This follows the existing `SIM_PROBE_*` precedent.
- Tests are stdlib `unittest`; pytest is NOT installed in `.venv`. Run with `python -m unittest ...`.
- No new Python test may require `gcc` or a built `.exe`; `python -m unittest discover -s tests -p "test_*.py"` must stay green on a machine with no C toolchain.
- No source file exceeds 4096 tokens (workspace `code-style.md`).
- Python naming: classes `PascalCase`, functions `snake_case`, constants `UPPER_SNAKE_CASE`, type hints in every signature.
- The RTDE wire constants are duplicated in C and Python by necessity. Exact values, which MUST match `datalogger/rtde_fallback_monitor.c`:
  - recipe `timestamp,actual_TCP_pose,actual_TCP_force,runtime_state`
  - types `DOUBLE,VECTOR6D,VECTOR6D,UINT32`
  - payload size `108`; offsets `timestamp 0`, `actual_TCP_pose 8`, `actual_TCP_force 56`, `runtime_state 104`
  - package types `86` version, `77` text, `85` data, `79` setup outputs, `83` start
  - header: 2-byte big-endian total size, then 1 byte type
- Force-model parameters are **plausible, not measured**. Every place they appear must say so.
- `etalement.script`, `etalement.urp`, `design/export.py` and the exported artifacts are NOT modified by any task in this plan.

---

### Task 1: RTDE wire encoder

**Model: Sonnet.** New file, code and tests given in full, nothing existing to read.

Pins the byte layout shared with the C monitor. This is the load-bearing task: if these constants drift from the C side, a lab CSV fills with plausible wrong numbers.

**Files:**
- Create: `ur5_sim/rtde_server.py`
- Test: `tests/test_rtde_server.py`

**Interfaces:**
- Consumes: nothing
- Produces: constants `RTDE_HEADER_SIZE`, `RTDE_PAYLOAD_SIZE`, `RTDE_OUTPUT_RECIPE`, `RTDE_OUTPUT_TYPES`, `FIELD_OFF_TIMESTAMP`, `FIELD_OFF_TCP_POSE`, `FIELD_OFF_TCP_FORCE`, `FIELD_OFF_RUNTIME_STATE`, `RTDE_REQUEST_PROTOCOL_VERSION`, `RTDE_TEXT_MESSAGE`, `RTDE_DATA_PACKAGE`, `RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS`, `RTDE_CONTROL_PACKAGE_START`, `RT_STOPPING`, `RT_STOPPED`, `RT_PLAYING`, `RT_PAUSING`, `RT_PAUSED`, `RT_RESUMING`; functions `encode_packet(pkg_type: int, payload: bytes) -> bytes` and `encode_data_payload(timestamp: float, pose6: Sequence[float], force6: Sequence[float], runtime_state: int) -> bytes`

- [ ] **Step 1: Write the failing test**

Create `tests/test_rtde_server.py`:

```python
"""
Tests for the RTDE emulator's wire layer.

The byte layout asserted here is duplicated in datalogger/rtde_fallback_monitor.c
by necessity (two languages, one protocol). Pinning it from both sides turns a
silent drift into a failing test instead of wrong numbers in a lab CSV.
"""

import struct
import unittest

from ur5_sim import rtde_server as rs


class WireConstantsTests(unittest.TestCase):
    """Every value here must equal the C constant of the same name."""

    def test_payload_size_and_field_offsets(self) -> None:
        self.assertEqual(rs.RTDE_PAYLOAD_SIZE, 108)
        self.assertEqual(rs.FIELD_OFF_TIMESTAMP, 0)
        self.assertEqual(rs.FIELD_OFF_TCP_POSE, 8)
        self.assertEqual(rs.FIELD_OFF_TCP_FORCE, 56)
        self.assertEqual(rs.FIELD_OFF_RUNTIME_STATE, 104)

    def test_recipe_matches_the_monitor(self) -> None:
        self.assertEqual(
            rs.RTDE_OUTPUT_RECIPE,
            "timestamp,actual_TCP_pose,actual_TCP_force,runtime_state",
        )
        self.assertEqual(rs.RTDE_OUTPUT_TYPES, "DOUBLE,VECTOR6D,VECTOR6D,UINT32")

    def test_package_type_codes(self) -> None:
        self.assertEqual(rs.RTDE_REQUEST_PROTOCOL_VERSION, 86)
        self.assertEqual(rs.RTDE_TEXT_MESSAGE, 77)
        self.assertEqual(rs.RTDE_DATA_PACKAGE, 85)
        self.assertEqual(rs.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, 79)
        self.assertEqual(rs.RTDE_CONTROL_PACKAGE_START, 83)

    def test_runtime_state_enumeration(self) -> None:
        self.assertEqual(
            (rs.RT_STOPPING, rs.RT_STOPPED, rs.RT_PLAYING,
             rs.RT_PAUSING, rs.RT_PAUSED, rs.RT_RESUMING),
            (0, 1, 2, 3, 4, 5),
        )


class EncoderTests(unittest.TestCase):

    def test_header_is_big_endian_size_then_type(self) -> None:
        pkt = rs.encode_packet(rs.RTDE_CONTROL_PACKAGE_START, b"\x01")
        self.assertEqual(len(pkt), 4)
        self.assertEqual(pkt[0], 0x00)
        self.assertEqual(pkt[1], 0x04)          # total size includes the header
        self.assertEqual(pkt[2], rs.RTDE_CONTROL_PACKAGE_START)
        self.assertEqual(pkt[3], 0x01)

    def test_empty_payload_gives_a_three_byte_packet(self) -> None:
        pkt = rs.encode_packet(rs.RTDE_CONTROL_PACKAGE_START, b"")
        self.assertEqual(pkt, b"\x00\x03\x53")

    def test_data_payload_size_and_decoded_values(self) -> None:
        pose = (0.412345, -0.298765, 0.101234, 0.0, -3.1416, 0.0)
        force = (-0.123456, 0.234567, -6.012345, 0.0, 0.0, 0.0)
        body = rs.encode_data_payload(123456.789, pose, force, rs.RT_PLAYING)

        self.assertEqual(len(body), rs.RTDE_PAYLOAD_SIZE)
        # Decode field by field at the offsets the C monitor uses.
        (ts,) = struct.unpack_from(">d", body, rs.FIELD_OFF_TIMESTAMP)
        got_pose = struct.unpack_from(">6d", body, rs.FIELD_OFF_TCP_POSE)
        got_force = struct.unpack_from(">6d", body, rs.FIELD_OFF_TCP_FORCE)
        (state,) = struct.unpack_from(">I", body, rs.FIELD_OFF_RUNTIME_STATE)

        self.assertAlmostEqual(ts, 123456.789, places=9)
        self.assertEqual(got_pose, pose)
        self.assertEqual(got_force, force)
        self.assertEqual(state, rs.RT_PLAYING)

    def test_known_byte_sequence_for_one_point_zero(self) -> None:
        """Pins big-endian IEEE-754 against a hand-checkable literal."""
        body = rs.encode_data_payload(1.0, (0.0,) * 6, (0.0,) * 6, rs.RT_STOPPED)
        self.assertEqual(
            body[rs.FIELD_OFF_TIMESTAMP:rs.FIELD_OFF_TIMESTAMP + 8],
            b"\x3f\xf0\x00\x00\x00\x00\x00\x00",
        )
        self.assertEqual(
            body[rs.FIELD_OFF_RUNTIME_STATE:rs.FIELD_OFF_RUNTIME_STATE + 4],
            b"\x00\x00\x00\x01",
        )


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_rtde_server -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ur5_sim.rtde_server'`

- [ ] **Step 3: Write minimal implementation**

Create `ur5_sim/rtde_server.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest tests.test_rtde_server -v`
Expected: PASS, 6 tests.

- [ ] **Step 5: Commit**

```bash
git add ur5_sim/rtde_server.py tests/test_rtde_server.py
git commit -m "Add RTDE wire encoder with the byte layout pinned against the C monitor"
```

---

### Task 2: runtime_state machine

**Model: Sonnet.** Appends a self-contained class; code and tests given in full.

**Files:**
- Modify: `ur5_sim/rtde_server.py` (append)
- Modify: `ur5_sim/config.py` (add `RTDE_EMU_TRANSITION_PACKETS`)
- Test: `tests/test_rtde_server.py` (append)

**Interfaces:**
- Consumes: `RT_*` constants from Task 1
- Produces: `class RunStateMachine` with `request(target: int) -> None` and `next_state() -> int`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_rtde_server.py`, before the `if __name__` block:

```python
class RunStateMachineTests(unittest.TestCase):
    """
    A real controller passes through PAUSING / RESUMING / STOPPING rather than
    jumping between the stable states. Emitting them drives the monitor through
    the same enum sequence its own C suite asserts pair by pair.
    """

    def _drain(self, machine: "rs.RunStateMachine", n: int) -> list:
        return [machine.next_state() for _ in range(n)]

    def test_starts_stopped(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        self.assertEqual(self._drain(m, 3), [rs.RT_STOPPED] * 3)

    def test_start_goes_straight_to_playing(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        m.request(rs.RT_PLAYING)
        self.assertEqual(self._drain(m, 3), [rs.RT_PLAYING] * 3)

    def test_pause_passes_through_pausing(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        m.request(rs.RT_PLAYING)
        m.next_state()
        m.request(rs.RT_PAUSED)
        self.assertEqual(
            self._drain(m, 4),
            [rs.RT_PAUSING, rs.RT_PAUSING, rs.RT_PAUSED, rs.RT_PAUSED],
        )

    def test_resume_passes_through_resuming(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        m.request(rs.RT_PLAYING)
        m.next_state()
        m.request(rs.RT_PAUSED)
        self._drain(m, 3)
        m.request(rs.RT_PLAYING)
        self.assertEqual(
            self._drain(m, 4),
            [rs.RT_RESUMING, rs.RT_RESUMING, rs.RT_PLAYING, rs.RT_PLAYING],
        )

    def test_stop_passes_through_stopping(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        m.request(rs.RT_PLAYING)
        m.next_state()
        m.request(rs.RT_STOPPED)
        self.assertEqual(
            self._drain(m, 4),
            [rs.RT_STOPPING, rs.RT_STOPPING, rs.RT_STOPPED, rs.RT_STOPPED],
        )

    def test_requesting_the_current_state_changes_nothing(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        m.request(rs.RT_PLAYING)
        m.next_state()
        m.request(rs.RT_PLAYING)
        self.assertEqual(self._drain(m, 2), [rs.RT_PLAYING] * 2)

    def test_repeated_request_during_a_transition_is_ignored(self) -> None:
        m = rs.RunStateMachine(transition_packets=2)
        m.request(rs.RT_PLAYING)
        m.next_state()
        m.request(rs.RT_PAUSED)
        m.request(rs.RT_PAUSED)      # operator double-click must not restart it
        self.assertEqual(
            self._drain(m, 4),
            [rs.RT_PAUSING, rs.RT_PAUSING, rs.RT_PAUSED, rs.RT_PAUSED],
        )
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_rtde_server.RunStateMachineTests -v`
Expected: FAIL — `AttributeError: module 'ur5_sim.rtde_server' has no attribute 'RunStateMachine'`

- [ ] **Step 3: Add the constant**

In `ur5_sim/config.py`, append a new section:

```python
# --------------------------------------------------------------------------
# RTDE emulator (simulation only; never reaches the exported script, so these
# live here and not in design/params.py - same rule as SIM_PROBE_* above).
# --------------------------------------------------------------------------

# Packets spent in PAUSING / RESUMING / STOPPING before the stable state.
# 2 packets at 125 Hz is about 16 ms, the order a real CB3 takes.
RTDE_EMU_TRANSITION_PACKETS: int = 2
```

- [ ] **Step 4: Write minimal implementation**

Append to `ur5_sim/rtde_server.py`:

```python
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `python -m unittest tests.test_rtde_server -v`
Expected: PASS, 13 tests.

- [ ] **Step 6: Commit**

```bash
git add ur5_sim/rtde_server.py ur5_sim/config.py tests/test_rtde_server.py
git commit -m "Add the runtime_state machine with real CB3 transition states"
```

---

### Task 3: Pose interpolation to the controller rate

**Model: Sonnet.** Pure function, code and tests given in full.

`DT = 0.05` means the trajectory is sampled at 20 Hz, below the 50 Hz the monitor targets. Decimation cannot create samples that were never sent, so the emulator interpolates, as a real CB3 does between waypoints.

**Files:**
- Modify: `ur5_sim/rtde_server.py` (append)
- Test: `tests/test_rtde_server.py` (append)

**Interfaces:**
- Consumes: nothing
- Produces: `interpolate_pose(poses: Sequence[Sequence[float]], times: Sequence[float], t: float) -> tuple[float, ...]`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_rtde_server.py`:

```python
class InterpolationTests(unittest.TestCase):
    """
    Translation is interpolated; orientation is taken from the nearest frame,
    because the exported trajectory holds orientation constant within a cycle.
    """

    POSES = [
        (0.0, 0.0, 0.0, 0.0, -3.1416, 0.0),
        (1.0, 2.0, 3.0, 0.0, -3.1416, 0.0),
        (2.0, 4.0, 6.0, 0.0, -3.0000, 0.0),
    ]
    TIMES = [0.0, 0.05, 0.10]

    def test_exact_frame_times_return_that_frame(self) -> None:
        for i, t in enumerate(self.TIMES):
            got = rs.interpolate_pose(self.POSES, self.TIMES, t)
            self.assertEqual(tuple(got), tuple(self.POSES[i]))

    def test_midpoint_translation_is_the_average(self) -> None:
        got = rs.interpolate_pose(self.POSES, self.TIMES, 0.025)
        self.assertAlmostEqual(got[0], 0.5, places=9)
        self.assertAlmostEqual(got[1], 1.0, places=9)
        self.assertAlmostEqual(got[2], 1.5, places=9)

    def test_quarter_point_translation(self) -> None:
        got = rs.interpolate_pose(self.POSES, self.TIMES, 0.0625)
        self.assertAlmostEqual(got[0], 1.25, places=9)
        self.assertAlmostEqual(got[1], 2.50, places=9)
        self.assertAlmostEqual(got[2], 3.75, places=9)

    def test_orientation_comes_from_the_nearer_frame(self) -> None:
        before_half = rs.interpolate_pose(self.POSES, self.TIMES, 0.060)
        after_half = rs.interpolate_pose(self.POSES, self.TIMES, 0.090)
        self.assertAlmostEqual(before_half[4], -3.1416, places=9)
        self.assertAlmostEqual(after_half[4], -3.0000, places=9)

    def test_clamps_outside_the_trajectory(self) -> None:
        self.assertEqual(
            tuple(rs.interpolate_pose(self.POSES, self.TIMES, -5.0)),
            tuple(self.POSES[0]),
        )
        self.assertEqual(
            tuple(rs.interpolate_pose(self.POSES, self.TIMES, 99.0)),
            tuple(self.POSES[-1]),
        )

    def test_single_frame_trajectory(self) -> None:
        got = rs.interpolate_pose([self.POSES[0]], [0.0], 1.0)
        self.assertEqual(tuple(got), tuple(self.POSES[0]))
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_rtde_server.InterpolationTests -v`
Expected: FAIL — `AttributeError: module 'ur5_sim.rtde_server' has no attribute 'interpolate_pose'`

- [ ] **Step 3: Write minimal implementation**

Add `import bisect` at the top of `ur5_sim/rtde_server.py`, then append:

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m unittest tests.test_rtde_server -v`
Expected: PASS, 19 tests.

- [ ] **Step 5: Commit**

```bash
git add ur5_sim/rtde_server.py tests/test_rtde_server.py
git commit -m "Interpolate the 20 Hz trajectory up to the 125 Hz controller rate"
```

---

### Task 4: FT-300 force surrogate

**Model: Sonnet.** New standalone module, code and tests given in full. The physics judgment is already spent — the parameters and their justification are fixed in the spec.

**Files:**
- Create: `ur5_sim/force_model.py`
- Modify: `ur5_sim/config.py` (add `FORCE_MODEL_*`)
- Test: `tests/test_force_model.py`

**Interfaces:**
- Consumes: `FORCE_Z_TARGET_N` from `ur5_sim/config.py`
- Produces: `class ForceModel` with `__init__(stiffness_n_per_m: float, tau_s: float, friction_mu: float, noise_n: float, seed: int, target_n: float)` and `step(dt: float, in_contact: bool, penetration_m: float, vx: float, vy: float) -> tuple[float, float, float]` returning `(fx, fy, fz)` newtons

- [ ] **Step 1: Write the failing test**

Create `tests/test_force_model.py`:

```python
"""
Tests for the FT-300 force surrogate.

The model is a named surrogate driven by the simulator's own penetration
depth, NOT a physics simulation and NOT measured data. These tests pin its
stated behavior, not the realism of its parameters.
"""

import unittest

from ur5_sim.config import FORCE_Z_TARGET_N
from ur5_sim.force_model import ForceModel


def _model(noise: float = 0.0) -> ForceModel:
    """A noiseless model by default, so behavior is asserted without slack."""
    return ForceModel(
        stiffness_n_per_m=4000.0,
        tau_s=0.05,
        friction_mu=0.8,
        noise_n=noise,
        seed=20260814,
        target_n=FORCE_Z_TARGET_N,
    )


class ForceModelTests(unittest.TestCase):

    DT = 1.0 / 125.0

    def _settle(self, model: ForceModel, seconds: float, **kwargs) -> tuple:
        out = (0.0, 0.0, 0.0)
        for _ in range(int(seconds / self.DT)):
            out = model.step(dt=self.DT, **kwargs)
        return out

    def test_transit_force_is_zero(self) -> None:
        m = _model()
        fx, fy, fz = self._settle(
            m, 1.0, in_contact=False, penetration_m=0.0, vx=0.05, vy=0.0
        )
        self.assertAlmostEqual(fz, 0.0, places=3)
        self.assertAlmostEqual(fx, 0.0, places=3)
        self.assertAlmostEqual(fy, 0.0, places=3)

    def test_contact_converges_to_the_force_target(self) -> None:
        m = _model()
        _, _, fz = self._settle(
            m, 1.0, in_contact=True, penetration_m=0.0, vx=0.0, vy=0.0
        )
        # Negative while pressing into the plate, matching the monitor's own
        # sample row (ForceZ = -6.012345).
        self.assertAlmostEqual(fz, -FORCE_Z_TARGET_N, places=3)

    def test_penetration_adds_a_stiffness_transient(self) -> None:
        m = _model()
        _, _, fz = self._settle(
            m, 1.0, in_contact=True, penetration_m=0.001, vx=0.0, vy=0.0
        )
        # 1 mm deeper at 4000 N/m is 4 N on top of the 6 N target.
        self.assertAlmostEqual(fz, -(FORCE_Z_TARGET_N + 4.0), places=3)

    def test_response_is_gradual_not_instant(self) -> None:
        m = _model()
        _, _, after_one_step = m.step(
            dt=self.DT, in_contact=True, penetration_m=0.0, vx=0.0, vy=0.0
        )
        self.assertLess(abs(after_one_step), FORCE_Z_TARGET_N)
        self.assertGreater(abs(after_one_step), 0.0)

    def test_friction_opposes_travel(self) -> None:
        m = _model()
        fx, fy, fz = self._settle(
            m, 1.0, in_contact=True, penetration_m=0.0, vx=0.05, vy=0.0
        )
        self.assertLess(fx, 0.0)                     # opposes +x travel
        self.assertAlmostEqual(fy, 0.0, places=6)
        self.assertAlmostEqual(fx, -0.8 * abs(fz), places=3)

    def test_friction_follows_the_travel_direction(self) -> None:
        m = _model()
        fx, fy, _ = self._settle(
            m, 1.0, in_contact=True, penetration_m=0.0, vx=0.0, vy=-0.05
        )
        self.assertAlmostEqual(fx, 0.0, places=6)
        self.assertGreater(fy, 0.0)                  # opposes -y travel

    def test_no_friction_at_rest(self) -> None:
        m = _model()
        fx, fy, _ = self._settle(
            m, 1.0, in_contact=True, penetration_m=0.0, vx=0.0, vy=0.0
        )
        self.assertAlmostEqual(fx, 0.0, places=9)
        self.assertAlmostEqual(fy, 0.0, places=9)

    def test_noise_is_present_and_bounded(self) -> None:
        m = _model(noise=0.05)
        samples = [
            m.step(dt=self.DT, in_contact=False, penetration_m=0.0, vx=0.0, vy=0.0)[2]
            for _ in range(500)
        ]
        self.assertGreater(len(set(samples)), 400)   # actually varying
        self.assertLess(max(abs(s) for s in samples), 0.5)

    def test_same_seed_gives_the_same_sequence(self) -> None:
        a = _model(noise=0.05)
        b = _model(noise=0.05)
        for _ in range(50):
            self.assertEqual(
                a.step(dt=self.DT, in_contact=True, penetration_m=0.0, vx=0.01, vy=0.0),
                b.step(dt=self.DT, in_contact=True, penetration_m=0.0, vx=0.01, vy=0.0),
            )


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_force_model -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ur5_sim.force_model'`

- [ ] **Step 3: Add the constants**

Append to the RTDE emulator section of `ur5_sim/config.py`:

```python
# --------------------------------------------------------------------------
# FT-300 force surrogate for the RTDE emulator.
#
# WARNING: these are PLAUSIBLE values, not measurements of this silicone
# finger on this plate. They are isolated here so measured values can replace
# them without touching any logic. Data produced with them resembles FT-300
# output; it is not FT-300 output.
# --------------------------------------------------------------------------

# Contact stiffness of the silicone hemispheric finger. 4000 N/m places the
# 6 N target at about 1.5 mm penetration.
FORCE_MODEL_STIFFNESS_N_PER_M: float = 4000.0

# First-order rise of the force_mode regulation.
FORCE_MODEL_TAU_S: float = 0.05

# Coulomb friction coefficient, silicone on a smooth plate.
FORCE_MODEL_FRICTION_MU: float = 0.8

# Sensor noise. The FT-300's stated resolution is 0.1 N; this sigma gives
# roughly that peak to peak.
FORCE_MODEL_NOISE_N: float = 0.05

# Fixed so a recorded CSV is reproducible and --verify-csv is stable.
FORCE_MODEL_SEED: int = 20260814
```

- [ ] **Step 4: Write minimal implementation**

Create `ur5_sim/force_model.py`:

```python
"""
FT-300 force surrogate for the RTDE emulator.

The simulator has no physics layer (ARCHITECTURE.md section 5). This module
does not add one. It synthesises a force signal from state the simulator
already computes - contact flag, penetration depth, TCP velocity - so the
result responds to real trajectory events, including the deliberate 5 mm
recontact overshoot, rather than replaying a canned waveform.

WARNING: the parameters are plausible, not measured. Output resembles FT-300
data; it is not FT-300 data. See ur5_sim/config.py FORCE_MODEL_*.
"""

from __future__ import annotations

import math
import random


class ForceModel:
    """
    Contact regulation plus Coulomb friction plus sensor noise.

    In contact, Fz relaxes toward the regulated target with a first-order
    response, offset by a stiffness term proportional to how far the tool
    actually is below the plane. In transit it relaxes toward zero. The
    tangential components oppose the direction of travel, scaled by the normal
    force. Sign convention: Fz is negative while pressing into the plate.
    """

    def __init__(
        self,
        stiffness_n_per_m: float,
        tau_s: float,
        friction_mu: float,
        noise_n: float,
        seed: int,
        target_n: float,
    ) -> None:
        self._stiffness = float(stiffness_n_per_m)
        self._tau = max(float(tau_s), 1e-9)
        self._mu = float(friction_mu)
        self._noise = float(noise_n)
        self._target = float(target_n)
        self._rng = random.Random(seed)
        self._fz = 0.0

    def step(
        self,
        dt: float,
        in_contact: bool,
        penetration_m: float,
        vx: float,
        vy: float,
    ) -> tuple[float, float, float]:
        """
        ----------------------------------------------------------------------
        Purpose:
            Advance the model by one emitter tick and return the synthesised
            force triple.

        Inputs:
            dt (float): tick length, seconds.
            in_contact (bool): True between force_mode and end_force_mode.
            penetration_m (float): depth below the plane, positive downward.
            vx (float): TCP velocity along world X, m/s.
            vy (float): TCP velocity along world Y, m/s.

        Outputs:
            force (tuple[float, float, float]): (Fx, Fy, Fz) in newtons.
        ----------------------------------------------------------------------
        """
        if in_contact:
            target = -(self._target + self._stiffness * penetration_m)
        else:
            target = 0.0

        # Exponential approach, exact for the step response of a first-order
        # lag, so the result does not depend on the tick length.
        alpha = 1.0 - math.exp(-dt / self._tau)
        self._fz += (target - self._fz) * alpha

        speed = math.hypot(vx, vy)
        if speed > 1e-9:
            tangential = -self._mu * abs(self._fz)
            fx = tangential * (vx / speed)
            fy = tangential * (vy / speed)
        else:
            fx = 0.0
            fy = 0.0

        if self._noise > 0.0:
            fx += self._rng.gauss(0.0, self._noise)
            fy += self._rng.gauss(0.0, self._noise)
            return (fx, fy, self._fz + self._rng.gauss(0.0, self._noise))
        return (fx, fy, self._fz)
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `python -m unittest tests.test_force_model -v`
Expected: PASS, 9 tests.

- [ ] **Step 6: Commit**

```bash
git add ur5_sim/force_model.py ur5_sim/config.py tests/test_force_model.py
git commit -m "Add the FT-300 force surrogate driven by simulator penetration depth"
```

---

### Task 5: TCP server, handshake and emitter thread

**Model: Opus.** The largest task and the one with real traps: a background thread sharing state under a lock, blocking-versus-non-blocking send (a partial send desyncs the framing permanently), `SO_REUSEADDR` having the opposite meaning on Windows to the one most developers expect, and a handshake that must satisfy a client written in another language. Do not downgrade.

**Files:**
- Modify: `ur5_sim/rtde_server.py` (append)
- Modify: `ur5_sim/config.py` (add remaining `RTDE_EMU_*`)
- Test: `tests/test_rtde_server.py` (append)

**Interfaces:**
- Consumes: everything from Tasks 1-4
- Produces: `class RtdeServer` with `__init__(host: str, port: int, rate_hz: float, force_model: ForceModel | None = None, transition_packets: int = 2)`, `start() -> bool`, `host -> str` (attribute), `port -> int` (property, resolved after bind), `load_run(poses: Sequence[Sequence[float]], times: Sequence[float], in_contact: Sequence[bool], penetration_m: Sequence[float]) -> None`, `set_run_state(running: bool, sim_time: float, finished: bool) -> None`, `stop() -> None`, `dropped_packets -> int` (property)

- [ ] **Step 1: Write the failing test**

Append to `tests/test_rtde_server.py`. Add `import socket`, `import time` at the top of the file:

```python
class _FakeMonitor:
    """Minimal client that performs the handshake the C monitor performs."""

    def __init__(self, port: int, protocol_version: int = 2) -> None:
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5.0)
        self.sock.settimeout(5.0)
        self.protocol_version = protocol_version
        self.recipe_id = 0

    def _recv_exactly(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("server closed")
            buf += chunk
        return buf

    def recv_packet(self) -> tuple:
        head = self._recv_exactly(rs.RTDE_HEADER_SIZE)
        size, pkg_type = struct.unpack(">HB", head)
        return pkg_type, self._recv_exactly(size - rs.RTDE_HEADER_SIZE)

    def handshake(self) -> None:
        self.sock.sendall(rs.encode_packet(
            rs.RTDE_REQUEST_PROTOCOL_VERSION,
            struct.pack(">H", self.protocol_version)))
        pkg_type, body = self.recv_packet()
        assert pkg_type == rs.RTDE_REQUEST_PROTOCOL_VERSION and body[0] == 1

        payload = b""
        if self.protocol_version >= 2:
            payload += struct.pack(">d", 125.0)
        payload += rs.RTDE_OUTPUT_RECIPE.encode("ascii")
        self.sock.sendall(
            rs.encode_packet(rs.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, payload))
        pkg_type, body = self.recv_packet()
        assert pkg_type == rs.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS
        if self.protocol_version >= 2:
            self.recipe_id = body[0]
            types = body[1:].decode("ascii")
        else:
            types = body.decode("ascii")
        assert types == rs.RTDE_OUTPUT_TYPES, types

        self.sock.sendall(rs.encode_packet(rs.RTDE_CONTROL_PACKAGE_START, b""))
        pkg_type, body = self.recv_packet()
        assert pkg_type == rs.RTDE_CONTROL_PACKAGE_START and body[0] == 1

    def next_sample(self) -> tuple:
        """Return (timestamp, pose6, force6, runtime_state) of the next frame."""
        while True:
            pkg_type, body = self.recv_packet()
            if pkg_type != rs.RTDE_DATA_PACKAGE:
                continue
            if self.protocol_version >= 2:
                body = body[1:]
            values = struct.unpack(">d6d6dI", body)
            return values[0], values[1:7], values[7:13], values[13]

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class RtdeServerTests(unittest.TestCase):

    POSES = [(float(i) * 0.01, 0.0, 0.3, 0.0, -3.1416, 0.0) for i in range(20)]
    TIMES = [i * 0.05 for i in range(20)]

    def _server(self) -> "rs.RtdeServer":
        server = rs.RtdeServer(host="127.0.0.1", port=0, rate_hz=125.0)
        self.assertTrue(server.start())
        self.addCleanup(server.stop)
        server.load_run(
            poses=self.POSES,
            times=self.TIMES,
            in_contact=[True] * len(self.POSES),
            penetration_m=[0.0] * len(self.POSES),
        )
        return server

    def test_binds_loopback_only(self) -> None:
        """It must be unreachable from the lab VLAN, never mistaken for a robot."""
        server = self._server()
        self.assertEqual(server.host, "127.0.0.1")

    def test_handshake_then_streaming(self) -> None:
        server = self._server()
        client = _FakeMonitor(server.port)
        self.addCleanup(client.close)
        client.handshake()

        ts0, pose, force, state = client.next_sample()
        self.assertEqual(len(pose), 6)
        self.assertEqual(len(force), 6)
        self.assertEqual(state, rs.RT_STOPPED)

        ts1, _, _, _ = client.next_sample()
        self.assertGreater(ts1, ts0)

    def test_protocol_version_1_is_accepted(self) -> None:
        server = self._server()
        client = _FakeMonitor(server.port, protocol_version=1)
        self.addCleanup(client.close)
        client.handshake()
        _, _, _, state = client.next_sample()
        self.assertEqual(state, rs.RT_STOPPED)

    def test_stream_continues_while_stopped(self) -> None:
        """The monitor needs a STOPPED packet to see the edge that opens a file."""
        server = self._server()
        client = _FakeMonitor(server.port)
        self.addCleanup(client.close)
        client.handshake()
        for _ in range(5):
            _, _, _, state = client.next_sample()
            self.assertEqual(state, rs.RT_STOPPED)

    def test_run_state_reaches_the_client(self) -> None:
        server = self._server()
        client = _FakeMonitor(server.port)
        self.addCleanup(client.close)
        client.handshake()
        client.next_sample()

        server.set_run_state(running=True, sim_time=0.0, finished=False)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            _, _, _, state = client.next_sample()
            if state == rs.RT_PLAYING:
                break
        else:
            self.fail("never reached PLAYING")

    def test_timestamp_advances_while_stopped(self) -> None:
        """A real controller's clock does not stop when the program does."""
        server = self._server()
        client = _FakeMonitor(server.port)
        self.addCleanup(client.close)
        client.handshake()
        first, _, _, _ = client.next_sample()
        for _ in range(20):
            last, _, _, _ = client.next_sample()
        self.assertGreater(last, first)

    def test_client_disconnect_does_not_kill_the_server(self) -> None:
        server = self._server()
        first = _FakeMonitor(server.port)
        first.handshake()
        first.close()

        second = _FakeMonitor(server.port)
        self.addCleanup(second.close)
        second.handshake()
        _, _, _, state = second.next_sample()
        self.assertEqual(state, rs.RT_STOPPED)

    def test_port_in_use_degrades_instead_of_raising(self) -> None:
        """The visualizer must never be blocked by the emulator."""
        first = self._server()
        second = rs.RtdeServer(host="127.0.0.1", port=first.port, rate_hz=125.0)
        self.addCleanup(second.stop)
        self.assertFalse(second.start())

    def test_stop_is_idempotent(self) -> None:
        server = self._server()
        server.stop()
        server.stop()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_rtde_server.RtdeServerTests -v`
Expected: FAIL — `AttributeError: module 'ur5_sim.rtde_server' has no attribute 'RtdeServer'`

- [ ] **Step 3: Add the remaining constants**

Append to the RTDE emulator section of `ur5_sim/config.py`:

```python
# Loopback only. Binding 0.0.0.0 would make the emulator reachable from the
# lab VLAN, where it could be mistaken for the robot at 192.168.4.38.
RTDE_EMU_HOST: str = "127.0.0.1"

# The controller's real RTDE port, so the monitor's command line is identical
# to the one used in the lab.
RTDE_EMU_PORT: int = 30004

# CB3 control-loop rate.
RTDE_EMU_RATE_HZ: float = 125.0

# STOPPED dwell between consecutive --runs, and the hold of --pause-at.
RTDE_EMU_IDLE_S: float = 1.0
```

- [ ] **Step 4: Write minimal implementation**

Add `import socket`, `import threading`, `import time` to the top of `ur5_sim/rtde_server.py`, then append:

```python
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
        force_model: "object | None" = None,
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
            if self._dropped:
                print(f"[rtde-emu] {self._dropped} packet(s) dropped to a slow client")

    @property
    def port(self) -> int:
        return self._port

    @property
    def dropped_packets(self) -> int:
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

    def _sample_index(self, t: float) -> int:
        i = bisect.bisect_right(self._times, t) - 1
        return min(max(i, 0), len(self._poses) - 1)

    def _build_payload(self) -> bytes:
        with self._lock:
            sim_t = self._sim_time
            poses, times = self._poses, self._times
            contact, penetration = self._contact, self._penetration
            state = self._machine.next_state()

        pose = interpolate_pose(poses, times, sim_t)
        idx = self._sample_index(sim_t)

        vx = vy = 0.0
        if self._prev_xy is not None:
            vx = (pose[0] - self._prev_xy[0]) / self._period
            vy = (pose[1] - self._prev_xy[1]) / self._period
        self._prev_xy = (pose[0], pose[1])

        moving = state in (RT_PLAYING, RT_PAUSING, RT_RESUMING)
        if self._force_model is not None:
            fx, fy, fz = self._force_model.step(
                dt=self._period,
                in_contact=bool(contact[idx]) and moving,
                penetration_m=float(penetration[idx]),
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `python -m unittest tests.test_rtde_server -v`
Expected: PASS, 28 tests.

- [ ] **Step 6: Run the whole suite for regressions**

Run: `python -m unittest discover -s tests -p "test_*.py"`
Expected: PASS, no new failures (47 pre-existing plus the new ones).

- [ ] **Step 7: Commit**

```bash
git add ur5_sim/rtde_server.py ur5_sim/config.py tests/test_rtde_server.py
git commit -m "Add the RTDE TCP server, handshake and 125 Hz emitter thread"
```

---

### Task 6: PAUSE control in the viewer

**Model: Opus.** Steps 1-4 are mechanical, but step 5 edits `viewer.py`, a ~900-line matplotlib file with existing widget wiring, button state and HUD text that must be read before a third control can be added coherently. The plan describes that edit; it does not hand over the whole file.

Without it the monitor's "a pause does not split the file" cannot be exercised locally. `paused_sim_t` already participates in the clock at `viewer.py:699` and is never set to anything but zero.

**Files:**
- Create: `ur5_sim/visualization/playback_clock.py`
- Modify: `ur5_sim/visualization/viewer.py` (the `set_stop` / `set_start` region, around lines 741-810)
- Test: `tests/test_playback_clock.py`

**Interfaces:**
- Consumes: nothing
- Produces: `class PlaybackClock` with `start(now: float) -> None`, `pause(now: float) -> None`, `stop() -> None`, `elapsed(now: float) -> float`, `running -> bool` (property), `paused -> bool` (property)

The clock logic is extracted so it can be tested; matplotlib widget wiring cannot.

- [ ] **Step 1: Write the failing test**

Create `tests/test_playback_clock.py`:

```python
"""
Tests for the playback clock behind the viewer's START / PAUSE / STOP.

PAUSE must preserve elapsed simulation time; STOP must discard it, keeping the
viewer's documented behavior that the next START replays from frame 0.
"""

import unittest

from ur5_sim.visualization.playback_clock import PlaybackClock


class PlaybackClockTests(unittest.TestCase):

    def test_starts_idle_at_zero(self) -> None:
        clock = PlaybackClock()
        self.assertFalse(clock.running)
        self.assertFalse(clock.paused)
        self.assertEqual(clock.elapsed(now=100.0), 0.0)

    def test_elapsed_advances_while_running(self) -> None:
        clock = PlaybackClock()
        clock.start(now=10.0)
        self.assertTrue(clock.running)
        self.assertAlmostEqual(clock.elapsed(now=12.5), 2.5, places=9)

    def test_pause_freezes_elapsed(self) -> None:
        clock = PlaybackClock()
        clock.start(now=10.0)
        clock.pause(now=12.5)
        self.assertFalse(clock.running)
        self.assertTrue(clock.paused)
        self.assertAlmostEqual(clock.elapsed(now=99.0), 2.5, places=9)

    def test_resume_continues_from_the_pause_point(self) -> None:
        clock = PlaybackClock()
        clock.start(now=10.0)
        clock.pause(now=12.5)
        clock.start(now=50.0)
        self.assertTrue(clock.running)
        self.assertFalse(clock.paused)
        self.assertAlmostEqual(clock.elapsed(now=51.0), 3.5, places=9)

    def test_stop_discards_elapsed(self) -> None:
        clock = PlaybackClock()
        clock.start(now=10.0)
        clock.pause(now=12.5)
        clock.stop()
        self.assertFalse(clock.running)
        self.assertFalse(clock.paused)
        self.assertEqual(clock.elapsed(now=99.0), 0.0)

    def test_start_after_stop_replays_from_zero(self) -> None:
        clock = PlaybackClock()
        clock.start(now=10.0)
        clock.stop()
        clock.start(now=20.0)
        self.assertAlmostEqual(clock.elapsed(now=20.75), 0.75, places=9)

    def test_speed_factor_scales_elapsed(self) -> None:
        clock = PlaybackClock(speed=2.0)
        clock.start(now=0.0)
        self.assertAlmostEqual(clock.elapsed(now=1.0), 2.0, places=9)

    def test_double_pause_is_harmless(self) -> None:
        clock = PlaybackClock()
        clock.start(now=0.0)
        clock.pause(now=1.0)
        clock.pause(now=5.0)
        self.assertAlmostEqual(clock.elapsed(now=9.0), 1.0, places=9)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_playback_clock -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ur5_sim.visualization.playback_clock'`

- [ ] **Step 3: Write minimal implementation**

Create `ur5_sim/visualization/playback_clock.py`:

```python
"""
Playback clock behind the viewer's START / PAUSE / STOP controls.

Extracted from viewer.py so the timing rules can be tested without a
matplotlib figure. The distinction it encodes is the one the RTDE emulator
needs: a PAUSE keeps elapsed simulation time (same run, so the monitor must
not split the CSV), a STOP discards it (next START replays from frame 0,
which is the viewer's long-standing documented behavior).
"""

from __future__ import annotations


class PlaybackClock:
    """Wall-clock to simulation-time mapping with pause support."""

    def __init__(self, speed: float = 1.0) -> None:
        self._speed = float(speed)
        self._running = False
        self._paused = False
        self._offset = 0.0        # simulation time banked by earlier segments
        self._anchor = 0.0        # wall time the current segment started

    @property
    def running(self) -> bool:
        return self._running

    @property
    def paused(self) -> bool:
        return self._paused

    def start(self, now: float) -> None:
        """Start, or resume from a pause without losing banked time."""
        if self._running:
            return
        self._anchor = float(now)
        self._running = True
        self._paused = False

    def pause(self, now: float) -> None:
        """Bank the elapsed simulation time and hold it."""
        if not self._running:
            return
        self._offset = self.elapsed(now)
        self._running = False
        self._paused = True

    def stop(self) -> None:
        """Hard stop: discard banked time so the next start replays from zero."""
        self._offset = 0.0
        self._running = False
        self._paused = False

    def elapsed(self, now: float) -> float:
        """Simulation time, seconds."""
        if not self._running:
            return self._offset
        return (float(now) - self._anchor) * self._speed + self._offset
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest tests.test_playback_clock -v`
Expected: PASS, 8 tests.

- [ ] **Step 5: Wire PAUSE into the viewer**

In `ur5_sim/visualization/viewer.py`:

1. Add a PAUSE button beside the existing START/STOP button, using the same
   `matplotlib.widgets.Button` pattern already present. Label it `PAUSE`, and
   flip it to `RESUME` while paused.
2. Its callback, mirroring the existing `set_stop` / `set_start` style:

```python
    def set_pause() -> None:
        # PAUSE keeps the run alive: paused_sim_t banks the elapsed sim time so
        # RESUME continues instead of replaying. STOP still discards it.
        if not state["running"]:
            return
        paused_sim_t[0] = (time.perf_counter() - clock_t0[0]) * SIM_SPEED + paused_sim_t[0]
        state["running"] = False
        state["paused"] = True
        pause_btn.label.set_text("RESUME")
        status_text.set_text("STATE = PAUSE")
        status_text.set_color("#a06000")
        _publish_run_state()

    def on_pause_button(_event) -> None:
        if state["running"]:
            set_pause()
        elif state.get("paused"):
            state["paused"] = False
            pause_btn.label.set_text("PAUSE")
            set_start()
        fig.canvas.draw_idle()
```

3. `set_start()` must NOT reset `paused_sim_t` (it already does not) and must
   clear `state["paused"]`. `set_stop()` keeps zeroing `paused_sim_t` and must
   clear `state["paused"]` and reset the button label to `PAUSE`.
4. Add `"paused": False` to the `state` dict initialisers at `viewer.py:352`
   and `viewer.py:891`.

- [ ] **Step 6: Verify by hand**

Run: `python -m ur5_sim --visualize`
Expected: START plays; PAUSE freezes the robot and the HUD shows `STATE = PAUSE`; RESUME continues from where it stopped rather than restarting; STOP still resets to frame 0.

- [ ] **Step 7: Commit**

```bash
git add ur5_sim/visualization/playback_clock.py ur5_sim/visualization/viewer.py tests/test_playback_clock.py
git commit -m "Add a real PAUSE to the viewer, using the clock offset that was already threaded through"
```

---

### Task 7: Wire the emulator into the CLI and the viewer

**Model: Opus.** Touches both `cli.py` and `viewer.py`, and the snippets below name buffers — `poses_xyzrpy_per_frame`, `penetration_per_frame`, `times`, `total_sim_time`, `contact_flags`, `depths` — that are **illustrative, not verified against the real files**. The implementer must read `viewer.py` and `cli.py`, find the actual buffers holding the surface-clamped poses and the per-frame penetration, and use those names. Transcribing the snippets literally will not run. Do not downgrade.

**Files:**
- Modify: `ur5_sim/cli.py`
- Modify: `ur5_sim/visualization/viewer.py`
- Test: `tests/test_rtde_server.py` (append the headless-runner test)

**Interfaces:**
- Consumes: `RtdeServer`, `ForceModel`, `PlaybackClock`
- Produces: `ur5_sim.rtde_server.run_headless(server: RtdeServer, total_sim_time: float, runs: int, pause_at: float | None, idle_s: float) -> None`; CLI flags `--rtde-serve` / `--no-rtde-serve` / `--rtde-port` / `--emulate` / `--runs` / `--pause-at`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_rtde_server.py`:

```python
class HeadlessRunnerTests(unittest.TestCase):
    """
    The headless runner is what makes the two remaining pendant behaviors
    checkable unattended: two runs producing two files, and a pause that does
    not split one.
    """

    POSES = [(float(i) * 0.01, 0.0, 0.3, 0.0, -3.1416, 0.0) for i in range(6)]
    TIMES = [i * 0.05 for i in range(6)]

    def _server_and_client(self):
        server = rs.RtdeServer(host="127.0.0.1", port=0, rate_hz=125.0)
        self.assertTrue(server.start())
        self.addCleanup(server.stop)
        server.load_run(
            poses=self.POSES, times=self.TIMES,
            in_contact=[True] * 6, penetration_m=[0.0] * 6,
        )
        client = _FakeMonitor(server.port)
        self.addCleanup(client.close)
        client.handshake()
        return server, client

    def _collect_states(self, client, seconds: float) -> list:
        seen, deadline = [], time.time() + seconds
        while time.time() < deadline:
            _, _, _, state = client.next_sample()
            if not seen or seen[-1] != state:
                seen.append(state)
        return seen

    def test_two_runs_emit_two_stopped_to_playing_edges(self) -> None:
        server, client = self._server_and_client()
        thread = threading.Thread(
            target=rs.run_headless,
            kwargs=dict(server=server, total_sim_time=0.25, runs=2,
                        pause_at=None, idle_s=0.2),
            daemon=True)
        thread.start()
        seen = self._collect_states(client, 2.0)
        thread.join(timeout=3.0)

        edges = sum(
            1 for a, b in zip(seen, seen[1:])
            if a == rs.RT_STOPPED and b == rs.RT_PLAYING
        )
        self.assertEqual(edges, 2)

    def test_pause_at_emits_the_pause_sequence_without_stopping(self) -> None:
        server, client = self._server_and_client()
        thread = threading.Thread(
            target=rs.run_headless,
            kwargs=dict(server=server, total_sim_time=0.5, runs=1,
                        pause_at=0.2, idle_s=0.2),
            daemon=True)
        thread.start()
        seen = self._collect_states(client, 2.0)
        thread.join(timeout=3.0)

        self.assertIn(rs.RT_PAUSED, seen)
        # No STOPPED between the pause and the resume: it is still one run.
        pause_i = seen.index(rs.RT_PAUSED)
        resume_i = len(seen) - 1 - seen[::-1].index(rs.RT_PLAYING)
        self.assertGreater(resume_i, pause_i)
        self.assertNotIn(rs.RT_STOPPED, seen[pause_i:resume_i])
```

Add `import threading` to the test file's imports.

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_rtde_server.HeadlessRunnerTests -v`
Expected: FAIL — `AttributeError: module 'ur5_sim.rtde_server' has no attribute 'run_headless'`

- [ ] **Step 3: Write the headless runner**

Append to `ur5_sim/rtde_server.py`:

```python
def run_headless(
    server: "RtdeServer",
    total_sim_time: float,
    runs: int,
    pause_at: float | None,
    idle_s: float,
) -> None:
    """
    --------------------------------------------------------------------------
    Purpose:
        Drive the server through complete runs in real time with no GUI, so an
        unattended script can exercise the monitor's file boundaries. Each run
        is STOPPED, then PLAYING for total_sim_time, then STOPPED again, with
        an optional single pause partway.

    Inputs:
        server (RtdeServer): a started server.
        total_sim_time (float): duration of one run, seconds.
        runs (int): number of consecutive runs.
        pause_at (float | None): simulation time to pause once at, or None.
        idle_s (float): STOPPED dwell between runs, and the pause hold.

    Outputs:
        None.
    --------------------------------------------------------------------------
    """
    tick = 0.01
    for run_index in range(max(1, int(runs))):
        server.set_run_state(running=False, sim_time=0.0, finished=True)
        time.sleep(idle_s)

        sim_t, paused_done = 0.0, pause_at is None
        wall0 = time.perf_counter()
        while sim_t < total_sim_time:
            sim_t = time.perf_counter() - wall0
            server.set_run_state(running=True, sim_time=sim_t, finished=False)
            if not paused_done and sim_t >= float(pause_at):
                server.set_run_state(running=False, sim_time=sim_t, finished=False)
                time.sleep(idle_s)
                wall0 = time.perf_counter() - sim_t   # resume, do not restart
                paused_done = True
            time.sleep(tick)

        server.set_run_state(running=False, sim_time=total_sim_time, finished=True)
        time.sleep(idle_s)
        print(f"[rtde-emu] run {run_index + 1}/{runs} complete")
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest tests.test_rtde_server -v`
Expected: PASS, 30 tests.

- [ ] **Step 5: Add the CLI flags**

In `ur5_sim/cli.py`, add to the argument parser:

```python
    parser.add_argument(
        "--rtde-serve", dest="rtde_serve", action="store_true", default=None,
        help="serve the RTDE emulator (default: on with --visualize)")
    parser.add_argument(
        "--no-rtde-serve", dest="rtde_serve", action="store_false",
        help="do not open the RTDE emulator socket")
    parser.add_argument(
        "--rtde-port", type=int, default=RTDE_EMU_PORT,
        help=f"RTDE emulator port (default {RTDE_EMU_PORT}, loopback only)")
    parser.add_argument(
        "--emulate", action="store_true",
        help="headless real-time RTDE emulation, no Swift and no matplotlib")
    parser.add_argument(
        "--runs", type=int, default=1,
        help="with --emulate: number of consecutive runs")
    parser.add_argument(
        "--pause-at", type=float, default=None,
        help="with --emulate: pause once at this simulation time, seconds")
```

Build the server the same way for both paths, after the trajectory and the
surface constraint have been computed (the emulator must receive the clamped
poses, the same array the IK sees):

```python
def _build_rtde_server(port: int) -> "RtdeServer | None":
    """Returns a started server, or None if the port was unavailable."""
    model = ForceModel(
        stiffness_n_per_m=FORCE_MODEL_STIFFNESS_N_PER_M,
        tau_s=FORCE_MODEL_TAU_S,
        friction_mu=FORCE_MODEL_FRICTION_MU,
        noise_n=FORCE_MODEL_NOISE_N,
        seed=FORCE_MODEL_SEED,
        target_n=FORCE_Z_TARGET_N,
    )
    server = RtdeServer(
        host=RTDE_EMU_HOST, port=port, rate_hz=RTDE_EMU_RATE_HZ,
        force_model=model,
        transition_packets=RTDE_EMU_TRANSITION_PACKETS,
    )
    print("[rtde-emu] force values are a documented surrogate with plausible, "
          "NOT measured parameters (ur5_sim/config.py FORCE_MODEL_*)")
    return server if server.start() else None
```

Resolve the default explicitly (`--rtde-serve` has `default=None`, so "not
given" is distinguishable from "given as false"):

```python
def _wants_rtde(args) -> bool:
    """
    --------------------------------------------------------------------------
    Purpose:
        Decide whether to open the emulator socket. On by default with
        --visualize and implied by --emulate; never with --check, which has no
        real-time pacing, so a three-minute protocol would stream in seconds
        and produce timestamps resembling no trial.

    Inputs:
        args (argparse.Namespace): parsed command line.

    Outputs:
        wanted (bool): True to start the RTDE server.
    --------------------------------------------------------------------------
    """
    if args.emulate:
        return True
    if args.rtde_serve is not None:
        return bool(args.rtde_serve)
    return bool(args.visualize)
```

In the `--emulate` branch, build the polyline exactly as `--visualize` does
(parse, `transform`, `rotate_translation_y`, `apply_surface_constraint`), then:

```python
    server = _build_rtde_server(args.rtde_port)
    if server is None:
        return 1
    try:
        server.load_run(poses=poses_xyzrpy, times=times,
                        in_contact=contact_flags, penetration_m=depths)
        run_headless(server=server, total_sim_time=total_sim_time,
                     runs=args.runs, pause_at=args.pause_at,
                     idle_s=RTDE_EMU_IDLE_S)
    finally:
        server.stop()
    return 0
```

- [ ] **Step 6: Publish run state from the viewer**

In `viewer.py`, add an `rtde_server=None` keyword parameter to `visualize()`.
Immediately after the trajectory buffers are ready (right after the
`in_contact_per_frame` normalisation near `viewer.py:157-164`, and again inside
`_finalize_recompute` so a configuration change re-arms the emulator with the
new buffer):

```python
    def _load_rtde_run() -> None:
        # The emulator must receive the SAME clamped poses the IK sees, so a
        # recorded CSV can be checked against the commanded path.
        if rtde_server is None:
            return
        rtde_server.load_run(
            poses=[tuple(p) for p in poses_xyzrpy_per_frame],
            times=list(times),
            in_contact=list(in_contact_per_frame),
            penetration_m=list(penetration_per_frame),
        )
```

Call `_load_rtde_run()` once before the animation starts and once at the end of
`_finalize_recompute`. Then add one call beside the existing UDP emit
(`viewer.py:516-535`):

```python
        if rtde_server is not None:
            rtde_server.set_run_state(
                running=bool(state["running"]),
                sim_time=sim_elapsed,
                finished=bool(sim_elapsed >= total_sim_time),
            )
```

Wrap the animation in `try/finally` so `rtde_server.stop()` always runs.

- [ ] **Step 7: Verify by hand**

Terminal 1: `python -m ur5_sim --emulate --runs 2`
Terminal 2: `datalogger\rtde_fallback_monitor.exe 127.0.0.1 30004 datalogger\sim_runs`
Expected: two `ACQ_rtde_*.csv` files appear, each with rows, and the monitor prints one "run started" and one "run finished" per run.

- [ ] **Step 8: Run the whole suite**

Run: `python -m unittest discover -s tests -p "test_*.py"`
Expected: PASS.

- [ ] **Step 9: Commit**

```bash
git add ur5_sim/cli.py ur5_sim/visualization/viewer.py ur5_sim/rtde_server.py tests/test_rtde_server.py
git commit -m "Serve RTDE from --visualize and add the headless --emulate runner"
```

---

### Task 8: `--verify-csv`

**Model: Sonnet.** The module and its tests are given in full. Step 5 reuses the polyline the `--verify-csv` branch already has to hand; if that array is not obvious in `cli.py`, stop and flag it rather than guessing.

Comparing CSV row *N* to trajectory time *N* breaks as soon as a pause exists, because controller time advances while simulation time freezes. The primary check is therefore geometric.

**Files:**
- Create: `ur5_sim/verify_csv.py`
- Modify: `ur5_sim/cli.py`
- Test: `tests/test_verify_csv.py`

**Interfaces:**
- Consumes: nothing from earlier tasks
- Produces: `parse_monitor_csv(path: str) -> tuple[list[str], list[tuple[float, ...]]]` returning `(header_lines, rows)` where each row is `(time, fx, fy, fz, x, y, z)`; `distance_to_polyline(point: tuple[float, float, float], polyline: Sequence[Sequence[float]]) -> float`; `verify(csv_path: str, polyline: Sequence[Sequence[float]], tol_m: float) -> dict`

- [ ] **Step 1: Write the failing test**

Create `tests/test_verify_csv.py`:

```python
"""
Tests for the recorded-CSV checker.

The geometric check is the primary one because it survives a pause: controller
time keeps advancing while simulation time freezes, so any purely time-indexed
comparison would report a false failure on a paused run.
"""

import os
import tempfile
import unittest

from ur5_sim.verify_csv import (
    distance_to_polyline,
    parse_monitor_csv,
    verify,
)

CSV_TEXT = """# Robot Model: UR5 CB3
# Data Source: RTDE fallback monitor (192.168.4.14)
# Robot RTDE Endpoint: 127.0.0.1:30004
# Time Column: RTDE timestamp field, relative to the first sample of this file (s)
Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ
0.000,-0.100000,0.000000,-6.000000,0.000000,0.000000,0.300000
0.020,-0.100000,0.000000,-6.000000,0.050000,0.000000,0.300000
0.040,-0.100000,0.000000,-6.000000,0.100000,0.000000,0.300000
"""

POLYLINE = [
    (0.0, 0.0, 0.3),
    (0.1, 0.0, 0.3),
]


class ParseTests(unittest.TestCase):

    def setUp(self) -> None:
        fd, self.path = tempfile.mkstemp(suffix=".csv")
        with os.fdopen(fd, "w", encoding="ascii") as handle:
            handle.write(CSV_TEXT)
        self.addCleanup(os.remove, self.path)

    def test_separates_header_from_rows(self) -> None:
        header, rows = parse_monitor_csv(self.path)
        self.assertTrue(any("Robot Model" in line for line in header))
        self.assertEqual(len(rows), 3)

    def test_row_values(self) -> None:
        _, rows = parse_monitor_csv(self.path)
        self.assertAlmostEqual(rows[1][0], 0.020, places=9)
        self.assertAlmostEqual(rows[1][3], -6.0, places=9)
        self.assertAlmostEqual(rows[1][4], 0.05, places=9)
        self.assertAlmostEqual(rows[1][6], 0.30, places=9)


class DistanceTests(unittest.TestCase):

    def test_point_on_the_line_is_zero(self) -> None:
        self.assertAlmostEqual(
            distance_to_polyline((0.05, 0.0, 0.3), POLYLINE), 0.0, places=12)

    def test_point_off_the_line(self) -> None:
        self.assertAlmostEqual(
            distance_to_polyline((0.05, 0.002, 0.3), POLYLINE), 0.002, places=12)

    def test_point_beyond_an_end_uses_the_endpoint(self) -> None:
        self.assertAlmostEqual(
            distance_to_polyline((0.2, 0.0, 0.3), POLYLINE), 0.1, places=12)

    def test_single_point_polyline(self) -> None:
        self.assertAlmostEqual(
            distance_to_polyline((0.0, 0.0, 0.4), [(0.0, 0.0, 0.3)]), 0.1, places=12)


class VerifyTests(unittest.TestCase):

    def setUp(self) -> None:
        fd, self.path = tempfile.mkstemp(suffix=".csv")
        with os.fdopen(fd, "w", encoding="ascii") as handle:
            handle.write(CSV_TEXT)
        self.addCleanup(os.remove, self.path)

    def test_matching_csv_passes(self) -> None:
        report = verify(self.path, POLYLINE, tol_m=1e-5)
        self.assertTrue(report["passed"])
        self.assertEqual(report["rows"], 3)
        self.assertLess(report["max_dev_m"], 1e-5)
        self.assertTrue(report["time_monotonic"])

    def test_offset_trajectory_fails(self) -> None:
        shifted = [(x, y + 0.01, z) for (x, y, z) in POLYLINE]
        report = verify(self.path, shifted, tol_m=1e-5)
        self.assertFalse(report["passed"])
        self.assertGreater(report["max_dev_m"], 1e-5)

    def test_reports_the_effective_rate(self) -> None:
        report = verify(self.path, POLYLINE, tol_m=1e-5)
        self.assertAlmostEqual(report["rate_hz"], 50.0, places=3)

    def test_flags_a_simulated_source(self) -> None:
        report = verify(self.path, POLYLINE, tol_m=1e-5)
        self.assertTrue(report["simulated_source"])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_verify_csv -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ur5_sim.verify_csv'`

- [ ] **Step 3: Write minimal implementation**

Create `ur5_sim/verify_csv.py`:

```python
"""
Check a CSV recorded by datalogger/rtde_fallback_monitor.exe against the
trajectory the simulator commanded.

The primary check is geometric - the distance from each recorded point to the
commanded polyline - because it survives a pause. Controller time keeps
advancing while simulation time freezes, so a purely time-indexed comparison
would report a false failure on any paused run.
"""

from __future__ import annotations

import math
from typing import Sequence


def parse_monitor_csv(path: str) -> tuple[list[str], list[tuple[float, ...]]]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Split a monitor CSV into its comment header and its data rows.

    Inputs:
        path (str): CSV written by the monitor.

    Outputs:
        header (list[str]): the leading comment lines.
        rows (list[tuple[float, ...]]): (Time, Fx, Fy, Fz, X, Y, Z) per row.
    --------------------------------------------------------------------------
    """
    header: list[str] = []
    rows: list[tuple[float, ...]] = []
    with open(path, "r", encoding="ascii") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                header.append(line)
            elif line.startswith("Time,"):
                continue
            else:
                rows.append(tuple(float(v) for v in line.split(",")))
    return header, rows


def _segment_distance(
    p: tuple[float, float, float],
    a: Sequence[float],
    b: Sequence[float],
) -> float:
    abx, aby, abz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    apx, apy, apz = p[0] - a[0], p[1] - a[1], p[2] - a[2]
    denom = abx * abx + aby * aby + abz * abz
    t = 0.0 if denom <= 0.0 else (apx * abx + apy * aby + apz * abz) / denom
    t = min(1.0, max(0.0, t))
    dx = apx - abx * t
    dy = apy - aby * t
    dz = apz - abz * t
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def distance_to_polyline(
    point: tuple[float, float, float],
    polyline: Sequence[Sequence[float]],
) -> float:
    """Shortest distance from a point to a polyline, metres."""
    if len(polyline) == 1:
        a = polyline[0]
        return math.dist(point, (a[0], a[1], a[2]))
    return min(
        _segment_distance(point, polyline[i], polyline[i + 1])
        for i in range(len(polyline) - 1)
    )


def verify(
    csv_path: str,
    polyline: Sequence[Sequence[float]],
    tol_m: float,
) -> dict:
    """
    --------------------------------------------------------------------------
    Purpose:
        Check that every recorded point lies on the commanded path, that time
        increases, and report the effective rate and any pause gaps.

    Inputs:
        csv_path (str): CSV written by the monitor.
        polyline (Sequence[Sequence[float]]): commanded XYZ, surface-clamped.
        tol_m (float): maximum acceptable deviation, metres.

    Outputs:
        report (dict): passed, rows, max_dev_m, rms_dev_m, rate_hz,
            time_monotonic, gaps, simulated_source.
    --------------------------------------------------------------------------
    """
    header, rows = parse_monitor_csv(csv_path)
    if not rows:
        return {"passed": False, "rows": 0, "max_dev_m": float("inf"),
                "rms_dev_m": float("inf"), "rate_hz": 0.0,
                "time_monotonic": False, "gaps": [], "simulated_source": False,
                "reason": "no data rows"}

    deviations = [
        distance_to_polyline((r[4], r[5], r[6]), polyline) for r in rows
    ]
    max_dev = max(deviations)
    rms_dev = math.sqrt(sum(d * d for d in deviations) / len(deviations))

    times = [r[0] for r in rows]
    monotonic = all(b > a for a, b in zip(times, times[1:]))
    span = times[-1] - times[0]
    rate = (len(times) - 1) / span if span > 0 else 0.0

    # A gap well beyond the 20 ms grid is a pause, not a dropout.
    gaps = [(a, b) for a, b in zip(times, times[1:]) if (b - a) > 0.100]

    simulated = any("127.0.0.1" in line or "SIMULATED SOURCE" in line
                    for line in header)

    return {
        "passed": bool(max_dev <= tol_m and monotonic),
        "rows": len(rows),
        "max_dev_m": max_dev,
        "rms_dev_m": rms_dev,
        "rate_hz": rate,
        "time_monotonic": monotonic,
        "gaps": gaps,
        "simulated_source": simulated,
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest tests.test_verify_csv -v`
Expected: PASS, 11 tests.

- [ ] **Step 5: Wire `--verify-csv` into the CLI**

In `ur5_sim/cli.py`, add the flag and a branch that runs before any IK work:

```python
    parser.add_argument(
        "--verify-csv", nargs="?", const="auto", default=None,
        metavar="PATH",
        help="check a monitor CSV against the commanded trajectory "
             "('auto' takes the newest ACQ_rtde_*.csv)")
```

```python
def _newest_recorded_csv() -> str | None:
    """Newest ACQ_rtde_*.csv under datalogger/sim_runs/, then the cwd."""
    candidates: list[str] = []
    for folder in (os.path.join("datalogger", "sim_runs"), "."):
        candidates.extend(glob.glob(os.path.join(folder, "ACQ_rtde_*.csv")))
    if not candidates:
        return None
    return max(candidates, key=os.path.getmtime)


def _run_verify_csv(arg: str, polyline) -> int:
    path = _newest_recorded_csv() if arg == "auto" else arg
    if path is None:
        print("[verify] no ACQ_rtde_*.csv found")
        return 1

    report = verify(path, polyline, tol_m=1e-5)
    print(f"[verify] {path}")
    print(f"[verify]   rows          : {report['rows']}")
    print(f"[verify]   max deviation : {report['max_dev_m'] * 1000:.4f} mm")
    print(f"[verify]   rms deviation : {report['rms_dev_m'] * 1000:.4f} mm")
    print(f"[verify]   effective rate: {report['rate_hz']:.2f} Hz")
    print(f"[verify]   time monotonic: {report['time_monotonic']}")
    for a, b in report["gaps"]:
        print(f"[verify]   pause gap     : {a:.3f} s -> {b:.3f} s")
    if report["simulated_source"]:
        print("[verify]   SOURCE        : SIMULATED (ur5_sim emulator), "
              "not robot data")
    print(f"[verify] {'PASS' if report['passed'] else 'FAIL'}")
    return 0 if report["passed"] else 1
```

The branch runs before any IK work. It builds the commanded polyline with the
existing parse plus `transform` plus `rotate_translation_y` plus
`apply_surface_constraint` chain — the same clamped array the emulator was
handed, not the raw one — then returns the exit code above so a script can gate
on it. Add `import glob` and `import os` to `cli.py` if absent.

- [ ] **Step 6: Verify by hand**

Run: `python -m ur5_sim --verify-csv auto`
Expected: a report naming the file, the row count, max and RMS deviation below 1e-5 m, a rate near 50 Hz, and the simulated-source note.

- [ ] **Step 7: Commit**

```bash
git add ur5_sim/verify_csv.py ur5_sim/cli.py tests/test_verify_csv.py
git commit -m "Add --verify-csv: geometric, pause-immune check of a recorded CSV"
```

---

### Task 9: Simulated-source provenance in the monitor

**Model: Sonnet.** Small, localized C change with the full replacement function and its tests given. The existing C suite catches a mistake immediately.

Emulator CSVs otherwise carry the same prefix and the same header as lab recordings. For a research dataset that is a hazard worth designing out.

**Files:**
- Modify: `datalogger/rtde_fallback_monitor.c`
- Test: `datalogger/tests/test_rtde_fallback_monitor.c`

**Interfaces:**
- Consumes: existing `is_valid_ipv4`, `format_csv_header`
- Produces: `int is_loopback_ipv4(const char *s)`; `format_csv_header` gains a trailing `int simulated` parameter

- [ ] **Step 1: Write the failing test**

In `datalogger/tests/test_rtde_fallback_monitor.c`, add before `test_csv_filename_is_stamped_and_prefixed`:

```c
/*
 * An emulator CSV must never be mistakable for a lab recording: same prefix,
 * same schema, same folder.  A loopback endpoint is the marker, since the
 * robot is at 192.168.4.38 and the emulator binds 127.0.0.1 only.
 */
static void test_loopback_detection(void)
{
    GROUP("is_loopback_ipv4");
    CHECK(is_loopback_ipv4("127.0.0.1") == 1);
    CHECK(is_loopback_ipv4("127.1.2.3") == 1);
    CHECK(is_loopback_ipv4("192.168.4.38") == 0);
    CHECK(is_loopback_ipv4("10.0.0.1") == 0);
}

static void test_simulated_header_is_marked(void)
{
    char buf[1024];

    GROUP("format_csv_header: simulated source");
    CHECK(format_csv_header(buf, sizeof(buf), "127.0.0.1", 30004,
                            "2026-08-14", "10:15:30", 1.5, 1) > 0);
    CHECK(strstr(buf, "# WARNING: SIMULATED SOURCE - ur5_sim RTDE emulator, "
                      "not robot data\n") != NULL);

    CHECK(format_csv_header(buf, sizeof(buf), "192.168.4.38", 30004,
                            "2026-08-14", "10:15:30", 1.5, 0) > 0);
    CHECK(strstr(buf, "SIMULATED SOURCE") == NULL);
}
```

Update the two existing `format_csv_header` calls to pass a final `0`, and register both new tests in `main()`.

- [ ] **Step 2: Run test to verify it fails**

Run: `datalogger\tests\build_and_run_tests.bat`
Expected: BUILD FAILED — undefined reference to `is_loopback_ipv4`, and too many arguments to `format_csv_header`.

- [ ] **Step 3: Write minimal implementation**

In `datalogger/rtde_fallback_monitor.c`, add beside `is_valid_ipv4`:

```c
/*
 * 127.0.0.0/8.  The emulator in ur5_sim binds loopback only, so this is what
 * separates a simulated recording from a lab one in the CSV header.
 */
static int is_loopback_ipv4(const char *s)
{
    return (s[0] == '1' && s[1] == '2' && s[2] == '7' && s[3] == '.');
}
```

Replace `format_csv_header` in full:

```c
static int format_csv_header(char *buf, size_t buflen, const char *robot_ip,
                             int robot_port, const char *date_str,
                             const char *time_str, double rtde_t0,
                             int simulated)
{
    int n = snprintf(buf, buflen,
        "# Robot Model: UR5 CB3\n"
        "# PolyScope Version: 3.11.0.82155 (20 August 2019)\n"
        "# Data Source: RTDE fallback monitor (192.168.4.14)\n"
        "# Robot RTDE Endpoint: %s:%d\n"
        "%s"
        "# File Creation Date: %s\n"
        "# File Creation Time: %s\n"
        "# Target Acquisition Frequency: " TARGET_HZ_LABEL "\n"
        "# Time Column: RTDE timestamp field, relative to the first sample"
        " of this file (s)\n"
        "# RTDE Timestamp At First Sample: %.6f s (controller uptime)\n"
        CSV_SCHEMA_LINE,
        robot_ip, robot_port,
        simulated ? "# WARNING: SIMULATED SOURCE - ur5_sim RTDE emulator, "
                    "not robot data\n" : "",
        date_str, time_str, rtde_t0);

    if (n < 0 || (size_t)n >= buflen) {
        return -1;
    }
    return n;
}
```

In `csv_open`, pass the flag through:

```c
    if (format_csv_header(header, sizeof(header), robot_ip, robot_port,
                          date_str, time_str, rtde_ts,
                          is_loopback_ipv4(robot_ip)) < 0) {
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `datalogger\tests\build_and_run_tests.bat`
Expected: TESTS PASSED, 161 checks.

- [ ] **Step 5: Rebuild the executable**

Run: `datalogger\build.bat`
Expected: `Built rtde_fallback_monitor.exe`, no warnings.

- [ ] **Step 6: Commit**

```bash
git add datalogger/rtde_fallback_monitor.c datalogger/tests/test_rtde_fallback_monitor.c
git commit -m "Mark a loopback endpoint as a simulated source in the CSV header"
```

---

### Task 10: Launcher wiring and documentation

**Model: Sonnet.** Batch snippets given in full; the documentation edits are additive and their content is listed section by section. Follow the surrounding style of each file rather than inventing one.

**Files:**
- Modify: `validate.bat`
- Modify: `ARCHITECTURE.md`, `README.md`, `CLAUDE.md`, `datalogger/README.md`
- Create: `datalogger/sim_runs/.gitkeep`
- Modify: `.gitignore`

- [ ] **Step 1: Add the monitor helper to `validate.bat`**

Add a `:start_monitor` label that never blocks the visualizer:

```bat
:start_monitor
if not exist "%~dp0datalogger\rtde_fallback_monitor.exe" (
    echo [WARN] datalogger\rtde_fallback_monitor.exe absent : lancez datalogger\build.bat
    echo        La visualisation continue sans le moniteur RTDE.
    goto :eof
)
if not exist "%~dp0datalogger\sim_runs" mkdir "%~dp0datalogger\sim_runs"
start "UR5 - Moniteur RTDE" "%~dp0datalogger\rtde_fallback_monitor.exe" 127.0.0.1 30004 "%~dp0datalogger\sim_runs"
goto :eof
```

Call it before options 3 and 4 launch the visualizer. The monitor retries every
2 s, so starting it first is safe.

- [ ] **Step 2: Add the two new menu entries**

```bat
echo   7. Emulateur seul     (headless, 2 essais + moniteur)
echo   8. Verifier le CSV    (dernier ACQ_rtde_*.csv)
```

Option 7 calls `:start_monitor` then
`python -m ur5_sim --emulate --runs 2 --pause-at 30`.
Option 8 runs `python -m ur5_sim --verify-csv auto`.

- [ ] **Step 3: Ignore emulator output**

Append to `.gitignore`:

```
datalogger/sim_runs/*.csv
```

Create `datalogger/sim_runs/.gitkeep` so the folder exists after a clone.

- [ ] **Step 4: Update `ARCHITECTURE.md`**

- New section after §7, "RTDE emulation contract (simulator to monitor)": the
  recipe, the byte layout, the loopback bind, the `runtime_state` mapping, and
  the statement that this is a separate channel from §7's UDP overlay, which is
  unchanged.
- §2 tables: add `rtde_server.py`, `force_model.py`, `verify_csv.py`,
  `visualization/playback_clock.py`.
- §9 test table: add `test_rtde_server.py`, `test_force_model.py`,
  `test_playback_clock.py`, `test_verify_csv.py`.
- §10: add a rule — the RTDE wire layout is duplicated in C and Python, and
  must stay pinned by the offset assertions in `tests/test_rtde_server.py`.

- [ ] **Step 5: Update the READMEs**

- `datalogger/README.md`: a "Testing locally against `ur5_sim`" section with the
  two-terminal procedure, and a note that a loopback CSV is marked simulated.
- `README.md` and `CLAUDE.md`: the new commands (`--emulate`, `--verify-csv`,
  `--no-rtde-serve`) and the new test modules.

- [ ] **Step 6: Full verification**

```bash
python -m unittest discover -s tests -p "test_*.py"
datalogger\tests\build_and_run_tests.bat
python -m ur5_sim --check
```
Expected: all green, and `--check` unchanged in behavior.

- [ ] **Step 7: End-to-end**

Run `validate.bat` option 7, wait for both runs, then option 8.
Expected: two CSVs in `datalogger\sim_runs\`, the paused run NOT split, and
`--verify-csv` reporting max deviation below 1e-5 m with the simulated-source
note.

- [ ] **Step 8: Commit**

```bash
git add validate.bat .gitignore datalogger/sim_runs/.gitkeep ARCHITECTURE.md README.md CLAUDE.md datalogger/README.md
git commit -m "Wire the emulator and monitor into validate.bat and document the contract"
```

---

## Verification summary

| Spec section | Task |
|---|---|
| §3 architecture, module boundaries | 1, 4, 5 |
| §4 wire contract | 1 (offsets pinned), 5 (handshake) |
| §5 timing, interpolation, always-on stream | 3, 5 |
| §6 runtime_state machine and PAUSE | 2, 6 |
| §7 force model | 4 |
| §8 provenance and loopback bind | 5 (bind), 9 (header marker), 10 (sim_runs) |
| §9 CLI surface | 7, 8 |
| §10 verify-csv | 8 |
| §11 error handling | 5 (port in use, disconnect, drop-not-stall) |
| §12 testing | every task; C side in 9 |
| §13 validate.bat | 10 |
| §14 documentation | 10 |
