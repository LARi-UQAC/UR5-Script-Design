# Plan — RTDE fallback monitor for UR5 CB3 (standalone C executable on a lab computer)

This plan is **independent**: it can be executed in a fresh session with no knowledge
of any other plan in this repo. It designs and builds exactly one deliverable: a
small, dependency-free command-line tool that runs on a lab computer, on the same
network as a UR5 robot, and passively records TCP force and pose data to its own CSV
file as a redundant/fallback data path.

## Context

A UR5 CB3 robot (PolyScope 3.11.0.82155, 20 August 2019) runs an experimental
ISO/COLIPA cosmetic-spread protocol, 20-30 trials of ~3 min each. The **primary**
way this data is captured is a separate, already-planned on-robot logger (URScript +
an on-controller Python daemon writing to the robot's own USB key) — see
[`plan_acq_datalogger.md`](plan_acq_datalogger.md) if that context is ever wanted,
but this plan does not depend on it in any way: no shared code, no shared process, no
shared file. That plan and this one are two independent, required deliverables of the
overall data-logging effort (user decision, 2026-08-06): **both** must exist; this is
not a fallback that replaces the other, it is a second, redundant path.

This plan's tool is the **fallback**: it reads the robot's Real-Time Data Exchange
(RTDE) stream from a second machine over the network, entirely without touching the
robot's controller, its USB key, or any URScript program. It exists so operators still
get force/pose data even if the on-robot path is ever unavailable, and so the two
sources can cross-validate each other during commissioning.

Variables to capture, same convention as the main path so the two CSVs are directly
comparable: `Time, ForceX, ForceY, ForceZ, PoseX, PoseY, PoseZ` (TCP force X/Y/Z and
TCP pose X/Y/Z only — no rotation components, no torque components), target 50 Hz.

## 0. Facts established before this plan (no further questions needed to start)

1. **Network topology** (confirmed 2026-08-06): both machines sit on the same isolated
   VLAN behind an IE5000 industrial switch, VLAN 4, **no DHCP, no Internet route**.
   - Robot controller: static IP **`192.168.4.38`**, switch port 8. RTDE listens on
     this IP, **TCP port 30004**, always on (stock CB3 service, no configuration
     needed on the robot side for this plan).
   - Lab computer (runs this tool): static IP **`192.168.4.14`**, switch port 4, wired
     (not Wi-Fi — the site's Wi-Fi, `laimi-robot`, conflicts with other local networks
     and is unreliable; wired is the only planned access path).
   - Subnet mask was not specified; both addresses are consistent with a `/24`
     (`192.168.4.0/24`). Confirm the real mask against the switch's VLAN 4 config at
     deployment time (§6 step 5) — this does not block writing or building the tool.
2. **Lab computer OS and available tooling** (confirmed 2026-08-06): **Windows**,
   `cmd.exe` confirmed present, **no Python installed**, PowerShell version/
   availability **uncertain**. No internet access on this VLAN, so nothing can be
   installed on this machine at all — whatever runs there must already work with
   zero additional dependency.
3. **Language decision: C, compiled to one static executable.** Not PowerShell:
   version/availability on the target machine is uncertain, and this removes that
   question entirely (only dependency left is `cmd.exe` + the core Windows
   `ws2_32.dll`, both universal on any Windows version). Not Rust: would need
   `rustup`/`cargo` installed on the *build* machine first; a plain MinGW-w64 `gcc` is
   already available with no setup. Not Python (no interpreter on the target and no
   way to get one there). The tool is built once, on a normal networked machine that
   has `gcc`, and only the resulting single `.exe` is copied to the lab computer —
   nothing else ships, no DLLs (static link), no install step on that machine.
4. **Can the robot's own data tell this tool when a trial starts and stops, so it can
   open/close CSV files automatically without any manual action?** Yes. RTDE exposes
   an output field named `runtime_state`, a `UINT32` program-execution-state enum:
   `0` STOPPING, `1` STOPPED, `2` PLAYING (= running), `3` PAUSING, `4` PAUSED,
   `5` RESUMING. Requesting `runtime_state` in the same output recipe as `timestamp`,
   `actual_TCP_pose`, `actual_TCP_force` costs nothing extra — it arrives in the same
   stream, no polling, no second connection. Rule used by this tool: open a new CSV on
   a transition **into** `PLAYING` from `STOPPED` (a genuine new run), keep the same
   file open across `PAUSING`/`PAUSED`/`RESUMING` (still the same run, e.g. an
   operator pausing mid-trial), and close/finalize the file on a transition **into**
   `STOPPED`. No manual "close current file, start new file" action is ever needed.
   The Dashboard Server (`port 29999`, `running` query) was considered and rejected:
   it is request/response text, needs its own polling loop and connection, and gives
   strictly less information than the RTDE field already present in the stream this
   tool reads anyway.
   Sources: [RTDE Guide (docs.universal-robots.com), handshake and field types](https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html),
   [RTDE `runtime_state` enumeration, confirmed on real UR3 CB-series (UR forum)](https://forum.universal-robots.com/t/rtde-runtime-state-enumeration/6634).
5. **RTDE availability on this exact controller**: RTDE requires PolyScope >= 3.3; the
   robot runs 3.11.0.82155 (2019), so it qualifies. Not yet tested from this specific
   network path — user is testing 2026-08-06 (§6 step 4).

## 1. Architecture

```
Lab computer (192.168.4.14, wired to IE5000 port 4, same VLAN 4 as the robot, cmd.exe)
 └─ rtde_fallback_monitor.exe   -> single static binary, no GUI, connects OUT to the robot
      run from cmd:  rtde_fallback_monitor.exe 192.168.4.38 30004 .
      1. Winsock2 TCP connect to 192.168.4.38:30004 (RTDE)
      2. handshake: RTDE_REQUEST_PROTOCOL_VERSION, then
         RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS "timestamp,actual_TCP_pose,
         actual_TCP_force,runtime_state", then RTDE_CONTROL_PACKAGE_START
      3. per RTDE_DATA_PACKAGE: unpack via a hand-written big-endian reader (see
         below), keep timestamp + pose[0:3] + force[0:3] + runtime_state
      4. runtime_state transition STOPPED->PLAYING: open ACQ_rtde_YYYYMMDD_HHMMSS.csv
         (computer's own wall clock -- reliable, this machine is a normal Windows PC)
      5. runtime_state transition ->STOPPED: finalize + close current file
      6. PAUSING/PAUSED/RESUMING: same file stays open, samples keep appending
```

- **Read-only toward the robot**: the tool only ever reads the RTDE output stream. It
  sends no motion command, no register write, nothing that could affect a running
  program — safe to leave connected indefinitely, and safe to run at the same time as
  the (independent) on-robot logging path for cross-validation.
- **No shared state with anything else**: no shared file, no shared socket, no shared
  process with the robot's own USB-based logging. A fault in one cannot corrupt the
  other's output.

## 2. Design details

- **Build**: `gcc -O2 -static -o rtde_fallback_monitor.exe rtde_fallback_monitor.c
  -lws2_32` (Winsock2 import lib; `ws2_32.dll` itself is a core Windows system DLL
  present on every Windows install, not a dependency concern — `-static` only needs to
  cover the MinGW C runtime, which it does). Exact command lives in
  `datalogger/README.md` and `datalogger/build.bat`; no Makefile needed for one
  source file.
- **No `ur_rtde` library usage of any kind** (C++ or Python): the RTDE wire protocol
  is hand-implemented directly with Winsock2 (`socket`, `connect`, `send`, `recv`) —
  2-byte size + 1-byte type header, then a body whose field layout is fixed once the
  output recipe is acknowledged (`DOUBLE` = 8 bytes, `VECTOR6D` = 6 doubles, `UINT32`
  = 4 bytes).
- **Endianness**: RTDE is big-endian (network byte order); x86/x64 is little-endian.
  Every multi-byte field goes through one small, tested helper pair —
  `uint32_t read_be_u32(const unsigned char *p)` and
  `double read_be_double(const unsigned char *p)` (byte-reverse into a local buffer,
  then `memcpy` into the target type — never a pointer-cast, which would be undefined
  behavior/strict-aliasing UB in C). Every field decode goes through these two
  functions, never an inline cast, so there is exactly one place to get this right and
  exactly one place the tests need to pin down with known byte sequences.
- **CSV writing**: plain `fopen`/`fprintf`/`fclose`. Header exactly
  `Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ`, matching the on-robot path's schema
  (§0 Context) so the two sources are directly comparable, with one added metadata
  line identifying this source (`# Data Source: RTDE fallback monitor
  (192.168.4.14)`). Filename prefix `ACQ_rtde_` keeps this tool's output from ever
  being mistaken for the on-robot path's `ACQ_log_*` files even if both land in the
  same folder.
- **Unsigned-executable warning**: an `.exe` copied from removable media and run for
  the first time can trigger Windows SmartScreen ("Windows protected your PC");
  documented in the README as an expected one-time "More info -> Run anyway" click,
  not a failure.
- **Sampling rate**: RTDE output frequency is requested as a divisor of the
  controller's base control-loop rate; the exact base rate for this CB3/PolyScope 3.11
  build (125 Hz historically for CB3, vs 500 Hz documented for e-Series) is not
  confirmed from public docs for this specific build. Rather than gamble on a divisor
  formula, the tool requests the recipe at the server's default (full) rate and
  software-decimates to a 20 ms cadence using each packet's own `timestamp` field, so
  output is timestamp-exact regardless of the underlying base rate. Confirm the actual
  base rate empirically on first connect (log the raw inter-packet interval once) and
  revisit requesting a direct-frequency recipe as an optimization if it proves
  reliable.
- **Parsing/decision logic factored into small pure functions** (byte decode,
  state-transition action, CSV row formatting), each callable without a live socket,
  specifically so the tests (§4) can exercise them directly without needing a real or
  fake network connection for every case.

## 3. Deliverables (new `datalogger/` folder in the repo)

| File | Content |
|---|---|
| `datalogger/rtde_fallback_monitor.c` | The tool itself (§1, §2). CLI args: `rtde_fallback_monitor.exe <robot-ip> <rtde-port> <out-dir>`. |
| `datalogger/build.bat` | One-liner: `gcc -O2 -static -o rtde_fallback_monitor.exe rtde_fallback_monitor.c -lws2_32`. Run on a normal networked machine with `gcc`; only the resulting `.exe` is copied to the lab computer. |
| `datalogger/tests/test_rtde_fallback_monitor.c` | Hand-rolled C test harness (no framework — zero new dependency), `assert()`-based, non-zero exit on any failure. Two layers: (1) **unit** — known byte sequences through `read_be_u32`/`read_be_double` against expected values (endianness correctness, pinned down once); state-transition table (`STOPPED->PLAYING`=OPEN, `PLAYING->PAUSED`=NONE, `PAUSED->PLAYING`=NONE, `PLAYING->STOPPED`=CLOSE, etc.) exercised for every pair, not just the happy path. (2) **integration** — a local Winsock2 `TcpListener` on `127.0.0.1` in the test binary itself replays the documented RTDE handshake and a scripted `runtime_state` sequence; asserts one CSV per run, correct header/schema, decimation close to 20 ms, and that a forced mid-stream socket close leaves the partial file intact with a diagnostic printed (not silently dropped). |
| `datalogger/tests/build_and_run_tests.bat` | `gcc -O2 -static -o test_rtde_fallback_monitor.exe test_rtde_fallback_monitor.c -lws2_32 && test_rtde_fallback_monitor.exe`. |
| `datalogger/README.md` | Deployment procedure: build, wire the lab computer, set static IP, confirm reachability, copy the `.exe`, run it, confirm automatic file creation. Documents the SmartScreen prompt and the subnet-mask confirmation step. If `plan_acq_datalogger.md`'s own `datalogger/README.md` already exists by the time this is implemented, add a clearly separated section to it rather than creating a conflicting second file of the same name — this tool's README content is additive, not a replacement. |

This is a **C-language exception** to this repo's otherwise Python-only test suite
(`python -m unittest discover -s tests`) and its stdlib-`unittest` convention — the
target machine having no Python and uncertain PowerShell forces the tool itself into
C, and its tests follow the same language so they can call the tool's own functions
directly rather than shelling out. Document this exception plainly in the top-level
`README.md`/`CLAUDE.md` test instructions when this plan is executed.

## 4. Implementation order

1. `datalogger/rtde_fallback_monitor.c` + `datalogger/tests/test_rtde_fallback_monitor.c`
   + `datalogger/build.bat` + `datalogger/tests/build_and_run_tests.bat` — TDD: write
   the pure functions (`read_be_u32`, `read_be_double`, the state-transition decision,
   CSV row formatting) and their unit tests first, then the handshake/socket loop and
   its integration test against a local fake RTDE listener.
2. `datalogger/tests/build_and_run_tests.bat` — must pass before moving on.
3. `datalogger/README.md` — deployment procedure (§3, §6).
4. Confirm the tool never touches `etalement*.script`, `etalement*.urp`, `ur5_sim`, or
   the design UI in any way — it is a fully standalone program.

## 5. CSV format (written by this tool)

```
# Robot Model: UR5 CB3
# PolyScope Version: 3.11.0.82155 (20 August 2019)
# Data Source: RTDE fallback monitor (192.168.4.14)
# File Creation Date: YYYY-MM-DD
# File Creation Time: HH:MM:SS
# Target Acquisition Frequency: 50 Hz
Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ
0.000,-0.123456,0.234567,-6.012345,0.412345,-0.298765,0.101234
...
```

- `Time` = seconds since this file was opened (or the RTDE `timestamp` field directly
  — decide at implementation, document whichever is chosen in the file's own header
  comment so it never needs cross-referencing another document to interpret).
- Floats: 6 decimals (m and N — engineering precision), matching the on-robot path.
- No `Maximum Buffer Size` / `Actual Number of Collected Samples` metadata lines: this
  tool has no fixed-size buffer (it streams straight to disk row by row via
  `fprintf`), so those two lines from the on-robot path's format do not apply here.

## 6. Verification

Offline (dev/build machine):
1. `datalogger\tests\build_and_run_tests.bat` — handshake parsing, big-endian
   unpacking (known-value pinned), auto file boundaries on every `runtime_state`
   transition pair, decimation, partial-file preservation. Must pass before any
   on-robot step.

On the lab network:
2. `datalogger\build.bat` on a machine with `gcc` (need not be the lab computer).
3. Wire the lab computer to IE5000 switch port 4, set static IP `192.168.4.14/24`.
4. `ping 192.168.4.38` — confirms L2/L3 reachability before touching RTDE.
5. Confirm the actual VLAN 4 subnet mask against the switch config (§0.1); adjust the
   static IP's prefix length if it is not `/24`.
6. Copy only `rtde_fallback_monitor.exe` to the lab computer; run from `cmd.exe`:
   `rtde_fallback_monitor.exe 192.168.4.38 30004 .` (accept the one-time SmartScreen
   prompt if it appears).
7. Start a trial from the pendant; confirm a `ACQ_rtde_*.csv` appears automatically,
   with no manual file action on the lab computer.
8. Pause and resume the program from the pendant mid-trial: confirm the file does NOT
   split into two.
9. Stop the trial: confirm the file closes; start a second trial: confirm a second,
   distinct file opens automatically.
10. If the on-robot path (`plan_acq_datalogger.md`) is also running, confirm both
    produce plausible, independently readable CSVs for the same trial (cross-check,
    not a strict equality requirement — the two paths sample independently).

## 7. Risks and mitigations

- **Lab computer has no Python, PowerShell version uncertain** (confirmed) — resolved
  by writing this tool in C, statically compiled to a single `.exe` (§0.3). Only
  runtime requirement is `cmd.exe` (confirmed present) and the core Windows
  `ws2_32.dll`, both universal on any Windows version.
- **Subnet mask for `192.168.4.14`**: not given; deferred to the on-site deployment
  step (§6 step 5) rather than blocking this plan — `/24` is the working assumption
  until confirmed against the switch.
- **Unsigned `.exe` from removable media triggers Windows SmartScreen** on first run:
  documented as an expected one-time "More info -> Run anyway" click (§2, §6), not a
  failure to debug.
- **Endianness bug in hand-rolled parsing**: an unreversed field produces a
  plausible-looking but wrong number instead of an error, and a raw pointer-cast
  instead of `memcpy` would additionally be undefined behavior in C. Mitigated by
  centralizing every conversion through the two tested `read_be_u32`/`read_be_double`
  helpers (§2), with fixed known-value unit tests, not by reasoning about it inline at
  each call site.
- **RTDE base output rate for this exact CB3 build is unconfirmed** (§2): the tool
  decimates from whatever full rate the controller streams rather than requesting an
  assumed divisor, so this does not block correctness, only removes a possible
  optimization (requesting the exact rate server-side) until confirmed on first
  connect.
- **RTDE field name mismatch**: if `runtime_state` or another requested field is not
  supported on this build, RTDE returns type `NOT_FOUND` for it at recipe setup
  (documented, clean failure) rather than silently mis-parsing — the tool must check
  for this at connect time and abort with a clear message, not start logging garbage.
- **Controller reboot / RTDE disconnect mid-trial**: the tool must detect a closed
  socket, print a clear diagnostic, and keep whatever was already written to the
  current CSV intact (no truncation, no silent data loss) rather than crash.

## 8. Requirement traceability

- Small CLI tool, no GUI, on a lab computer monitoring the robot's port -> §1, §3.
- Detect trial start/stop from the data stream itself, avoid manual file
  open/close -> §0.4, `runtime_state` transition rule (§1 steps 4-6).
- No dependency installable on the target (no Python, no internet) -> §0.2-3, C
  static executable.
- Same variables monitored as the main path (interpreted as: same CSV schema/columns,
  not a literal shared-memory link between two separate machines, which would not be
  physically meaningful) -> §2 CSV writing, §5.
- Both main and fallback paths required, not either/or -> this plan (fallback) +
  `plan_acq_datalogger.md` (main), two independent deliverables, no shared code.

## 9. Original prompt context

This plan descends from a larger single prompt that originally requested a
pure-URScript on-robot logger (now `plan_acq_datalogger.md`); the RTDE fallback
requirement was added afterward, once it was established that a lab computer with no
Python could reach the robot's controller over an isolated VLAN. See
`plan_acq_datalogger.md` section 10 for the original verbatim prompt text (kept there,
not duplicated here, since it describes the other plan's deliverable).
