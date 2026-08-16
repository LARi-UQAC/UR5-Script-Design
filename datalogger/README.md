# datalogger

Data-acquisition tooling for the ISO/COLIPA cosmetic-spread trials on the UR5 CB3.

Two independent paths record the same variables. Both are required; neither replaces the
other, and they share no code, no process and no file, so a fault in one cannot corrupt the
output of the other.

| Path | Where it runs | Output | Status |
|---|---|---|---|
| On-robot logger (main) | Robot controller + USB key | `ACQ_log_*.csv` | Implemented, in [`../onrobot/`](../onrobot/README.md) - Python, not part of this folder |
| RTDE fallback monitor | Lab computer, over the network | `ACQ_rtde_*.csv` | Implemented, described below |

**This folder is C only.** It holds one executable, which records what the robot transmits
on its RTDE port, plus its build script and its C test harness. The on-robot acquisition
path is Python and lives in [`../onrobot/`](../onrobot/README.md); the two share the seven
CSV columns and nothing else - no code, no process, no file.

---

## RTDE fallback monitor

`rtde_fallback_monitor.exe` reads the robot's Real-Time Data Exchange stream from a second
machine and writes one CSV per robot program run. It touches nothing on the controller: no
URScript program, no USB key, no installed software. Design and rationale are in
[`plan_rtde_fallback_monitor.md`](../docs/superpower/plans/plan_rtde_fallback_monitor.md).

The tool is **read-only toward the robot**. It opens one outbound TCP connection, requests
an output recipe and reads packets. The RTDE input path is never used, so no motion
command, no register write and nothing else that could disturb a running program can leave
this tool. It is safe to leave connected during a trial and safe to run at the same time as
the on-robot logger, which is how the two sources cross-validate each other.

### What it records

`timestamp`, `actual_TCP_pose`, `actual_TCP_force` and `runtime_state`, written as the
seven columns shared with the on-robot path:

```
Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ
```

Only the translational components are kept (no rotations, no torques), at a 50 Hz cadence.

### Automatic file boundaries, no operator action

`runtime_state` travels in the same stream as the measurements, so the tool finds its own
file boundaries with no polling, no second connection and no manual "start a new file"
step:

| Transition | Action |
|---|---|
| `STOPPED` to `PLAYING` | Open a new `ACQ_rtde_*.csv` |
| `PAUSING`, `PAUSED`, `RESUMING` | Nothing: same trial, same file, samples keep appending |
| anything to `STOPPED` | Finalize and close the file |

Starting the tool while a program is already running also opens a file, rather than waiting
for the next program start.

If two trials land in the same wall-clock second, the second file gets a `_1`, `_2` suffix.
No trial can overwrite another.

---

## Build

Needs MinGW-w64 `gcc` (`x86_64-w64-mingw32`). Build on any normal networked machine; the
lab computer needs nothing installed.

```
datalogger\build.bat
```

or directly:

```
gcc -O2 -static -Wall -Wextra -o rtde_fallback_monitor.exe rtde_fallback_monitor.c -lws2_32
```

`-static` links the MinGW C runtime in, so the result is a single self-contained 64-bit
executable. Confirmed dependencies are `KERNEL32.dll`, `msvcrt.dll` and `WS2_32.dll` only,
all core Windows libraries present on every Windows 10 install.

## Test

```
datalogger\tests\build_and_run_tests.bat
```

Builds and runs the harness (147 checks; non-zero exit on any failure). No robot, no
network, no framework. It covers big-endian decoding against known byte sequences, every
ordered `runtime_state` transition pair, the decimation cadence, the CSV format, and an
integration layer that replays the RTDE handshake from a fake server on `127.0.0.1`:
one-run/one-file, pause not splitting a file, two runs producing two files, a reset
connection leaving the partial file intact, a `NOT_FOUND` recipe field aborting before any
data is logged, and the protocol-version-1 fallback.

Run it before any on-robot step.

---

## Deployment procedure

1. **Build** on a machine that has `gcc` (need not be the lab computer): `build.bat`.
2. **Wire** the lab computer to IE5000 switch port 4 (VLAN 4). Wired only: the site's
   `laimi-robot` Wi-Fi conflicts with other local networks and is not a supported path.
3. **Set the static IP** `192.168.4.14/24` on that interface. There is no DHCP on this
   VLAN.
4. **Confirm the subnet mask** against the switch's VLAN 4 configuration. `/24` is the
   working assumption, consistent with both addresses, but it was never confirmed against
   the switch; adjust the prefix length if VLAN 4 uses something else.
5. **Confirm reachability** before touching RTDE:
   ```
   ping 192.168.4.38
   ```
6. **Copy only** `rtde_fallback_monitor.exe` to the lab computer. Nothing else ships: no
   DLL, no installer, no source.
7. **Run it** from `cmd.exe`, from the folder where the CSVs should land:
   ```
   rtde_fallback_monitor.exe 192.168.4.38 30004 .
   ```
   Stop it with Ctrl+C; the file in progress is finalized before it exits.

### First-run SmartScreen prompt

An unsigned executable copied from removable media raises "Windows protected your PC" the
first time it runs. Click **More info**, then **Run anyway**. This is expected once, not a
failure to debug.

## Verifying on the robot

1. Start a trial from the pendant. An `ACQ_rtde_*.csv` appears on its own, with no action on
   the lab computer.
2. Pause and resume from the pendant mid-trial. The file must **not** split in two.
3. Stop the trial. The file closes; the console prints its row count.
4. Start a second trial. A second, distinct file opens automatically.
5. If the on-robot path is also running, confirm both CSVs are plausible and independently
   readable for the same trial. The two sample independently, so this is a cross-check, not
   an equality test.

On the first connection the tool prints the measured inter-packet interval, for example:

```
[RTDE] stream interval 0.0080 s (125.0 Hz), decimating to 50 Hz
```

That line settles the controller's real base output rate, which is not confirmed from
public documentation for this PolyScope build.

---

## CSV format

```
# Robot Model: UR5 CB3
# PolyScope Version: 3.11.0.82155 (20 August 2019)
# Data Source: RTDE fallback monitor (192.168.4.14)
# Robot RTDE Endpoint: 192.168.4.38:30004
# File Creation Date: 2026-08-14
# File Creation Time: 10:15:30
# Target Acquisition Frequency: 50 Hz
# Time Column: RTDE timestamp field, relative to the first sample of this file (s)
# RTDE Timestamp At First Sample: 123456.797000 s (controller uptime)
Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ
0.000,-0.123456,0.234567,-6.012345,0.412345,-0.298765,0.101234
```

- `Time` is the robot's own RTDE `timestamp`, referenced to the first sample of the file, so
  it starts at `0.000` exactly like the on-robot path's tick time and the two CSVs align
  without any clock conversion. The absolute controller value of that first sample is kept
  in the header, so the offset loses nothing. Three decimals, matching the on-robot path.
- Forces are newtons and poses are metres, six decimals.
- No `Maximum Buffer Size` or `Actual Number of Collected Samples` line: this tool has no
  fixed-size buffer, it streams straight to disk row by row.

### Cadence

The controller streams at its own base rate and the tool imposes the 50 Hz cadence from
each packet's own `timestamp`, so the output is timestamp-exact whatever that base rate
turns out to be. The rule is grid-based, emitting the first packet at or after each 20 ms
boundary. A gap-based rule would give 125/3 = 41.7 Hz on a 125 Hz stream instead of 50 Hz.

## Behavior when things go wrong

| Situation | Behavior |
|---|---|
| Robot unreachable, or not yet powered | Prints the reason, retries every 2 s. Leave it running; it connects when the robot comes up. |
| Controller reboot or cable pull mid-trial | Prints a diagnostic, finalizes the current CSV with every row already written, then reconnects. No truncation, no silent loss. |
| A recipe field unsupported on this build | RTDE answers `NOT_FOUND`; the tool refuses the session and exits rather than decode at wrong offsets and fill a CSV with plausible garbage. |
| Mistyped robot IP | Rejected at startup, before the reconnect loop, so a typo fails fast instead of retrying forever. |

## Notes for maintainers

- **The C source is an exception to this repo's Python-only test convention.** The lab
  computer has no Python and nothing can be installed on it (no internet on that VLAN), so
  the tool is C; its tests are C as well, so they can call its own functions directly rather
  than shell out. They are not part of `python -m unittest discover -s tests`; run
  `datalogger\tests\build_and_run_tests.bat` instead.
- Every multi-byte field is decoded through `read_be_u16` / `read_be_u32` /
  `read_be_double`, never an inline pointer cast. RTDE is big-endian and x86-64 is not; a
  cast would also violate alignment and strict aliasing, since fields sit at arbitrary
  offsets behind a 3-byte header. One place to get right, pinned by known-value tests.
- The tool negotiates RTDE protocol version 2 and falls back to version 1. Version 1 is not
  a degraded mode here: it carries no `output_frequency` field at all, which suits a tool
  that takes whatever the controller streams and imposes its own cadence.
- The tool is standalone. It reads and writes nothing belonging to `etalement.script`,
  `etalement.urp`, `ur5_sim` or the design UI.
