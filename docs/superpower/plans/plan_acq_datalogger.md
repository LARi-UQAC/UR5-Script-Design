# Plan — 50 Hz force/pose data logger for UR5 CB3 (on-robot main path)

## Context

Experimental need: record TCP force (Fx,Fy,Fz) and TCP position (X,Y,Z) at 50 Hz during the
ISO/COLIPA spread protocol runs on the UR5 CB3 (RobotIQ FT-300 + 2F-85 passive finger),
20-30 trials of ~3 min each, one uniquely named CSV per trial saved to the robot's USB key.
This plan covers the **main path only**: fully on-controller, no dependency on any
second machine being present, wired, or working.

**Split (2026-08-06)**: this plan was originally combined with a separate fallback
tool (a lab-computer-side monitor reading the robot's RTDE stream). The two are now
independent plans/issues so each can be executed standalone in a fresh session:
- **This plan** — the main path, on-robot only.
- [`plan_rtde_fallback_monitor.md`](plan_rtde_fallback_monitor.md) — the fallback, a
  standalone C executable on a lab computer. Tracked separately (see that plan's own
  issue). Neither plan depends on the other being implemented first; both are required
  deliverables of the overall data-logging effort, executed independently.

The original prompt (kept in section 10) asked for a pure-URScript solution that buffers
11700 samples and writes `/ramdisk/usb/ACQ_log_YYYYMMDD_HHMMSS.csv`. Validation against
the CB3 URScript language shows that as written it is **impossible**; this plan is the
corrected, feasible design that preserves every requirement's intent.

**Output policy (user decision)**: the current, working generation of `etalement.script`
/ `etalement.urp` is **not modified in any way**. The exporter additionally produces a
second pair, `etalement_acq.script` / `etalement_acq.urp`, which is the original program
plus the data-acquisition process. The simulator (`ur5_sim --check/--visualize`) keeps
consuming `etalement.script` only — nothing changes on the sim side.

## 0. Questions to ask the user at execution start (blocking, per prompt requirement)

1. **Exact PolyScope version** — ANSWERED: **PolyScope 3.11.0.82155 (20 August 2019)**,
   CB3 controller. Confirmed facts for this exact build (all hold for the 3.x branch):
   8 ms thread tick, `socket_send_line` available (list arg auto-serialized), no file
   I/O API, no date/time builtin, no `list_append`, no list slicing. Nothing in this
   plan needs revisiting for a newer/older CB3 build; only an e-Series controller
   (5.x, 2 ms tick) would change §4.
2. **Confirm the urmagic daemon approach is acceptable** (a `urmagic_*.sh` on the USB key
   runs as root on insertion — standard UR mechanism, script fully auditable, stdlib-only
   Python). If the user has SSH access to the controller instead, install the daemon that
   way and skip the magic file.
3. **FT-300 URCap installed and streaming?** — ANSWERED: **URCap 1.11.0.29 (2017)**.
   Researched facts (Robotiq FT Sensor Instruction Manual, DoF forum, see citations
   below): port 63351 is a **fixed** controller-side TCP socket opened once the FT
   Sensor URCap is active; it is not a configurable field anywhere in PolyScope. In
   PolyScope: **Program Robot -> Installation tab -> FT Sensor** (left pane) opens the
   sensor dashboard (RS-485 wiring status, Calibration tab with a zeroing wizard); no
   port number ever appears in that UI, by design. Data-stream mode is started at the
   sensor register level (write `0x0200` to Modbus register 410) and the URCap keeps
   it running once installed; the daemon in this plan just connects to
   `127.0.0.1:63351` and reads whatever is already streaming (see §3/daemon FT-300
   reader). Minimum PolyScope for the URCap is 3.5 -- our 3.11.0.82155 (2019) qualifies,
   and the sensor manual line predates and covers the 2017 URCap release. Deployment
   risk kept in the plan (§8): the exact wire format (field order/separator) is not
   confirmed from public docs for this URCap build, so `test_acq_logger_daemon.py`
   must exercise the parser against the documented Robotiq format and the daemon
   should log a raw sample once at connect time for the first on-robot smoke test, so a
   format mismatch is caught before the file-safety net removes it and NOT silently
   miscolumned into the CSV.
   Sources: [Robotiq FT Sensor Instruction Manual](https://assets.robotiq.com/website-assets/support_documents/document/FT_Sensor_Instruction_Manual_PDF_20181218.pdf),
   [FT Sensor Installation (PolyScope tab path, min. version 3.5)](https://assets.robotiq.com/website-assets/support_documents/document/online/FT_Sensor_Instruction_Manual_Web_20181218.zip/FT_Sensor_Instruction_Manual_Web/Content/Installation.htm),
   [Export Data from FT 300 and FT 150 (port 63351, DoF forum)](https://dof.robotiq.com/discussion/494/export-data-from-ft-300-and-ft-150).

## 1. Validation of the original prompt — issues found (CB3 / PolyScope 3.x facts)

| # | Prompt requirement | CB3 reality | Resolution |
|---|---|---|---|
| 1 | Write CSV to `/ramdisk/usb/` from URScript | URScript has **no file API at all** (motion, math, sockets, textmsg only — deliberate sandbox) | On-controller Python daemon writes the file; URScript talks to it over loopback socket |
| 2 | Filename from controller date/time in URScript | **No date/time function** in URScript 3.x | Daemon uses the controller's Linux clock (`datetime`) for `ACQ_log_YYYYMMDD_HHMMSS.csv` |
| 3 | Preallocate 11700-sample arrays, "no list_append" | `list_append` **does not exist** on CB3 anyway; lists are fixed-size literals, nested lists (`force_log[i] = [Fx,Fy,Fz]`) unsupported, practical list sizes are hundreds not thousands, and an 11700-element literal blows the PolyScope program memory budget | Buffer lives in the daemon's RAM (unlimited). URScript stays allocation-free: one reusable 7-element list per sample |
| 4 | Exact 20 ms sampling via timing control | Thread `sync()` tick is 8 ms on CB3 → 20 ms is 2.5 ticks, not reachable exactly | Alternate 2-tick/3-tick periods (16/24 ms, mean exactly 20 ms, jitter ±4 ms); every sample carries its true tick-time stamp, so analysis is exact |
| 5 | Timestamp = "actual acquisition timestamp" | No wall clock in URScript | Column 1 = robot tick time (s, from the 8 ms sync counter, authoritative relative time). Daemon adds a second column with controller wall-clock arrival time (absolute, ±ms) |
| 6 | `get_tcp_force()` as force source | Built-in is a joint-current **estimate** (multi-newton error) while a calibrated FT-300 streams on local port 63351 at 100 Hz | Primary: daemon reads FT-300 stream and merges latest force into each sample (user-approved). Fallback flag: URScript sends `get_tcp_force()` values instead |
| 7 | String CSV rows built in URScript | CB3 3.x lacks `to_str`/`str_cat` (version-dependent) | `socket_send_line(buf)` with a list auto-serializes (`[t,x,y,z,...]`) on CB3 — no string building at all |
| 8 | "No blocking ops in acquisition loop" | `socket_send` can block only if the peer stalls | Peer is a loopback daemon that drains continuously; 64 KB TCP buffer absorbs any hiccup; send is ~50 µs |
| 9 | No `stopl`/`movel` in threads, no `[0:3]` slicing (project CB3 rules) | Respected: logger thread only reads pose + sends; no slicing anywhere | — |

Conclusion: intent of every requirement is preserved; only the **locus of the buffer and
of the file write moves from URScript (impossible) to a daemon on the same controller**.

## 2. Architecture (2 components, both on the robot / USB key)

```
USB key
 ├─ urmagic_acqlogger.sh      -> runs on insertion, launches daemon (nohup, python2)
 ├─ acq_logger_daemon.py      -> loopback TCP server 127.0.0.1:50100
 └─ ACQ_log_*.csv               <- written here at end of each trial

URScript (data_logger thread, 50 Hz)
   --[loopback socket, one list per sample]--> daemon RAM buffer
                                               + FT-300 reader (127.0.0.1:63351, keeps latest Fx,Fy,Fz)
   --"STOP" sentinel at end of motion------->  daemon writes metadata + CSV to USB, replies "OK <filename>"
```

- During motion: **zero USB I/O** (daemon buffers in RAM only).
- After motion: daemon writes the whole file in one pass, fsync, close, reply.
- Repeat trials: each program run opens a new session -> new timestamped file; daemon adds
  `_1`, `_2` suffix on the (unlikely) same-second collision. No overwrite possible.

## 3. Deliverables (new `datalogger/` folder + additive changes in `design/`)

| File | Content |
|---|---|
| `datalogger/acq_logger_daemon.py` | Python **2.7-compatible**, stdlib only (CB3 ships Python 2.7). Threads: (a) FT-300 reader — connects 63351, parses `(Fx, Fy, Fz, Mx, My, Mz)` text stream, keeps latest triple, auto-reconnect; (b) log server on 50100 — accepts URScript connection, parses `[t,x,y,z]` (or 7-field fallback) lines, appends to in-RAM list, on `STOP` writes CSV (metadata block + header + rows) to detected USB mount, replies status. USB mount auto-detected from `/proc/mounts` (vfat/exfat under `/media` or `/programs`); falls back to `/tmp` with a warning if no USB. |
| `datalogger/urmagic_acqlogger.sh` | Kills previous instance, launches daemon with `nohup`, logs to USB. Short, auditable. |
| `design/params.py` (additions only) | New constants per ARCHITECTURE rule 1: `ACQ_LOG_PORT = 50100`, `ACQ_SAMPLE_TARGET_MS = 20`, `ACQ_MAX_SAMPLES = 11700`, `ACQ_SCRIPT_PATH` / `ACQ_URP_PATH` (`etalement_acq.script` / `.urp`). No existing constant touched. |
| `design/export.py` (additions only) | New `_build_acq_lines(base_lines)` wraps the untouched output of `_build_urscript_lines()`: inserts (a) header comment block for the acquisition process, (b) BeforeStart socket-open + abort-popup if daemon absent, (c) `thread data_logger():` definition, (d) `run data_logger()` right after `set_tcp(...)`, (e) shutdown + STOP handshake after the final retreat, before program end. Insertion via stable anchor lines already emitted by the builder (e.g. the `set_tcp` line and the retreat comment block); a missing anchor is a hard export error, not a silent skip. New `generate_urscript_acq()` / `generate_urp_acq()` reuse `_validate_script_memory()` on the acq files. `generate_urscript()` / `generate_urp()` are **not modified**. |
| `design/app.py` (additions only) | `--export` / `--export-urp` now also write the `_acq` twin after the original (original files written first, byte-identical to today). |
| `datalogger/README.md` | Deployment procedure (USB insertion, daemon check, load `etalement_acq.script`, run, retrieve CSV), version-dependency notes, FT-300 vs `get_tcp_force()` switch, and the rule: simulation always uses `etalement.script`. |
| `tests/test_acq_logger_daemon.py` | stdlib `unittest`, offline: fake FT-300 server + fake URScript client on loopback; asserts CSV metadata block, header `Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ`, row count == sent count, no trailing preallocated rows, filename format, collision suffix, STOP handshake, buffer cap at 11700 with clean auto-stop. Must run on Windows dev machine (pure sockets, temp dir as fake USB). Daemon code stays Python-2.7-valid but the test may run it under the project venv's Python 3 — write it 2/3-compatible (no f-strings, `print()` function, etc.). |
| `tests/test_acq_export.py` | Three guarantees: (1) **regression** — lines produced for `etalement.script` are identical with and without the acq feature present (original untouched); (2) **equivalence** — `parse_poses(etalement_acq.script)` returns exactly the same 4-tuples as `parse_poses(etalement.script)` (lenient parser ignores the thread block; motion unchanged); (3) **content** — acq script contains the thread def, `keep_logging`, bounds guard `log_index < ACQ_MAX_SAMPLES`, socket open before motion, STOP after retreat, and no `movel`/`stopl`/slicing inside the thread block. Memory budget asserted on the acq file too. |

## 4. Acquisition block design (emitted into `etalement_acq.script` by `_build_acq_lines`)

Structure (CB3-safe, fully commented per prompt's code-quality section). The motion is
the real, unchanged 6-cycle spread program — no placeholder needed:

1. **Config block**: `LOG_PORT=50100`, `SAMPLE_MS=20`, `MAX_SAMPLES=11700` (daemon enforces
   too), `USE_INTERNAL_FORCE=False` (fallback switch).
2. **Globals**: `keep_logging = True`, `log_index = 0`, `t_ticks = 0`, one reusable
   `sample_buf` list. All created before motion. No allocation in the loop.
3. **`thread data_logger():`**
   - open socket to `127.0.0.1:LOG_PORT` (opened before motion starts, in BeforeStart
     region, so a missing daemon aborts the run with a popup before the robot moves).
   - loop while `keep_logging` and `log_index < MAX_SAMPLES`:
     - alternate `sync()` x2 / x3 (16/24 ms -> mean 20.000 ms); `t_ticks` += ticks.
     - `p = get_actual_tcp_pose()`; fill `sample_buf` indices: `t_ticks*0.008`, `p[0]`,
       `p[1]`, `p[2]` (+ `get_tcp_force()[0..2]` only if fallback switch on).
     - `socket_send_line(sample_buf, ...)` — list auto-serialized, no strings built.
     - `log_index = log_index + 1`.
   - loop exit on either condition == automatic stop at buffer cap. No motion commands,
     no slicing, no stopl/movel in thread (project rule, section 4 of ARCHITECTURE.md).
4. **Main program**: original lines from `_build_urscript_lines()` verbatim; acq inserts
   `lt = run data_logger()` right after `set_tcp(...)` and, after the final retreat,
   shutdown: `keep_logging = False`, `sleep(0.1)`, `kill lt`, confirm `log_index` frozen.
5. **Export handshake**: send `STOP <log_index>` line; `socket_read_string` the daemon
   reply with timeout; `popup` shows `OK ACQ_log_....csv (N samples)` or the daemon's
   error text (file open failed, no USB, ...). Data stays in daemon RAM on failure, and
   a `RETRY` line can be sent from a re-run popup path.
6. **Version-dependency notes block** (prompt item 15): confirmed against PolyScope
   3.11.0.82155 (CB3, Aug 2019) — 8 ms tick vs 2 ms on e-Series; `socket_send_line` list
   serialization available; no file/date APIs on this build; FT-300 port dependence on
   the Robotiq URCap version installed.

## 5. CSV format (written by daemon)

```
# Robot Model: UR5 CB3
# PolyScope Version: 3.11.0.82155 (20 August 2019)
# File Creation Date: YYYY-MM-DD
# File Creation Time: HH:MM:SS
# Target Acquisition Frequency: 50 Hz
# Maximum Buffer Size: 11700
# Actual Number of Collected Samples: N
Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ
0.000,-0.123456,0.234567,-6.012345,0.412345,-0.298765,0.101234
...
```

- `Time` = robot tick time (s, 3 decimals — 8 ms resolution). Optional extra column
  `HostTime` (daemon arrival clock) can be added; decide at execution (keeps header
  exactly as specified by default).
- Floats: 6 decimals (m and N — engineering precision).
- Only `N = log_index` rows written; the RAM buffer is a Python list appended per received
  line, so "unused preallocated entries" cannot exist by construction.
- This exact 7-column header (`Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ`) is also
  used by the independent fallback tool in `plan_rtde_fallback_monitor.md`, with a
  different filename prefix (`ACQ_rtde_` vs `ACQ_log_`) so the two outputs are never
  mistaken for each other even if both land in the same folder.

## 6. Implementation order

1. `datalogger/acq_logger_daemon.py` + `tests/test_acq_logger_daemon.py` (TDD: fake
   FT-300 + fake robot client; run `python -m unittest tests.test_acq_logger_daemon -v`).
2. `datalogger/urmagic_acqlogger.sh` (trivial, reviewed for root-execution safety).
3. `design/params.py` additions (ACQ_* constants, acq output paths).
4. `design/export.py`: `_build_acq_lines()` + `generate_urscript_acq()` /
   `generate_urp_acq()`; `design/app.py` wiring so `--export` / `--export-urp` also emit
   the `_acq` twins. Zero edits inside `_build_urscript_lines()`, `generate_urscript()`,
   `generate_urp()`.
5. `tests/test_acq_export.py` (regression, pose-equivalence, content — see §3).
6. `datalogger/README.md` deployment guide.
7. Run full existing suite (`python -m unittest discover -s tests -p "test_*.py"`) —
   expect green; then `python ur5_etalementv6.py --export --no-show` and confirm the
   regenerated `etalement.script` is byte-identical to the current one (`git diff` /
   `fc`), with `etalement_acq.script` appearing alongside.
8. `python -m ur5_sim --check` against `etalement.script` — unchanged behavior (sim
   never reads the acq file).
9. `pip-audit -r requirements.txt` (no new deps expected — daemon is stdlib).

Simulation stays exactly as today: `ur5_sim` consumes `etalement.script` only. The acq
twin is robot-only; `test_acq_export.py` guarantee (2) proves its motion is identical.

## 7. Verification

Offline (dev machine):
- `python -m unittest tests.test_acq_logger_daemon -v` — CSV content, header,
  metadata, filename format, collision suffix, 11700 cap, STOP/OK handshake, FT-300 merge.
- Manual smoke: run daemon locally, `python` snippet plays 9000 fake samples at 50 Hz,
  inspect produced CSV timing columns.

On robot (procedure in README, user executes):
1. Copy the 3 files to USB root; insert into pendant; verify daemon start (log file on USB).
2. Load `etalement_acq.script`; run one spread trial (smoke); check CSV appears,
   Time column monotonic with 0.016/0.024 s steps averaging 0.020.
3. Full-protocol trial: row count == duration x 50, force column plausible vs 6 N target
   during contact strokes, transit phases near zero.
4. Two back-to-back trials: two distinct filenames.
5. Pull USB out only between trials.

## 8. Risks and mitigations

- **urmagic runs as root**: script is 5 lines, stdlib daemon, no network beyond loopback;
  user reviews before first insertion.
- **Controller clock wrong** (isolated network, no NTP): filenames still unique
  (monotonic RTC); absolute date may be off — documented; user can set clock in PolyScope.
- **USB mount path varies across CB3 images**: daemon auto-detects from `/proc/mounts`;
  first smoke test confirms; `/tmp` fallback prevents data loss.
- **FT-300 stream absent/format drift**: daemon degrades to zeros + warning in metadata;
  URScript fallback switch to `get_tcp_force()` available.
- **Force/pose sync**: daemon merges the latest 100 Hz FT-300 triple into each 50 Hz pose
  sample → worst-case 10 ms skew; acceptable for the 6 N spread analysis; documented.
- **PolyScope version**: confirmed 3.11.0.82155 (Aug 2019), CB3 — resolved, no
  outstanding version risk (§0.1).

## 9. Requirement traceability (prompt -> plan)

- 50 Hz, timestamp-first, explicit sub-sampling, no sync()-only assumption -> §4.3, issue 4.
- 11700 cap, auto-stop, no overflow -> URScript loop guard + daemon cap (§4.3, §3 tests).
- No allocation in loop, indexed assignment only -> reusable `sample_buf` (§4.2); the
  "indexed arrays" become daemon-side by necessity (issue 3).
- Thread start/stop/clean termination, no writes after stop -> §4.4 (flag -> sleep -> kill).
- Export only after motion + thread death -> STOP handshake ordering (§4.5).
- Unique datetime filename, no overwrite -> daemon clock + collision suffix (§2).
- Metadata block + exact header + only-collected-rows -> §5.
- File-open validation, meaningful diagnostics, preserve data on failure, close+verify ->
  daemon try/except + fsync + status reply + RAM retention + RETRY (§3, §4.5).
- Version-dependent behavior documented -> §4.6 notes block + README.
- Both main and fallback paths required, not either/or -> this plan (main) +
  `plan_rtde_fallback_monitor.md` (fallback), two independent deliverables.

## 10. Original prompt

Kept verbatim in the conversation; the deliverable script must still satisfy its
code-quality section (full comments, purpose/timing/memory notes per section). The
original prompt covered both the main path and the (now separately planned) fallback
monitor in one request; this document addresses the main path only.
