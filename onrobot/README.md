# onrobot

The on-controller acquisition path: record TCP force and TCP position at 50 Hz during an
ISO/COLIPA spread trial, one uniquely named CSV per trial, written to the robot's USB key.

Everything here runs without a second machine. The independent fallback path, a C recorder
reading the robot's RTDE stream from a lab computer, is in [`../datalogger/`](../datalogger/README.md);
the two share the seven CSV columns and nothing else - no code, no process, no file - so a
fault in one cannot corrupt the other's output. Design and rationale:
[`plan_acq_datalogger.md`](../docs/superpower/plans/plan_acq_datalogger.md).

| File | Where it runs | Role |
|---|---|---|
| `acq_logger_daemon.py` | Robot controller, from the USB key | Loopback TCP server on port 50100 receiving samples from `etalement_acq.script`, plus an FT-300 reader on port 63351. Buffers in RAM for the whole trial, writes the CSV on `STOP`. |
| `urmagic_acqlogger.sh` | Robot controller, as root | UR magic file: the controller runs it when the key is inserted, it launches the daemon and returns. |
| `acq_emulator.py` | Development machine only | Fake FT-300 plus a fake robot replaying the real poses of `etalement.script`. Never copied to the robot. |

The URScript half is not here: `design/export.py` emits `etalement_acq.script`, the original
6-cycle program plus a `data_logger` thread. The original `etalement.script` is untouched and
remains what `ur5_sim` validates.

## Why the buffer is in the daemon and not in URScript

URScript on a CB3 has no file API, no date function, no `list_append`, no list slicing, and a
practical list size in the hundreds. The 11700-sample buffer and the file write are therefore
impossible in the robot program as originally specified. They live in a Python process on the
same controller; the robot program only opens a socket and sends one list per sample. During
motion there is zero USB I/O.

## Deployment

1. **Copy three files to the root of the USB key**: `acq_logger_daemon.py`,
   `urmagic_acqlogger.sh`, and the exported `etalement_acq.script`. Nothing else. The daemon
   imports only the standard library, so nothing is installed on the controller.
2. **Read `urmagic_acqlogger.sh` before the first insertion.** The controller runs it as
   root. It is short and every line is commented for exactly that reason.
3. **Insert the key** into the pendant. The daemon starts and writes its log beside itself on
   the key. Check that log before trusting a trial: the first FT-300 sample is printed raw,
   which is how a wire-format mismatch is caught before it silently lands in a CSV column.
4. **Load `etalement_acq.script`** in PolyScope and run it. If the daemon is not answering,
   the program aborts with a popup **before the robot moves**.
5. **At the end of the program**, a popup reports `OK ACQ_log_<date>_<time>.csv (N samples)`
   or the daemon's error text. On error the samples stay in the daemon's RAM.
6. **Remove the key only between trials**, never during one.

Each run produces a new timestamped file; a same-second collision appends `_1`, `_2`. No
overwrite is possible.

## CSV

```
# Robot Model: UR5 CB3
# PolyScope Version: 3.11.0.82155 (20 August 2019)
# File Creation Date: YYYY-MM-DD
# File Creation Time: HH:MM:SS
# Target Acquisition Frequency: 50 Hz
# Maximum Buffer Size: 11700
# Actual Number of Collected Samples: N
Time,ForceX,ForceY,ForceZ,PoseX,PoseY,PoseZ
0.016,-0.082475,0.050368,-6.040525,-0.017788,0.586142,0.050004
```

`Time` is robot tick time in seconds, from the controller's own 8 ms thread counter. It steps
0.016 and 0.024 alternately, because 20 ms is 2.5 ticks and cannot be produced exactly: the
thread alternates 2 and 3 ticks for a mean of exactly 20.000 ms. Every sample carries its true
tick time, so the analysis is exact even though the spacing alternates. Forces are newtons,
poses metres, both to 6 decimals. Only collected rows are written.

The same seven columns are produced by the RTDE fallback monitor under the `ACQ_rtde_` prefix,
so two recordings of one trial can be compared directly.

## Protocol on port 50100

| Line received | Daemon action | Reply |
|---|---|---|
| `[t,x,y,z]` or `[t,x,y,z,fx,fy,fz]` | append, merging the latest FT-300 triple when the sample carries no force | none |
| unparsable | counted in `n_bad`, ignored | none |
| `[-1.0,n,0.0,0.0]` | counting sentinel: records what the robot says it sent | none |
| `STOP` | write the CSV, fsync, close | `OK <file> <rows>` |
| `STOP` after a write failure | keep the buffer in RAM | `ERR <reason>` |
| `RETRY` | rewrite from the retained buffer | `OK ...` or `ERR ...` |

The sentinel exists because a CB3 cannot build the string `STOP <n>`: PolyScope 3.x has
neither `to_str` nor `str_cat`. The count arrives as a list whose first field is negative,
which no real sample can be, immediately before the literal `STOP`. A disagreement between
that count and the rows received is reported, never corrected: it is the only evidence
available that lines were lost.

## Offline exercise, no robot

Three shells:

```
python onrobot/acq_logger_daemon.py
python onrobot/acq_emulator.py --ft
python onrobot/acq_emulator.py --robot etalement.script --samples 200
```

The third replays the real poses of the exported program at 50 Hz with the same alternating
tick times, ends with the sentinel and `STOP`, and exits non-zero unless the daemon answers
`OK`. `--samples` exercises the buffer cap without waiting four minutes.

## Tests

```
python -m unittest tests.test_acq_logger_daemon -v   # 30 tests, daemon
python -m unittest tests.test_acq_export -v          # 13 tests, emitted script
```

Both are collected by `python -m unittest discover -s tests -p "test_*.py"`, unlike the C
harness of `../datalogger/`. They run offline on Windows: fake sockets, a fake clock, a temp
directory standing in for the USB key. `test_acq_export.py` is the one that matters most for
safety - it proves the acq twin's motion is identical to the original's, pose for pose.

## Known limitations

- **`RETRY` recovers only within the same connection.** If the socket drops before `STOP`,
  the buffered trial is lost with it.
- **The FT-300 wire format is not confirmed** from public documentation for URCap 1.11.0.29.
  The parser follows the documented Robotiq format and the daemon logs one raw sample at
  connect time so a mismatch is visible rather than silently miscolumned.
- **Force and pose are merged, not simultaneous.** The FT-300 streams at 100 Hz and its
  latest triple is attached to each 50 Hz pose, so the worst-case skew is 10 ms.
- **No USB mount detected** falls back to `/tmp` with a warning in the metadata rather than
  losing the trial. The mount point varies across CB3 images, which is why it is detected
  from `/proc/mounts` instead of hardcoded.
- **The controller clock may be wrong** on an isolated network with no NTP. Filenames stay
  unique regardless; only the absolute date is affected.
