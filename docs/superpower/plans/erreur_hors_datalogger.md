# Plan - defects found outside the data-logger scope

## Purpose

Register of faults found in the existing code while preparing
[`plan_acq_datalogger.md`](plan_acq_datalogger.md), which that plan must **not** absorb.
Mixing them in would make the acquisition work unreviewable: a diff that both adds a
feature and repairs unrelated code cannot be judged on either count. Each entry carries a
severity, the exact location, the reason it is out of scope, a proposed correction, and the
tests that would pin it. Nothing here is fixed yet; this file is the backlog, not a report
of work done.

Convention of this repo: plans live in `docs/superpower/plans/` (singular "superpower",
per [`CLAUDE.md`](../../../CLAUDE.md)).

## Writing protocol - two sessions share this file

More than one Claude Code session appends here, and neither sees the other's buffer. A
session that writes from a copy read at its own start will silently erase whatever landed
in between. The rules below exist to make that impossible rather than unlikely.

1. **Re-read the file immediately before writing**, not at the start of the session. This
   is the rule that actually prevents loss; the rest is bookkeeping.
2. **Append, never rewrite.** Add new entries directly above `## Execution note`. Do not
   renumber, reword, merge or delete an entry you did not write, even to fix it - add a new
   entry that references it instead.
3. **The `## Purpose`, coverage and protocol sections above are shared.** Leave them alone
   unless the change is yours to make; a session that widens the audit says so inside its
   own entry rather than editing the shared table.
4. **Identifiers are `F<n>`, claimed by appending.** If two sessions claim the same number
   concurrently, the one that notices renames its own entry to the next free number and
   keeps the other's, because inbound references to an existing id must not break.
5. **One finding per entry**, in the established shape: Where, Consequence, Why out of
   scope, Proposed correction, Potential tests. State the audit basis when the file
   concerned is listed as unaudited above.
6. **Nothing here is fixed by the session that logs it.** This file is the backlog; the fix
   is a separate, separately reviewed change.

## Audit coverage (state it, so the gaps are known)

| Read in full or in the relevant region | Not audited |
|---|---|
| `design/export.py`, `design/settings.py`, `design/settings_spec.py` (bounds), `ur5_sim/config.py` (settings binding), `ur5_sim/visualization/viewer.py` (clock and pause paths), repo file inventory (`git ls-files`) | `design/trajectory.py`, `design/geometry.py`, `design/app.py`, `design/ui_*.py`, `ur5_sim/kinematics/*`, `ur5_sim/parsing/urscript.py`, `ur5_sim/visualization/surface.py`, `datalogger/rtde_fallback_monitor.c` (already covered by 147 checks) |

Severity scale used below: **High** = can reach the robot or destroy operator work;
**Medium** = wrong result or non-deterministic gate; **Low** = hygiene or a missing pin on
an intentional behavior.

---

## F1. Settings loaded from JSON are never bounds-checked (High)

**Where.** `design/settings.py:167-182` (`Settings.from_file`) applies every known key with
a bare `setattr(s, name, value)`. `validate()` exists at `design/settings.py:212` and is
called from exactly one place, `design/ui_widgets.py:342`, the settings window's apply
button. Neither `get_settings()`, `reload_settings()`, `generate_urscript()` nor
`generate_urp()` calls it.

**Consequence.** `etalement_settings.json` is gitignored, hand-editable and documented as
such (one workstation, one trial), with a versioned example file inviting the operator to
copy and edit it. A value edited there bypasses every bound in `SPECS`. Concretely,
`force_z_target` is declared `lo=2.0, hi=20.0` N in `design/settings_spec.py:51`; a file
carrying `"force_z_target": 200.0` produces an exported `force_mode(...)` block asking the
UR5 for 200 N against the plate, with no warning at any point in the chain. A wrong **type**
is as bad in a different way: a string reaches `to_overrides()`, whose list branch
(`design/settings.py:133-138`) iterates the value and raises deep inside the export, or
formats straight into the emitted script.

**Why out of scope.** The acquisition plan adds `ACQ_*` constants that are deliberately kept
out of `SPECS`, so it neither creates nor worsens this path. Fixing it here would change
the meaning of every existing export.

**Proposed correction.** Validate on the load path, not only in the UI: `from_file` runs
`validate()` and refuses the file (falling back to defaults, with the message naming each
faulty field) rather than returning out-of-bounds settings. Add a hard gate in
`generate_urscript()` / `generate_urp()` so an invalid `Settings` returns `False` and writes
nothing, since an export is what reaches the robot. Type coercion is checked before the
bound: `int` / `float` / list length per `FieldSpec`.

**Potential tests** (`tests/test_settings_validation.py`, stdlib `unittest`):

1. `from_file` on a JSON with `force_z_target = 200.0` returns defaults and prints one
   message naming the field and its bounds; it never returns 200.0.
2. `from_file` on `force_z_target = "six"` gives the same clean refusal, no `TypeError`,
   no traceback.
3. `from_file` on `p_ref` of the wrong length, and on `p_ref` set to a scalar, is refused
   (this is the branch that currently raises inside `to_overrides`).
4. `from_file` on a `null` value for any field is refused.
5. `generate_urscript(cycles, settings=<invalid>)` returns `False`, and the target file's
   mtime and bytes are unchanged (nothing is written before the gate).
6. Boundary pair per numeric spec: exactly `lo` and exactly `hi` are accepted, `lo - eps`
   and `hi + eps` are refused.
7. Non-regression: a valid `etalement_settings.json` still exports byte-identical output to
   the current reference, so the gate did not move any value.
8. Every `FieldSpec` in `SPECS` is reachable by the validation path (a spec added later
   without bounds must fail this test rather than pass silently).

---

## F2. The PolyScope memory gate measures the wrong bytes on Windows (Medium)

**Where.** `design/export.py:570` writes with `Path.write_text(content, encoding='utf-8')`,
which applies the platform newline translation, and `design/export.py:60` gates on
`filename.stat().st_size`.

**Consequence.** The current `etalement.script` on disk holds 817 CRLF pairs and 0 bare LF,
so the gate counts 817 bytes that do not exist in the string it was given, and would count
zero of them on a Linux or macOS workstation. The budget check
(`URSCRIPT_MAX_BYTES = 200_000`, `design/params.py:110`) is therefore **platform-dependent**:
the same trajectory can pass on one machine and fail on another, and today's file sits at
113 131 bytes against 112 314 bytes of actual content. At 0.4 % of the budget this changes
no verdict now, but it is a gate whose result depends on the operating system that ran the
export, which is not something to leave in a protocol tool. The generated file also carries
CRLF onto a Linux controller.

**Why out of scope.** The acquisition plan reuses `_validate_script_memory()` on the acq
twin, so it inherits the behavior, but changing how every export is written is a separate
change with its own byte-identity consequences (the reference `etalement.script` would be
rewritten).

**Proposed correction.** Write with an explicit `newline='\n'` and gate on
`len(content.encode('utf-8'))`, the size the controller will see, keeping `st_size` only as
a cross-check. Note that applying this rewrites `etalement.script` (817 bytes smaller), so
it must be a deliberate, separately reviewed commit, and the export-state digest file has to
be regenerated with it.

**Potential tests** (`tests/test_export_newlines.py`):

1. The exported file contains no `\r` byte.
2. The size used by the memory gate equals `len(content.encode('utf-8'))`, asserted by
   monkeypatching the print or by returning the measured value.
3. Content sized one byte under `urscript_max_bytes` passes, one byte over fails, and the
   verdict is identical whether the test writes LF or CRLF to disk.
4. Round trip of the overwrite guard still holds after the change: export, re-read, digest
   equal, no spurious "modified by hand" warning (see F3).
5. Golden-file test updated in the same commit, not before or after
   (`tests/fixtures/golden_headless.script`).

---

## F3. The hand-edit guard fails open (Medium)

**Where.** `design/export.py:547-550`: if reading the existing file raises `OSError`,
`check_overwrite` returns `None`, which `_write_export` reads as "safe to overwrite".
`design/export.py:544-546` returns `None` as well when the file is absent from the state
file, and `_load_export_state` (`design/export.py:502-508`) returns `{}` on any read or JSON
error.

**Consequence.** This guard exists for one reason, stated in its own comment
(`design/export.py:492-494`): `etalement.urp` is adjusted by hand on the pendant between
robot trials and used to be overwritten silently. All three fallbacks defeat it in the case
it was written for. A `.urp` held open by PolyScope or by an editor (`OSError` on read) is
overwritten with no message. And `.etalement_export_state.json` sits untracked at the repo
root, so deleting it, or corrupting it, disarms the guard for every file at once, silently.

**Why out of scope.** The acquisition plan only requires that `etalement_acq.urp` go through
the same guard; it does not depend on how the guard behaves in its failure modes.

**Proposed correction.** Fail closed on `OSError`: report that the file could not be read
and refuse without `force=True`. When the state file is missing or corrupt while the output
file exists, warn once naming the situation instead of returning silently, and keep allowing
the export (refusing the first export on a fresh workstation is the nuisance the docstring
correctly rejects). Consider tracking the state file, or writing the digest as a comment
line inside the generated file itself, so the guard cannot be disarmed by deleting one
untracked file.

**Potential tests** (extend `tests/test_export_settings.py` or a new
`tests/test_overwrite_guard.py`):

1. Existing output file, unreadable (permission denied or open exclusively): export refuses
   and the file is unchanged.
2. Same case with `force=True`: export proceeds, exactly one warning.
3. State file deleted, output file present and hand-modified: assert the chosen behavior
   explicitly (today it is a silent overwrite) so a future change is a test change.
4. State file present but corrupt JSON: same assertion, and no traceback.
5. File modified by hand: refusal message names the file and both digests.
6. File untouched since export: no warning, export proceeds.
7. `etalement.urp` specifically: never written by any code path that was not asked for it
   (guards the operator memory that this file is hand-tuned).

---

## F4. `ur5_sim/config.py` freezes settings at import, with no test pinning it (Low)

**Where.** `ur5_sim/config.py:22`, `_S = get_settings()`, then module constants derived from
`_S` at import time.

**Assessment.** This is **intentional and documented** in the surrounding comment: one
process, one read, with `_READ_AT` published by `settings_summary()` so the `--check` report
states when the values were read. It is the correct trade for a separate simulator process.
The defect is not the design but the absence of a test pinning it: nothing fails if someone
later expects `reload_settings()` to affect `ur5_sim.config`, which it cannot, since the
constants were already computed.

**Potential tests** (extend `tests/test_sim_reads_settings.py`):

1. Import `ur5_sim.config`, call `reload_settings()` against a different file, assert the
   module constants are unchanged, with the reason in the assertion message.
2. `settings_summary()` reports a read timestamp and the source file actually used.
3. A settings file written **before** import is reflected; one written after is not.

---

## F5. Dead artifacts tracked in git (Low)

**Where.** `git ls-files` shows `etalement.script.bak`, `ur5_etalementv6.py.original`
(69 657 bytes, the pre-refactor monolith), `tcp_live.json` and an empty `tcp_live/` at the
repo root, plus `validate_etalement.py`, a deprecated shim that duplicates `run_validate.py`
(both call `ur5_sim.cli.main`). `ARCHITECTURE.md` and `CLAUDE.md` state the
`tcp_live/tcp_live.json` file IPC is retired in favor of UDP, yet
`ur5_sim/kinematics/ik_multisolve.py:27` still points a comment at "../../tcp_live
trajectory cache".

**Assessment.** Git history already holds every one of these; a `.bak` and a `.original`
committed beside the live file are a standing invitation to edit the wrong one.
`etalement_vINIT.script` is **not** in this list: it is the reference for the legacy
`movel(T(p[...]))` emit format that `tests/test_urscript_parse.py` still covers, and it
stays.

**Proposed correction.** Remove the four dead files in one commit that touches nothing else,
fix the stale comment, and decide explicitly whether `validate_etalement.py` is still needed
as a command-line alias (if yes, say so in its docstring and in `README.md`; if no, delete
it with the others).

**Potential tests** (`tests/test_repo_hygiene.py`):

1. No tracked file matches `*.bak`, `*.original` or `*.orig`.
2. `tcp_live.json` and `tcp_live/` are absent from the tracked set.
3. Every path named in a `see ...` comment inside `ur5_sim/` resolves on disk (catches the
   `ik_multisolve.py:27` reference and any future one).
4. If `validate_etalement.py` is kept: it exits with the same status as
   `python -m ur5_sim --check` on the reference script.

---

## F6. Source files above the workspace file-size ceiling (Low)

**Where.** `ur5_sim/visualization/viewer.py` is 916 lines;
`datalogger/rtde_fallback_monitor.c` is 909 and its test 1118.

**Assessment.** The workspace `code-style.md` sets a ceiling of 4096 tokens per source file,
roughly a quarter of the local model's measured context window, and
[`spec_rtde_emulator.md`](../specs/spec_rtde_emulator.md) explicitly claims its two new
modules stay under it, so the rule is treated as binding in this repo. `viewer.py` is well
past it, which matters concretely: `plan_rtde_emulator.md` adds a PAUSE control to that same
file, and a local model handed it whole silently loses the tail of the prompt. The C files
are in the same position, with no ceiling exemption in the C-language exception recorded in
`CLAUDE.md` (that exception is about language and test runner, not size).

**Proposed correction.** Split `viewer.py` along the seam it already has: clock and run-state
management, matplotlib panel assembly, and Swift scene wiring. Do this **before**
`plan_rtde_emulator.md` Task 6 touches it, not after, or the split and the feature land in
one unreviewable diff. For the C files, note the measurement and decide, rather than assume
the rule does not apply.

**Potential tests** (`tests/test_file_size_ceiling.py`):

1. Every tracked `.py` under `design/` and `ur5_sim/` is under the ceiling, with the offender
   named and its size reported in the failure message.
2. The threshold is read from the live measurement
   (`.claude/local-model-config.json`, `retained_num_ctx // 4`) with an explicit failure when
   that file is absent, never a silently hardcoded 4096.
3. The test is skipped, not failed, for the C sources until the decision above is taken, so
   the suite states the open question instead of hiding it.

---

## F7. Dead `paused_sim_t`, no pause in the viewer (already tracked, do not fix twice)

**Where.** `ur5_sim/visualization/viewer.py:430` initializes `paused_sim_t = [0.0]`, read at
lines 543 and 699, and every other occurrence (743, 760, 796) writes `0.0` back. No code
path ever gives it a non-zero value, so the elapsed-time term it contributes is always zero
and a pause cannot exist: `set_stop()` is a hard stop that restarts from frame 0.

**Status.** Already diagnosed in [`spec_rtde_emulator.md`](../specs/spec_rtde_emulator.md)
§2, fact 3, and scheduled as Task 6 of `plan_rtde_emulator.md` on branch
`feat/rtde-emulator`. Listed here only so this audit is complete. **Do not open a second
correction for it**; if that branch is abandoned, move this entry up into the active list.

**Potential tests** (owned by that plan, restated so they are not lost): pause freezes
simulation time while controller time keeps advancing; resume continues from the frozen
frame rather than frame 0; a pause does not split the monitor's CSV file.

---

## F8. The CSV provenance line hardcodes one machine's address, and a test pins it (Medium)

**Audit basis.** Found while implementing `datalogger/rtde_fallback_monitor.c` on branch
`feat/rtde-emulator`. The coverage table above lists that file as unaudited on the grounds
that 147 checks already cover it; those checks are the very thing that pins this defect in
place.

**Where.** `datalogger/rtde_fallback_monitor.c:245`, inside `format_csv_header`, emits the
literal `# Data Source: RTDE fallback monitor (192.168.4.14)`. The address is a string
constant; nothing reads it from the machine. `datalogger/tests/test_rtde_fallback_monitor.c`
(`test_csv_header_carries_the_schema_and_provenance`) asserts that exact line.

**Consequence.** The line names the lab computer whatever machine actually wrote the file.
Run the executable anywhere else - a spare laptop during commissioning, a developer machine,
the emulator rig of [`plan_rtde_emulator.md`](plan_rtde_emulator.md) - and every CSV still
claims it came from `192.168.4.14`. For a dataset whose provenance is the point, a
provenance field that is a constant is worse than none: it reads as authoritative and cannot
be falsified from the file alone. Worse, the neighbouring `# Robot RTDE Endpoint:` line *is*
derived from `argv`, so the two can disagree, and the wrong one is the one labelled "Data
Source". The test asserting the literal makes the incorrect value the specified behavior, so
a future correction fails the suite and reads as a regression.

**Why out of scope.** `plan_rtde_emulator.md` Task 9 adds a simulated-source marker driven
by the endpoint address, which separates emulator from robot but says nothing about *which
machine recorded the file*; the two are independent. Changing this line changes every CSV
header and the C test that pins it, which is a deliberate change to a shipped, tested tool.

**Proposed correction.** Derive both halves at runtime: `gethostname()` for the name, and
`getsockname()` on the connected socket for the address actually used to reach the robot,
which is the right one on a multi-homed machine where a hardcoded guess is wrong by
construction. Emit for example
`# Data Source: RTDE fallback monitor on LABPC-03 (192.168.4.14)`. Fall back to the hostname
alone if the address cannot be read, never to a literal. Update the test to assert the
*shape* and the agreement with the socket, not a fixed address.

**Potential tests** (extend `datalogger/tests/test_rtde_fallback_monitor.c`):

1. The header carries the running host's name as reported by `gethostname()`.
2. The address in the header equals the local address of the live socket, obtained through
   the existing loopback fake server, rather than any constant.
3. Against a fake server reached on a second local address, the header follows the socket
   and not the first one.
4. `getsockname()` failure degrades to hostname only: exactly one `Data Source` line, no
   crash, no empty parentheses.
5. Non-regression: `Data Source` and `Robot RTDE Endpoint` are never the same field and
   never contradict each other; the first is local, the second is the peer.

---

## F9. No `.gitattributes`, so line endings are decided per workstation (Low)

**Audit basis.** Observed on every commit made from branch `feat/rtde-emulator`: git prints
`warning: LF will be replaced by CRLF the next time Git touches it` for each new text file.

**Where.** Repo root; the file is absent. Nothing in the repository states an end-of-line
policy, so each contributor's `core.autocrlf` decides.

**Consequence.** Three, in decreasing severity. The exported `.script` is loaded by a Linux
controller, and F2 above already measures 817 CRLF pairs in the current `etalement.script`,
so the newline question is not theoretical here. `datalogger/*.bat` *needs* CRLF to be safe
under `cmd.exe`, while the C sources and the Python modules do not, and nothing today
distinguishes them. And two workstations configured differently produce whole-file diffs on
anything either one touches, which buries real changes in noise. This entry is the
repo-wide half of F2: F2 fixes one writer (`design/export.py`), this fixes what git stores
for everything else, including the C sources and the modules
[`plan_rtde_emulator.md`](plan_rtde_emulator.md) is about to add.

**Why out of scope.** No current plan changes how files are stored; F2 changes a single
export path and would still leave every other file governed by local configuration.

**Proposed correction.** Add `.gitattributes` with `* text=auto eol=lf` as the default,
`*.bat text eol=crlf`, explicit entries for `*.script` and `*.urp`, and `binary` for `*.stl`
and `*.pptx`. Land it in its own commit and run `git add --renormalize .` there, because
normalization rewrites blobs and must not be mixed with a behavior change. Sequence it with
F2, which rewrites `etalement.script` anyway, so the reference file moves once rather than
twice.

**Potential tests** (extend `tests/test_repo_hygiene.py` from F5):

1. `.gitattributes` exists and assigns an explicit `eol` to `*.bat`.
2. `git ls-files --eol` reports `i/lf` for every tracked `.py` and `.c`. Assert on the index
   side, not the working tree, which legitimately differs per platform.
3. `git ls-files --eol` reports `i/crlf` for every tracked `.bat`.
4. No tracked file falls through without an attribute match, so a new extension added later
   is a deliberate decision rather than a silent default.

---

## F10. The committed `etalement.script` cannot be reproduced by any documented command (Medium)

**Where.** `etalement.script` at the repo root, tracked, 817 lines and 113 131 bytes.
`python ur5_etalementv6.py --export --no-show` produces 502 lines and 61 366 characters
instead, and that output matches `tests/fixtures/golden_headless.script` byte for byte
(measured 2026-08-15 on commit `acfe0e3`, with no `etalement_settings.json` present, so the
export ran on pure `design/params.py` defaults).

**Consequence.** The artifact the operator loads on the robot is roughly twice the program
the tool generates today, and nothing in the repository records which inputs produced it.
The plausible explanation is an export from the interactive UI with a richer cycle
configuration than the headless defaults, but that is an inference, not a record: the
traceability header (`_settings_header_lines`, `design/export.py:73`) is emitted only when a
settings field differs from its `params.py` default, so a UI export that changed the
per-cycle sliders leaves no trace at all. Three consequences: nobody can regenerate the file
that ran a given trial, nobody can tell whether the committed script and the current code
still agree on the protocol, and any test written as "regenerate and compare to
`etalement.script`" fails for a reason unrelated to the change under test. This was hit while
writing the acquisition plan's verification step, now pinned to the golden fixture instead.

**Why out of scope.** The acquisition plan adds a second output pair and must leave the first
untouched; deciding what the committed artifact should be is a protocol question for the
operator, not a side effect of adding a logger.

**Proposed correction.** Decide, then record. Either the committed `etalement.script` is the
trial-of-record, and the cycle configuration that produced it is captured beside it so a
command regenerates it, or it is stale and is regenerated from current defaults and committed
as such. Either way the emitted header should carry the cycle configuration, not only the
settings overrides, so the file states what produced it. Interacts with F2 and F9: any
regeneration also rewrites the line endings, so do it in one deliberate commit.

**Potential tests** (`tests/test_export_reproducibility.py`):

1. A headless export equals `tests/fixtures/golden_headless.script` byte for byte (passes
   today; pins the current baseline).
2. After the decision above: regenerating the committed artifact from its recorded
   configuration reproduces it byte for byte, the test naming the configuration source.
3. The emitted header of any export names enough to reproduce it: cycle count, per-cycle type
   and waypoint counts, plus the settings fingerprint.
4. Guard: a test comparing an export against the wrong one of the two references fails with a
   message that says which is which, so the confusion cannot recur silently.

---

## F11. The shipped acq twin was not the twin of the shipped original (High) - FIXED

**Audit basis.** Found 2026-08-16 while preparing the F2/F10 fix, by counting poses in the
two tracked artifacts. Introduced by commit `bccdbb2` of this same branch, which tracked
`etalement_acq.script` "for the student".

**Where.** The repo root pair. `etalement.script` parsed to **661** poses (663 `movel`
lines, 818 lines); `etalement_acq.script` parsed to **346** (348 `movel`, 570 lines). Both
declare six cycles, so nothing about their shape gives the difference away at a glance.

**Consequence.** They were not the same trajectory. `etalement.script` is the operator's
export from the interactive UI, the trial-of-record; the acq twin had been produced by a
headless `--export` from `design/params.py` defaults, which is a coarser program with about
half the waypoints. A student loading `etalement_acq.script` to record an instrumented run
would have executed a different motion from the validated protocol, and the resulting force
and position data would not describe the trial anyone thinks it describes. The failure is
silent by construction: both files are valid, both pass every existing test, both run on the
robot, and the only visible difference is a pose count nobody counts. This is the direct
descendant of F10 - because no artifact records the configuration that produced it, nothing
could flag that these two came from different ones.

**Cause, plainly.** The plan owner ran `--export --no-show` during verification, which
regenerated the acq twin from defaults, then committed it beside an original that had not
been regenerated. Guarantee 2 of the acquisition plan (pose equivalence) was verified on
freshly generated pairs, never on the tracked pair, so the guarantee was true of the
generator and false of the shipped files.

**Correction applied.** The twin was rebuilt from the reference itself rather than from
defaults: `_build_acq_lines()` was applied to the lines of the committed `etalement.script`,
which is exactly what the exporter does internally, so the acq block wraps the trial-of-record
motion. Result: 661 poses on both sides, identical pose by pose, cycle index by cycle index,
`in_contact` flag by flag; 114 794 bytes, 57.4% of the PolyScope budget. `etalement.script`
itself was not touched.

**Tests** (`tests/test_artifact_consistency.py`, written with this entry):

1. The two **tracked** artifacts carry identical motion: same pose count, same poses, same
   cycle indices, same `in_contact` flags. This is guarantee 2 applied to the shipped files
   rather than to a freshly generated pair, which is the gap that let F11 through.
2. The tracked acq twin stays inside the PolyScope memory budget.
3. The acq twin contains the logger thread, so a twin accidentally replaced by a plain copy
   of the original is caught too.

**Residual risk, for the operator.** Regenerating one file without the other reintroduces
this immediately. `--export` now writes both, so the safe habit is to always export the pair
from the same UI state, never one alone. Test 1 above fails loudly if that slips.

---

## Execution note

Same split as [`plan_acq_datalogger.md`](plan_acq_datalogger.md) §0-bis: the corrections
themselves are small and mechanical enough for a Sonnet subagent, **except F1**, where the
question of what a refused settings file should do to a running UI is a design decision, and
F6, where the split of `viewer.py` decides a module boundary. All test files go to Sonnet;
running them and accepting the result stays with Opus. F1 and F3 change behavior the
operator depends on, so neither ships without the golden-file and byte-identity checks of
the acquisition plan's V1 passing first.
