# Sponsor
Ask for my book (French version): Vibe Design. 30$ contribution via:

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/s/89b1e1cc6c)

[![PayPal](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.me/MartinJDOtis)

# UR5-Script-Design

Tooling around the ISO/COLIPA cosmetic-spread protocol executed on a Universal Robots
UR5 (PolyScope 3.11.0.82155, CB3 controller) with a RobotIQ FT-300 force/torque sensor
and a 2F-85 gripper holding a silicone hemispheric finger.

Full architecture (package layout, coordinate-frame invariants, extension rules) lives in
[ARCHITECTURE.md](ARCHITECTURE.md). This file is the practical entry point: what the
project does, how to install it, and how to run it.

## What is in this repository

Two cooperating Python tools plus the generated on-robot artifacts:

- **`design/`** - interactive design UI (matplotlib) to tune the 6 spreading cycles
  (3 boustrophedon + epicycloid, 3 linear) on a 50x50 mm plate, then export the
  trajectory as `etalement.script` (URScript) and/or `etalement.urp` (PolyScope XML). A
  Paramètres button opens a settings window covering the protocol's tunable variables;
  operator overrides are kept in `etalement_settings.json` (gitignored, specific to one
  workstation and one trial), with a versioned example at
  `etalement_settings.example.json`.
- **`ur5_sim/`** - offline validator and replay. Parses `etalement.script`, runs
  sequential IK against a UR5 model, reports failures, and (with `--visualize`) renders
  the robot in Swift (WebGL) alongside matplotlib panels (XYZ vs time, XY trail, IK
  branch selector, test-surface overlay).

## Export options

Today the exporter produces one pair of artifacts:

| File | Content |
|---|---|
| `etalement.script` | URScript program for the real robot: 6 spread cycles, `force_mode(...)` regulating 6 N along Z during contact strokes, no gripper actuation (passive finger support). |
| `etalement.urp` | Same program packaged as PolyScope XML, loadable directly on the pendant. |

`ur5_sim` always validates and replays `etalement.script`; nothing in the simulator
depends on any other export target.

A second, additive export option is planned (tracked in
[issue #6](https://github.com/LARi-UQAC/UR5-Script-Design/issues/6), full design in
[docs/superpower/plans/plan_acq_datalogger.md](docs/superpower/plans/plan_acq_datalogger.md)):
`etalement_acq.script` / `etalement_acq.urp`, the same spread program with a 50 Hz TCP
force/pose data logger attached (background thread on the robot, CSV written to the USB
key once the run completes). It does not replace or modify `etalement.script` /
`etalement.urp`; both pairs will keep being generated side by side, and the simulator
keeps using the plain `etalement.script`.

A second, fully independent fallback tool is **implemented** in
[datalogger/](datalogger/) (issue
[#7](https://github.com/LARi-UQAC/UR5-Script-Design/issues/7), design in
[docs/superpower/plans/plan_rtde_fallback_monitor.md](docs/superpower/plans/plan_rtde_fallback_monitor.md)):
`rtde_fallback_monitor.exe`, a standalone statically compiled C executable run from
`cmd.exe` on a lab computer wired to the same isolated VLAN. It passively reads the
robot's RTDE stream and writes its own CSV with automatic per-run file boundaries, driven
by the `runtime_state` field carried in that same stream (no Python, no PowerShell, no
install of any kind on that machine). It is read-only toward the robot. Neither the main
path nor this fallback depends on the other; build and deployment procedure in
[datalogger/README.md](datalogger/README.md).

## Dependencies

Python 3.13, isolated in a local `.venv` (see [requirements.txt](requirements.txt)):
`numpy`, `scipy`, `matplotlib`, `roboticstoolbox-python`, `spatialmath-python`,
`spatialgeometry`, `swift-sim>=1.1.0`, `websockets<13`, `pyright`, `pip-audit`.

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

Two pinning constraints to respect (details and file paths in `requirements.txt`):

- `swift-sim==1.1.0` requires `websockets<13`; keep this project's `.venv` isolated from
  any global environment that needs `websockets>=13` (for example an LLM SDK).
- `roboticstoolbox-python==1.1.1` on Python 3.13 with `numpy>=2` needs two small in-place
  patches (`DistanceTransformPlanner.py`, `xacro/xmlutils.py`), and `swift-sim` on
  Windows needs a local patch to its `/retrieve/<path>` handler so UR5 meshes load in
  the browser. Apply once after `pip install`; see the header comment of
  `requirements.txt` for the exact patch locations.

Validate the installed dependencies with `pip-audit -r requirements.txt` before any
release.

## Starting the software

Quickest path on Windows, an interactive menu that activates `.venv` automatically:

```bash
.\validate.bat
```

Menu options: kinematic check (target or identity anchor), 3D visualizer (Swift +
matplotlib), design UI alone, or design UI and viewer started together.

Equivalent manual commands:

```bash
# Offline check (parse + IK, no GUI)
python -m ur5_sim --check

# Full visualizer (Swift 3D + matplotlib panels)
python -m ur5_sim --visualize

# Design UI (also re-exports etalement.script and .urp)
python ur5_etalementv6.py
python ur5_etalementv6.py --export        # write etalement.script
python ur5_etalementv6.py --export-urp    # write etalement.urp
python ur5_etalementv6.py --no-show       # headless
python ur5_etalementv6.py --export --force  # overwrite a hand-edited output file
```

The design window's Paramètres button opens the settings editor described below. The
Sortie field next to Exporter URScript names the output file (default `etalement`), so a
trial variant can be produced without touching the reference `etalement.script`. Settings
persist to `etalement_settings.json` at the repo root (gitignored, specific to one
workstation and one trial); a versioned example lives at
`etalement_settings.example.json`. Both `--export` and `--export-urp` (and the equivalent
Enregistrer/Exporter buttons in the settings window) refuse to overwrite an output file
whose content has drifted from the last recorded export - a `.urp` retouched by hand on
the pendant, for instance - unless `--force` is passed.

## Settings

The Paramètres button opens a window with five tabs, generated from a single metadata
table so a new field never needs its own hand-written row: Force (target Z force and the
`force_mode` deviation limits), Sondage (Z-probe timing, plus the parked 3-point probe
fields), Mouvement (URScript accelerations, phase speeds, blend radius, circular waypoint
density), Surface (plate size, transit and retreat heights, cycle durations and counts),
and Calibration (robot origin, base rotation, `P_REF`, and the four TCP tool-length
offsets). Each row shows the field next to its hard-coded default and valid range, so a
saved override stays legible against the protocol reference in `design/params.py`.

Two fields stay read-only on every tab: `urscript_max_tcp_speed` and `urscript_max_bytes`
are PolyScope controller limits, not preferences, and letting the operator edit them would
misrepresent what the real controller enforces. The Calibration tab is locked behind an
explicit "Deverrouiller la calibration" checkbox with a confirmation dialog, because its
fields move the robot's anchor in the world rather than tune the spreading protocol; a
session always reopens locked, since nothing there is meant to persist by accident.
`tcp_z` is
computed from the four tool-length fields above it (FT-300, coupling, 2F-85 gripper,
silicone finger) and cannot be edited on its own.

The Sondage tab's dropdown still lists a `plane3` probing mode, but `validate()` refuses
that value when Appliquer is clicked, and the four fields `plane3` alone would need
(approach height, tilt limit, retry count, probe points) stay grayed out regardless of the
selected mode: the 3-point probe is parked, incorrect as implemented (fixed in Z, no
tilt), and its rework is separate future work (see [ARCHITECTURE.md](ARCHITECTURE.md),
section 6).

Réinitialiser resets either the current tab or the whole window to the `design/params.py`
defaults. Deleting `etalement_settings.json` has the same effect for the whole file: the
settings layer falls back to those defaults whenever the file is absent or unreadable.

## Tests

Stdlib `unittest`, no pytest:

```bash
python -m unittest discover -s tests -p "test_*.py"
```

One exception, in C: the RTDE fallback monitor in [datalogger/](datalogger/) is a C tool,
because the lab computer it runs on has no Python and nothing can be installed there. Its
tests are C as well, so they call the tool's own functions directly instead of shelling
out, and they are not collected by the discovery above. Run them with MinGW-w64 `gcc` on
`PATH`:

```bash
datalogger\tests\build_and_run_tests.bat
```

## Where to go next

- [ARCHITECTURE.md](ARCHITECTURE.md) for the package layout, the three coordinate
  frames every new pose must travel through, the CB3 URScript generation constraints,
  the force-mode kinematic surrogate, and the rules for extending the project.
- [CLAUDE.md](CLAUDE.md) for the same material kept as an operational summary.
