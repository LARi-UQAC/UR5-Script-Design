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
  trajectory as `etalement.script` (URScript) and/or `etalement.urp` (PolyScope XML).
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
[issue #1](https://github.com/LARi-UQAC/UR5-Script-Design/issues/1)): `etalement_acq.script`
/ `etalement_acq.urp`, the same spread program with a 50 Hz TCP force/pose data logger
attached (background thread on the robot, CSV written to the USB key once the run
completes). It does not replace or modify `etalement.script` / `etalement.urp`; both
pairs will keep being generated side by side, and the simulator keeps using the plain
`etalement.script`.

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
```

## Tests

Stdlib `unittest`, no pytest:

```bash
python -m unittest discover -s tests -p "test_*.py"
```

## Where to go next

- [ARCHITECTURE.md](ARCHITECTURE.md) for the package layout, the three coordinate
  frames every new pose must travel through, the CB3 URScript generation constraints,
  the force-mode kinematic surrogate, and the rules for extending the project.
- [CLAUDE.md](CLAUDE.md) for the same material kept as an operational summary.
