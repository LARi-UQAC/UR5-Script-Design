# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository. The full architecture (package layout, invariants, extension rules) lives in [ARCHITECTURE.md](ARCHITECTURE.md); this file keeps the operational summary.

## Project purpose

Tooling around the ISO/COLIPA cosmetic-spread protocol executed on a Universal Robots UR5 with a RobotIQ FT-300 force/torque sensor and a 2F-85 gripper holding a silicone hemispheric finger. Two cooperating Python tools:

- **`design/`** — interactive design UI (matplotlib) that lets the operator tune the 6 spreading cycles (3 boustrophedon + epicycloid, 3 linear) on a 50x50 mm plate, then exports the trajectory as `etalement.script` (URScript) and/or `etalement.urp` (PolyScope XML). The on-robot script wraps the contact strokes in `force_mode(...)` / `end_force_mode()` to maintain 6 N along Z during XY motion. `ur5_etalementv6.py` is now a 2-line shim over `design.app.main` kept for backward compatibility; the modules are `params.py` (single source of truth for all protocol constants), `geometry.py`, `trajectory.py`, `export.py`, `live_ipc.py`, `app.py`.
- **`ur5_sim/`** — offline validator + replay. Parses `etalement.script`, runs sequential IK against the UR5 model (`roboticstoolbox.models.UR5`), reports failures, and (with `--visualize`) renders the robot in Swift (WebGL, browser tab) while matplotlib shows XYZ-vs-time, XY trail, the IK-branch selector and the test-surface overlay.

The Swift viewer streams the live TCP to the design UI over UDP loopback (`ur5_sim/ipc_config.py`, port 47811); the old `tcp_live/tcp_live.json` file IPC is retired.

## Plans and specs

Every plan (design-doc produced by plan mode, or written ahead of a feature) goes in
`docs/superpower/plans/plan_<topic>.md`. A matching specification, when one exists, goes
in `docs/superpower/specs/spec_<topic>.md`. Do not leave a plan at the repo root or in
`~/.claude/plans/` once work starts on it; move or copy it into `docs/superpower/plans/`
so it stays with the code it describes and survives outside the local Claude Code plan
history. Cross-link a plan and its spec by relative path when both exist.

## Common commands

```bash
# Interactive menu (Windows) - runs everything from the local .venv
.\validate.bat

# Offline check (parse + IK, no GUI)
python -m ur5_sim --check

# Full visualizer (Swift 3D + matplotlib panels)
python -m ur5_sim --visualize

# Override P_REF with P_ANCHOR_OLD to verify the refactor (identity transform)
python -m ur5_sim --check --identity
python -m ur5_sim --visualize --identity

# Design UI (also re-exports etalement.script and .urp)
python ur5_etalementv6.py
python ur5_etalementv6.py --export        # write etalement.script
python ur5_etalementv6.py --export-urp    # write etalement.urp
python ur5_etalementv6.py --no-show       # CI-friendly, headless

# Tests (project uses stdlib unittest; pytest is NOT installed in .venv)
python -m unittest discover -s tests -p "test_*.py"
python -m unittest tests.test_surface_constraint -v   # single module
python -m unittest tests.test_surface_constraint.SurfaceFrameTests.test_snap_projects_pose_onto_plane

# Dependency audit (CLAUDE rule from parent dir)
pip-audit -r requirements.txt
```

`run_validate.py` is a thin shim that calls `ur5_sim.cli.main`; both `python run_validate.py` and `python -m ur5_sim --check` are equivalent.

## High-level architecture

### Layered structure of `ur5_sim/`

```
parsing/       URScript reader. parse_poses() now returns 4-tuples
               (lineno, pose, cycle_idx, in_contact). Lenient regex
               handles three emit formats: movel(T(p[...])) (vINIT),
               movel(apply_correction(p[...], dx, dy)) (current),
               and plain movel(p[...]). force_mode / end_force_mode
               toggle the in_contact flag inside each def cycle_N():.
kinematics/    SE(3) helpers (transforms.rotate_translation_y is the
               single source of truth for the SIM_TRAJ_ROT_Y_RAD
               remap), sequential IK (ik.run_ik), IK branch
               enumeration (ik_multisolve).
visualization/ viewer.visualize() (Swift + matplotlib), surface.py
               (test plate geometry + kinematic force surrogate),
               interactions.py (matplotlib widget callbacks).
meshes/        FT-300 + 2F-85 + custom Support_doigt.stl pipeline
               for Swift; decimation, color extraction, link loader.
reporting/     text_report.report() prints surface events first
               (SURFACE_DEVIATION / SURFACE_CLAMP) then IK / joint
               limit failures.
cli.py         Argparse entry point, anchors P_ANCHOR_OLD / P_REF,
               applies the surface constraint pre-IK, checks TCP
               speed globals, filters expected recontact deviations.
config.py      Paths, anchors, sim DT/speed, surface constants.
               Shared protocol constants are imported from
               design.params — never re-defined here or in
               submodules.
ipc_config.py  UDP host/port/payload shared by viewer (writer)
               and design/live_ipc.py (reader).
probe.py       3-point probe simulation — PARKED (incorrect
               algorithm, guarded by SIM_PROBE_ENABLE = False;
               export now uses the 1-point probe_surface_z).
```

### Coordinate frames (critical, all three are at play simultaneously)

1. **Plate frame** (mm, +Z out of the plate) — defined in `design/params.py` via `SURFACE_W`, `SURFACE_H`, `Z_CONTACT`, `Z_TRANSIT`. All trajectory generators (`circular_cycle`, `linear_cycle`, `triangular_cycle` in `design/trajectory.py`) work in this frame.
2. **Robot base frame** (m) — `plate_to_robot()` maps plate-mm to robot-m by rotating `ROBOT_BASE_ROTATION_DEG = 225` around the plate origin and adding `ROBOT_X_ORIGIN`, `ROBOT_Y_ORIGIN`, then `Z = ROBOT_Z_SURFACE` for the contact plane. This is the `p_orig` of the legacy URScript form `movel(T(p[...]))`.
3. **Absolute world frame** (m) — `_abs_pose()` pre-bakes `pose_trans(P_REF, pose_trans(pose_inv(P_ANCHOR_OLD), p_orig))`. Current `etalement.script` carries absolute poses directly (no `T(...)` wrapper). `ur5_sim/cli.py` still composes `transform(p, P_ANCHOR_OLD, P_REF)` on every pose, then applies `rotate_translation_y(_, SIM_TRAJ_ROT_Y_RAD)` to remap the playback onto the XY plane shown in the design UI.

When adding any new geometry (surface, fixture, tool tip), it must travel through the **exact same chain** the trajectory poses take, otherwise it will land in a different frame. See `ur5_sim/visualization/surface._plate_corner_world` for the canonical recipe (`plate_to_robot` -> `_abs_pose` -> `transform` -> `rotate_translation_y`).

### Force-mode kinematic surrogate

The simulator has no physics layer. The 6 N Z regulation enforced by the real `force_mode(...)` block is emulated kinematically by `ur5_sim/visualization/surface.apply_surface_constraint`:

- `in_contact = True` (between `force_mode(...)` and `end_force_mode()`): the TCP target is **snapped** onto the surface plane (bidirectional projection along the surface normal). Any pre-snap deviation `|n . (t - O)| > CONTACT_SNAP_TOL_M` is logged as `SURFACE_DEVIATION` (signed mm).
- `in_contact = False` (transit phases): only **clamped from below**; pre-clamp penetration is logged as `SURFACE_CLAMP` (positive mm).

The clamp runs in `cli.py` *before* `run_ik`, so the IK solver receives feasible targets. Toggle with `SURFACE_ENABLE_CLAMP` in `config.py`. The `FORCE_Z_TARGET_N` constant is the HUD label only (no force is actually applied in sim).

### IPC contract with the design UI

UDP unicast on `127.0.0.1:47811` (constants in `ur5_sim/ipc_config.py`, port overridable via `UR5_SIM_IPC_PORT`). The viewer emits one JSON datagram per frame; `design/live_ipc.py` drains the non-blocking socket on each matplotlib timer tick. Drop-tolerant by design: only the latest frame matters. Keys consumed today: `running`, `cycle`, `frame`, `x_anchor_m`/`y_anchor_m`, `trail_anchor_m`. Emitted but not yet consumed: `in_contact`, `force_z_n`, `surface_depth_mm`. The design UI inverts `plate_to_robot` to recover plate-mm coordinates from these world-m values.

### Dependency pinning (do not relax without reading `requirements.txt` comments)

- `swift-sim==1.1.0` requires `websockets<13`. The Anthropic/Google SDKs in the user's global env need `websockets>=13`; keep them isolated in this project's `.venv`.
- `roboticstoolbox-python==1.1.1` on Python 3.13 + numpy>=2 has two breakages that this project patches in-place (see `requirements.txt` for the file paths): `mobile/DistanceTransformPlanner.py` `from numpy import disp`, and `tools/xacro/xmlutils.py` `_write_data` signature.
- `swift-sim` on Windows ships a `/retrieve/<path>` handler that mishandles drive letters; without the local patch the UR5 meshes 404 in the browser.

### Tooling and rules inherited from the parent directory

The `.claude/rules/` folder in this repo currently lists project rules copied from the SimulCuve / CostEstimator project (paths like `web/SimulCuve.Web/...`, `api/routes/...`). These do **not** apply to UR5Script. Treat them as inert until they are cleaned up; the relevant rules for this repo live in:
- `requirements.txt` (dependency invariants),
- the comment headers of each `ur5_sim/*.py` module,
- the `_abs_pose` / `transform` / `rotate_translation_y` chain documented above.

### Tests

Stdlib `unittest`, no `conftest.py`, no pytest plugins. Tests live in `tests/`:
- `test_urscript_parse.py` covers the 4-tuple shape, the legacy 3-tuple shim, the `force_mode` toggle and the `apply_correction` wrapper.
- `test_surface_constraint.py` covers `compute_surface_frame`, the normal orientation, `snap`/`clamp` math, and the dispatch.
- `test_ik_smoke.py` runs the first 10 poses through IK end-to-end.
- `test_transforms.py` sanity-checks the SE(3) helpers.
- `test_motion_segments.py` covers segment densification; `test_limits.py` covers joint and TCP-speed limits.
- `test_force_target_filter.py` covers the recontact-depth deviation filter.
- `test_udp_ipc.py` covers the UDP frame round-trip.
- `test_probe_sim.py` is parked with the disabled 3-point probe (see ARCHITECTURE.md, section 6).

No CI; run them locally before pushing changes that touch parsing, transforms, export, or the surface module.

### URScript generation constraints (CB3 / PolyScope 3.x)

`design/export.py` targets a CB3 controller: no `stopl`/`movel` inside URScript threads, no list slicing (`[0:3]`), PolyScope memory budget checked, TCP speed clamped to `URSCRIPT_MAX_TCP_SPEED`. The generated program never actuates the 2F-85 (passive finger support — no `rq_*`, `set_payload`, or tool RS485). See ARCHITECTURE.md, section 4.
