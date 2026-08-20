"""Hybrid Swift + matplotlib viewer for the UR5 trajectory replay.

Architecture
------------
* **Swift backend (WebGL, browser tab)** - handles the 3D rendering of the
  UR5 robot. Swift ships with the UR5 URDF + meshes, gives GPU-accelerated
  real-time playback similar in feel to RoboDK / URSim, and supports user
  camera control natively (orbit/pan/zoom).
* **matplotlib (native window)** - keeps the two 2D panels (XYZ vs time
  and XY trail), the configuration selector (IK branches), the START/STOP
  toggle, the PAUSE/RESUME toggle, and the HUD text.

Compute stage assembles a per-frame buffer of TCP positions and joint
configurations once per selected IK branch. Display stage is wall-clock
driven: every tick pushes ``robot.q = trajectory[frame]`` to Swift and
updates the matplotlib line data. Slow renders drop frames instead of
stalling the simulation clock.

The simulation starts in the STOP state; the user must click START to
begin the playback. PAUSE holds it without ending the run (RESUME
continues where it stopped); STOP ends it, and the next START replays
from frame 0.

This file is the assembly point only. The work lives in five siblings, one
per concern, so no single file passes the workspace file-size ceiling:

* :mod:`ur5_sim.visualization.swift_scene` - 3D scene: launch, meshes,
  keepalive, per-frame pose push, teardown.
* :mod:`ur5_sim.visualization.mpl_display` - figure, panels, overlays,
  widgets, and the per-frame artist repaint.
* :mod:`ur5_sim.visualization.playback` - trajectory buffer, wall clock,
  HUD, and the matplotlib timer callback.
* :mod:`ur5_sim.visualization.controls` - the START / STOP,
  PAUSE / RESUME and configuration widget callbacks.
* :mod:`ur5_sim.visualization.ipc_live` - the UDP loopback status feed to
  the design UI.
"""

from __future__ import annotations

from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb

from ur5_sim.config import DT
from ur5_sim.visualization.interactions import attach_pan, attach_scroll_zoom
from ur5_sim.visualization.mpl_display import build_display
from ur5_sim.visualization.playback import build_playback
from ur5_sim.visualization.swift_scene import (
    setup_swift_scene,
    start_heartbeat,
    stop_heartbeat,
    teardown_swift_scene,
)


def _fit_per_frame(seq: list | None, n_frames: int, fill: Any) -> list:
    """Pad with ``fill`` or truncate ``seq`` so it aligns with the trajectory.

    A sequence that already has the right length is returned unchanged, as
    the inline form this replaces did.
    """
    if seq is None:
        return [fill] * n_frames
    if len(seq) < n_frames:
        return list(seq) + [fill] * (n_frames - len(seq))
    if len(seq) > n_frames:
        return list(seq[:n_frames])
    return seq


def visualize(
    robot: rtb.Robot,
    trajectories: list[tuple[str, list[np.ndarray]]],
    dt: float = DT,
    cycle_per_frame: list[int] | None = None,
    plate_xy_per_frame: list[tuple[float, float]] | None = None,
    surface: dict | None = None,
    in_contact_per_frame: list[bool] | None = None,
) -> None:
    """Run the live animation with multi-configuration selection.

    Parameters
    ----------
    robot:
        roboticstoolbox UR5 model.
    trajectories:
        List of ``(label, joint_trajectory)`` pairs - one entry per IK
        branch that reaches the initial pose. The label appears in the
        configuration selector.
    dt:
        Nominal time step between successive joint configurations.
    cycle_per_frame:
        Optional list aligned with the trajectory length, containing the
        1-based index of the URScript cycle each frame belongs to
        (``0`` for frames that fall outside any ``def cycle_N():`` block).
        When provided, the viewer publishes a live TCP status to the design
        UI over UDP loopback (``ur5_sim/ipc_config.py``, 127.0.0.1:47811).
        The ``tcp_live.json`` file exchange this once used is retired.
    surface:
        Optional surface frame returned by
        :func:`ur5_sim.visualization.surface.compute_surface_frame`. When
        present, the test plate is rendered both in Swift (50x50x2 mm
        cuboid) and in the matplotlib XY panel (semi-transparent polygon),
        and the HUD marks each frame as contact (green) or transit (blue).
    in_contact_per_frame:
        Optional list aligned with the trajectory; ``True`` between
        ``force_mode(...)`` and ``end_force_mode()``. Drives the HUD
        ``F_Z`` field (6.0 N during contact, 0.0 N during transit) and the
        live marker colour. Defaults to all-False if omitted.
    """
    if not trajectories:
        raise ValueError("visualize() requires at least one trajectory")

    n_frames = len(trajectories[0][1])
    cycle_per_frame = _fit_per_frame(cycle_per_frame, n_frames, 0)
    n_cycles_detected = max(cycle_per_frame) if cycle_per_frame else 0
    plate_xy_per_frame = _fit_per_frame(plate_xy_per_frame, n_frames, (0.0, 0.0))
    in_contact_per_frame = _fit_per_frame(in_contact_per_frame, n_frames, False)

    # --- 3D rendering : Swift (browser tab, WebGL) ---
    scene = setup_swift_scene(robot, surface)
    env = scene["env"]

    # Keepalive must run while the figure is built (5-15 s on Windows).
    hb_active, hb_thread = start_heartbeat(env)

    # --- 2D panels + UI : matplotlib ---
    display = build_display(
        trajectories, n_frames, dt, n_cycles_detected, surface, env,
    )
    fig = display["fig"]

    playback = build_playback(
        robot, trajectories, dt, surface,
        cycle_per_frame, plate_xy_per_frame, in_contact_per_frame,
        n_cycles_detected, scene, display,
    )

    display["radio"].on_clicked(playback["on_radio"])
    display["btn"].on_clicked(playback["on_button"])
    display["pause_btn"].on_clicked(playback["on_pause_button"])
    display["swift_btn"].on_clicked(playback["on_swift_btn"])

    # Navigation souris sur la vue X-Y (vue dessus) uniquement : molette = zoom
    # centre sur le curseur (haut = zoom avant, comme Swift), glisser-bouton
    # gauche = translation. Pas de rotation (plan 2D). Limite a ax_xy pour ne
    # pas perturber le graphe X,Y,Z-vs-temps ni les widgets.
    attach_scroll_zoom(fig, [display["ax_xy"]], [])
    attach_pan(fig, display["ax_xy"])

    playback["render_frame"](0)
    playback["write_hud"](0, 0.0)

    print(
        f"Viewer ready ({len(trajectories)} configurations). "
        f"Click START to play. Close the matplotlib window to exit."
    )
    timer = fig.canvas.new_timer(interval=30) # Intervalle de 30ms (environ 33 FPS)
    timer.add_callback(playback["tick"])
    playback["state"]["timer"] = timer
    timer.start()
    # Matplotlib timer now owns the heartbeat - stop the background thread.
    stop_heartbeat(hb_active, hb_thread)
    plt.show()

    # Cleanup on window close : flag the IPC so the design UI hides
    # its live marker even if the simulator window closes mid-cycle.
    playback["emit_idle_ipc"]()
    teardown_swift_scene(env, scene["ee_temp_dir"])
