"""
ur5_sim/visualization/playback.py - horloge, etat de lecture et pompe a frames.

Ce module tient la moitie « cadence et etat » de l'ancien viewer.py : le
tampon par branche IK, l'horloge murale, le rendu par frame, le HUD et le
rappel du timer matplotlib.

Le decoupage suit les seams du fichier d'origine : `mpl_display.py` construit
et repeint les artistes, `swift_scene.py` monte et alimente la scene 3D,
`ipc_live.py` publie l'etat TCP au design UI, `controls.py` porte les rappels
des boutons, et `viewer.visualize` ne fait plus que cabler le tout.

Les fermetures restent des fermetures et partagent l'etat comme avant ; elles
sont simplement construites ici par `build_playback`. Les cellules d'horloge
sont des listes d'un element, donc `controls.py` les mute a travers le
dictionnaire `core`.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

from ur5_sim.config import (
    END_LINK,
    FORCE_Z_TARGET_N,
    SIM_SPEED,
    P_ANCHOR_OLD_RAW,
    P_REF_RAW,
)
from ur5_sim.kinematics.transforms import tcp_tool_offset
from ur5_sim.parsing.urscript import urscript_pose
from ur5_sim.visualization.controls import build_controls
from ur5_sim.visualization.ipc_live import (
    build_tcp_live_payload,
    send_tcp_live,
    send_tcp_live_idle,
)
from ur5_sim.visualization.mpl_display import update_frame_artists
from ur5_sim.visualization.swift_scene import push_pose_to_swift


def build_playback(
    robot,
    trajectories: list[tuple[str, list[np.ndarray]]],
    dt: float,
    surface: dict | None,
    cycle_per_frame: list[int],
    plate_xy_per_frame: list[tuple[float, float]],
    in_contact_per_frame: list[bool],
    n_cycles_detected: int,
    scene: dict[str, Any],
    display: dict[str, Any],
) -> dict[str, Any]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Build the animation closures of the viewer, bound to the Swift scene
        and the matplotlib artists, and compute the first trajectory buffer
        before returning, as the viewer always did.

    Inputs:
        robot: roboticstoolbox UR5 model.
        trajectories (list): (label, joint_trajectory) pairs, one per IK
            branch. Mutated in place when a recompute lands.
        dt (float): nominal time step, seconds.
        surface (dict | None): surface frame, or None.
        cycle_per_frame (list[int]): 1-based URScript cycle per frame.
        plate_xy_per_frame (list[tuple]): script pose in the P_ANCHOR_OLD frame.
        in_contact_per_frame (list[bool]): inside a force_mode block.
        n_cycles_detected (int): number of cycles found in the script.
        scene (dict): handles from swift_scene.setup_swift_scene.
        display (dict): handles from mpl_display.build_display.

    Outputs:
        playback (dict): tick, render_frame, write_hud, emit_idle_ipc, state,
            plus the widget callbacks from controls.build_controls.
    --------------------------------------------------------------------------
    """
    env = scene["env"]
    ee_handles = scene["ee_handles"]
    tcp_marker = scene["tcp_marker"]

    fig = display["fig"]
    ax_xyz = display["ax_xyz"]
    ax_xy = display["ax_xy"]
    sim_text = display["sim_text"]
    max_palette_cycles = display["max_palette_cycles"]

    n_frames = len(trajectories[0][1])
    times = np.arange(n_frames) * dt

    state = {
        "idx": 0,
        "running": False,
        "trajectory": None,
        "tcp_pts": None,
        "xs": None, "ys": None, "zs": None,
        "timer": None,
    }

    p_anchor_old = urscript_pose(*P_ANCHOR_OLD_RAW)
    p_ref = urscript_pose(*P_REF_RAW)
    tool_offset = tcp_tool_offset()
    tool_offset_A = tool_offset.A

    # Cross-thread handoff for the background recompute. The worker thread fills
    # ``done``/``payload``/``error`` under ``lock``; the matplotlib timer
    # (main thread) polls and finalises. Nothing here touches matplotlib/Swift.
    recompute = {
        "active": False,
        "lock": threading.Lock(),
        "done": 0,
        "total": 1,
        "ready": False,
        "payload": None,
        "error": None,
    }

    def compute_state(idx: int) -> None:
        traj = trajectories[idx][1]
        tcp_pts = np.zeros((len(traj), 3))
        for i, q in enumerate(traj):
            tcp_pts[i] = (robot.fkine(q, end=END_LINK).A @ tool_offset_A)[:3, 3]
        xs_, ys_, zs_ = tcp_pts[:, 0], tcp_pts[:, 1], tcp_pts[:, 2]
        state["idx"] = idx
        state["trajectory"] = traj
        state["tcp_pts"] = tcp_pts
        state["xs"], state["ys"], state["zs"] = xs_, ys_, zs_
        # Indices des frames par cycle (1-based) -> base pour les trails colores.
        cycle_arr = np.asarray(cycle_per_frame[: len(traj)], dtype=int)
        state["cycle_indices"] = [
            np.where(cycle_arr == c)[0]
            for c in range(max_palette_cycles + 1)
        ]

        ax_xyz.set_ylim(
            min(xs_.min(), ys_.min(), zs_.min()) - 0.05,
            max(xs_.max(), ys_.max(), zs_.max()) + 0.05,
        )
        pad = 0.05
        ax_xy.set_xlim(xs_.min() - pad, xs_.max() + pad)
        ax_xy.set_ylim(ys_.min() - pad, ys_.max() + pad)

        # Reset des trails colores par cycle.
        for ln in display["cycle_xy_lines"]:
            ln.set_data([], [])

        # Push initial joint config to Swift so the robot is visible at q[0].
        if env is not None and len(traj) > 0:
            try:
                push_pose_to_swift(
                    env, robot, traj[0], ee_handles, tcp_marker, tool_offset_A,
                )
            except Exception as exc:  # pragma: no cover
                print(f"[viewer] Swift step failed: {exc!r}")

    print("Computing trajectory buffer for configuration #1...")
    t0 = time.perf_counter()
    compute_state(0)
    print(f"  ready in {time.perf_counter() - t0:.1f} s.")

    total_sim_time = max(n_frames - 1, 1) * dt
    clock_t0 = [time.perf_counter()]
    paused_sim_t = [0.0]
    last_drawn = [-1]
    paint_count = [0]
    paint_start = [time.perf_counter()]
    last_wall: list[float | None] = [None]
    last_dt_real = [0.0]
    dt_real_window: list[float] = []
    dt_real_avg = [0.0]
    fps_text = [""]

    def render_frame(frame: int) -> None:
        xs_, ys_, zs_ = state["xs"], state["ys"], state["zs"]
        traj = state["trajectory"]
        # 3D : push joint config to Swift.
        if env is not None and traj is not None:
            try:
                push_pose_to_swift(
                    env, robot, traj[frame], ee_handles, tcp_marker, tool_offset_A,
                )
            except Exception:  # pragma: no cover
                pass
        in_contact = (
            in_contact_per_frame[frame]
            if frame < len(in_contact_per_frame) else False
        )
        surface_depth_m = 0.0
        if surface is not None:
            surface_depth_m = float(
                np.dot(
                    surface["normal"],
                    np.array([xs_[frame], ys_[frame], zs_[frame]])
                    - surface["center"],
                )
            )
        # 2D matplotlib lines, per-cycle trail, live marker colour.
        cycle_indices = state.get("cycle_indices")
        update_frame_artists(
            display, frame, times, xs_, ys_, zs_,
            cycle_indices, in_contact, surface_depth_m,
        )
        # IPC : publish TCP state to the design UI.
        send_tcp_live(build_tcp_live_payload(
            bool(state["running"]), frame, n_frames, xs_, ys_, zs_,
            cycle_per_frame, plate_xy_per_frame, cycle_indices,
            n_cycles_detected, in_contact, surface_depth_m,
        ))
        # Memorise pour write_hud.
        state["_last_in_contact"] = in_contact
        state["_last_surface_depth_m"] = surface_depth_m

    def write_hud(frame: int, sim_elapsed: float) -> None:
        run_flag = "RUN " if state["running"] else "STOP"
        cfg_label = trajectories[state["idx"]][0]
        wall_elapsed = (time.perf_counter() - clock_t0[0]) if state["running"] else paused_sim_t[0]
        sync_err_ms = (wall_elapsed - sim_elapsed) * 1000.0
        backend_tag = "Swift" if env is not None else "2D"
        current_cycle = cycle_per_frame[frame] if frame < len(cycle_per_frame) else 0
        in_contact = bool(state.get("_last_in_contact", False))
        surface_depth_mm = float(state.get("_last_surface_depth_m", 0.0)) * 1000.0
        f_z = FORCE_Z_TARGET_N if in_contact else 0.0
        sim_text.set_text(
            f"[{run_flag}] {backend_tag}  cfg={cfg_label[:22]:<22s}  |  "
            f"cycle {current_cycle}/{n_cycles_detected}  |  "
            f"F_Z = {f_z:4.1f} N  dz = {surface_depth_mm:+6.2f} mm  |  "
            f"PC t = {wall_elapsed:6.3f} s  |  "
            f"SIM t = {sim_elapsed:6.3f} / {total_sim_time:5.2f} s  |  "
            f"delta = {sync_err_ms:+6.1f} ms  |  "
            f"frame {frame:>3d}/{n_frames - 1}  |  "
            f"dt_real = {last_dt_real[0] * 1000:5.1f} ms  "
            f"avg = {dt_real_avg[0] * 1000:5.1f} ms  "
            f"target = {dt * 1000:5.1f} ms{fps_text[0]}"
        )

    def _finalize_recompute(payload: dict) -> None:
        """Swap in the freshly computed trajectory (main thread only)."""
        nonlocal n_frames, times, total_sim_time, cycle_per_frame
        nonlocal plate_xy_per_frame, n_cycles_detected, in_contact_per_frame
        trajectories[:] = payload["trajectories"]
        cycle_per_frame = payload["cycle"]
        plate_xy_per_frame = payload["plate"]
        in_contact_per_frame = payload["contact"]
        n_cycles_detected = max(cycle_per_frame) if cycle_per_frame else 0
        n_frames = len(trajectories[0][1])
        times = np.arange(n_frames) * dt
        total_sim_time = max(n_frames - 1, 1) * dt
        ax_xyz.set_xlim(0, times[-1] if n_frames > 1 else 1.0)
        if state["idx"] >= len(trajectories):
            state["idx"] = 0
        compute_state(state["idx"])

    # Les rappels de boutons partagent cet etat ; ils vivent dans controls.py.
    controls = build_controls({
        "state": state,
        "recompute": recompute,
        "display": display,
        "clock_t0": clock_t0,
        "paused_sim_t": paused_sim_t,
        "last_drawn": last_drawn,
        "last_wall": last_wall,
        "dt_real_window": dt_real_window,
        "dt_real_avg": dt_real_avg,
        "last_dt_real": last_dt_real,
        "compute_state": compute_state,
        "render_frame": render_frame,
        "write_hud": write_hud,
        "robot": robot,
        "env": env,
        "trajectories": trajectories,
        "surface": surface,
        "p_anchor_old": p_anchor_old,
        "p_ref": p_ref,
    })
    set_stop = controls["set_stop"]
    set_start = controls["set_start"]
    reset_playback_to_start = controls["reset_playback_to_start"]

    def tick() -> None:
        # Background recompute in flight (script changed): keep Swift + the GUI
        # alive, show progress, and finalise on this main thread when done.
        if recompute["active"]:
            if env is not None:
                try:
                    env.step(0)
                except Exception:
                    pass
            with recompute["lock"]:
                done = recompute["done"]
                total = recompute["total"]
                ready = recompute["ready"]
                error = recompute["error"]
                payload = recompute["payload"]
            if not ready:
                pct = int(100 * done / total) if total else 0
                sim_text.set_text(
                    f"Recomputing trajectory... {pct}% ({done}/{total} IK)"
                )
                fig.canvas.draw_idle()
                return
            recompute["active"] = False
            if error is not None or payload is None:
                print(f"[viewer] recompute failed ({error}); keeping buffer.")
                sim_text.set_text("Recompute failed - previous buffer kept.")
                reset_playback_to_start()
                set_start()
                fig.canvas.draw_idle()
                return
            _finalize_recompute(payload)
            reset_playback_to_start()
            set_start()
            fig.canvas.draw_idle()
            return

        if not state["running"]:
            # On maintient la boucle Swift active pour l'interactivite (camera)
            # meme quand la simulation est a l'arret. Cela evite que le
            # serveur WebSocket de Swift ne se ferme par inactivite.
            if env is not None:
                try:
                    env.step(0)
                except Exception as e:
                    print(f"[viewer] Swift env.step(0) failed in idle mode: {e!r}")
            return
        wall = time.perf_counter()
        sim_elapsed = (wall - clock_t0[0]) * SIM_SPEED + paused_sim_t[0]
        if sim_elapsed > total_sim_time:
            # Fin de trajectoire : on MAINTIENT l'effecteur sur la derniere pose
            # (le retrait Z+3 cm au-dessus du dernier waypoint) et on arrete la
            # lecture, au lieu de boucler vers la frame 0 (debut de trajectoire).
            # Reproduit le comportement reel : apres etalement() le programme
            # s'arrete sur le retrait, laissant la plaque accessible.
            # Un nouveau START rejoue depuis le debut (cf. on_button ->
            # reset_playback_to_start).
            sim_elapsed = total_sim_time
            frame = n_frames - 1
            if frame != last_drawn[0]:
                last_drawn[0] = frame
                render_frame(frame)
                write_hud(frame, sim_elapsed)
            set_stop()
            fig.canvas.draw_idle()
            return
        frame = min(int(sim_elapsed / dt), n_frames - 1)
        if frame == last_drawn[0]:
            return

        if last_wall[0] is not None:
            last_dt_real[0] = wall - last_wall[0]
            dt_real_window.append(last_dt_real[0])
            if len(dt_real_window) > 20:
                del dt_real_window[0]
            dt_real_avg[0] = sum(dt_real_window) / len(dt_real_window)
        last_wall[0] = wall

        paint_count[0] += 1
        if wall - paint_start[0] >= 1.0:
            fps = paint_count[0] / (wall - paint_start[0])
            fps_text[0] = f"  |  draw {fps:5.1f} fps"
            paint_count[0] = 0
            paint_start[0] = wall

        last_drawn[0] = frame
        render_frame(frame)
        write_hud(frame, sim_elapsed)
        fig.canvas.draw_idle()

    def emit_idle_ipc() -> None:
        """Cleanup on window close: tell the design UI the run is over."""
        send_tcp_live_idle(n_frames, n_cycles_detected)

    return {
        "state": state,
        "tick": tick,
        "render_frame": render_frame,
        "write_hud": write_hud,
        "on_radio": controls["on_radio"],
        "on_button": controls["on_button"],
        "on_swift_btn": controls["on_swift_btn"],
        "emit_idle_ipc": emit_idle_ipc,
    }
