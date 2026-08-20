"""
ur5_sim/visualization/controls.py - rappels des widgets de lecture du viewer.

START / STOP, PAUSE / RESUME, le selecteur de configuration IK et le bouton
« Reouvrir 3D ». STOP jette le temps ecoule (le prochain START rejoue depuis
la frame 0, comportement documente de longue date) ; PAUSE le banque dans
`paused_sim_t` pour que RESUME reprenne ou la lecture s'est arretee. La regle
elle-meme est enoncee et testee dans `playback_clock.PlaybackClock`.
Ces fermetures etaient inlinees dans `viewer.visualize` ; elles vivent ici
parce qu'elles forment la surface de commande de la lecture, distincte de la
pompe a frames (`playback.py`) et de la construction des artistes
(`mpl_display.py`).

Elles partagent l'etat de lecture avec `playback.py` par le dictionnaire
`core` que celui-ci assemble : les cellules d'horloge y sont des listes d'un
element, donc mutables de part et d'autre de la frontiere, exactement comme
elles l'etaient entre fermetures d'une meme fonction.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import matplotlib.pyplot as plt

from ur5_sim.config import SIM_SPEED
from ur5_sim.visualization.mpl_display import clear_frame_artists
from ur5_sim.visualization.recompute import recompute_all_branches
from ur5_sim.visualization.swift_scene import reopen_swift_tab


def build_controls(core: dict[str, Any]) -> dict[str, Any]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Build the widget callbacks that drive the playback state.

    Inputs:
        core (dict): shared playback context assembled by
            ur5_sim.visualization.playback.build_playback - the state dict,
            the recompute handoff, the display handles, the one-element
            clock cells, and the compute_state / render_frame / write_hud
            functions.

    Outputs:
        controls (dict): set_stop, set_start, set_pause,
            reset_playback_to_start, on_radio, on_button,
            on_pause_button, on_swift_btn.
    --------------------------------------------------------------------------
    """
    state = core["state"]
    recompute = core["recompute"]
    display = core["display"]
    fig = display["fig"]
    sim_text = display["sim_text"]
    status_text = display["status_text"]
    btn = display["btn"]
    pause_btn = display["pause_btn"]
    labels = display["labels"]

    clock_t0 = core["clock_t0"]
    paused_sim_t = core["paused_sim_t"]
    last_drawn = core["last_drawn"]
    last_wall = core["last_wall"]
    dt_real_window = core["dt_real_window"]
    dt_real_avg = core["dt_real_avg"]
    last_dt_real = core["last_dt_real"]

    compute_state = core["compute_state"]
    # Reports START / PAUSE / STOP to the RTDE emulator; a no-op when none is
    # served (ur5_sim.visualization.rtde_link).
    publish_run_state = core["publish_run_state"]
    render_frame = core["render_frame"]
    write_hud = core["write_hud"]

    robot = core["robot"]
    env = core["env"]
    trajectories = core["trajectories"]
    surface = core["surface"]
    p_anchor_old = core["p_anchor_old"]
    p_ref = core["p_ref"]

    def set_stop() -> None:
        # STOP is always a hard stop; next START must restart from frame 0.
        # finished=True, so the monitor closes its CSV and the next START
        # opens a new one - which is exactly what a replay from frame 0 is.
        publish_run_state(False, 0.0, True)
        paused_sim_t[0] = 0.0
        state["running"] = False
        state["paused"] = False
        pause_btn.label.set_text("PAUSE")
        btn.label.set_text("START")
        btn.color = "#cce5cc"
        btn.hovercolor = "#a6d6a6"
        status_text.set_text("STATE = STOP")
        status_text.set_color("#a02020")

    def reset_playback_to_start() -> None:
        # Efface toutes les anciennes traces et repositionne la simulation au debut.
        clear_frame_artists(display)

        paused_sim_t[0] = 0.0
        clock_t0[0] = time.perf_counter()
        last_drawn[0] = -1
        last_wall[0] = None
        dt_real_window.clear()
        dt_real_avg[0] = 0.0
        last_dt_real[0] = 0.0

        render_frame(0)
        write_hud(0, 0.0)

    def set_start() -> None:
        # Deliberately does NOT reset paused_sim_t: a RESUME must keep the
        # simulation time banked by set_pause. Only set_stop discards it.
        state["running"] = True
        state["paused"] = False
        # The banked time is the resume point, and 0.0 on a fresh START.
        publish_run_state(True, paused_sim_t[0], False)
        clock_t0[0] = time.perf_counter()
        last_wall[0] = None
        dt_real_window.clear()
        dt_real_avg[0] = 0.0
        last_dt_real[0] = 0.0
        btn.label.set_text("STOP")
        btn.color = "#f5c6c6"
        btn.hovercolor = "#e89999"
        status_text.set_text("STATE = RUN")
        status_text.set_color("#207020")

    def set_pause() -> None:
        # PAUSE keeps the run alive: paused_sim_t banks the elapsed sim time so
        # RESUME continues instead of replaying. STOP still discards it.
        if not state["running"]:
            return
        paused_sim_t[0] = (time.perf_counter() - clock_t0[0]) * SIM_SPEED + paused_sim_t[0]
        state["running"] = False
        state["paused"] = True
        pause_btn.label.set_text("RESUME")
        status_text.set_text("STATE = PAUSE")
        status_text.set_color("#a06000")
        # finished=False with a positive banked time: runtime_state goes
        # PAUSING then PAUSED, so the monitor keeps ONE file across the hold
        # instead of splitting the run in two.
        publish_run_state(False, paused_sim_t[0], False)

    def on_pause_button(_event) -> None:
        if state["running"]:
            set_pause()
        elif state.get("paused"):
            state["paused"] = False
            pause_btn.label.set_text("PAUSE")
            set_start()
        fig.canvas.draw_idle()

    def on_radio(label: str) -> None:
        if recompute["active"]:
            return
        idx = labels.index(label)
        set_stop()
        sim_text.set_text("Rebuilding buffer for selected configuration...")
        fig.canvas.draw_idle()
        plt.pause(0.01)
        print(f"Rebuilding buffer for configuration '{label}'...")
        t0 = time.perf_counter()
        compute_state(idx)
        print(f"  ready in {time.perf_counter() - t0:.1f} s.")
        paused_sim_t[0] = 0.0
        clock_t0[0] = time.perf_counter()
        last_drawn[0] = -1
        render_frame(0)
        write_hud(0, 0.0)
        fig.canvas.draw_idle()

    def on_button(_event) -> None:
        # STOP branch: a running playback is halted immediately. A PAUSED run
        # counts as running here - the button still reads STOP, and set_pause
        # left state["running"] False, so testing that flag alone would send a
        # STOP click down the START path and silently restart the trajectory.
        if state["running"] or state.get("paused"):
            set_stop()
            write_hud(0, 0.0)
            fig.canvas.draw_idle()
            return
        # A background recompute is already in flight -> ignore extra clicks.
        if recompute["active"]:
            fig.canvas.draw_idle()
            return
        # START always re-reads ``etalement.script`` and re-runs IK for every
        # branch, so the viewer never replays a trajectory that is out of date
        # with the file on disk (e.g. after the design UI re-exports it). The
        # work runs off the GUI thread; ``tick`` shows the progress and swaps
        # the fresh buffer in when the worker finishes (no freeze).
        set_stop()
        recompute["active"] = True
        recompute["ready"] = False
        recompute["error"] = None
        recompute["payload"] = None
        recompute["done"] = 0
        recompute["total"] = 1
        sim_text.set_text("Recomputing trajectory... 0%")
        status_text.set_text("STATE = BUSY")
        status_text.set_color("#a06000")
        btn.label.set_text("...")
        fig.canvas.draw_idle()
        worker = threading.Thread(
            target=recompute_all_branches,
            args=(trajectories, surface, p_anchor_old, p_ref, recompute),
            daemon=True,
        )
        worker.start()

    def on_swift_btn(_event) -> None:
        if env is None:
            print("[viewer] Reouvrir 3D: Swift indisponible (env=None).")
            return
        reopen_swift_tab(
            env, robot, state["trajectory"],
            last_drawn[0] if last_drawn[0] >= 0 else 0,
        )

    return {
        "set_stop": set_stop,
        "set_start": set_start,
        "set_pause": set_pause,
        "reset_playback_to_start": reset_playback_to_start,
        "on_radio": on_radio,
        "on_button": on_button,
        "on_pause_button": on_pause_button,
        "on_swift_btn": on_swift_btn,
    }
