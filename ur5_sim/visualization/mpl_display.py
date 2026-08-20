"""
ur5_sim/visualization/mpl_display.py — Construction et mise a jour des panneaux matplotlib.

Responsabilités :
  - Crée la figure et les axes (XYZ vs temps, trajectoire XY).
  - Ajoute l'empreinte de la surface de test, les marques de mesure et les
    étoiles de sondage.
  - Instancie les widgets (RadioButtons, boutons START/STOP, PAUSE/RESUME,
    Reouvrir 3D).
  - Retourne un dictionnaire `display` avec tous les handles d'artistes.
  - Met à jour ces artistes à chaque frame.

Aucune logique d'animation ici : la cadence, l'horloge et l'état de lecture
appartiennent à `playback.py`, qui appelle `update_frame_artists` à chaque
tick.
"""

from __future__ import annotations

from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle as MplCircle, Polygon as MplPolygon
from matplotlib.widgets import Button, RadioButtons

from ur5_sim.config import (
    CONTACT_SNAP_TOL_M,
    P_ANCHOR_OLD_RAW,
    P_REF_RAW,
    SURFACE_COLOR_RGBA,
)
from ur5_sim.parsing.urscript import urscript_pose
from ur5_sim.visualization.surface import (
    compute_probe_points_world,
    compute_test_points_world,
    make_corners_xy_polygon,
    test_point_radius_world_m,
)


def build_display(
    trajectories: list[tuple[str, list[np.ndarray]]],
    n_frames: int,
    dt: float,
    n_cycles_detected: int,
    surface: dict | None,
    env: Any,
) -> dict[str, Any]:
    """
    --------------------------------------------------------------------------
    Purpose:
        Build the matplotlib figure, its two panels, the static overlays and
        every widget, then hand back the artist handles.

    Inputs:
        trajectories (list): (label, joint_trajectory) pairs; the labels
            populate the IK configuration selector.
        n_frames (int): trajectory length, sets the time axis.
        dt (float): nominal time step, seconds.
        n_cycles_detected (int): number of URScript cycles found.
        surface (dict | None): surface frame, drives the plate overlays.
        env (Any): Swift environment, or None; only decides the 3D button text.

    Outputs:
        display (dict): fig, ax_xyz, ax_xy, times, line_x/y/z, cycle_xy_lines,
            xy_now, sim_text, status_text, radio, btn, pause_btn,
            swift_btn, labels, cycle_palette, max_palette_cycles.
    --------------------------------------------------------------------------
    """
    times = np.arange(n_frames) * dt

    fig = plt.figure(figsize=(12, 6))
    gs = GridSpec(1, 2, wspace=0.25, left=0.07, right=0.78, top=0.88, bottom=0.13)
    ax_xyz = fig.add_subplot(gs[0, 0])
    ax_xy = fig.add_subplot(gs[0, 1])

    ax_xyz.set_xlim(0, times[-1] if n_frames > 1 else 1.0)
    ax_xyz.set_xlabel("Temps (s)")
    ax_xyz.set_ylabel("Position TCP (m)")
    ax_xyz.set_title("X, Y, Z vs temps")
    ax_xyz.grid(True, alpha=0.3)
    line_x, = ax_xyz.plot([], [], "r-", lw=1.2, label="X")
    line_y, = ax_xyz.plot([], [], "g-", lw=1.2, label="Y")
    line_z, = ax_xyz.plot([], [], "b-", lw=1.2, label="Z")
    ax_xyz.legend(loc="upper right")

    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.set_title("Trajectoire X-Y (vue dessus)")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.grid(True, alpha=0.3)
    # Trail XY colore par cycle : une ligne distincte par cycle URScript
    # detecte, plus un marqueur de la position courante. Les couleurs
    # matchent celles utilisees par le design UI (circulaires : bleu/orange/vert ;
    # rectilignes : rouge/violet/brun).
    cycle_palette = [
        "#888888", # Gris pour Cycle 0 (Initialisation/Palpage)
        "#1f77b4", "#ff7f0e", "#2ca02c",
        "#d62728", "#9467bd", "#8c564b",
        "#7f7f7f", "#bcbd22", "#17becf",
    ]
    max_palette_cycles = max(6, n_cycles_detected if n_cycles_detected else 6)
    cycle_xy_lines: list = []
    for c_i in range(max_palette_cycles + 1):
        label = "Initialisation" if c_i == 0 else f"Cycle {c_i}"
        ln, = ax_xy.plot(
                [], [],
                color=cycle_palette[c_i % len(cycle_palette)],
                lw=1.0, alpha=0.9,
                label=label,
            )
        cycle_xy_lines.append(ln)
    # Legende construite plus bas, apres l'ajout des etoiles de sondage, pour
    # que l'entree "Sondage" y figure.

    # Empreinte de la surface de test sur la vue XY (polygone semi-transparent).
    surface_xy = make_corners_xy_polygon(surface)
    if surface_xy is not None:
        ax_xy.add_patch(
            MplPolygon(
                surface_xy,
                closed=True,
                facecolor=SURFACE_COLOR_RGBA[:3] + (0.15,),
                edgecolor=SURFACE_COLOR_RGBA[:3] + (0.9,),
                linewidth=1.2,
                zorder=2,
            )
        )

    # 9-point test pattern (cf. meshes/TestMeasure.png). Marks are drawn
    # as static circles on top of the plate polygon so the operator can
    # see the live trajectory crossing each measurement point.
    if surface is not None:
        try:
            p_anchor_old_tp = urscript_pose(*P_ANCHOR_OLD_RAW)
            p_ref_tp = urscript_pose(*P_REF_RAW)
            tp_radius_m = test_point_radius_world_m(surface)
            for label, xyz in compute_test_points_world(p_anchor_old_tp, p_ref_tp):
                ax_xy.add_patch(
                    MplCircle(
                        (float(xyz[0]), float(xyz[1])),
                        radius=tp_radius_m,
                        facecolor="none",
                        edgecolor="#c66a3a",
                        linewidth=1.2,
                        zorder=3,
                    )
                )
                ax_xy.text(
                    float(xyz[0]), float(xyz[1]), str(label),
                    color="#1f4f8a", fontsize=8, fontweight="bold",
                    ha="center", va="center", zorder=4,
                )
        except Exception as exc:  # pragma: no cover - non-fatal overlay
            print(f"[viewer] test-point overlay skipped: {exc!r}")

    # Poses de sondage (probe_surface_plane) : etoiles noires sur la plaque,
    # aux 3 points que le robot palpe pour mesurer le plan. Memes anchors /
    # meme chaine plate->monde que les points de test, donc co-localises.
    if surface is not None:
        try:
            p_anchor_old_pb = urscript_pose(*P_ANCHOR_OLD_RAW)
            p_ref_pb = urscript_pose(*P_REF_RAW)
            for k, xyz in enumerate(
                compute_probe_points_world(p_anchor_old_pb, p_ref_pb)
            ):
                ax_xy.plot(
                    float(xyz[0]), float(xyz[1]),
                    marker="*", linestyle="None",
                    color="black", markersize=12, zorder=5,
                    label="Sondage" if k == 0 else "_nolegend_",
                )
        except Exception as exc:  # pragma: no cover - non-fatal overlay
            print(f"[viewer] probe-point overlay skipped: {exc!r}")

    ax_xy.legend(loc="best", fontsize=7, frameon=True)

    xy_now, = ax_xy.plot([], [], "o", ms=6, zorder=6,
                        markerfacecolor="black", markeredgecolor="black")

    sim_text = fig.text(
        0.42, 0.96, "", ha="center", va="center",
        fontsize=10, family="monospace",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="0.6"),
    )

    labels = [t[0] for t in trajectories]
    radio_ax = fig.add_axes([0.80, 0.50, 0.18, 0.42])
    radio_ax.set_title("Configurations IK", fontsize=10, loc="left")
    radio = RadioButtons(radio_ax, labels=labels, active=0)
    for label_widget in radio.labels:
        label_widget.set_fontsize(8)

    # START/STOP et PAUSE partagent la meme rangee : STOP jette le temps
    # ecoule (le prochain START rejoue depuis la frame 0), PAUSE le conserve.
    button_ax = fig.add_axes([0.80, 0.40, 0.087, 0.06])
    btn = Button(button_ax, "START", color="#cce5cc", hovercolor="#a6d6a6")

    pause_btn_ax = fig.add_axes([0.893, 0.40, 0.087, 0.06])
    pause_btn = Button(pause_btn_ax, "PAUSE", color="#ffe0b0", hovercolor="#f5c98a")

    status_text = fig.text(
        0.80, 0.36, "STATE = STOP", fontsize=10, family="monospace", color="#a02020",
    )

    swift_btn_ax = fig.add_axes([0.80, 0.28, 0.18, 0.05])
    swift_btn = Button(
        swift_btn_ax,
        "Reouvrir 3D" if env is not None else "Swift indispo",
        color="#cce5e5" if env is not None else "#e5e5e5",
        hovercolor="#a6d6d6" if env is not None else "#e5e5e5",
    )

    return {
        "fig": fig,
        "ax_xyz": ax_xyz,
        "ax_xy": ax_xy,
        "times": times,
        "line_x": line_x,
        "line_y": line_y,
        "line_z": line_z,
        "cycle_xy_lines": cycle_xy_lines,
        "xy_now": xy_now,
        "sim_text": sim_text,
        "status_text": status_text,
        "radio": radio,
        "btn": btn,
        "pause_btn": pause_btn,
        "swift_btn": swift_btn,
        "labels": labels,
        "cycle_palette": cycle_palette,
        "max_palette_cycles": max_palette_cycles,
    }


def update_frame_artists(
    display: dict[str, Any],
    frame: int,
    times: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    zs: np.ndarray,
    cycle_indices: list | None,
    in_contact: bool,
    surface_depth_m: float,
) -> None:
    """
    --------------------------------------------------------------------------
    Purpose:
        Repaint the 2D panels for one frame: the three XYZ curves, the XY
        trail up to that frame, and the live marker whose colour states the
        force regime (green in contact, blue in transit, red off-plane).

    Inputs:
        display (dict): handles returned by build_display.
        frame (int): index of the frame to paint.
        times (np.ndarray): time axis, seconds.
        xs, ys, zs (np.ndarray): world TCP coordinates, metres.
        cycle_indices (list | None): frame indices grouped by URScript cycle.
        in_contact (bool): inside a force_mode block.
        surface_depth_m (float): signed distance to the plate, metres.

    Outputs:
        None. The artists are mutated in place; the caller redraws.
    --------------------------------------------------------------------------
    """
    display["line_x"].set_data(times[: frame + 1], xs[: frame + 1])
    display["line_y"].set_data(times[: frame + 1], ys[: frame + 1])
    display["line_z"].set_data(times[: frame + 1], zs[: frame + 1])
    # Trail XY par cycle : on ne trace que les frames deja jouees.
    cycle_xy_lines = display["cycle_xy_lines"]
    if cycle_indices is not None:
        for c_i, idxs in enumerate(cycle_indices):
            visible_idxs = idxs[idxs <= frame]
            if len(visible_idxs) > 0:
                cycle_xy_lines[c_i].set_data(xs[visible_idxs], ys[visible_idxs])
            else:
                cycle_xy_lines[c_i].set_data([], [])
    xy_now = display["xy_now"]
    xy_now.set_data([xs[frame]], [ys[frame]])
    # Marqueur live colore selon le regime force_mode et l'ecart au plan.
    tol = CONTACT_SNAP_TOL_M
    if in_contact:
        marker_color = "red" if abs(surface_depth_m) > tol else "#1faa00"
    else:
        marker_color = "red" if surface_depth_m < -tol else "#1f6dff"
    xy_now.set_markerfacecolor(marker_color)
    xy_now.set_markeredgecolor(marker_color)


def clear_frame_artists(display: dict[str, Any]) -> None:
    """Efface toutes les anciennes traces (retour au debut de la lecture)."""
    for ln in display["cycle_xy_lines"]:
        ln.set_data([], [])
    display["line_x"].set_data([], [])
    display["line_y"].set_data([], [])
    display["line_z"].set_data([], [])
    display["xy_now"].set_data([], [])
