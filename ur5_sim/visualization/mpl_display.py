"""
ur5_sim/visualization/mpl_display.py — Construction des panneaux matplotlib.

Responsabilités :
  - Crée la figure et les axes (XYZ vs temps, trajectoire XY).
  - Ajoute l'empreinte de la surface de test et les marques de mesure.
  - Instancie les widgets (RadioButtons, Boutons START/STOP, Reouvrir 3D).
  - Retourne un dictionnaire `plot_objects` avec tous les handles d'artistes.

Cette fonction ne contient aucune logique d'animation : elle construit les
objets matplotlib et les retourne à `viewer.py` qui les intègre dans les
closures d'animation.
"""

from __future__ import annotations

from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle as MplCircle, Polygon as MplPolygon
from matplotlib.widgets import Button, RadioButtons

from ur5_sim.config import P_ANCHOR_OLD_RAW, P_REF_RAW, SURFACE_COLOR_RGBA
from ur5_sim.parsing.urscript import urscript_pose
from ur5_sim.visualization.surface import (
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
    Construit la figure matplotlib et tous ses widgets.

    Retourne un dictionnaire avec les clés suivantes :
      fig, ax_xyz, ax_xy,
      line_x, line_y, line_z,
      cycle_xy_lines,
      xy_now,
      sim_text, status_text,
      radio, btn, swift_btn,
      labels,
      cycle_palette,
      max_palette_cycles,
    """
    times = np.arange(n_frames) * dt
    labels = [t[0] for t in trajectories]

    cycle_palette = [
        "#888888",
        "#1f77b4", "#ff7f0e", "#2ca02c",
        "#d62728", "#9467bd", "#8c564b",
        "#7f7f7f", "#bcbd22", "#17becf",
    ]
    max_palette_cycles = max(6, n_cycles_detected if n_cycles_detected else 6)

    fig = plt.figure(figsize=(12, 6))
    gs = GridSpec(1, 2, wspace=0.25, left=0.07, right=0.78, top=0.88, bottom=0.13)
    ax_xyz = fig.add_subplot(gs[0, 0])
    ax_xy = fig.add_subplot(gs[0, 1])

    # --- Panneau XYZ ---
    ax_xyz.set_xlim(0, times[-1] if n_frames > 1 else 1.0)
    ax_xyz.set_xlabel("Temps (s)")
    ax_xyz.set_ylabel("Position TCP (m)")
    ax_xyz.set_title("X, Y, Z vs temps")
    ax_xyz.grid(True, alpha=0.3)
    line_x, = ax_xyz.plot([], [], "r-", lw=1.2, label="X")
    line_y, = ax_xyz.plot([], [], "g-", lw=1.2, label="Y")
    line_z, = ax_xyz.plot([], [], "b-", lw=1.2, label="Z")
    ax_xyz.legend(loc="upper right")

    # --- Panneau XY ---
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.set_title("Trajectoire X-Y (vue dessus)")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.grid(True, alpha=0.3)

    cycle_xy_lines: list[Any] = []
    for c_i in range(max_palette_cycles + 1):
        label = "Initialisation" if c_i == 0 else f"Cycle {c_i}"
        ln, = ax_xy.plot(
            [], [],
            color=cycle_palette[c_i % len(cycle_palette)],
            lw=1.0, alpha=0.9,
            label=label,
        )
        cycle_xy_lines.append(ln)
    ax_xy.legend(loc="best", fontsize=7, frameon=True)

    # --- Overlay surface ---
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

    if surface is not None:
        try:
            p_anchor_tp = urscript_pose(*P_ANCHOR_OLD_RAW)
            p_ref_tp = urscript_pose(*P_REF_RAW)
            tp_radius_m = test_point_radius_world_m(surface)
            for lbl, xyz in compute_test_points_world(p_anchor_tp, p_ref_tp):
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
                    float(xyz[0]), float(xyz[1]), str(lbl),
                    color="#1f4f8a", fontsize=8, fontweight="bold",
                    ha="center", va="center", zorder=4,
                )
        except Exception as exc:
            print(f"[mpl_display] Overlay points de test ignoré : {exc!r}")

    xy_now, = ax_xy.plot([], [], "o", ms=6, zorder=6,
                         markerfacecolor="black", markeredgecolor="black")

    # --- HUD ---
    sim_text = fig.text(
        0.42, 0.96, "", ha="center", va="center",
        fontsize=10, family="monospace",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="0.6"),
    )

    # --- Widgets ---
    radio_ax = fig.add_axes([0.80, 0.50, 0.18, 0.42])
    radio_ax.set_title("Configurations IK", fontsize=10, loc="left")
    radio = RadioButtons(radio_ax, labels=labels, active=0)
    for lw in radio.labels:
        lw.set_fontsize(8)

    button_ax = fig.add_axes([0.80, 0.40, 0.18, 0.06])
    btn = Button(button_ax, "START", color="#cce5cc", hovercolor="#a6d6a6")

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
        "swift_btn": swift_btn,
        "labels": labels,
        "cycle_palette": cycle_palette,
        "max_palette_cycles": max_palette_cycles,
    }
