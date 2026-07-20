"""
design/app.py — Interface graphique matplotlib pour le design de trajectoires.

Responsabilités :
  - Affiche 6 sous-graphiques (un par cycle) avec trajectoires colorées.
  - Sliders : points de discrétisation, CIRC_R_CIRCLE, N_CIRCULAR_CYCLES,
    CIRC_N_PASSES, CIRC_N_CIRCLES.
  - Boutons : Afficher Waypoints, Tri/Rect, Exporter URScript.
  - Overlay live TCP depuis le simulateur (via design.live_ipc).
  - Fonction main() : point d'entrée CLI.
"""

from __future__ import annotations

import argparse
from typing import Any, List

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider

import design.params as P
from design.export import generate_urscript, generate_urp
from design.live_ipc import build_ipc_overlay
from design.trajectory import (
    build_full_trajectory,
    circular_cycle,
    get_waypoint_indices,
    linear_cycle,
    triangular_cycle,
)


# ---------------------------------------------------------------------------
# Helpers de style
# ---------------------------------------------------------------------------

def _style_ax(ax, title: str) -> None:
    ax.set_facecolor('#16213e')
    ax.add_patch(mpatches.Rectangle(
        (0, 0), P.SURFACE_W, P.SURFACE_H,
        linewidth=1.5, edgecolor='white', facecolor='none', linestyle='--'))
    ax.set_xlim(-5, P.SURFACE_W + 5)
    ax.set_ylim(-5, P.SURFACE_H + 5)
    ax.set_aspect('equal')
    ax.set_title(title, color='white', fontsize=9, pad=4)
    ax.tick_params(colors='#aaaaaa', labelsize=7)
    ax.set_xlabel('X (mm)', color='#aaaaaa', fontsize=7)
    ax.set_ylabel('Y (mm)', color='#aaaaaa', fontsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor('#444466')


def draw_validation_grid(ax) -> None:
    """Ajoute la grille de validation 9 points sur un axe matplotlib."""
    coords = np.linspace(P.MARGIN, P.SURFACE_W - P.MARGIN, 3)
    idx = 1
    for y in reversed(coords):
        for x in coords:
            circ = mpatches.Circle((x, y), 2.5, color='yellow', alpha=0.3,
                                   ec='white', linestyle='--', zorder=2)
            ax.add_patch(circ)
            ax.text(x, y, str(idx), color='white', weight='bold',
                    ha='center', va='center', fontsize=8, zorder=3)
            idx += 1


def _resample_points(pts: np.ndarray, n_points: int) -> np.ndarray:
    """Ré-échantillonne sur la distance curviligne pour un pas spatial uniforme."""
    n_points = max(2, int(n_points))
    deltas = np.diff(pts[:, :2], axis=0)
    seg_len = np.sqrt((deltas ** 2).sum(axis=1))
    s = np.concatenate(([0.0], np.cumsum(seg_len)))
    keep = np.concatenate(([True], np.diff(s) > 1e-12))
    s_u = s[keep]
    pts_u = pts[keep]
    if len(s_u) < 2 or s_u[-1] <= 1e-12:
        return np.repeat(pts_u[:1], n_points, axis=0)
    s_target = np.linspace(0.0, s_u[-1], n_points)
    x = np.interp(s_target, s_u, pts_u[:, 0])
    y = np.interp(s_target, s_u, pts_u[:, 1])
    z = np.interp(s_target, s_u, pts_u[:, 2])
    return np.column_stack((x, y, z))


# ---------------------------------------------------------------------------
# Construction de la figure principale
# ---------------------------------------------------------------------------

def plot_static(cycles: list[dict]) -> Any:
    """
    Construit la figure matplotlib avec 6 sous-graphiques et tous les widgets.
    Retourne la figure.
    """
    fig, axes = plt.subplots(2, 3, figsize=(14, 11))
    fig.patch.set_facecolor('#1a1a2e')
    total_s = P.N_CIRCULAR_CYCLES * P.CIRC_DURATION + P.N_LINEAR_CYCLES * P.LIN_DURATION_ODD
    fig.suptitle(
        f"UR5 - Protocole d'etalement cosmetique\n"
        f"{int(P.SURFACE_W)} x {int(P.SURFACE_H)} mm | 6 cycles | Durée totale = {total_s:.0f} s",
        color='white', fontsize=13, fontweight='bold', y=0.98)

    min_pts, max_pts = 50, 2000
    natural_pts = max(len(cycles[i]['pts']) for i in range(3))
    init_pts = min(max_pts, max(min_pts, natural_pts))

    waypoint_scatters: List[Any] = []
    cycle_lines: List[Any] = []
    start_markers: List[Any] = []
    end_markers: List[Any] = []
    current_circular_pts: List[Any] = [None, None, None]
    current_linear_pts: List[Any] = [None, None, None]
    current_linear_waypoint_indices: List[Any] = [None, None, None]
    linear_mode = {'triangular': False}

    for idx, (ax, cyc) in enumerate(zip(axes.flat, cycles)):
        _style_ax(ax, cyc['label'])
        pts = cyc['pts']
        if idx < 3:
            pts = _resample_points(pts, init_pts)
            current_circular_pts[idx] = pts

        line, = ax.plot(pts[:, 0], pts[:, 1], color=cyc['color'], linewidth=0.5, alpha=0.9)
        cycle_lines.append(line)

        if idx < 3:
            indices = np.arange(len(pts))
        else:
            indices = get_waypoint_indices(len(pts), cyc['type'])
            current_linear_pts[idx - 3] = pts
            current_linear_waypoint_indices[idx - 3] = get_waypoint_indices(len(pts), cyc['type'])
        sc = ax.scatter(pts[indices, 0], pts[indices, 1], c='yellow', s=20,
                        edgecolor='black', linewidth=0.3, zorder=4, visible=False)
        waypoint_scatters.append(sc)

        start_marker = ax.scatter(pts[0, 0], pts[0, 1], c='lime', s=60, zorder=5)
        end_marker = ax.scatter(pts[-1, 0], pts[-1, 1], c='red', s=60, zorder=5, marker='X')
        start_markers.append(start_marker)
        end_markers.append(end_marker)

        handles = [mpatches.Patch(color='lime', label='Depart'),
                   mpatches.Patch(color='red', label='Fin'),
                   mpatches.Patch(color='yellow', label='Waypoints')]
        ax.legend(handles=handles, fontsize=6, loc='lower right',
                  facecolor='#0f3460', edgecolor='none', labelcolor='white')

    # --- Sliders ---
    ax_slider = fig.add_axes([0.22, 0.49, 0.56, 0.022], facecolor='#0f3460')
    pts_slider = Slider(ax=ax_slider, label='Cycles 1-3 : points de discrétisation',
                        valmin=min_pts, valmax=max_pts, valinit=init_pts,
                        valstep=10, color='#3a86ff')
    pts_slider.label.set_color('white')
    pts_slider.valtext.set_color('white')

    ax_r_circle = fig.add_axes([0.22, 0.455, 0.56, 0.02], facecolor='#0f3460')
    r_circle_slider = Slider(ax=ax_r_circle, label='CIRC_R_CIRCLE',
                              valmin=0.0, valmax=10.0, valinit=float(P.CIRC_R_CIRCLE),
                              valstep=0.1, color='#ff8fab')
    r_circle_slider.label.set_color('white')
    r_circle_slider.valtext.set_color('white')

    ax_n_cycles = fig.add_axes([0.22, 0.422, 0.56, 0.02], facecolor='#0f3460')
    n_cycles_slider = Slider(ax=ax_n_cycles, label='N_CIRCULAR_CYCLES',
                              valmin=0, valmax=10, valinit=int(P.N_CIRCULAR_CYCLES),
                              valstep=1, color='#90be6d')
    n_cycles_slider.label.set_color('white')
    n_cycles_slider.valtext.set_color('white')

    ax_n_passes = fig.add_axes([0.22, 0.389, 0.56, 0.02], facecolor='#0f3460')
    n_passes_slider = Slider(ax=ax_n_passes, label='CIRC_N_PASSES',
                              valmin=0, valmax=10, valinit=int(P.CIRC_N_PASSES),
                              valstep=1, color='#f9c74f')
    n_passes_slider.label.set_color('white')
    n_passes_slider.valtext.set_color('white')

    ax_n_circles = fig.add_axes([0.22, 0.356, 0.56, 0.02], facecolor='#0f3460')
    n_circles_slider = Slider(ax=ax_n_circles, label='CIRC_N_CIRCLES',
                               valmin=1, valmax=60, valinit=int(P.CIRC_N_CIRCLES),
                               valstep=1, color='#43aa8b')
    n_circles_slider.label.set_color('white')
    n_circles_slider.valtext.set_color('white')

    # --- Callbacks sliders ---
    def _build_circular_with_params(rotation_deg, radius_mm, n_passes, n_circles):
        import design.params as _P
        old_r, old_np, old_nc = _P.CIRC_R_CIRCLE, _P.CIRC_N_PASSES, _P.CIRC_N_CIRCLES
        try:
            _P.CIRC_R_CIRCLE = float(radius_mm)
            _P.CIRC_N_PASSES = max(2, int(n_passes))
            _P.CIRC_N_CIRCLES = max(1, int(n_circles))
            return circular_cycle(rotation_deg=rotation_deg)
        finally:
            _P.CIRC_R_CIRCLE = old_r
            _P.CIRC_N_PASSES = old_np
            _P.CIRC_N_CIRCLES = old_nc

    def update_discretization(val):
        n_points = int(pts_slider.val)
        show_waypoints = waypoint_scatters[0].get_visible()
        active_cycles = min(3, max(0, int(round(n_cycles_slider.val))))
        radius = float(r_circle_slider.val)
        n_passes = int(n_passes_slider.val)
        n_circles = int(n_circles_slider.val)
        rotations = [0, 90, 0]

        for i in range(3):
            if i < active_cycles:
                pts_full = _build_circular_with_params(rotations[i], radius, n_passes, n_circles)
                pts_resampled = _resample_points(pts_full, n_points)
                cycle_lines[i].set_visible(True)
                current_circular_pts[i] = pts_resampled
            else:
                pts_resampled = np.empty((0, 3))
                cycle_lines[i].set_visible(False)
                current_circular_pts[i] = None

            if len(pts_resampled) > 0:
                cycle_lines[i].set_data(pts_resampled[:, 0], pts_resampled[:, 1])
                start_markers[i].set_offsets(np.array([[pts_resampled[0, 0], pts_resampled[0, 1]]]))
                end_markers[i].set_offsets(np.array([[pts_resampled[-1, 0], pts_resampled[-1, 1]]]))
                start_markers[i].set_visible(True)
                end_markers[i].set_visible(True)
            else:
                cycle_lines[i].set_data([], [])
                start_markers[i].set_visible(False)
                end_markers[i].set_visible(False)

            waypoint_scatters[i].remove()
            indices = np.arange(len(pts_resampled))
            waypoint_scatters[i] = axes.flat[i].scatter(
                pts_resampled[indices, 0] if len(indices) else [],
                pts_resampled[indices, 1] if len(indices) else [],
                c='yellow', s=14, edgecolor='black', linewidth=0.25,
                zorder=4, visible=show_waypoints,
            )
        fig.canvas.draw_idle()

    def _update_linear_cycles_shape(show_waypoints=None):
        if show_waypoints is None:
            show_waypoints = waypoint_scatters[0].get_visible()
        rotations = [0, 90, 0]
        for i in range(3):
            cyc_idx = i + 3
            if linear_mode['triangular']:
                pts = triangular_cycle(rotation_deg=rotations[i])
                shape_label = 'Triangulé'
            else:
                pts = linear_cycle(rotation_deg=rotations[i])
                shape_label = 'Rectiligne'

            current_linear_pts[i] = pts
            if linear_mode['triangular']:
                current_linear_waypoint_indices[i] = list(range(len(pts)))
            else:
                current_linear_waypoint_indices[i] = get_waypoint_indices(len(pts), 'linear')

            cycles[cyc_idx]['pts'] = pts
            cycles[cyc_idx]['label'] = f'Cycle {cyc_idx + 1} - {shape_label} ({rotations[i]}deg)'
            cycles[cyc_idx]['type'] = 'linear'

            cycle_lines[cyc_idx].set_data(pts[:, 0], pts[:, 1])
            axes.flat[cyc_idx].set_title(cycles[cyc_idx]['label'], color='white', fontsize=9, pad=4)
            start_markers[cyc_idx].set_offsets(np.array([[pts[0, 0], pts[0, 1]]]))
            end_markers[cyc_idx].set_offsets(np.array([[pts[-1, 0], pts[-1, 1]]]))
            start_markers[cyc_idx].set_visible(True)
            end_markers[cyc_idx].set_visible(True)

            waypoint_scatters[cyc_idx].remove()
            idx_wp = (np.arange(len(pts)) if linear_mode['triangular']
                      else current_linear_waypoint_indices[i])
            waypoint_scatters[cyc_idx] = axes.flat[cyc_idx].scatter(
                pts[idx_wp, 0], pts[idx_wp, 1],
                c='yellow', s=20, edgecolor='black', linewidth=0.3,
                zorder=4, visible=show_waypoints,
            )
        fig.canvas.draw_idle()

    pts_slider.on_changed(update_discretization)
    r_circle_slider.on_changed(update_discretization)
    n_cycles_slider.on_changed(update_discretization)
    n_passes_slider.on_changed(update_discretization)
    n_circles_slider.on_changed(update_discretization)
    update_discretization(None)

    # --- Boutons ---
    ax_btn = fig.add_axes([0.42, 0.015, 0.16, 0.04])
    btn = Button(ax_btn, 'Afficher Waypoints', color='#0f3460', hovercolor='#16213e')
    btn.label.set_color('white')
    btn.label.set_fontsize(8)

    ax_btn_shape = fig.add_axes([0.18, 0.015, 0.16, 0.04])
    btn_shape = Button(ax_btn_shape, 'Tri/Rect: Rect', color='#0f3460', hovercolor='#16213e')
    btn_shape.label.set_color('white')
    btn_shape.label.set_fontsize(8)

    ax_btn_export = fig.add_axes([0.60, 0.015, 0.20, 0.04])
    btn_export = Button(ax_btn_export, 'Exporter URScript', color='#0f3460', hovercolor='#16213e')
    btn_export.label.set_color('white')
    btn_export.label.set_fontsize(8)

    def toggle_wp(event):
        new_vis = not waypoint_scatters[0].get_visible()
        for sc in waypoint_scatters:
            sc.set_visible(new_vis)
        fig.canvas.draw_idle()

    def toggle_shape_mode(event):
        linear_mode['triangular'] = not linear_mode['triangular']
        btn_shape.label.set_text('Tri/Rect: Tri' if linear_mode['triangular'] else 'Tri/Rect: Rect')
        _update_linear_cycles_shape()

    def export_current_urscript(event):
        active_cycles = min(3, max(0, int(round(n_cycles_slider.val))))
        export_cycles = []
        for i in range(active_cycles):
            pts = current_circular_pts[i]
            if pts is None or len(pts) < 2:
                continue
            export_cycles.append({
                'label': f'Cycle {i + 1} - Circulaire (UI)',
                'color': cycles[i]['color'],
                'pts': pts,
                'type': 'circular',
                'waypoint_indices': list(range(len(pts))),
            })
        for i in range(3):
            pts_lin = current_linear_pts[i]
            wp_lin = current_linear_waypoint_indices[i]
            if pts_lin is None or wp_lin is None:
                continue
            export_cycles.append({
                'label': cycles[i + 3]['label'],
                'color': cycles[i + 3]['color'],
                'pts': pts_lin,
                'type': 'linear',
                'waypoint_indices': wp_lin,
            })
        if not export_cycles:
            print('[EXPORT] Aucun cycle à exporter.')
            return
        generate_urscript(export_cycles, filename=P.SCRIPT_PATH)
        print(f"[EXPORT] URScript généré depuis l'UI ({len(export_cycles)} cycles) -> {P.SCRIPT_PATH}")

    btn.on_clicked(toggle_wp)
    btn_shape.on_clicked(toggle_shape_mode)
    btn_export.on_clicked(export_current_urscript)

    # Références persistantes pour éviter la désactivation des boutons
    fig._toggle_btn = btn
    fig._shape_btn = btn_shape
    fig._export_btn = btn_export
    fig._pts_slider = pts_slider
    fig._r_circle_slider = r_circle_slider
    fig._n_cycles_slider = n_cycles_slider
    fig._n_passes_slider = n_passes_slider
    fig._n_circles_slider = n_circles_slider

    _update_linear_cycles_shape(show_waypoints=False)

    # --- Overlay live TCP (IPC UDP) ---
    live_tcp_scatters: List[Any] = []
    live_tcp_trails: List[Any] = []
    for ax in axes.flat:
        ln, = ax.plot([], [], color='cyan', lw=1.3, alpha=0.85, zorder=5)
        live_tcp_trails.append(ln)
        sc_live = ax.scatter([], [], c='cyan', s=140, edgecolor='black',
                             linewidth=1.0, zorder=6, visible=False, marker='*')
        live_tcp_scatters.append(sc_live)

    poll_callback = build_ipc_overlay(fig, axes, live_tcp_scatters, live_tcp_trails)
    live_timer = fig.canvas.new_timer(interval=20)
    live_timer.add_callback(poll_callback)
    live_timer.start()
    fig._live_timer = live_timer
    fig._live_tcp_scatters = live_tcp_scatters

    # --- Layout ---
    def _apply_ui_layout():
        fig.subplots_adjust(left=0.06, right=0.98, top=0.93, bottom=0.10,
                            wspace=0.22, hspace=0.62)
        button_y, button_h = 0.005, 0.045
        ax_btn_shape.set_position([0.04, button_y, 0.28, button_h])
        ax_btn.set_position([0.36, button_y, 0.28, button_h])
        ax_btn_export.set_position([0.68, button_y, 0.28, button_h])
        bottom_row_y, bottom_row_h = 0.07, 0.22
        for ax in axes[1, :]:
            pos = ax.get_position()
            ax.set_position([pos.x0, bottom_row_y, pos.width, bottom_row_h])

    _apply_ui_layout()
    fig._layout_resize_cid = fig.canvas.mpl_connect(
        'resize_event', lambda e: (_apply_ui_layout(), fig.canvas.draw_idle())
    )
    return fig


def plot_overlay(cycles: list[dict]) -> Any:
    """Vue superposée — couverture globale de tous les cycles."""
    fig, ax = plt.subplots(figsize=(7, 7))
    fig.patch.set_facecolor('#1a1a2e')
    _style_ax(ax, "Vue superposée - couverture globale")
    for cyc in cycles:
        pts = cyc['pts']
        ax.plot(pts[:, 0], pts[:, 1], color=cyc['color'],
                linewidth=0.5, alpha=0.6, label=cyc['label'])
    ax.legend(fontsize=7, facecolor='#0f3460', edgecolor='none',
              labelcolor='white', loc='upper left')
    plt.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Entrée principale
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(description="UR5 étalement - simulation & export")
    parser.add_argument('--export', action='store_true', help='Exporter le URScript (.script)')
    parser.add_argument('--export-urp', action='store_true', help='Exporter le programme PolyScope (.urp)')
    parser.add_argument('--no-show', action='store_true', help='Ne pas afficher les graphiques')
    args = parser.parse_args()

    cycles = build_full_trajectory()
    exit_code = 0

    if args.export:
        if not generate_urscript(cycles):
            exit_code = 1
    if args.export_urp:
        if not generate_urp(cycles):
            exit_code = 1

    if not args.no_show:
        plot_static(cycles)
        plot_overlay(cycles)
        plt.show()

    return exit_code


if __name__ == '__main__':
    import sys
    sys.exit(main() or 0)
