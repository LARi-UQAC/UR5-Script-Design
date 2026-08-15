"""
design/trajectory.py — Générateurs de trajectoire pour le protocole d'étalement.

Fonctions exportées :
  - circular_cycle()    : boustrophedon avec épicycloïde continue
  - linear_cycle()      : passes rectilignes en boustrophedon
  - triangular_cycle()  : variante réduite des cycles rectilignes
  - build_full_trajectory() : assemble les 6 cycles dans l'ordre protocole
  - get_waypoint_indices()  : calcule les indices pour l'export URScript
  - rotate_points()     : rotation 2D autour du centre de la surface
"""

from __future__ import annotations

import numpy as np

from design.params import (
    LIN_N_PASSES,
    LIN_N_POINTS_PER_SEGMENT,
    N_LINEAR_CYCLES,
    Z_CONTACT,
)
from design.settings import get_settings


def get_waypoint_indices(total_pts_count: int, cycle_type: str) -> list[int]:
    """
    Calcule les indices des points exportés vers l'URScript.
    Pour les cycles linéaires, retourne les points aux coins (début/fin de passe).
    Pour les cycles circulaires, applique le mode de densité réglé :
    'subsample' sous-échantillonne à URSCRIPT_N_WAYPOINTS_CIRCULAR (défaut,
    comportement de l'export headless), 'all' garde tous les points du tracé
    (comportement historique de l'export depuis l'interface).
    """
    if cycle_type == 'linear':
        pts_per_pass = max(1, int(total_pts_count // max(1, LIN_N_PASSES)))
        indices: set[int] = set()
        for i in range(LIN_N_PASSES):
            start_idx = min(i * pts_per_pass, total_pts_count - 1)
            end_idx = min((i + 1) * pts_per_pass - 1, total_pts_count - 1)
            indices.add(start_idx)
            indices.add(end_idx)
        return sorted(list(indices))
    cfg = get_settings()
    if cfg.circular_waypoint_mode == 'all':
        return list(range(total_pts_count))
    step = max(1, total_pts_count // max(1, cfg.urscript_n_waypoints_circular))
    return sorted(list(set(
        [0] + list(range(step, total_pts_count, step)) + [total_pts_count - 1]
    )))


def rotate_points(
    pts: np.ndarray,
    angle_deg: float,
    cx: float | None = None,
    cy: float | None = None,
) -> np.ndarray:
    """Rotation 2D autour du centre de la surface."""
    cfg = get_settings()
    if cx is None:
        cx = cfg.surface_w / 2.0
    if cy is None:
        cy = cfg.surface_h / 2.0
    a = np.radians(angle_deg)
    c, s = np.cos(a), np.sin(a)
    x, y = pts[:, 0] - cx, pts[:, 1] - cy
    return np.column_stack([c * x - s * y + cx, s * x + c * y + cy])


def circular_cycle(rotation_deg: float = 0, n_pts_pass: int = 600, n_pts_turn: int = 100) -> np.ndarray:
    """
    Boustrophedon avec épicycloïde continue + sections droites début/fin.

    Structure :
      [section droite bas]  carrier descend de y_start -> y_bot
      [passes en U]         n_passes verticales reliées par demi-tours
      [section droite bas]  carrier remonte de y_bot -> y_start
    """
    # Lu a l'appel, pour que les reglages de l'interface prennent effet
    # (plan, section 2 : un import par valeur figerait ces constantes).
    cfg = get_settings()
    R = cfg.circ_r_circle
    n_passes = max(2, int(cfg.circ_n_passes))

    x_first = R
    x_last = cfg.surface_w - R
    xs = np.linspace(x_first, x_last, n_passes)
    spacing = xs[1] - xs[0]
    r_turn = spacing / 2.0

    y_bot = r_turn + R
    y_top = cfg.surface_h - r_turn - R
    # Guard: when passes are few and R is large the turns fill the full height,
    # making y_top <= y_bot (degenerate). Give a minimal drawable range.
    if y_top <= y_bot:
        y_top = y_bot + 1.0
    y_start = cfg.circ_y_start
    n_straight = (
        max(2, int(n_pts_pass * abs(y_bot - y_start) / (y_top - y_bot)))
        if y_start < y_bot else 2
    )

    carrier_x, carrier_y = [], []

    if y_start < y_bot:
        ys_init = np.linspace(y_start, y_bot, n_straight)
        carrier_x.append(np.full(n_straight, xs[0]))
        carrier_y.append(ys_init)

    for i in range(n_passes):
        ys = (
            np.linspace(y_bot, y_top, n_pts_pass) if i % 2 == 0
            else np.linspace(y_top, y_bot, n_pts_pass)
        )
        carrier_x.append(np.full(n_pts_pass, xs[i]))
        carrier_y.append(ys)

        if i < n_passes - 1:
            cx_turn = (xs[i] + xs[i + 1]) / 2.0
            if i % 2 == 0:
                angles = np.linspace(np.pi, 0, n_pts_turn)
                ty = y_top + r_turn * np.sin(angles)
            else:
                angles = np.linspace(np.pi, 2 * np.pi, n_pts_turn)
                ty = y_bot + r_turn * np.sin(angles)
            carrier_x.append(cx_turn + r_turn * np.cos(angles))
            carrier_y.append(ty)

    if y_start < y_bot:
        ys_final = np.linspace(y_bot, y_start, n_straight)
        carrier_x.append(np.full(n_straight, xs[-1]))
        carrier_y.append(ys_final)

    carrier_x_arr = np.concatenate(carrier_x)
    carrier_y_arr = np.concatenate(carrier_y)

    dx = np.diff(carrier_x_arr, prepend=carrier_x_arr[0])
    dy = np.diff(carrier_y_arr, prepend=carrier_y_arr[0])
    ds = np.sqrt(dx ** 2 + dy ** 2)
    s = np.cumsum(ds)
    s_norm = s / s[-1]
    total_turns = max(1, int(cfg.circ_n_circles)) * n_passes
    theta = 2 * np.pi * total_turns * s_norm
    x = carrier_x_arr + R * np.cos(theta)
    y = carrier_y_arr + R * np.sin(theta)

    pts = np.column_stack([x, y])
    if rotation_deg != 0:
        pts = rotate_points(pts, rotation_deg)
    return np.column_stack([pts, np.full(len(pts), Z_CONTACT)])


def linear_cycle(rotation_deg: float = 0) -> np.ndarray:
    """LIN_N_PASSES allers-retours rectilignes en boustrophedon."""
    cfg = get_settings()
    n_pts = LIN_N_POINTS_PER_SEGMENT
    positions_y = np.linspace(cfg.margin, cfg.surface_h - cfg.margin, LIN_N_PASSES)
    all_pts = []
    for i, y in enumerate(positions_y):
        xs = (
            np.linspace(cfg.margin, cfg.surface_w - cfg.margin, n_pts) if i % 2 == 0
            else np.linspace(cfg.surface_w - cfg.margin, cfg.margin, n_pts)
        )
        all_pts.append(np.column_stack([xs, np.full(n_pts, y)]))
    pts = np.vstack(all_pts)
    if rotation_deg != 0:
        pts = rotate_points(pts, rotation_deg)
    return np.column_stack([pts, np.full(len(pts), Z_CONTACT)])


def triangular_cycle(rotation_deg: float = 0) -> np.ndarray:
    """
    Variante "triangulée" : réduit les cycles rectilignes en conservant
    uniquement le point milieu une fois sur deux parmi les coins.
    """
    rect_pts = np.asarray(linear_cycle(rotation_deg=0), dtype=float)
    n_rect = int(rect_pts.shape[0])
    corner_idx = np.asarray(get_waypoint_indices(n_rect, 'linear'), dtype=int)
    corners = rect_pts[corner_idx]
    n_corners = int(corners.shape[0])

    if n_corners <= 2:
        pts = corners.copy()
    else:
        rows = [corners[0]]
        j = 1
        while j + 1 < n_corners:
            rows.append((corners[j] + corners[j + 1]) / 2.0)
            j += 2
        if j < n_corners - 1:
            rows.append(corners[j])
        rows.append(corners[-1])
        pts = np.asarray(rows, dtype=float)

    if rotation_deg != 0:
        xy = rotate_points(pts[:, :2], rotation_deg)
        pts = np.column_stack([xy, pts[:, 2]])
    return pts


def build_full_trajectory() -> list[dict]:
    """Assemble les cycles dans l'ordre du protocole."""
    cfg = get_settings()
    rotations = [0, 90, 0]
    colors_circ = ['#1f77b4', '#ff7f0e', '#2ca02c']
    colors_lin = ['#d62728', '#9467bd', '#8c564b']

    # Les listes de rotations et de couleurs sont dimensionnees pour les trois
    # cycles du protocole. Le nombre de cycles circulaires etant reglable
    # (0 a 10), on cycle dessus au lieu d'indexer directement, sinon un reglage
    # au-dela de 3 leve un IndexError.
    n_circ = max(0, int(cfg.n_circular_cycles))
    cycles = []
    for i in range(n_circ):
        rot = rotations[i % len(rotations)]
        cycles.append({
            'label': f'Cycle {i + 1} - Circulaire ({rot}deg)',
            'color': colors_circ[i % len(colors_circ)],
            'pts': circular_cycle(rotation_deg=rot),
            'type': 'circular',
        })
    for i in range(N_LINEAR_CYCLES):
        rot = rotations[i % len(rotations)]
        cycles.append({
            'label': f'Cycle {n_circ + i + 1} - Rectiligne ({rot}deg)',
            'color': colors_lin[i % len(colors_lin)],
            'pts': linear_cycle(rotation_deg=rot),
            'type': 'linear',
        })
    return cycles
