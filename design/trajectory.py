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
    CIRC_N_CIRCLES,
    CIRC_N_PASSES,
    CIRC_R_CIRCLE,
    CIRC_Y_START,
    LIN_N_PASSES,
    LIN_N_POINTS_PER_SEGMENT,
    MARGIN,
    N_CIRCULAR_CYCLES,
    N_LINEAR_CYCLES,
    SURFACE_H,
    SURFACE_W,
    URSCRIPT_N_WAYPOINTS_CIRCULAR,
    Z_CONTACT,
)


def get_waypoint_indices(total_pts_count: int, cycle_type: str) -> list[int]:
    """
    Calcule les indices des points exportés vers l'URScript.
    Pour les cycles linéaires, retourne les points aux coins (début/fin de passe).
    Pour les cycles circulaires, sous-échantillonne à URSCRIPT_N_WAYPOINTS_CIRCULAR.
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
    else:
        step = max(1, total_pts_count // URSCRIPT_N_WAYPOINTS_CIRCULAR)
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
    if cx is None:
        cx = SURFACE_W / 2.0
    if cy is None:
        cy = SURFACE_H / 2.0
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
    # Read at call time so that app.py slider patches to design.params take effect.
    import design.params as _P
    R = _P.CIRC_R_CIRCLE
    n_passes = _P.CIRC_N_PASSES

    x_first = R
    x_last = SURFACE_W - R
    xs = np.linspace(x_first, x_last, n_passes)
    spacing = xs[1] - xs[0]
    r_turn = spacing / 2.0

    y_bot = r_turn + R
    y_top = SURFACE_H - r_turn - R
    # Guard: when passes are few and R is large the turns fill the full height,
    # making y_top <= y_bot (degenerate). Give a minimal drawable range.
    if y_top <= y_bot:
        y_top = y_bot + 1.0
    y_start = CIRC_Y_START
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
    total_turns = _P.CIRC_N_CIRCLES * n_passes
    theta = 2 * np.pi * total_turns * s_norm
    x = carrier_x_arr + R * np.cos(theta)
    y = carrier_y_arr + R * np.sin(theta)

    pts = np.column_stack([x, y])
    if rotation_deg != 0:
        pts = rotate_points(pts, rotation_deg)
    return np.column_stack([pts, np.full(len(pts), Z_CONTACT)])


def linear_cycle(rotation_deg: float = 0) -> np.ndarray:
    """LIN_N_PASSES allers-retours rectilignes en boustrophedon."""
    n_pts = LIN_N_POINTS_PER_SEGMENT
    positions_y = np.linspace(MARGIN, SURFACE_H - MARGIN, LIN_N_PASSES)
    all_pts = []
    for i, y in enumerate(positions_y):
        xs = (
            np.linspace(MARGIN, SURFACE_W - MARGIN, n_pts) if i % 2 == 0
            else np.linspace(SURFACE_W - MARGIN, MARGIN, n_pts)
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
    """Assemble les 6 cycles dans l'ordre du protocole."""
    rotations = [0, 90, 0]
    colors_circ = ['#1f77b4', '#ff7f0e', '#2ca02c']
    colors_lin = ['#d62728', '#9467bd', '#8c564b']

    cycles = []
    for i in range(N_CIRCULAR_CYCLES):
        cycles.append({
            'label': f'Cycle {i + 1} - Circulaire ({rotations[i]}deg)',
            'color': colors_circ[i],
            'pts': circular_cycle(rotation_deg=rotations[i]),
            'type': 'circular',
        })
    for i in range(N_LINEAR_CYCLES):
        cycles.append({
            'label': f'Cycle {i + 4} - Rectiligne ({rotations[i]}deg)',
            'color': colors_lin[i],
            'pts': linear_cycle(rotation_deg=rotations[i]),
            'type': 'linear',
        })
    return cycles
