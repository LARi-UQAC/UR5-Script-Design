"""
design/live_ipc.py — Réception UDP et overlay live TCP du simulateur.

Responsabilités :
  - Ouvre le socket UDP non-bloquant sur TCP_LIVE_PORT (loopback seulement).
  - Draine les datagrammes à chaque tick du timer matplotlib.
  - Convertit les coordonnées monde (m) reçues du viewer en coordonnées
    plaque (mm) via l'inverse de plate_to_robot().
  - Met à jour les marqueurs étoile et les traces sur les sous-graphiques.

Le module ne contient pas de référence directe à `fig` ou `axes` : la
fonction `build_ipc_overlay()` retourne un callable à passer au timer
matplotlib de l'application.
"""

from __future__ import annotations

import json
import re
import socket
import time as _time
from typing import Any

import numpy as np

from design.geometry import _pose_inv, _pose_trans
from design.params import (
    ROBOT_BASE_ROTATION_DEG,
    ROBOT_RX, ROBOT_RY, ROBOT_RZ,
    ROBOT_X_ORIGIN, ROBOT_Y_ORIGIN, ROBOT_Z_SURFACE,
    P_REF,
    SCRIPT_PATH,
)
from ur5_sim.ipc_config import TCP_LIVE_HOST, TCP_LIVE_MAX_BYTES, TCP_LIVE_PORT


# ---------------------------------------------------------------------------
# Conversion monde -> plaque
# ---------------------------------------------------------------------------

def _world_to_plate_mm(wx_m: float, wy_m: float) -> tuple[float, float]:
    """
    Inverse de plate_to_robot() + _abs_pose().
    Récupère les coordonnées plate-frame (mm) depuis une pose monde (m).
    """
    p_anchor_old = [
        ROBOT_X_ORIGIN, ROBOT_Y_ORIGIN, ROBOT_Z_SURFACE,
        ROBOT_RX, ROBOT_RY, ROBOT_RZ,
    ]
    world_pose = [wx_m, wy_m, ROBOT_Z_SURFACE, ROBOT_RX, ROBOT_RY, ROBOT_RZ]
    p_orig = _pose_trans(p_anchor_old, _pose_trans(_pose_inv(P_REF), world_pose))
    rx_m, ry_m = p_orig[0], p_orig[1]
    a = np.radians(ROBOT_BASE_ROTATION_DEG)
    dx = rx_m - ROBOT_X_ORIGIN
    dy = ry_m - ROBOT_Y_ORIGIN
    plate_x_m = dx * np.cos(a) + dy * np.sin(a)
    plate_y_m = -dx * np.sin(a) + dy * np.cos(a)
    return plate_x_m * 1000.0, plate_y_m * 1000.0


# ---------------------------------------------------------------------------
# Mapping runtime cycle_N -> sous-graphique logique
# ---------------------------------------------------------------------------

def _build_cycle_slot_map(script_path) -> dict[int, int]:
    """
    Parse etalement.script pour mapper chaque def cycle_N() -> sous-graphique.
    """
    try:
        with open(script_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except OSError:
        return {}

    fn_re = re.compile(r'^\s*def\s+cycle_(\d+)\s*\(\s*\)\s*:')
    lbl_re = re.compile(r'Cycle\s+(\d+)\b', re.IGNORECASE)
    out: dict[int, int] = {}
    current_runtime_idx = None

    for line in lines:
        m_fn = fn_re.match(line)
        if m_fn:
            current_runtime_idx = int(m_fn.group(1))
            continue
        if current_runtime_idx is None:
            continue
        m_lbl = lbl_re.search(line)
        if m_lbl:
            logical_cycle = int(m_lbl.group(1))
            subplot_idx = logical_cycle - 1
            out[current_runtime_idx] = subplot_idx
            current_runtime_idx = None

    return out


# ---------------------------------------------------------------------------
# Overlay live : fonction principale
# ---------------------------------------------------------------------------

def build_ipc_overlay(
    fig: Any,
    axes: Any,
    live_tcp_scatters: list[Any],
    live_tcp_trails: list[Any],
) -> Any:
    """
    Construit et retourne le callable à passer au timer matplotlib.

    Paramètres
    ----------
    fig              : figure matplotlib
    axes             : tableau 2D d'axes (2x3)
    live_tcp_scatters : liste de scatter (un par sous-graphique)
    live_tcp_trails   : liste de Line2D (un par sous-graphique)

    Retourne
    --------
    poll_callback : callable() à appeler depuis le timer (interval=20 ms)
    """
    trail_buffers: list[tuple[list, list]] = [([], []) for _ in range(len(live_tcp_scatters))]
    poll_state: dict[str, Any] = {
        'last_cycle': 0,
        'last_frame': -1,
        'last_running': False,
    }
    cycle_slot_cache: dict[str, Any] = {'mtime': None, 'map': {}}
    ipc_state: dict[str, Any] = {'sock': None, 'bind_failed': False}

    def _ensure_socket() -> socket.socket | None:
        if ipc_state['sock'] is not None or ipc_state['bind_failed']:
            return ipc_state['sock']
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((TCP_LIVE_HOST, TCP_LIVE_PORT))
            sock.setblocking(False)
            ipc_state['sock'] = sock
        except OSError as exc:
            ipc_state['bind_failed'] = True
            print(f"[live_ipc] UDP bind {TCP_LIVE_HOST}:{TCP_LIVE_PORT} "
                  f"échoué ({exc!r}); overlay live désactivé.")
        return ipc_state['sock']

    def _refresh_slot_map() -> None:
        try:
            mtime = SCRIPT_PATH.stat().st_mtime
        except OSError:
            cycle_slot_cache['mtime'] = None
            cycle_slot_cache['map'] = {}
            return
        if cycle_slot_cache['mtime'] == mtime:
            return
        cycle_slot_cache['map'] = _build_cycle_slot_map(SCRIPT_PATH)
        cycle_slot_cache['mtime'] = mtime

    def _reset_trails() -> None:
        for i in range(len(live_tcp_trails)):
            trail_buffers[i] = ([], [])
            live_tcp_trails[i].set_data([], [])

    def _hide_all() -> None:
        changed = False
        for sc in live_tcp_scatters:
            if sc.get_visible():
                sc.set_visible(False)
                changed = True
        if changed:
            fig.canvas.draw_idle()

    def poll_callback() -> None:
        _refresh_slot_map()
        sock = _ensure_socket()
        if sock is None:
            _hide_all()
            return

        last_payload = None
        while True:
            try:
                data_bytes, _addr = sock.recvfrom(TCP_LIVE_MAX_BYTES)
                last_payload = data_bytes
            except BlockingIOError:
                break
            except OSError:
                break
        if last_payload is None:
            return

        try:
            data = json.loads(last_payload.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return

        running = bool(data.get('running', False))
        ts = float(data.get('ts', 0.0))
        stale = (_time.time() - ts) > 1.5
        cycle = int(data.get('cycle', 0))
        frame = int(data.get('frame', 0))

        if (
            running
            and poll_state['last_running']
            and frame < poll_state['last_frame']
            and frame != 0
        ):
            return

        if (not poll_state['last_running']) and running:
            _reset_trails()
        elif running and frame == 0 and poll_state['last_frame'] > 0:
            _reset_trails()

        poll_state['last_running'] = running
        poll_state['last_frame'] = frame

        if (not running) or stale or cycle < 1 or cycle > len(live_tcp_scatters):
            _hide_all()
            poll_state['last_cycle'] = cycle
            return

        rx = float(data.get('x_anchor_m', 0.0))
        ry = float(data.get('y_anchor_m', 0.0))
        px_mm, py_mm = _world_to_plate_mm(rx, ry)
        sub_idx = cycle_slot_cache['map'].get(cycle, cycle - 1)
        if sub_idx < 0 or sub_idx >= len(live_tcp_scatters):
            _hide_all()
            poll_state['last_cycle'] = cycle
            return

        trail_anchor_m = data.get('trail_anchor_m')
        if isinstance(trail_anchor_m, list) and len(trail_anchor_m) > 0:
            xs_mm, ys_mm = [], []
            for pair in trail_anchor_m:
                if not isinstance(pair, (list, tuple)) or len(pair) < 2:
                    continue
                try:
                    tx_mm, ty_mm = _world_to_plate_mm(float(pair[0]), float(pair[1]))
                    xs_mm.append(tx_mm)
                    ys_mm.append(ty_mm)
                except (TypeError, ValueError):
                    continue
            if xs_mm:
                trail_buffers[sub_idx] = (xs_mm, ys_mm)
                live_tcp_trails[sub_idx].set_data(xs_mm, ys_mm)
                px_mm, py_mm = xs_mm[-1], ys_mm[-1]
        else:
            trail_buffers[sub_idx][0].append(px_mm)
            trail_buffers[sub_idx][1].append(py_mm)
            live_tcp_trails[sub_idx].set_data(
                trail_buffers[sub_idx][0], trail_buffers[sub_idx][1]
            )

        for i, sc in enumerate(live_tcp_scatters):
            if i == sub_idx:
                sc.set_offsets(np.array([[px_mm, py_mm]]))
                sc.set_visible(True)
            elif sc.get_visible():
                sc.set_visible(False)

        poll_state['last_cycle'] = cycle
        fig.canvas.draw_idle()

    return poll_callback
