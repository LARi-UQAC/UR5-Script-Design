"""
ur5_sim/visualization/ipc_live.py - UDP loopback IPC vers le design UI.

Contrat : un datagramme JSON par frame sur ``127.0.0.1:47811``
(``ur5_sim/ipc_config.py``), tolérant à la perte, seule la dernière frame
compte. Le fichier ``tcp_live/tcp_live.json`` que cela remplaçait est retiré.

Le socket et ``send_tcp_live`` vivaient dans ``swift_scene.py``, la
construction du payload dans ``viewer.render_frame`` : ni l'un ni l'autre
n'a de rapport avec Swift ou avec matplotlib, d'où ce module. Les deux noms
historiques restent importables depuis ``swift_scene`` (ré-export).

Dépendances : bibliothèque standard plus ``ur5_sim.config``.
"""

from __future__ import annotations

import json
import socket
import time

from ur5_sim.config import FORCE_Z_TARGET_N
from ur5_sim.ipc_config import TCP_LIVE_HOST, TCP_LIVE_PORT

_TCP_LIVE_SOCKET: socket.socket | None = None


def get_tcp_live_socket() -> socket.socket:
    """Retourne le socket UDP sortant (créé au premier appel)."""
    global _TCP_LIVE_SOCKET
    if _TCP_LIVE_SOCKET is None:
        _TCP_LIVE_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _TCP_LIVE_SOCKET.setblocking(False)
    return _TCP_LIVE_SOCKET


def send_tcp_live(payload: dict) -> None:
    """Envoie ``payload`` en JSON sur le socket UDP loopback."""
    try:
        sock = get_tcp_live_socket()
        sock.sendto(
            json.dumps(payload).encode("utf-8"),
            (TCP_LIVE_HOST, TCP_LIVE_PORT),
        )
    except OSError:
        pass


def build_tcp_live_payload(
    running: bool,
    frame: int,
    n_frames: int,
    xs,
    ys,
    zs,
    cycle_per_frame: list[int],
    plate_xy_per_frame: list[tuple[float, float]],
    cycle_indices,
    n_cycles_detected: int,
    in_contact: bool,
    surface_depth_m: float,
) -> dict:
    """
    --------------------------------------------------------------------------
    Purpose:
        Assemble the per-frame status datagram the design UI consumes.

    Inputs:
        running (bool): playback state flag.
        frame (int): index of the frame being painted.
        n_frames (int): trajectory length.
        xs, ys, zs (np.ndarray): world TCP coordinates, metres.
        cycle_per_frame (list[int]): 1-based URScript cycle per frame.
        plate_xy_per_frame (list[tuple]): script pose in the P_ANCHOR_OLD frame.
        cycle_indices (list | None): frame indices grouped by cycle.
        n_cycles_detected (int): number of cycles found in the script.
        in_contact (bool): inside a force_mode block.
        surface_depth_m (float): signed distance to the plate, metres.

    Outputs:
        payload (dict): JSON-serialisable frame status.
    --------------------------------------------------------------------------
    """
    cycle_idx = cycle_per_frame[frame] if frame < len(cycle_per_frame) else 0
    plate_xy = (
        plate_xy_per_frame[frame]
        if frame < len(plate_xy_per_frame)
        else (0.0, 0.0)
    )
    trail_anchor_m = []
    if cycle_indices is not None and cycle_idx < len(cycle_indices):
        cycle_visible = cycle_indices[cycle_idx]
        cycle_visible = cycle_visible[cycle_visible <= frame]
        trail_anchor_m = [
            [
                float(plate_xy_per_frame[int(k)][0]),
                float(plate_xy_per_frame[int(k)][1]),
            ]
            for k in cycle_visible
            if int(k) < len(plate_xy_per_frame)
        ]
    return {
        "running": bool(running),
        "cycle": int(cycle_idx),
        "frame": int(frame),
        "n_frames": int(n_frames),
        # World TCP from forward kinematics (P_REF frame, m).
        "x_world": float(xs[frame]),
        "y_world": float(ys[frame]),
        "z_world": float(zs[frame]),
        # Script pose in P_ANCHOR_OLD frame (m). The design UI
        # inverts plate_to_robot() on these to recover plate mm.
        "x_anchor_m": float(plate_xy[0]),
        "y_anchor_m": float(plate_xy[1]),
        "trail_anchor_m": trail_anchor_m,
        "n_cycles": int(n_cycles_detected),
        # Regulateur de force / surface (surrogate cinematique).
        "in_contact": bool(in_contact),
        "force_z_n": float(FORCE_Z_TARGET_N if in_contact else 0.0),
        "surface_depth_mm": float(surface_depth_m * 1000.0),
        "ts": time.time(),
    }


def send_tcp_live_idle(n_frames: int, n_cycles: int) -> None:
    """Flag the IPC as stopped so the design UI hides its live marker.

    Sent when the matplotlib window closes, even mid-cycle.
    """
    send_tcp_live({
        "running": False,
        "cycle": 0,
        "frame": 0,
        "n_frames": int(n_frames),
        "x_world": 0.0,
        "y_world": 0.0,
        "z_world": 0.0,
        "x_anchor_m": 0.0,
        "y_anchor_m": 0.0,
        "trail_anchor_m": [],
        "n_cycles": int(n_cycles),
        "in_contact": False,
        "force_z_n": 0.0,
        "surface_depth_mm": 0.0,
        "ts": time.time(),
    })
