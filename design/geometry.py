"""
design/geometry.py — Transformations SE(3) et conversion coordonnées.

Contient les primitives partagées entre le générateur de trajectoire,
l'export URScript et l'IPC live :
  - plate_to_robot()   : plate-frame (mm) -> robot base frame (m)
  - _abs_pose()        : équivalent Python de T(p_orig) URScript
  - _rotvec_to_matrix(), _matrix_to_rotvec()  : rotation-vector <-> SO(3)
  - _pose_trans(), _pose_inv()                : primitives URScript
  - _fmt_pose(), _fmt_raw_pose()              : formatage littéral URScript
  - mm_to_m()                                 : conversion d'unité
"""

from __future__ import annotations

import numpy as np

from design.params import (
    P_REF,
    ROBOT_BASE_ROTATION_DEG,
    ROBOT_RX, ROBOT_RY, ROBOT_RZ,
    ROBOT_X_ORIGIN, ROBOT_Y_ORIGIN, ROBOT_Z_SURFACE,
)


def mm_to_m(v: float) -> float:
    return round(v / 1000.0, 6)


def plate_to_robot(x_mm: float, y_mm: float) -> tuple[float, float]:
    """
    Convertit des coordonnées plaque (mm) en coordonnées repère robot (m).
    Applique ROBOT_BASE_ROTATION_DEG autour de l'origine plaque, puis ajoute
    ROBOT_X/Y_ORIGIN.
    """
    angle = np.radians(ROBOT_BASE_ROTATION_DEG)
    dx = mm_to_m(x_mm)
    dy = mm_to_m(y_mm)
    rx = ROBOT_X_ORIGIN + dx * np.cos(angle) - dy * np.sin(angle)
    ry = ROBOT_Y_ORIGIN + dx * np.sin(angle) + dy * np.cos(angle)
    return round(rx, 6), round(ry, 6)


# ---------------------------------------------------------------------------
# Primitives rotation-vector (équivalent Python de pose_trans / pose_inv)
# ---------------------------------------------------------------------------

def _rotvec_to_matrix(rv: list[float] | np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rv))
    if theta < 1e-12:
        return np.eye(3)
    k = np.asarray(rv, dtype=float) / theta
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def _matrix_to_rotvec(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float)
    cos_theta = (np.trace(R) - 1.0) / 2.0
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = float(np.arccos(cos_theta))
    if theta < 1e-12:
        return np.zeros(3)
    if abs(theta - np.pi) < 1e-9:
        diag = np.array([R[0, 0], R[1, 1], R[2, 2]])
        i = int(np.argmax(diag))
        axis = np.zeros(3)
        axis[i] = np.sqrt(max(0.0, (R[i, i] + 1.0) / 2.0))
        for j in range(3):
            if j != i and axis[i] > 1e-12:
                axis[j] = R[i, j] / (2.0 * axis[i])
        return axis * theta
    sin_theta = np.sin(theta)
    rx = (R[2, 1] - R[1, 2]) / (2.0 * sin_theta)
    ry = (R[0, 2] - R[2, 0]) / (2.0 * sin_theta)
    rz = (R[1, 0] - R[0, 1]) / (2.0 * sin_theta)
    return np.array([rx, ry, rz]) * theta


def _pose_trans(
    a: list[float] | np.ndarray,
    b: list[float] | np.ndarray,
) -> list[float]:
    ta = np.asarray(a[:3], dtype=float)
    tb = np.asarray(b[:3], dtype=float)
    Ra = _rotvec_to_matrix(a[3:])
    Rb = _rotvec_to_matrix(b[3:])
    R = Ra @ Rb
    t = ta + Ra @ tb
    return list(t) + list(_matrix_to_rotvec(R))


def _pose_inv(a: list[float] | np.ndarray) -> list[float]:
    ta = np.asarray(a[:3], dtype=float)
    Ra = _rotvec_to_matrix(a[3:])
    R_inv = Ra.T
    t_inv = -R_inv @ ta
    return list(t_inv) + list(_matrix_to_rotvec(R_inv))


def _abs_pose(p_orig: list[float] | np.ndarray) -> list[float]:
    """
    Équivalent du T(p_orig) URScript :
        pose_trans(P_REF, pose_trans(pose_inv(P_ANCHOR_OLD), p_orig))
    P_ANCHOR_OLD est l'ancre nominale = pose des constantes ROBOT_* d'origine plaque.
    """
    p_anchor_old = [
        ROBOT_X_ORIGIN, ROBOT_Y_ORIGIN, ROBOT_Z_SURFACE,
        ROBOT_RX, ROBOT_RY, ROBOT_RZ,
    ]
    return _pose_trans(P_REF, _pose_trans(_pose_inv(p_anchor_old), list(p_orig)))


def _fmt_raw_pose(p6: list[float]) -> str:
    """Formate une pose absolue déjà calculée comme littéral URScript p[...]."""
    return 'p[' + ', '.join(f'{v:.6f}' for v in p6) + ']'


def _fmt_pose(p_orig: list[float]) -> str:
    """Compose vers la pose absolue et formate comme littéral URScript p[...]."""
    abs_pose = _abs_pose(p_orig)
    return _fmt_raw_pose(abs_pose)
