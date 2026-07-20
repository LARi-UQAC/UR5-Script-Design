"""Velocity-faithful densification of URScript motion segments.

The URScript parser emits one :class:`~ur5_sim.parsing.urscript.MotionSegment`
per ``movel`` / ``movej`` call. Each segment carries a target pose plus the
linear/angular speed argument (``v=``) declared on the line. The viewer
advances one pose per fixed ``DT`` (see :mod:`ur5_sim.config`) so playing the
raw segment list ignores the declared velocity and shows the trajectory at
``segment_distance / DT`` instead of the real wall-clock pace.

This module bridges the two: it subdivides each segment into ``DT``-sized
substeps via SE(3) slerp, so feeding the densified pose list through the
existing IK + viewer pipeline yields an animation whose apparent TCP velocity
matches what PolyScope would execute on the real UR5. Per-segment durations
are clamped to ``URSCRIPT_MAX_TCP_SPEED_MPS`` (the PolyScope cap) and any
violation is reported as a ``SEGMENT_VELOCITY_EXCEEDED`` event for the text
report; segments whose ``v=`` could not be resolved emit
``SEGMENT_VELOCITY_UNKNOWN`` and fall back to the cap.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
from spatialmath import SE3

from ur5_sim.parsing.urscript import MotionSegment, urscript_pose

# Mouvement movej (joint-space) sans IK prealable : la distance articulaire
# n'est pas connue ici. Le simulateur traite movej comme une duree forfaitaire
# pour conserver un ordre de grandeur realiste (3-5 s sur le robot reel).
MOVEJ_NOMINAL_DURATION_S: float = 2.0
# Borne inferieure sur la duree d'un segment movel : segments quasi-stationnaires
# (distance ~ 0) ne doivent pas produire 0 substep ni inflate le buffer.
SEGMENT_MIN_DURATION_S: float = 1e-4


def interp_se3(a: SE3, b: SE3, t: float) -> SE3:
    """Interpolate between two SE3 poses at fraction ``t`` in [0, 1].

    Delegates to :meth:`spatialmath.SE3.interp` which slerp's the rotation
    component and lerp's the translation. Used by :func:`densify_segments`
    to sample intermediate frames along each motion segment.
    """
    return a.interp(b, t)


def _segment_duration(
    seg: MotionSegment,
    distance_m: float,
    v_cap_mps: float,
) -> tuple[float, Optional[str]]:
    """Return ``(t_seg, event_kind_or_None)`` for one motion segment.

    ``event_kind`` is ``"SEGMENT_VELOCITY_EXCEEDED"`` when ``v_value``
    exceeds the cap (clamp applied), ``"SEGMENT_VELOCITY_UNKNOWN"`` when
    the v= expression could not be resolved, ``None`` otherwise.
    """
    if seg.kind == "movej":
        return MOVEJ_NOMINAL_DURATION_S, None

    if seg.v_value is None:
        v = v_cap_mps
        return (
            max(distance_m / v, SEGMENT_MIN_DURATION_S),
            "SEGMENT_VELOCITY_UNKNOWN",
        )

    v = seg.v_value
    event: Optional[str] = None
    if v > v_cap_mps:
        event = "SEGMENT_VELOCITY_EXCEEDED"
        v = v_cap_mps
    if v <= 0.0:
        v = v_cap_mps
    return max(distance_m / v, SEGMENT_MIN_DURATION_S), event


def densify_segments(
    segments: list[MotionSegment],
    dt: float,
    v_cap_mps: float,
) -> tuple[
    list[tuple[int, tuple[float, ...], int, bool]],
    list[tuple[int, str, object]],
]:
    """Subdivide each segment into ``dt``-sized substeps via SE(3) slerp.

    Parameters
    ----------
    segments:
        Output of :func:`ur5_sim.parsing.urscript.parse_motion_segments`.
    dt:
        Simulation time step (``ur5_sim.config.DT``). Each densified frame
        is held for this duration by the viewer.
    v_cap_mps:
        PolyScope linear-velocity cap (``URSCRIPT_MAX_TCP_SPEED_MPS``).
        Any segment whose ``v_value`` exceeds it is clamped here so the
        sim playback never animates faster than the real robot would.

    Returns
    -------
    densified:
        ``[(lineno, pose_tuple, cycle_idx, in_contact), ...]``. Pose tuple
        is the 6-tuple ``(x, y, z, rx, ry, rz)`` that downstream code
        (``transform``, ``rotate_translation_y``, ``apply_surface_constraint``,
        ``run_ik``) expects. ``lineno`` is propagated from the source
        segment so HUD / events still map back to the script.
    events:
        ``[(lineno, kind, detail), ...]`` for the text report.
    """
    if not segments:
        return [], []

    densified: list[tuple[int, tuple[float, ...], int, bool]] = []
    events: list[tuple[int, str, object]] = []

    first = segments[0]
    densified.append(
        (first.lineno, first.pose, first.cycle_idx, first.in_contact)
    )
    prev_pose_se3 = urscript_pose(*first.pose)

    for seg in segments[1:]:
        target_se3 = urscript_pose(*seg.pose)
        distance_m = float(np.linalg.norm(
            np.asarray(target_se3.t) - np.asarray(prev_pose_se3.t)
        ))
        t_seg, event_kind = _segment_duration(seg, distance_m, v_cap_mps)
        if event_kind == "SEGMENT_VELOCITY_EXCEEDED":
            events.append((
                seg.lineno,
                event_kind,
                f"v = {seg.v_value:.4f} {seg.v_unit} > cap "
                f"{v_cap_mps:.4f} m/s (clamped, distance "
                f"{distance_m * 1000:.2f} mm)",
            ))
        elif event_kind == "SEGMENT_VELOCITY_UNKNOWN":
            events.append((
                seg.lineno,
                event_kind,
                f"v= expression unresolved on {seg.kind} "
                f"(distance {distance_m * 1000:.2f} mm, fallback to cap)",
            ))

        n_sub = max(1, int(round(t_seg / dt)))
        for k in range(1, n_sub + 1):
            frac = k / n_sub
            sub = interp_se3(prev_pose_se3, target_se3, frac)
            x, y, z = float(sub.t[0]), float(sub.t[1]), float(sub.t[2])
            # SE3 -> URScript axis-angle (rx, ry, rz). spatialmath exposes
            # the rotation vector via SO3.eulervec() (axis * angle).
            rvec = sub.R
            # Use a robust axis-angle conversion via the SO3 helper.
            from spatialmath import SO3  # local import keeps module light
            rx, ry, rz = SO3(rvec).eulervec()
            densified.append((
                seg.lineno,
                (x, y, z, float(rx), float(ry), float(rz)),
                seg.cycle_idx,
                seg.in_contact,
            ))

        prev_pose_se3 = target_se3

    return densified, events
