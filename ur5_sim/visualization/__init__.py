"""Visualization layer - matplotlib three-panel viewer + test surface."""

from ur5_sim.visualization.surface import (
    apply_surface_constraint,
    attach_surface_to_swift,
    clamp_pose_above_surface,
    compute_surface_frame,
    make_corners_xy_polygon,
    snap_pose_onto_surface,
)
from ur5_sim.visualization.viewer import visualize

__all__ = [
    "visualize",
    "apply_surface_constraint",
    "attach_surface_to_swift",
    "clamp_pose_above_surface",
    "compute_surface_frame",
    "make_corners_xy_polygon",
    "snap_pose_onto_surface",
]
