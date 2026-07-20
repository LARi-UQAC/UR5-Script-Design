"""
ur5_sim - Offline simulation and validation toolkit for the UR5 etalement script.

Layered architecture:
    parsing/        Read the URScript source, extract poses, replicate URScript
                    pose_trans / pose_inv semantics.
    kinematics/     Forward kinematics helpers and IK driver with joint-limit
                    enforcement.
    reporting/      Text reports of validation outcomes.
    meshes/         Visual mesh loading, decimation, colour extraction, and the
                    RobotIQ FT-300 / 2F-85 end-effector chain.
    visualization/  Matplotlib viewer with the three-panel layout, the
                    draggable splitters, the scroll-wheel zoom, the wall-clock
                    driven playback buffer.

Public API entry points live in ur5_sim.cli; for direct programmatic use, see
the per-module imports below.
"""

from ur5_sim.parsing.urscript import parse_poses, transform, urscript_pose
from ur5_sim.kinematics.ik import run_ik
from ur5_sim.reporting.text_report import report

__all__ = ["parse_poses", "transform", "urscript_pose", "run_ik", "report"]
