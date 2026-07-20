"""Mesh layer - load, decimate, colour and assemble visual geometry."""

from ur5_sim.meshes.colors import extract_color
from ur5_sim.meshes.decimation import decimate
from ur5_sim.meshes.endeffector import build_endeffector_meshes
from ur5_sim.meshes.link_loader import load_link_meshes

__all__ = [
    "extract_color",
    "decimate",
    "build_endeffector_meshes",
    "load_link_meshes",
]
