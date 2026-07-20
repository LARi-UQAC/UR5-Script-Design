"""Load the visual meshes shipped with the UR5 URDF.

Each ``rtb.models.UR5().links[i]`` holds the path of one or more DAE
files. The loader splits multi-material scenes into their constituent
sub-meshes so that per-part colours (UR blue joint caps, light-grey
body) survive into the viewer.
"""

from __future__ import annotations

import numpy as np

from ur5_sim.config import TARGET_FACES_LINK
from ur5_sim.meshes.colors import extract_color
from ur5_sim.meshes.decimation import decimate

try:
    import trimesh
    _HAS_TRIMESH = True
except ImportError:
    _HAS_TRIMESH = False


def load_link_meshes(robot, target_faces_per_part: int = TARGET_FACES_LINK) -> list[dict]:
    """Return one dict per visual sub-mesh.

    Keys:
        name  : link name, used downstream to look up the world transform.
        verts : (N, 3) vertices in the link's local frame, scaled to metres.
        faces : (M, 3) triangle indices.
        color : (R, G, B) tuple in [0, 1].
    """
    if not _HAS_TRIMESH:
        return []
    out: list[dict] = []
    total_in = 0
    total_out = 0
    for link in robot.links:
        for geom in getattr(link, "geometry", None) or []:
            fname = getattr(geom, "filename", None)
            if not fname:
                continue
            try:
                loaded = trimesh.load(fname)
            except Exception as e:
                print(f"  mesh load failed for {link.name}: {e}")
                continue

            submeshes = []
            if hasattr(loaded, "geometry") and hasattr(loaded, "graph"):
                for sub_name, sub in loaded.geometry.items():
                    T_scene = np.eye(4)
                    try:
                        T_scene = np.asarray(loaded.graph.get(sub_name)[0], dtype=float)
                    except Exception:
                        pass
                    submeshes.append((sub, T_scene))
            else:
                submeshes.append((loaded, np.eye(4)))

            scale = np.asarray(geom.scale, dtype=float)
            for sub, T_scene in submeshes:
                total_in += len(sub.faces)
                sub_dec = decimate(sub, target_faces_per_part)
                total_out += len(sub_dec.faces)
                verts = np.asarray(sub_dec.vertices, dtype=float)
                verts = (T_scene[:3, :3] @ verts.T).T + T_scene[:3, 3]
                verts = verts * scale
                faces = np.asarray(sub_dec.faces, dtype=int)
                color = extract_color(sub)
                out.append(
                    {"name": link.name, "verts": verts, "faces": faces, "color": color}
                )
    if total_in:
        print(
            f"  decimation: {total_in} -> {total_out} triangles "
            f"({100*total_out/total_in:.1f}%)."
        )
    return out
