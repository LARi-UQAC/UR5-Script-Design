"""Quadric decimation helper around trimesh.

Different trimesh versions accept different keyword arguments for the
``simplify_quadric_decimation`` call: ``face_count``, ``target_count`` or
just a positional integer. This wrapper tries each form and silently
returns the original mesh when no compatible signature works (e.g. when
the optional ``fast-simplification`` backend is missing).
"""

from __future__ import annotations


def decimate(mesh, target_faces: int):
    """Reduce ``mesh`` to at most ``target_faces`` triangles when possible."""
    if target_faces is None or target_faces <= 0:
        return mesh
    if len(getattr(mesh, "faces", [])) <= target_faces:
        return mesh
    for kwargs in ({"face_count": target_faces}, {"target_count": target_faces}, {}):
        try:
            if kwargs:
                return mesh.simplify_quadric_decimation(**kwargs)
            else:
                return mesh.simplify_quadric_decimation(target_faces)
        except TypeError:
            continue
        except Exception:
            break
    return mesh
