"""Best-effort colour extraction from a trimesh visual.

A DAE file produced by an off-the-shelf CAD tool can store its colour
information in a number of incompatible places. This helper probes them
in order and falls back to a neutral grey so the renderer never gets a
``None`` value.
"""

from __future__ import annotations

import numpy as np

_DEFAULT_COLOR: tuple[float, float, float] = (0.75, 0.78, 0.82)


def extract_color(mesh) -> tuple[float, float, float]:
    """Return an (R, G, B) tuple in [0, 1] from any trimesh visual variant."""
    visual = getattr(mesh, "visual", None)
    if visual is None:
        return _DEFAULT_COLOR

    fc = getattr(visual, "face_colors", None)
    if fc is not None:
        fc = np.asarray(fc)
        if fc.ndim >= 2 and fc.shape[0] > 0:
            c = np.asarray(fc[0], dtype=float)
            if c.max() > 1.5:
                c = c / 255.0
            return tuple(c[:3])

    mat = getattr(visual, "material", None)
    if mat is not None:
        for attr in ("main_color", "diffuse", "baseColorFactor"):
            c = getattr(mat, attr, None)
            if c is None:
                continue
            c = np.asarray(c, dtype=float)
            if c.size < 3:
                continue
            if c.max() > 1.5:
                c = c / 255.0
            return tuple(c[:3])

    return _DEFAULT_COLOR
