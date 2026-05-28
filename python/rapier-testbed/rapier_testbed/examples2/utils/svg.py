"""Minimal placeholder for the Rust ``utils::svg`` helper.

The Rust example uses ``lyon`` + ``usvg`` to tessellate the Rapier SVG
logo into trimeshes. There is no equivalent in pure Python without
pulling in heavy dependencies, so this helper provides a triangle-mesh
substitute that the ``trimesh2`` port can use for visual diversity.
"""
from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np


def rapier_logo() -> List[Tuple[np.ndarray, np.ndarray]]:
    """Return a list of ``(vertices, indices)`` pairs.

    Each pair describes a small trimesh that stands in for one glyph of
    the Rapier logo in the Rust testbed. The shapes are convex polygons
    (regular n-gons of varying sides) so the scene still has variety.
    """
    out: List[Tuple[np.ndarray, np.ndarray]] = []
    for sides in (5, 6, 7, 8, 9, 10):
        radius = 2.0
        verts = np.array(
            [
                [0.0, 0.0],
                *[
                    [math.cos(2.0 * math.pi * k / sides) * radius,
                     math.sin(2.0 * math.pi * k / sides) * radius]
                    for k in range(sides)
                ],
            ],
            dtype=np.float32,
        )
        idx = np.array(
            [[0, k + 1, ((k + 1) % sides) + 1] for k in range(sides)],
            dtype=np.uint32,
        )
        out.append((verts, idx))
    return out


__all__ = ["rapier_logo"]
