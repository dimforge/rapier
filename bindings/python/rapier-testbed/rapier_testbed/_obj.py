"""Minimal Wavefront OBJ loader for the testbed examples.

The Rust examples pull meshes from ``assets/3d/*.obj`` via the ``obj`` crate.
The Python bindings ship no OBJ parser, so a couple of examples used to fall
back to primitives. OBJ geometry is a trivial text format — vertex positions
(``v x y z``) and faces (``f ...``) — so we parse just that here: enough to
feed :func:`rapier3d.Collider.trimesh` / ``SharedShape.convex_decomposition``.

Only positions and face connectivity are read; texture/normal indices, groups,
materials, and free-form geometry are ignored. Polygonal faces are
fan-triangulated; OBJ's 1-based (and negative, relative) indices are
normalized to 0-based.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Tuple

import numpy as np


def find_asset(relpath: str) -> Path | None:
    """Locate an asset shipped in the repo's ``assets/`` tree.

    Honors ``RAPIER_REPO_ROOT`` first, then walks up from this file looking
    for ``<parent>/<relpath>`` (the testbed is normally run from a checkout).
    Returns ``None`` if nothing matches.
    """
    candidates = []
    env = os.environ.get("RAPIER_REPO_ROOT")
    if env:
        candidates.append(Path(env) / relpath)
    here = Path(__file__).resolve()
    for parent in [here] + list(here.parents):
        candidates.append(parent / relpath)
    for c in candidates:
        if c.is_file():
            return c
    return None


def _parse_face_index(token: str, num_vertices: int) -> int:
    """Resolve one OBJ face vertex reference to a 0-based position index.

    A token looks like ``v``, ``v/vt``, ``v//vn``, or ``v/vt/vn``; only the
    leading position component matters. OBJ indices are 1-based, and negative
    values count back from the end of the vertex list.
    """
    raw = int(token.split("/", 1)[0])
    if raw > 0:
        return raw - 1
    if raw < 0:
        return num_vertices + raw
    raise ValueError("OBJ face index 0 is invalid (indices are 1-based)")


def load_obj(path: os.PathLike | str) -> Tuple[np.ndarray, np.ndarray]:
    """Load an OBJ file into ``(vertices (N,3) float32, indices (M,3) uint32)``.

    Faces with more than three vertices are fan-triangulated.
    """
    positions: list[tuple[float, float, float]] = []
    tris: list[tuple[int, int, int]] = []

    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            # Strip comments and surrounding whitespace.
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            tag, _, rest = line.partition(" ")
            if tag == "v":
                xyz = rest.split()
                positions.append((float(xyz[0]), float(xyz[1]), float(xyz[2])))
            elif tag == "f":
                verts = rest.split()
                if len(verts) < 3:
                    continue
                n = len(positions)
                idx = [_parse_face_index(v, n) for v in verts]
                # Fan-triangulate: (i0, ik, ik+1).
                for k in range(1, len(idx) - 1):
                    tris.append((idx[0], idx[k], idx[k + 1]))

    vertices = np.asarray(positions, dtype=np.float32).reshape(-1, 3)
    indices = np.asarray(tris, dtype=np.uint32).reshape(-1, 3)
    return vertices, indices


def normalize_to_unit(vertices: np.ndarray, target_diag: float = 10.0) -> np.ndarray:
    """Center a vertex cloud at the origin and scale it to a target diagonal.

    Mirrors the Rust examples: subtract the AABB center, then scale so the
    AABB diagonal becomes ``target_diag`` — giving every model a similar size.
    """
    v = np.asarray(vertices, dtype=np.float32).reshape(-1, 3)
    if v.shape[0] == 0:
        return v
    mn = v.min(axis=0)
    mx = v.max(axis=0)
    center = (mn + mx) * 0.5
    diag = float(np.linalg.norm(mx - mn))
    scale = (target_diag / diag) if diag > 0.0 else 1.0
    return ((v - center) * scale).astype(np.float32)
