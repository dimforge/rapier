"""Mesh loader (`rapier.loaders.mesh`).

Wraps `rapier3d-meshloader`. 3D-only, f32-precision.
"""

from __future__ import annotations

from .._rapier3d import LoadedShape  # noqa: F401
from .._rapier3d import MeshConversionError, MeshLoaderError  # noqa: F401
from .._rapier3d import load_from_path, load_from_raw_mesh  # noqa: F401

__all__ = [
    "LoadedShape",
    "MeshConversionError",
    "MeshLoaderError",
    "load_from_path",
    "load_from_raw_mesh",
]
