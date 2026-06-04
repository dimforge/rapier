"""Loaders for external scene formats.

3D-only. The upstream `rapier3d-urdf` / `rapier3d-meshloader` /
`rapier3d-mjcf` crates only target the 3D engine and only ship an f32
build, so the loader symbols live exclusively in the 3D-f32 cdylib.

Sub-modules::

    from rapier3d.loaders import urdf, mjcf, mesh

Each sub-module re-exports the corresponding classes from the underlying
`_rapier3d` extension.
"""

from __future__ import annotations

from . import mesh, mjcf, urdf

__all__ = ["mesh", "urdf", "mjcf"]
