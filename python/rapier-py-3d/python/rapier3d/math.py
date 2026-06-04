"""`rapier.dim3.math` — math helpers (3D, f32)."""

from __future__ import annotations

from . import _rapier3d as _ext
from ._math_helpers import lerp, wrap_to_pi

rotation_from_angle = _ext.rotation_from_angle


def linear_interp(a, b, t):
    return lerp(a, b, t)


__all__ = ["lerp", "linear_interp", "rotation_from_angle", "wrap_to_pi"]
