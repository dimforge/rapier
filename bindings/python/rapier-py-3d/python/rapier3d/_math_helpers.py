"""Pure-Python conveniences that don't need to live in Rust.

These work on any dim/scalar variant since they only assume vector/point
arithmetic (`__mul__`, `__add__`, `__sub__`, scalar mul) and a `float`-like
angle for `wrap_to_pi`.
"""

from __future__ import annotations

import math


def lerp(a, b, t):
    """Linear interpolation: `a*(1-t) + b*t`.

    Works for any `Vec2`/`Vec3`/`Point2`/`Point3` (and plain floats) as long
    as `a` and `b` support `+`, `-`, and scalar `*`.
    """
    return a + (b - a) * t


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle in radians to the half-open interval `(-pi, pi]`."""
    a = (angle + math.pi) % (2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi
