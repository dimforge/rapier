"""`rapier3d.math`: math helpers, and transcendental functions computed by Rapier.

The functions :func:`sin`, :func:`cos`, :func:`tan`, :func:`asin`, :func:`acos`,
:func:`atan`, :func:`atan2`, :func:`exp`, :func:`log`, :func:`pow` and :func:`sqrt`
use the same math backend as the engine, in its precision (``f32``). When the bindings
are built with the ``determinism`` feature (see
:attr:`rapier3d.BuildFeatures.enhanced_determinism`), they give bit-identical results on
every platform, unlike Python's :mod:`math` module or NumPy: use them to compute the
initial state of a simulation that must be cross-platform deterministic.
"""

from __future__ import annotations

from . import _rapier3d as _ext
from ._math_helpers import lerp, wrap_to_pi

rotation_from_angle = _ext.rotation_from_angle

sin = _ext._math.sin
cos = _ext._math.cos
tan = _ext._math.tan
asin = _ext._math.asin
acos = _ext._math.acos
atan = _ext._math.atan
atan2 = _ext._math.atan2
exp = _ext._math.exp
log = _ext._math.log
pow = _ext._math.pow  # noqa: A001 (mirrors the name of `math.pow`)
sqrt = _ext._math.sqrt


def linear_interp(a, b, t):
    """Alias of :func:`lerp`."""
    return lerp(a, b, t)


__all__ = [
    "acos",
    "asin",
    "atan",
    "atan2",
    "cos",
    "exp",
    "lerp",
    "linear_interp",
    "log",
    "pow",
    "rotation_from_angle",
    "sin",
    "sqrt",
    "tan",
    "wrap_to_pi",
]
