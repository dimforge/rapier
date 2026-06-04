"""Make pyo3 pyclasses picklable from the correct dim/scalar namespace.

The PyO3 macros that emit all `rapier` pyclasses set `module = "rapier"` on
every class declaration. Since the four cdylibs (`_rapier2d`, `_rapier2d_f64`,
`_rapier3d`, `_rapier3d_f64`) each define their own copy of (e.g.)
`MassProperties` but all four share `__module__ == "rapier"`, Python's
default pickling cannot disambiguate which flavor to restore: pickle would
fetch `rapier.MassProperties` (== the default 3D-f32 flavor) and reject
non-matching pyclass instances with `PicklingError: it's not the same object
as rapier.MassProperties`.

To fix this without rewriting the macros, we patch `__module__` on every
exported class at import time so that each flavor's classes declare the
right module path:
  - `rapier.dim2.MassProperties.__module__ = "rapier.dim2"`
  - `rapier.dim2.f64.MassProperties.__module__ = "rapier.dim2.f64"`
  - `rapier.dim3.MassProperties.__module__ = "rapier.dim3"`
  - `rapier.dim3.f64.MassProperties.__module__ = "rapier.dim3.f64"`

Pickle then resolves each instance to its actual flavor.
"""

from __future__ import annotations

import types
from typing import Any


def retag_module(mod: Any, target_module: str) -> None:
    """Set `__module__` on every public class in `mod` to `target_module`."""
    for name in dir(mod):
        if name.startswith("_"):
            continue
        cls = getattr(mod, name)
        if not isinstance(cls, type):
            continue
        # Submodules can show up as types when they are nested package
        # objects, skip those.
        if isinstance(cls, types.ModuleType):
            continue
        try:
            cls.__module__ = target_module
        except (TypeError, AttributeError):
            # Some pyclasses or built-in types may refuse mutation; skip
            # them. We don't need pickle support for `bool`, `int`, etc.
            pass
