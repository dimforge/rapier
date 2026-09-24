"""Make pyo3 pyclasses picklable from the correct module namespace.

The PyO3 macros that emit the `rapier` pyclasses set `module = "rapier"` on
every class declaration, but the extension is imported as `rapier3d`. Left
unpatched, Python's pickling would look classes up under the wrong module
path and reject the instances with `PicklingError: it's not the same object
as rapier.MassProperties`.

To fix this without rewriting the macros, we patch `__module__` on every
exported class at import time so it declares the real module path:
  - `rapier3d.MassProperties.__module__ = "rapier3d"`

Pickle then resolves each instance correctly.
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
