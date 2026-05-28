# Phase 2 — Split the pure-Python layer into four packages

Goal: relocate `python/rapier-py/python/rapier/` (one package, four submodules)
into four top-level packages co-located with their crates. End state: each
package imports cleanly and its tests pass.

## Source → destination map

| Old (under `rapier/`)            | New                                        |
| -------------------------------- | ------------------------------------------ |
| `dim3/__init__.py` (f32)         | `rapier-py-3d/python/rapier3d/__init__.py` |
| `dim3/f64.py`                    | `rapier-py-3d-f64/python/rapier3d_f64/__init__.py` |
| `dim3/math.py`                   | `rapier3d/math.py` (+ f64 copy)            |
| `dim2/__init__.py` (f32)         | `rapier-py-2d/python/rapier2d/__init__.py` |
| `dim2/f64.py`                    | `rapier-py-2d-f64/python/rapier2d_f64/__init__.py` |
| `dim2/math.py`                   | `rapier2d/math.py` (+ f64 copy)            |
| `loaders/` (urdf, mjcf, mesh)    | `rapier3d/loaders/` + `rapier3d_f64/loaders/` (3D only) |
| `_event_handler.py`              | vendored into all four                     |
| `_debug_render.py`               | vendored into all four                     |
| `_math_helpers.py`               | vendored into all four                     |
| `_pickle_setup.py`               | vendored into all four                     |
| `math.py` (root, 3D f32)         | folded into `rapier3d/math.py`             |
| top-level `__init__.py` re-exports | dropped (no umbrella)                    |

## Per-package `__init__.py` changes

Each new top-level `__init__.py` is the old flavor module body with:

1. **Extension import**: `from . import _rapier3d as _ext` /
   `from ._rapier3d import (...)` (relative to the package, not `from ..`).
2. **Helper imports**: `from ._event_handler import …`,
   `from ._debug_render import …` (now sibling modules, not `from ..`).
3. **Retag**: `from ._pickle_setup import retag_module; retag_module(_ext, "rapier3d")`
   — target string is the package's own top-level name.
4. **Drop** the `f64` / `math` submodule re-exports that assumed the umbrella
   layout (`rapier3d` no longer contains `f64`; it's a separate package). Keep
   `math` as a sub-attribute (`rapier3d.math`) since `math.py` ships in-package.
5. Keep the big `__all__` lists as-is (names are unchanged).

## Loaders (3D only)
`loaders/{urdf,mjcf,mesh}.py` import the 3D extension. Ship them in **both**
`rapier3d` and `rapier3d-f64` (each importing its own `_ext`). Confirm the
loader modules reference `_ext` relatively, not a hard-coded `rapier._rapier3d`.

## Type stubs (`.pyi`)
Move each flavor's `.pyi` next to its package (`rapier3d/__init__.pyi`,
`rapier3d/_rapier3d.pyi`, `math.pyi`). Add `py.typed` marker files so the
stubs are recognized by type checkers.

## Cleanup at end of phase
Delete `python/rapier-py/` entirely (its Python sources are now relocated).

## Verification
For each package: `pip install -e python/rapier-py-3d` (maturin develop) then
`python -c "import rapier3d; rapier3d.RigidBodyBuilder; print(rapier3d.__version__)"`,
and a pickle round-trip of a `MassProperties` to confirm `__module__` resolves.
