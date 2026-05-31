# Phase 0 — Decisions & target layout

## Target directory layout

```
python/
  rapier-py-core/                 # Rust macros, shared (unchanged), publish=false
  rapier-py-2d/                   # was a thin cdylib crate; becomes a full maturin package
    Cargo.toml                    #   [lib] _rapier2d, crate-type=cdylib  (unchanged)
    pyproject.toml                #   name = "rapier2d"
    src/lib.rs                    #   unchanged
    python/rapier2d/              #   pure-Python wrapper (was rapier/dim2/*)
      __init__.py
      math.py
      _event_handler.py           #   vendored from shared
      _debug_render.py            #   vendored
      _math_helpers.py            #   vendored
      _pickle_setup.py            #   vendored
      __init__.pyi
    tests/                        #   2D test files
    examples/                     #   2D examples
  rapier-py-2d-f64/  -> name = "rapier2d-f64",  python/rapier2d_f64/
  rapier-py-3d/      -> name = "rapier3d",       python/rapier3d/      (+ loaders/)
  rapier-py-3d-f64/  -> name = "rapier3d-f64",   python/rapier3d_f64/  (+ loaders/)
```

Keep the Rust crate directory names (`rapier-py-2d`, …) so git history and
workspace members move minimally; only their *role* changes (each gains a
`pyproject.toml` + `python/<import_name>/`). The umbrella `python/rapier-py/`
crate and `scripts/build_all_cdylibs.sh` are deleted.

## Decisions

### Naming
- dist names use hyphens (`rapier2d-f64`); import names use the matching valid
  identifier (`rapier2d_f64`). This matches the Rust crate spelling and PEP 8.
- The internal cdylib keeps its current name (`module-name = "rapier2d._rapier2d"`).

### Shared pure-Python → vendor, don't add a 5th package
The shared helpers (`_event_handler.py`, `_debug_render.py`, `_math_helpers.py`,
`_pickle_setup.py`) are small and flavor-agnostic. **Vendor a copy into each
package** rather than introducing a `rapier-common` runtime dependency. This
keeps every package a self-contained, standard maturin project (the dimforge-JS
model) with no cross-package version coupling. The duplication is ~4 short
files and they change rarely.
- Trade-off noted: if these helpers start changing often, revisit and extract
  `rapier-common`. Not now.

### Pickling / `__module__`
The `define_*_types!` macros hard-code `module = "rapier"`. After the split,
each package retags its extension classes to its **own** top-level name at
import (`retag_module(_ext, "rapier3d")`), reusing the vendored
`_pickle_setup.retag_module`. Because each flavor is now a distinct top-level
module, the old cross-flavor collision that motivated the retag disappears —
the retag only needs to make `__module__` point at the real import path so
`pickle`/snapshots resolve. (Optional later cleanup: parameterize the macros
with a `MODULE = "…"` arg and drop the runtime retag. Out of scope here.)

### abi3
Keep `abi3-py39` on every crate → one wheel per (package, platform, arch).

### Versioning
All four packages share the workspace version (`version.workspace = true` in
Cargo; mirror the same literal in each `pyproject.toml [project].version`, or
use maturin's `dynamic = ["version"]` reading from Cargo). Bump the four
together. Decide in Phase 5.

### Determinism build
The `determinism` Cargo feature (libm transcendentals) stays a **from-source
opt-in** (`maturin build -F determinism`); published wheels are the fast path.
Documented in Phase 5; no separate `-determinism` dist for now.

## Public API migration (breaking)

| Before                              | After                          |
| ----------------------------------- | ------------------------------ |
| `import rapier` (3D f32 default)    | `import rapier3d`              |
| `from rapier import dim2`           | `import rapier2d`             |
| `from rapier.dim3 import f64`       | `import rapier3d_f64`        |
| `from rapier.dim2 import f64`       | `import rapier2d_f64`        |
| `rapier.RapierError` (cross-flavor) | per-package `rapier{2,3}d.RapierError` |

There is no longer a single exception base shared across flavors; each package
exposes its own `RapierError` tree. Note this prominently in the changelog.
