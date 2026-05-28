# Phase 3 — Re-point tests, examples, testbed

## Tests → single repo-level `python/tests/` suite (decided during implementation)
The 27 test files are **parity tests**: each parametrizes over flavors within
one file (`@pytest.fixture(params=[dim3, dim3_f64])`), so they inherently
import multiple packages at once. Rather than split (which would lose the
parity coverage), they became one repo-level suite at `python/tests/` that
imports all four packages. This is dev-only — it ships in no wheel, so the
packages stay independent. Import rewrites applied:

- `from rapier import dim3` → `import rapier3d as dim3`
- `from rapier.dim3 import f64 as dim3_f64` → `import rapier3d_f64 as dim3_f64`
- `from rapier import dim2` / `dim2.f64` → `import rapier2d` / `rapier2d_f64`
- `from rapier.loaders import …` → `from rapier3d.loaders import …`
- bare `import rapier` (3D-default) → `import rapier3d as rapier`

CI (`python-bindings.yml` `test` job) installs all four engine packages +
`rapier-testbed`, then runs the suite once per OS (ubuntu/macOS/**Windows**).
Result: **698 passed, 1 skipped**, plus **10** example-runner tests.

## Examples → `python/examples/`
`examples/` (mostly 3D: urdf, vehicle, character, render, serde + a couple of
parity scripts) moved to `python/examples/`, imports rewritten the same way.
`test_examples.py` runs each as a subprocess and asserts its final printed
line — all 10 pass.

## Testbed → standalone `rapier-testbed` package (decided during implementation)
`rapier/testbed/` is a Panda3D viewer + `examples2/` and `examples3/`. The
plan floated vendoring the framework into the f32 packages, but the framework
is **dimension-coupled**: `_testbed.py` imports both `import rapier3d` and
`import rapier2d`, and `_meshes.py` uses both. Splitting it per-dim would mean
butchering those files. The honest, idiomatic fix is a **standalone
`rapier-testbed` package** that depends on `rapier2d` + `rapier3d`.

Implemented:
- `python/rapier-testbed/` — pure-Python (hatchling backend), import name
  `rapier_testbed`, console script `rapier-testbed = rapier_testbed:run`.
- `dependencies = ["rapier2d", "rapier3d", "numpy", "panda3d"]`.
- Imports rewritten: `import rapier as _rp3` → `import rapier3d`,
  `from rapier import dim2` → `import rapier2d`, the hardcoded
  `rapier.testbed.examples*` autoload list → `rapier_testbed.examples*`.
- Headless smoke (`PANDA_NO_WINDOW=1`): 127 examples register across 9
  categories.
- Not necessarily published to PyPI (dev/demo tool); buildable + CI-installable.

## Verification
`pytest` green in each package dir; `python -m rapier3d.testbed <example>`
launches (manual / smoke-imported without opening a window in CI).
