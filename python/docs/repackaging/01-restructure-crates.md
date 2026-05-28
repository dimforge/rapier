# Phase 1 — Restructure crates into maturin packages

Goal: each cdylib crate becomes a self-contained maturin project. No Python
behavior change yet (Phase 2 moves the wrapper code); this phase is build
plumbing only. Tree must `cargo check` and `maturin build` per package.

## Steps

1. **Add a `pyproject.toml` to each cdylib crate dir.** Template
   (`rapier-py-3d/pyproject.toml`):
   ```toml
   [build-system]
   requires = ["maturin>=1.7,<2"]
   build-backend = "maturin"

   [project]
   name = "rapier3d"
   version = "0.32.0"
   description = "Python bindings for Rapier (3D, f32) — a fast, deterministic physics engine."
   readme = "README.md"
   license = { text = "Apache-2.0" }
   requires-python = ">=3.9"
   # classifiers, urls, authors: copy from the old umbrella pyproject

   [project.optional-dependencies]
   numpy = ["numpy>=1.21"]
   # rapier3d only: viz testbed extra (see Phase 3)

   [tool.maturin]
   manifest-path = "Cargo.toml"
   module-name = "rapier3d._rapier3d"
   python-source = "python"
   features = ["pyo3/extension-module"]
   ```
   The four `module-name` values: `rapier2d._rapier2d`,
   `rapier2d_f64._rapier2d_f64`, `rapier3d._rapier3d`,
   `rapier3d_f64._rapier3d_f64`. **No `include` block** — each wheel ships
   exactly one cdylib, so the secondary-bundling hack is gone.

2. **Update the workspace `Cargo.toml`.** Remove the `python/rapier-py`
   member (umbrella crate is deleted). The four cdylib crates + `rapier-py-core`
   remain members. `default-members` is unchanged (still excludes all
   `rapier-py-*` so bare `cargo build` stays Python-free).

3. **Delete** `python/rapier-py/` once Phase 2 has relocated its Python
   sources, and delete `scripts/build_all_cdylibs.sh`. (Sequence: do the
   Python move in Phase 2, then delete the umbrella dir at the end of Phase 2;
   this phase just stops referencing it.)

4. **`build.rs`** (`rapier-py-3d/build.rs`, determinism env-var translation)
   moves with its crate — no change.

## Verification
- `cargo check -p rapier-py-2d -p rapier-py-2d-f64 -p rapier-py-3d -p rapier-py-3d-f64`.
- `cargo clippy --no-deps -p rapier-py-core -p rapier-py-2d … -- -D warnings`
  (mirror the existing `python-bindings.yml` lint invocation, minus
  `rapier-py`).
- `maturin build -m python/rapier-py-3d/Cargo.toml` produces a single-cdylib
  wheel (after Phase 2 wires the Python source).

## Risks
- `module-name` must match the package's import name exactly or imports fail at
  runtime.
- Don't drop `pyo3/extension-module` from `[tool.maturin].features`; it is also
  in each crate's own `Cargo.toml` deps, but keep it explicit for clarity.
