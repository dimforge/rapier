# Repackaging the Python bindings into four PyPI packages

## Why

Today the bindings ship as **one** `rapier` wheel that bundles **four** Rust
cdylibs (`_rapier2d`, `_rapier2d_f64`, `_rapier3d`, `_rapier3d_f64`). Only
`_rapier3d` is built by maturin; the other three are pre-built by
`scripts/build_all_cdylibs.sh` and re-injected via `pyproject.toml`'s
`include`. That bundling is the non-idiomatic part of the setup and it fights
the standard wheel pipeline:

- `auditwheel` / `delocate` / `delvewheel` repair (manylinux/macOS/Windows
  tagging + vendoring of external libs) is built around a single
  maturin-produced module, not hand-injected `.so`/`.pyd` data files.
- Cross-compilation breaks: `build_all_cdylibs.sh` does a bare host-arch
  `cargo build` and never forwards `--target`, so the three secondary cdylibs
  would be the wrong architecture in a cross-built wheel.
- The secondaries must be built inside the same manylinux container as the
  primary or they don't satisfy the platform ABI.
- There is no publish workflow at all — CI only does `maturin develop` on two
  platforms (Linux x86_64, macOS arm64). Windows / Intel-macOS / Linux-aarch64
  are written-but-untested.

## Decision

Ship **four independent, standard single-crate maturin packages**, mirroring
the dimforge JS ecosystem (`@dimforge/rapier2d`, `rapier3d`, …):

| PyPI dist     | import name      | Rust crate          | cdylib          |
| ------------- | ---------------- | ------------------- | --------------- |
| `rapier2d`    | `rapier2d`       | `rapier-py-2d`      | `_rapier2d`     |
| `rapier3d`    | `rapier3d`       | `rapier-py-3d`      | `_rapier3d`     |
| `rapier2d-f64`| `rapier2d_f64`   | `rapier-py-2d-f64`  | `_rapier2d_f64` |
| `rapier3d-f64`| `rapier3d_f64`   | `rapier-py-3d-f64`  | `_rapier3d_f64` |

Each becomes a normal maturin project (one crate → one cdylib → one wheel),
so `PyO3/maturin-action` owns build + repair + tag end-to-end on every
platform. `rapier-py-core` (the `define_*_types!` macros) stays shared at the
Rust level — it is a normal path dependency, compiled into each cdylib.

This **replaces** the umbrella `rapier` package and its `rapier.dim2` /
`rapier.dim3.f64` API. See [00-target-layout.md](00-target-layout.md) for the
import-surface migration.

## Phases

0. [Decisions & target layout](00-target-layout.md)
1. [Restructure crates into maturin packages](01-restructure-crates.md)
2. [Split the pure-Python layer into four packages](02-pure-python-split.md)
3. [Re-point tests, examples, testbed](03-tests-examples-testbed.md)
4. [CI wheel build matrix (maturin-action)](04-ci-build-matrix.md)
5. [PyPI publishing](05-publishing-pypi.md)
6. [Docs & cleanup](06-docs-cleanup.md)

Each phase is independently reviewable and leaves the tree building. Phases
1–3 are the restructure (no behavior change, all tests must still pass);
4–5 add the publish pipeline; 6 is docs.

## Invariants to preserve through every phase

- abi3-py39 → **one wheel per (package, platform, arch)**, covering Python
  3.9+. Do not regress to per-minor-version wheels.
- The full pytest suite (currently 27 files) passes on each package after the
  split — re-pointed to per-package imports.
- Pickling / snapshot round-trips keep working (the `__module__` retag must
  point at each package's real top-level module).
