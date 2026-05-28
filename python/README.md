# Rapier Python bindings

The Rapier physics engine exposed to Python via [PyO3](https://pyo3.rs/) and
[maturin](https://www.maturin.rs/). The bindings ship as **four independent
PyPI packages**, one per `(dimension, scalar)` flavor — they share an
identical API differing only in dimension and scalar type:

| PyPI dist     | import name    | crate (`python/…`) | dim | scalar |
| ------------- | -------------- | ------------------ | --- | ------ |
| `rapier2d`    | `rapier2d`     | `rapier-py-2d`     | 2D  | f32    |
| `rapier3d`    | `rapier3d`     | `rapier-py-3d`     | 3D  | f32    |
| `rapier2d-f64`| `rapier2d_f64` | `rapier-py-2d-f64` | 2D  | f64    |
| `rapier3d-f64`| `rapier3d_f64` | `rapier-py-3d-f64` | 3D  | f64    |

`rapier-py-core` holds the shared PyO3 binding macros (compiled into each
package; it is not published on its own). The Panda3D visual testbed lives in
the separate `rapier-testbed` package.

Full documentation lives in [`docs/`](docs/); the design of this split is
recorded in [`docs/repackaging/`](docs/repackaging/).

> **Note:** these are not yet published on PyPI. The instructions below build
> the packages locally from this checkout. There is currently no source
> distribution (sdist), so building requires the full git repository plus a
> Rust toolchain — see [`docs/repackaging/05-publishing-pypi.md`](docs/repackaging/05-publishing-pypi.md).

## Prerequisites

- A **Rust toolchain** ([rustup](https://rustup.rs/)).
- **Python ≥ 3.9** and a virtual environment (extensions are compiled
  per-environment).
- [`maturin`](https://www.maturin.rs/) for building the Rust extensions.

```bash
# from the repository root
python3 -m venv .venv && source .venv/bin/activate
pip install maturin
```

## 1. Build the bindings

Each flavor is its own package. Build the one(s) you want, editable, with
`maturin develop` (add `--release` for an optimized build — slower to compile,
much faster at runtime):

```bash
maturin develop -m python/rapier-py-3d/Cargo.toml      # 3D f32 -> import rapier3d
```

Swap `rapier-py-3d` for `rapier-py-2d`, `rapier-py-3d-f64`, or
`rapier-py-2d-f64` to build the other flavors (see the table above for the
matching import name).

```bash
python -c "import rapier3d; print(rapier3d.__version__)"   # smoke check
```

### Run the test suite

The test suite is cross-flavor (it parametrizes over all four packages), so
install every flavor first:

```bash
for c in rapier-py-2d rapier-py-2d-f64 rapier-py-3d rapier-py-3d-f64; do
  maturin develop --release -m "python/$c/Cargo.toml"
done
pip install pytest pytest-timeout hypothesis numpy matplotlib
python -m pytest python/tests        # ~708 passed
```

## 2. Build and open the docs

The docs use Sphinx autodoc, so the engine packages must be **built first**
(step 1 — build all four to avoid import warnings). Then:

```bash
pip install sphinx furo sphinx-autodoc-typehints
cd python/docs
sphinx-build -b html . _build/html
open _build/html/index.html          # macOS; Linux: xdg-open; Windows: start
```

## 3. Run the testbed examples

The testbed drives both the 2D and 3D engines, so install it **after** the
engine packages. Use `--no-deps` (its `rapier2d`/`rapier3d` dependencies are
your local editable installs, not yet on PyPI):

```bash
pip install panda3d numpy
pip install --no-deps ./python/rapier-testbed
```

Then:

```bash
rapier-testbed                                  # interactive example picker (opens a window)
python -m rapier_testbed                        # same thing
python -m rapier_testbed.examples3.domino3      # launch one example directly
PANDA_NO_WINDOW=1 python -m rapier_testbed.examples3.domino3   # headless (no window, fixed steps)
```

Example modules are named `rapier_testbed.examples3.<name>` (3D) and
`rapier_testbed.examples2.<name>` (2D) — 127 examples across 9 categories.

## License

Apache-2.0
