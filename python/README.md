# Rapier Python bindings

The Rapier physics engine exposed to Python via [PyO3](https://pyo3.rs/) and
[maturin](https://www.maturin.rs/). The bindings ship as a single PyPI
package wrapping the 3D, `f32` engine:

| PyPI dist     | import name    | crate (`python/…`) | dim | scalar |
| ------------- | -------------- | ------------------ | --- | ------ |
| `rapier3d`    | `rapier3d`     | `rapier-py-3d`     | 3D  | f32    |

`rapier-py-core` holds the shared PyO3 binding macros (compiled into the
package; it is not published on its own). The Panda3D visual testbed lives in
the separate `rapier-testbed` package.

Full documentation lives in [`docs/`](docs/).

> **Note:** this is not yet published on PyPI. The instructions below build
> the package locally from this checkout. There is currently no source
> distribution (sdist), so building requires the full git repository plus a
> Rust toolchain.

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

## Quick start

[`python/dev.sh`](dev.sh) does everything below in one command — builds the
package, runs the test suite, builds the docs, and smoke-tests the testbed. It
creates and manages a `.venv` at the repo root on first run:

```bash
python/dev.sh                 # build + test + docs + testbed (headless smoke)
python/dev.sh build           # just build the package
python/dev.sh test            # build + run the test suite
python/dev.sh docs            # build + build the docs
python/dev.sh testbed         # build + install testbed, open the picker
python/dev.sh --help          # all commands and options (e.g. PROFILE=debug)
```

The sections below are the manual equivalents.

## 1. Build the bindings

Build the package, editable, with `maturin develop` (add `--release` for an
optimized build — slower to compile, much faster at runtime):

```bash
maturin develop -m python/rapier-py-3d/Cargo.toml      # 3D f32 -> import rapier3d
```

```bash
python -c "import rapier3d; print(rapier3d.__version__)"   # smoke check
```

### Run the test suite

```bash
maturin develop --release -m python/rapier-py-3d/Cargo.toml
pip install pytest pytest-timeout hypothesis numpy matplotlib
python -m pytest python/tests
```

## 2. Build and open the docs

The docs use Sphinx autodoc, so the engine package must be **built first**
(step 1). Then:

```bash
pip install sphinx furo sphinx-autodoc-typehints
cd python/docs
sphinx-build -b html . _build/html
open _build/html/index.html          # macOS; Linux: xdg-open; Windows: start
```

## 3. Run the testbed examples

`rapier-testbed` is a [Panda3D](https://www.panda3d.org/) visual gallery of
examples ported from the Rust `examples3d/`. It drives the 3D engine.

### Install

The testbed depends on `rapier3d`. Since it isn't on PyPI yet, build it first
(step 1 above), then install the testbed with `--no-deps` so pip uses your
local build instead of trying to fetch it.
Run this **from the repository root** (the `./python/...` path is relative to
it, like the build steps above):

```bash
pip install panda3d numpy
pip install --no-deps -e ./python/rapier-testbed
```

The `-e` (editable) install means edits to the testbed — examples, camera, etc.
— are picked up by `python -m rapier_testbed` with no reinstall. (`--no-deps`
must come before `-e`, or pip treats it as the install target.)

### Launch

Once installed, these run from **any** directory (they invoke the installed
`rapier_testbed` module, not a path), inside the virtual environment from the
[Prerequisites](#prerequisites):

```bash
# Interactive picker — browse every example by category (opens a window):
python -m rapier_testbed

# Jump straight into one example. 3D examples live under `examples3`:
python -m rapier_testbed.examples3.domino3

# Headless — run a fixed number of steps with no window (needs no display;
# this is what CI uses):
PANDA_NO_WINDOW=1 python -m rapier_testbed.examples3.domino3
```

To walk through **every** example in turn — each opens in a window, and
closing it launches the next (Ctrl-C to stop):

```bash
python python/examples_tour.py          # every 3D example; accepts --start NAME
python/dev.sh tour                      # same, but builds + installs the testbed first
```

### Controls (in the viewer window)

| Mouse | Action | | Key | Action |
| --- | --- | --- | --- | --- |
| left-drag | rotate camera | | `Space` | pause / resume |
| right-drag | pan | | `R` | reset the scene |
| wheel | zoom | | `Tab` | next example |
| | | | `W` | toggle wireframe |
| | | | `Esc` | quit |

The examples span categories such as Collisions, Dynamics, Joints, Controls,
Robotics, Stress Tests, Debug, and Misc. The picker lists them all; each
example's module name (for direct launch) matches its file under
`python/rapier-testbed/rapier_testbed/examples3/`.

## License

Apache-2.0
