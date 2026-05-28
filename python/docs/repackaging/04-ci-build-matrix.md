# Phase 4 — CI wheel build matrix (maturin-action)

Replace the `maturin develop`-only test job with a real wheel pipeline using
[`PyO3/maturin-action`](https://github.com/PyO3/maturin-action), which runs
manylinux/musllinux builds in containers and invokes
`auditwheel`/`delocate`/`delvewheel` repair automatically. Base it on
`maturin generate-ci github` output, then matrix over the four packages.

## Workflow shape (`.github/workflows/python-wheels.yml`)

Two axes: **package** × **platform target**.

- Packages: `rapier-py-2d`, `rapier-py-2d-f64`, `rapier-py-3d`,
  `rapier-py-3d-f64` (pass each via `working-directory` / `-m <crate>/Cargo.toml`).
- Targets:
  - **Linux**: `x86_64`, `aarch64` — each as manylinux **and** musllinux
    (`manylinux: auto`, plus a musllinux leg). Built in the action's container.
  - **macOS**: `x86_64` and `aarch64` (or a single `universal2`). Use
    `macos-14` (arm64) + cross to x86_64, or two runners.
  - **Windows**: `x64`.
- `args: --release --out dist` and rely on abi3 → one wheel per leg covers
  Python 3.9+. Do **not** add a Python-version axis.

```yaml
strategy:
  matrix:
    package:
      - { dir: rapier-py-3d,     name: rapier3d }
      - { dir: rapier-py-3d-f64, name: rapier3d-f64 }
      - { dir: rapier-py-2d,     name: rapier2d }
      - { dir: rapier-py-2d-f64, name: rapier2d-f64 }
    platform:
      - { runner: ubuntu-latest,  target: x86_64-unknown-linux-gnu,   manylinux: auto }
      - { runner: ubuntu-latest,  target: aarch64-unknown-linux-gnu,  manylinux: auto }
      - { runner: ubuntu-latest,  target: x86_64-unknown-linux-musl,  manylinux: musllinux_1_2 }
      - { runner: macos-14,       target: aarch64-apple-darwin }
      - { runner: macos-13,       target: x86_64-apple-darwin }
      - { runner: windows-latest, target: x86_64-pc-windows-msvc }
steps:
  - uses: actions/checkout@v4
  - uses: PyO3/maturin-action@v1
    with:
      command: build
      target: ${{ matrix.platform.target }}
      manylinux: ${{ matrix.platform.manylinux }}
      args: --release --out dist -m python/${{ matrix.package.dir }}/Cargo.toml
  - uses: actions/upload-artifact@v4
    with:
      name: wheels-${{ matrix.package.name }}-${{ matrix.platform.target }}
      path: dist
```

## Test legs
Keep a lighter PR job that, per package, builds with maturin and runs that
package's `pytest` on `ubuntu-latest` + `macos-14` + `windows-latest` (the
build matrix above only *builds*; this proves the wheels import and pass).
This finally exercises **Windows**, which the current setup never does.

## sdist
One `maturin sdist -m <crate>/Cargo.toml` per package (separate job). The sdist
must be buildable from a checkout — verify `rapier-py-core` and the engine
crates resolve via path deps inside the sdist (maturin vendors workspace path
deps into the sdist; confirm).

## Verification
- All matrix legs green; artifacts present for every (package × target).
- `twine check dist/*` passes.
- Download a manylinux wheel, `auditwheel show` reports a `manylinux_*` tag
  (not `linux_x86_64`).
