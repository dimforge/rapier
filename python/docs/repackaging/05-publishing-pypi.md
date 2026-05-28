# Phase 5 — PyPI publishing

> **Implemented in `.github/workflows/python-wheels.yml`.** `build` matrix
> (4 packages × 7 targets = 28 wheel legs) → `publish` job (trusted
> publishing) gated on `python-v*` tags. Versions are `dynamic` in each
> `pyproject.toml`, read from the workspace Cargo version, so one bump drives
> all four. **Wheels-only — no sdist** (see note below).

## No sdist (decided during implementation)
The rapier2d/3d engine crates declare `[lib] path = "../../src/lib.rs"`; the
engine source lives in the **repo-root `src/` tree**, outside any vendored
crate directory. maturin's sdist packer rejects `..` in `include`, so it
cannot pull that tree in — the resulting sdist would not build. Rather than
ship a broken sdist (which pip would try and fail on), we publish **wheels
only**. The matrix covers manylinux + musllinux (x86_64, aarch64), macOS
(x86_64, arm64), and Windows x64 — all mainstream targets. Niche platforms
build from a full git checkout. Restoring an sdist is a follow-up requiring
either an upstream layout change (move engine source under a crate dir) or
registry-based engine deps.

## Trigger
Publish on tag push `python-v*` (separate from Rust crate releases). The
wheel-build matrix (Phase 4) runs, then the `publish` job gathers the wheel
artifacts and uploads.

## Trusted publishing (OIDC) — preferred
Use PyPI Trusted Publishers (no long-lived token). Each of the four dist names
(`rapier2d`, `rapier3d`, `rapier2d-f64`, `rapier3d-f64`) must be registered on
PyPI with a trusted publisher pointing at this repo + workflow + environment.

```yaml
release:
  needs: [build, sdist]
  runs-on: ubuntu-latest
  permissions:
    id-token: write           # OIDC
  environment: pypi
  steps:
    - uses: actions/download-artifact@v4
      with: { path: dist, pattern: wheels-*, merge-multiple: true }
    - uses: actions/download-artifact@v4
      with: { path: dist, pattern: sdist-*, merge-multiple: true }
    - uses: pypa/gh-action-pypi-publish@release/v1
```
`gh-action-pypi-publish` uploads every dist in `dist/` regardless of name, so
all four packages publish from one job. (Alternatively `maturin upload`.)

First-time bootstrap: the four names don't exist on PyPI yet, so the very first
publish may need a token-based upload (or pre-create the projects), after which
trusted publishing takes over. Document the bootstrap step.

## Versioning
- Keep all four in lockstep on the workspace version (currently `0.32.0`).
- Prefer maturin `dynamic = ["version"]` reading from each crate's
  `version.workspace = true`, so a single workspace bump drives all four
  wheels — avoids four hand-edited `pyproject.toml` versions drifting.
- A release = bump workspace version → tag `python-vX.Y.Z` → CI publishes four.

## Determinism variant
Published wheels are the fast path (platform-native transcendentals). Users
needing cross-platform bit-reproducibility build from the sdist:
`pip install --no-binary rapier3d rapier3d` won't pass features, so document
`maturin build -F determinism` from a checkout, or
`MATURIN_PEP517_ARGS="--release -F determinism" pip install <sdist>`. No
separate `-determinism` dist for now (revisit if demand).

## Pre-publish checklist
- README.md present in each package dir (PyPI long description).
- License metadata + classifiers correct.
- `twine check` clean for all four.
- TestPyPI dry-run with a `0.32.0rcN` to validate the whole pipeline before the
  real tag.
