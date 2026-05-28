# Phase 6 — Docs & cleanup

## Sphinx docs
The Sphinx project (`python/rapier-py/docs/`) is written around the umbrella
`rapier` package (`dim_scalar.rst`, `getting_started.rst`, autodoc of
`rapier.dim3` …). Decide:
- **Option A (recommended):** one combined docs site documenting all four
  packages side by side, with a "choosing a package" page replacing
  `dim_scalar.rst`. Autodoc each top-level package. Lives at
  `python/docs/`.
- Option B: per-package mini-docs. More overhead; skip unless desired.

Rewrite:
- `getting_started.rst`: `pip install rapier3d`, `import rapier3d`.
- `dim_scalar.rst` → "which package": table of the four dists.
- `limitations.rst`: drop the "snapshots are wheel-flavor specific" umbrella
  caveat (now snapshots are package-specific by construction); keep the
  determinism note (Phase 5).
- `changelog.rst`: document the breaking API move (umbrella → four packages).

## READMEs
Write a short README per package (PyPI long description) from a shared
template, differing only in dim/scalar and install line. The root
`python/README.md` becomes an index pointing at the four.

## Remove dead artifacts
- `python/rapier-py/` (umbrella crate + pyproject + bundled `.so`/`.pyi` blobs).
- `scripts/build_all_cdylibs.sh`.
- The old `.github/workflows/python-bindings.yml` `maturin develop` job
  (replaced by Phase 4 build+test).
- Any `include = [...]` blocks (gone in Phase 1).
- Stale `__pycache__`, committed `*.so` test artifacts under the old tree.

## Final sweep
- `grep -rn "from rapier import\|rapier.dim2\|rapier.dim3\|_rapier3d as" python/`
  returns nothing outside the four packages' internal `_ext` imports.
- Workspace `Cargo.toml` has no `python/rapier-py` member.
- CI: lint + per-package test + wheel matrix + publish all reference the four
  crate dirs and nothing named `rapier-py`.
