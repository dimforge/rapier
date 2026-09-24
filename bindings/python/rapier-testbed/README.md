# rapier-testbed

A [Panda3D](https://www.panda3d.org/)-based visual testbed and example gallery
for the [Rapier](https://rapier.rs) Python bindings — examples ported from
the Rust `examples3d/`. It drives the 3D (f32) engine, so it depends on
`rapier3d`.

## Install

```bash
pip install rapier-testbed
```

From a checkout of the [Rapier repo](https://github.com/dimforge/rapier) (the
engine packages aren't on PyPI yet — build them first, then install the testbed
without re-resolving them). Run this **from the repository root** (the
`./python/...` path is relative to it):

```bash
pip install panda3d numpy
pip install --no-deps -e ./python/rapier-testbed   # -e: edits picked up without reinstalling
```

## Run

Once installed, these run from **any** directory (inside the venv), since they
invoke the installed `rapier_testbed` module rather than a path:

```bash
# Interactive picker — browse every example by category (opens a window):
rapier-testbed
python -m rapier_testbed                     # equivalent

# Jump straight into one example (under `examples3`):
python -m rapier_testbed.examples3.domino3

# Headless — run a fixed number of steps with no window (no display needed;
# used by the regression tests):
PANDA_NO_WINDOW=1 python -m rapier_testbed.examples3.domino3
```

## Controls (viewer window)

| Mouse | Action | | Key | Action |
| --- | --- | --- | --- | --- |
| left-drag | rotate camera | | `Space` | pause / resume |
| right-drag | pan | | `R` | reset the scene |
| wheel | zoom | | `Tab` | next example |
| | | | `W` | toggle wireframe |
| | | | `Esc` | quit |

## License

Apache-2.0
