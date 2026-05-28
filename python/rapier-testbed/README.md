# rapier-testbed

A [Panda3D](https://www.panda3d.org/)-based visual testbed and example gallery
for the [Rapier](https://rapier.rs) Python bindings. It drives both the 2D and
3D (f32) engines, so it depends on `rapier2d` and `rapier3d`.

```bash
pip install rapier-testbed
rapier-testbed            # opens the example picker
```

Set `PANDA_NO_WINDOW=1` to run an example headless (fixed number of physics
steps, no window) — used by the regression tests.

## License

Apache-2.0
