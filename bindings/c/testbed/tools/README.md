# Comparing C and native Rust step costs

`step_benchmark.c` runs the same C examples and their own physics loops as the
viewer, using a headless frame function to record timings and snapshots. `bindings/c/examples/compare_steps.rs` replays the C world's initial snapshot with
native `PhysicsWorld::step`. Both use one worker, disable sleeping, discard 120
warmup steps, and measure the next 300 steps. Setup, rendering, and serialization
are outside the measured region. Native Rust checks every final rigid-body pose
and velocity for exact equality with C, and both verify that dynamic bodies stay
awake. Use only trusted snapshots from the same checkout/configuration.

From the repository root, on a quiet machine:

```sh
cmake -S bindings/c -B build/compare3 -DRAPIER_BUILD_TESTBED=ON \
  -DRAPIER_TESTBED_GRAPHICS=OFF -DRAPIER_PROFILE=release \
  -DCMAKE_BUILD_TYPE=Release -DRAPIER_ENABLE_PARALLEL=ON -DRAPIER_SIMD_LANES=4
cmake --build build/compare3 --config Release --target rapier_testbed_step_benchmark
cargo build --release -p rapier3d-ffi --features parallel,profiler --example compare_steps \
  --target-dir build/compare3/cargo
build/compare3/testbed/rapier_testbed_step_benchmark primitives3 initial.bin final.bin
build/compare3/cargo/release/examples/compare_steps initial.bin final.bin
```

Repeat with `stress_tests_boxes3` or `stress_tests_keva3` (38,270 dynamic
bodies; a complete Keva pair takes a few minutes with one worker). For 2D, use a separate directory, set
`-DRAPIER_DIMENSION=2`, select `rapier2d-ffi`, and run `stress_tests_boxes2`.
Multi-configuration generators put the C executable under `testbed/Release/`;
Windows also needs the `.exe` suffix. The Rust example supports f32 builds only.
For a serial build, set `RAPIER_ENABLE_PARALLEL=OFF` and use Cargo's
`--features profiler`. Build both executables before timing, run them sequentially,
repeat the pair, and compare medians; do not benchmark while other builds run.

This isolates C-boundary/testbed stepping overhead for identical initial worlds.
It does not establish scene-construction parity for the whole catalog, benchmark
renderers, or support scenes with C callbacks or soft bodies. `wall_ms` measures
each complete call; `engine_ms` uses Rapier's own per-step counter, matching the
primary timing shown in both UIs. Dedicated-pool dispatch is outside that engine
counter. Snapshots are temporary benchmark artifacts, not a stable file format.
