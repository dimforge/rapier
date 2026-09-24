# C testbed timing investigation (2026-09-18)

Measured on macOS arm64 with Rust 1.98.0, the `soft-bodies` revision
`73a93dd9b`, and the C bindings in this worktree. Both paths used release builds,
f32, four-lane SIMD, parallel support with one dedicated worker, and native
profiling. Every dynamic body was awake; sleeping was disabled.

Each row is the median of three runs, each with 120 warmup steps followed by
300 measured steps. Times are complete step-call wall time, excluding rendering,
scene construction, and serialization. C used the actual testbed core and shared
library. Rust replayed the identical initial snapshot with `PhysicsWorld::step`
and checked every final rigid-body pose and velocity against C for exact equality.

| Scene | C ms/step | Rust ms/step | C / Rust |
| --- | ---: | ---: | ---: |
| `primitives3` | 1.954 | 2.002 | 0.976 |
| `stress_tests_boxes3` | 1.932 | 1.940 | 0.996 |
| `stress_tests_boxes2` | 1.396 | 1.413 | 0.988 |
| `stress_tests_keva3` | 120.229 | 121.134 | 0.993 |

Keva contained 38,271 rigid bodies, of which 38,270 were dynamic and awake.
A further C run with parallelism compiled out measured **122.037 ms/step**
(native counter: 122.030 ms), so the earlier serial build path also had similar
per-step cost. Its loaded library reported `parallel=0`, `profiling=1`, SIMD 4.

There was no 7x per-step overhead in these matched runs. These checks isolate the
C call path; they do not establish construction parity for every ported scene.

The old C viewer accumulated real elapsed time and executed up to eight physics
steps before drawing a frame. Its original Physics label timed that entire batch.
The Rust viewer executes one step per frame and displays the native single-step
counter. Replaying the old C loop on Keva showed **746.84 ms/frame for six steps**,
while the native counter read **124.70 ms/step** in that same frame. It executed
65 steps in a 12-frame smoke run. This reproduces a large apparent slowdown
without a comparable increase in cost per physics step.

The C viewer now advances one step per frame, matching Rust. The primary Physics
label uses the native per-step counter. The Performance panel separately reports
the complete simulation call time, step count, and draw CPU time. The corrected
Keva viewer executed exactly 60 steps in 60 frames; its screenshot showed
119.04 ms/step and 119.07 ms/frame for one step.

See [reproduction instructions](README.md) and [all measured runs](timing-results.json).
