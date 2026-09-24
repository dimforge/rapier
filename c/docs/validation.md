# Validation record

Validated locally on 2026-09-18 on macOS arm64, with Rust 1.98.0,
Apple Clang 17, CMake 3.31.3, and cbindgen 0.29.4.

## Mainline rebase (2026-09-24)

- Rebased the 23 C-binding commits onto `master` at `79fd711a9`, excluding the
  soft-body commits already merged upstream. The normal `c-bindings` branch is
  checked out in the main repository.
- All 130 default-feature Rust binding tests and 36 full-feature 3D tests pass.
  All four native ABI variants pass C/C++ shared/static linkage, symbol checks,
  version checks, and dimension coexistence. Header regeneration is unchanged.
- Both graphical testbeds build with FEM and parallelism (3D also with robotics),
  and all 28 CTests pass. One-step, no-sleep, single-worker smoke runs pass 88 2D
  and 113 3D scenes. The remaining `debug_deserialize3` scene needs its external
  `snapshot0.bincode` fixture.
- Audited all 214 upstream vendored files against pinned upstream Git blobs and
  reviewed nested licenses. Supplemental license texts and collected notices
  are included, and both viewers copy these plus LGPL fallback-header sources
  into their redistribution directories. See `testbed/vendor/README.md`.

## C release version reporting (2026-09-24)

- `c/VERSION` defines `0.35.3+c.2` independently of ABI version 1. The shared
  Cargo build script validates the Rust-version prefix and embeds the exact
  release string. `r2Version()` / `r3Version()` return borrowed static C strings.
- CMake reads the same file, compares the numeric base for package discovery,
  and exports the full release string as `Rapier_BINDINGS_VERSION`.
- All four native ABI variants pass C/C++ shared/static linking and exact
  version-string assertions, with 538 exports in 2D and 563 in 3D. Dimension
  coexistence, strict full-feature FFI Clippy, and formatting pass.
- A fresh Release 3D CMake build passes all six integration tests. A separate
  C consumer finds the installed package and verifies its version metadata
  matches the runtime string. Header regeneration and `git diff --check` pass.

## Full-width snapshot headers (2026-09-24)

- Snapshot headers now store ABI, dimension, and scalar byte size as little-endian
  `u32` values after the `RPRS` magic. No snapshot header field is narrowed to a
  byte. The payload begins after the shared 16-byte header length.
- Regression tests cover ABI values through `u32::MAX`, including the former
  1/257 collision, every mutated header byte, truncated headers/payloads, round
  trips, and rejection of the obsolete six-byte format. Public ABI remains 1.
- All 130 default-feature Rust tests and 36 full-feature 3D tests pass. Strict
  full-feature FFI Clippy, formatting, and all four native C/C++ ABI variants
  pass, including shared/static linkage and dimension coexistence. Header
  regeneration is unchanged and `git diff --check` passes.

## Initial ABI version (2026-09-20)

The unreleased bindings now use ABI version 1. Earlier ABI numbers below record
development checkpoints, not published compatibility guarantees. Future ABI
version increments apply to incompatible releases.

All four native variants pass C/C++ shared/static linking, export checks, and
dimension coexistence with ABI 1. The 2D/3D legacy snapshot tests and C# example
pass. Header regeneration is reproducible, and `git diff --check` passes.

## ABI 15: explicit free-function names (2026-09-20)

- Renamed `RemoveBody`, `InsertDeformable`, and `ActiveBodies` to
  `RemoveRigidBody`, `InsertDeformableCollider`, and `ActiveRigidBodies`.
  Integration-parameter accessors and world serialization now name their subject.
  `TimeStep` / `SetTimeStep` replace `Dt` / `SetDt`.
- All five collection `Len` functions now use `Count`, including callback-scoped
  rigid-body and collider queries. All 14 renames apply to both dimensions;
  replaced exports are absent and the ABI checks explicitly reject them.
- Migrated examples, tests, helpers, tools, and current documentation. Internal
  Rust binding entry points follow the same names; native Rapier methods retain
  their Rust naming conventions.
- All 122 default-feature Rust binding tests, three naming tests, and 34
  full-feature 3D tests pass. Formatting and strict full-feature FFI Clippy pass.
- Four native ABI variants pass C11/C++17 shared/static linking, exact export
  checks (537 in 2D, 562 in 3D), and dimension coexistence.
- Both Release viewers rebuild with all demos; all 28 CTests pass. The C#
  example passes with ABI 15. Header regeneration is byte-for-byte reproducible,
  and `git diff --check` passes.

## ABI 14: collider insertion from parent handles (2026-09-20)

- `InsertCollider(parent, &desc)` takes the parent handle by value and uses its
  world pointer. `InsertColliderWithoutParent(world, &desc)` creates a collider
  without a rigid-body parent. The nullable parent-pointer signature is removed.
- All demos, native tests, and documentation use the new signatures. Regression
  tests cover parent ownership, independent worlds, standalone colliders, null
  worlds, and invalid or removed parents without unintended insertion.
- All 122 default-feature Rust binding tests and three naming tests pass. All 34
  full-feature 3D tests, formatting, and strict full-feature FFI Clippy pass.
- Four native ABI variants pass C11/C++17 shared/static linking, exact export
  checks (537 in 2D, 562 in 3D), and dimension coexistence.
- Both Release viewers build and all 28 CTests pass. One-step, no-sleep,
  single-worker smoke runs pass all 88 2D demos and 113 3D demos; the remaining
  `debug_deserialize3` demo requires the external `snapshot0.bincode` fixture.
- The C# example passes with ABI 14. Header regeneration is byte-for-byte
  reproducible, and `git diff --check` passes.

## ABI 13: separate rigid-body and collider insertion (2026-09-20)

- Removed `r2Insert` / `r3Insert` and the `BodyCollider` result type. The body-only
  function is now `InsertRigidBody`. Examples insert the body, then call
  `InsertCollider` with its handle. No combined helper or compatibility alias remains.
- All demos, native tests, C++ helpers, the C# example, and documentation use the
  new sequence. Errors are checked between dependent calls in standalone examples
  and native tests. Collider insertion failure leaves an already-created body intact;
  an updated regression test verifies this behavior.
- All 118 default-feature Rust binding tests and three naming tests pass; all 33
  full-feature 3D tests and strict Clippy pass. Four native ABI variants pass
  C11/C++17 shared/static linkage, exact symbol checks (536 in 2D, 561 in 3D),
  and dimension coexistence. The ABI audit rejects the removed function/type.
- Both Release viewers build, and all 28 CTests pass. One-step, no-sleep,
  single-worker smoke runs pass all 88 2D demos and 113 3D demos. The remaining
  `debug_deserialize3` demo requires the existing external `snapshot0.bincode` fixture.
- C# P/Invoke passes with ABI 13. Header regeneration reproduces byte-for-byte,
  and `git diff --check` passes.

## ABI 12: configuration values and resource ownership (2026-09-20)

- URDF/MJCF options are copyable PODs with embedded body/collider descriptions.
  Default factories replace allocation, setters, and destructors. Options validate
  before file I/O and borrow blueprint geometry only through loading. `PodLayout`
  exposes their sizes (zero when robotics is unavailable).
- Removed redundant `Data` suffixes from nine configuration types and normalized
  material/description getter-setter pairs. `UserData` and the owned `TriMeshData`
  mesh buffer retain meaningful names. Public compatibility aliases are absent.
- Collider, callback collider, and MJCF visual shape getters now say `CloneShape`
  and document the owned wrapper. C++ owners cover all 14 remaining resource types.
  Demos, helpers, tests, documentation, and C# layout declarations are migrated.
- All 118 default-feature Rust binding tests and three naming tests pass. All 33
  full-feature 3D tests pass, including POD default preservation, independent copies,
  validation before I/O, borrowed geometry retention, URDF blueprint loading, and
  MJCF keyframes/actuators. Formatting and strict full-feature FFI Clippy pass.
- Four native ABIs pass C11/C++17 shared/static linking and dimension coexistence.
  Default exports remain 537 in 2D and 562 in 3D; the full-feature 3D header exactly
  matches all 601 exports. All 14 destructors have matching C++ owner aliases.
- Both Release viewers build with all demos; all 28 CTests pass. The final C++
  tests verify shape-clone lifetime after collider removal in both dimensions.
  URDF and MJCF demos pass one-step, no-sleep smoke tests with one worker.
- The .NET 8 example passes with ABI 12. Header regeneration is byte-for-byte
  reproducible, and `git diff --check` passes.

## ABI 11: constructor and destructor names (2026-09-20)

- Renamed 94 exported constructors/destructors, with no compatibility aliases:
  `NewWorld`, `FreeWorld`, `DynamicRigidBodyDesc`, `CuboidColliderDesc`, and
  `DefaultQueryOptions` illustrate the convention. All 16 destructors use `FreeType`.
  The inline pure-translation constructor is now `TranslationPose`.
- Instance methods retain `Type_Method`; loading constructors retain `TypeFromFile`.
  All demos, C/C++ helpers, tests, and the C# sample use the new names. The Rust
  implementation uses the same word order without additional export-macro logic.
- All 118 Rust binding tests across 2D/3D and f32/f64 pass, along with three naming
  tests. Formatting and strict FFI/macro Clippy checks pass.
- All four native variants pass C11/C++17 shared/static linking, exact export
  checks (537 in 2D, 562 in 3D), and 2D+3D coexistence. Header regeneration is
  byte-for-byte reproducible; no old lifecycle references remain in current source.
- Both Release viewers build with all demos and pass all 28 CTests. The .NET 8
  sample passes with ABI 11, reporting y=0.074563645 after 60 steps.

## ABI 10: instance-method names (2026-09-20)

- 414 instance methods use a separator between their receiver and method, for example
  `r3RigidBody_SetTranslation` and `r3JointDesc_SetMotorPosition`. Constructors,
  static helpers, and short world functions retain their existing names.
- The Rust export attribute records the receiver explicitly. The header generator
  reads the same annotation; obsolete method exports and aliases are absent.
  All C demos, shared testbed code, C++ helpers, and the C# example are migrated.
- Three macro tests cover method names, unchanged constructors/helpers, and invalid
  annotations. All four dimension/precision variants pass C11/C++17 shared/static
  linking, exact export checks (537 in 2D, 562 in 3D), and 2D+3D coexistence.
- Both Release testbeds build with all demos; all 28 CTests pass. The .NET 8
  example passes with ABI 10, reporting y=0.074563645 after 60 steps.
- Header regeneration is byte-for-byte reproducible. No obsolete method names
  remain in tracked source consumers. `git diff --check` passes.

## ABI 8: direct value returns (2026-09-19)

- 313 value-producing operations now return their results directly. Related outputs
  use POD aggregates; array fills retain caller buffers and return the element count.
  Mutators without a produced value still return status codes. Old signatures are removed.
- `LastStatus` records the most recent fallible call on each thread. Error callbacks
  cover both calling conventions, and nested callbacks preserve the original status
  and diagnostic. Failures return type defaults; short-buffer errors retain the required
  count. Infallible constructors and status/diagnostic reads preserve the recorded error.
- All 98 Rust binding tests pass across 2D/3D and f32/f64, both with default features
  and with FEM + parallel. Strict FFI Clippy passes in both configurations. Tests cover
  panic fallback, stale handles, thread-local status, callback reentrancy, and buffers.
- All four native variants pass C11/C++17 shared/static linking, complete symbol checks,
  and 2D+3D coexistence. Default exports: 537 in 2D and 562 in 3D. The generated header
  reproduces byte-for-byte; the ABI audit rejects scalar output-pointer declarations.
- The .NET 8 consumer passes with ABI 8, reporting y=0.074563645 after 60 steps.
  Both Release viewers and step benchmarks build; all 28 testbed tests pass.
- Independent three-step, no-sleep, one-worker runs pass all 88 2D demos and 113/114
  3D demos. `debug_deserialize3` still requires the external `snapshot0.bincode` fixture.

## Dimension-specific C names (2026-09-19)

- Public types and constants use `R2`/`R2_` or `R3`/`R3_`, with no old-name
  aliases. All demos and consumers use the new names. Shared C/C++ support uses
  `RAPIER_FN`, `RAPIER_TYPE`, and `RAPIER_CONST` selectors.
- The generator translates the shared Rust names without duplicating the Rust
  implementation. This source rename preserves layouts, symbols, and ABI 7.
- All four dimension/precision variants pass C11/C++17 shared/static linking,
  export checks, and 2D+3D coexistence. The audit rejects obsolete or wrong-dimension
  type and constant names. Header regeneration is reproducible.
- Both Release testbeds build with all demos and pass all 28 CTests.

## ABI 7: world ownership and scoped callback access (2026-09-19)

- The public C API exposes one simulation owner, `RprWorld`. Sets, pipelines,
  component getters, duplicate world accessors, and borrowed query views are
  removed. New entry points include `WorldNew`, `Step`, `InsertBody`,
  `RigidBodySetTranslation`, and `CastRay`; query options are owner-independent PODs.
- All C demos, the viewer, benchmarks, tests, C++ helpers, and the C# example use
  the new interface. Export checks explicitly reject obsolete public types and
  symbol prefixes; the generated header reproduces byte-for-byte.
- The world uses a nonblocking shared/exclusive access gate. Conflicting calls
  return `RPR_WORLD_BUSY` before native borrows are created. Tests cover hook-scoped
  reads, contact edits, rejected recursive stepping/mutation/destruction, nested
  read-only queries, cross-thread conflicts, and guard release after errors/panics.
- Rust `PhysicsWorld` now owns collision-only workspace and provides
  `detect_collisions` and `remove_body_with_colliders`. The C API uses these methods;
  collider-preserving removal and collision refresh without time advancement are tested.
- All 90 Rust binding tests pass across 2D/3D and f32/f64 with default features and
  with FEM + parallel. Strict FFI Clippy passes for both configurations.
- C11/C++17 shared/static behavior, POD layouts, all exports, and 2D+3D coexistence
  pass for all four native variants. Default export counts are 536 in 2D and 561 in 3D.
  C callbacks exercise scoped reads. The .NET 8 C# consumer passes with ABI 7,
  reporting y=0.074563645 after 60 steps.
- Both Release viewers and the standalone step benchmarks build. Both viewers pass
  all 14 CTests. Three-step no-sleep, one-worker smoke runs pass 88/88 2D and 113/114
  3D demos. The remaining `debug_deserialize3` demo needs the external
  `snapshot0.bincode` fixture; it is not counted as a pass.

## ABI 6: description constructors return values (2026-09-19)

- Collider, joint, and soft-body description constructors return POD values
  directly. Uniform material and volume-meshing parameter constructors do too.
  The 2D revolute constructor takes no axis, matching native Rust.
- All demos, standalone C/C++ examples, tests, and documentation use the new
  signatures. Output-pointer constructor signatures are removed. Invalid input
  is preserved in descriptions and rejected during build/insert or preview;
  construction does not allocate geometry or invoke the error callback.
- All 74 Rust tests pass across 2D/3D and f32/f64. Added coverage compares native
  constructor values and checks deferred validation of invalid shapes and axes.
  Strict FFI Clippy passes in all four default variants.
- All four variants pass C11/C++17 shared/static tests, symbol checks, layout
  checks, and dimensional coexistence. The C# example runs with the ABI 6 check.
- Both Release viewers build and pass all 14 CTests. Independent three-step,
  no-sleep, one-worker runs pass all 88 2D demos and 113/114 3D demos; the remaining
  snapshot demo requires the external `snapshot0.bincode` fixture.

## ABI 5: complete example migration and API cleanup (2026-09-19)

- Every C testbed example now uses POD construction, set-and-handle element
  access, native value initializers, and typed geometry inputs where applicable.
  The public header contains no owned builders, element-pointer accessors, or
  allocated query wrappers. Their replacement is mandatory, with no aliases.
- Shape, soft-body, binding, compound, and heightfield descriptions use typed
  views. Shared-shape geometry constructors use the same view types. Insertion
  copies borrowed inputs; examples retain temporary arrays and shared shapes
  through the last insertion that reads them.
- All four default variants pass C11/C++17 shared/static linking, ABI and POD
  layout checks, export checks, and 2D+3D coexistence. Default exported function
  counts are 560 in 2D and 584 in 3D.
- All 70 Rust tests pass with default features and again with FEM + parallel.
  Tests compare procedural recipes against native Rust defaults, check stale
  handles and failure atomicity, and cover infinite one-sided joint limits.
  Strict FFI Clippy passes for all variants with both feature configurations.
- Both Release viewers build and pass all 14 CTests, including dragging,
  deformable sensor rendering, UI, and threading. Builds use SIMD4 + parallel +
  FEM; 3D also enables robotics.
- Independent catalog runs pass 88/88 2D and 113/114 3D demos for three steps,
  with sleeping disabled and one physics worker. `debug_deserialize3` reports
  the missing external `snapshot0.bincode` fixture. These are smoke tests for
  stepping and finite state, not proof of numerical identity to every Rust demo.
- The .NET 8 C# example passes with ABI 5, reporting y=0.074563645 after 60 steps.
  Generated header reproduction and whitespace checks pass.

The entries below describe historical validation before ABI 5; their old API
names and export counts do not describe the current interface.

## Handle-based access (2026-09-19)

- All four native variants pass C/C++ tests, shared/static linkage, export checks,
  and dimension coexistence (758 exports in 2D, 781 in 3D).
- Handle tests cover storage growth, stepping, slot reuse, collider removal,
  soft-proxy restrictions, error callback delivery, invalid input preservation,
  batch capacity checks and all-or-nothing output on stale handles.
- Strict FFI Clippy passes in all variants. The C# example uses no borrowed body
  pointer and runs successfully; generated header reproduction passes.

## ABI 4: explicit setter names (2026-09-19)

- Property setters consistently include `Set`, including all owned builders and
  bulk POD configuration. Incremental translation and clearing use action verbs.
- Updated exports, header, examples, testbed, tests, C# ABI check, and documentation.
  Source audit found no references to replaced names; no legacy aliases remain.
- All four variants pass shared/static C/C++ ABI tests and dimension coexistence;
  62 Rust unit tests and strict FFI Clippy checks pass. Header generation is stable.
- Both release viewers build and pass all 11 CTests; the C# example runs against
  ABI 4. This is a development ABI change requiring consumers to rebuild.

## POD construction and configuration (2026-09-19)

- All four default ABIs pass C11/C++17 behavior tests, POD layout checks, every
  declared symbol, shared/static linking, and 2D+3D coexistence. Default exports
  are now **670 in 2D** and **693 in 3D**.
- 62 Rust tests pass across the four variants, both with default features and
  with FEM + parallel enabled. New tests compare construction
  defaults and shape/soft recipes against native Rust values, verify copied array
  lifetimes, reject recursive compound descriptions, and check failure atomicity.
- The C POD suite exercises direct set/world insertion, joint descriptions,
  integration read/apply, material read/apply, borrowed query predicates, shared
  shape retention, and deformable binding/vertex input lifetimes.
- Both Release f32 viewers pass all **11 CTests**, including mouse dragging,
  soft sensor rendering and UI behavior. Builds include parallel + SIMD4 + FEM;
  the 3D build also includes robotics.
- Full catalog smoke run: **88/88 2D** and **113/114 3D** demos pass three steps
  with sleeping disabled and one worker. The remaining `debug_deserialize3`
  requires the external `snapshot0.bincode` input; its failure reports the missing
  file. This smoke test checks successful stepping and finite state, not numerical
  identity of every demo with Rust.
- The updated C# example compiles/runs under .NET 8 using a blittable body
  description, verifies its native size, and produces y=0.074563645 after 60 steps.
- Strict default and FEM/parallel FFI Clippy checks pass for all four variants.
  Header regeneration is reproducible. No engine Rust source or existing
  object layout was changed; owned builder APIs remain available. ABI 4 subsequently
  renames setters to include `Set`.

## Deformable sensor transparency

- Deformable triangles and polyline ribbons preserve alpha and share the sorted
  transparency pass with rigid sensors, with depth writes disabled until flush.
- Both Release viewers pass all ten CTests. The renderer regression now checks
  the real Cluster meshes sensor and synthetic 2D deformable sensors: alpha 102,
  no transparent draws in the opaque pass, depth ordering, and depth-write restore.
- Inspected a 30-frame Cluster meshes capture: the deformable sensor shell reveals
  its enclosed solid mesh.

## Testbed controls and rendering

- Both Release f32 viewers pass all ten CTests. Mouse-grab coverage includes dynamic
  rigid bodies, articulated links, soft particles, sensor/fixed-body exclusion,
  release cleanup, and deletion during a drag. The UI test also checks filtered
  previous/next navigation and orbit-camera distance, target, panning, and zoom.
- Inspected rendered 2D deformable polylines and the 3D sensor demo with its solid
  collider visible through the transparent sensor surface.
- All four default variants pass shared/static C/C++ linkage, export comparison,
  and dimension coexistence. Default exports are 617 in 2D and 640 in 3D after
  adding soft-body ownership, cluster-proxy, and closed-mesh accessors.
- Clippy with `--no-deps -- -D warnings` passes for all four default FFI variants.

## Soft-mesh renderer follow-up

- Fixed renderer cache indexing for meshes without a collider. Mesh enumeration
  now exposes native mesh IDs so render-only skins remain independently accessible.
- Both Release viewers pass all nine CTests. The added renderer regression runs
  actual soft scenes without a GPU, bounds cache allocation, checks finite geometry,
  and verifies that queued triangles flush before back-face culling is restored.
- The Cluster meshes and Soft trimeshes windows ran for 30 and 90 frames,
  respectively; the latter screenshot was inspected after the two-sided fix.
- Both 3D precision variants pass all 12 Rust boundary/regression tests, including
  direct comparison of render-only mesh vertices and indices with the native API.
- All four default variants pass shared/static C/C++ linkage, export comparison,
  and dimension coexistence. Default exports are now 614 in 2D and 637 in 3D.
- Formatting and Clippy with `--no-deps -- -D warnings` pass for the updated bindings.

## ABI 3: complete example catalog

- All 202 catalog entries now have C ports: 88 2D and 114 3D. Added 62 entries.
- Release f32, SIMD4, one worker, sleeping disabled: all 201 scenes with available
  inputs passed three steps. This includes 2D/3D FEM, URDF, Cassie MJCF, and a
  locally installed Menagerie model. `debug_deserialize3` needs external snapshot
  files; its legacy rigid-state reader passed a generated-state round-trip and
  subsequent-step comparison in all four native variants.
- Longer runs exercised tearing (900 steps), self-intersection (260 steps),
  one-way platforms, soft joints, vehicle controllers/joints, shape replacement,
  articulated joints, ray casting, and OBJ-based scenes. These check execution
  and finite state; they are not a blanket proof of C/Rust numerical identity.
- All four FFI variants passed 11 Rust boundary/regression tests each. The 3D/f32
  `fem,robotics` configuration passed 12, including imported keyframes and
  actuator controls.
- Both Release viewers built with their optional features. All eight CTests
  passed per dimension, including native C/C++, ImGui input, worker/snapshot
  handling, and the new tests driving real IK, kinematic/PID, and voxel-edit loops.
- All four default ABIs passed shared/static C and C++ linkage, all-symbol
  linkage, export comparison, and 2D/3D coexistence. Default exports: 611 in 2D,
  634 in 3D. Optional-feature exports also matched their headers: 619 for 2D/FEM,
  699 for 3D/FEM/robotics/parallel.
- All 2D and 3D scene sources passed strict f64 C11 compilation with FEM enabled.
  Robotics stays 3D/f32, matching the native importer crates.
- Clippy passed with `--no-deps -- -D warnings` for all four default variants
  and for 3D/f32 with FEM/robotics. Existing engine dependency warnings remain.
- The raylib Menagerie viewer was run and its screenshot inspected for model
  framing, smooth normals, materials, and texture rendering.

The catalog counts implemented scenes, not external asset availability. FEM and
robotics remain opt-in build features. No new platform/engine certification is
implied by these local macOS checks.

## ABI 2: dimension prefixes and camelCase

- All four FFI crates: 28 boundary/validation tests passed. The export macro's
  2 naming tests also passed.
- `cargo clippy` for the export macro and all four FFI crates with
  `--no-deps -- -D warnings`: passed. Existing engine dependency warnings remain.
- `cargo fmt -p rapier-c-macros -p rapier3d-ffi -- --check`: passed.
- Regenerating `rapier.h` produces an identical header.
- `python3 c/tools/test-native.py`: every variant passed the C behavioral suite,
  C++ ownership suite, and declared-symbol linking with shared and static
  libraries. The default headers declare **564 functions for 2D** and
  **582 functions for 3D**; optional features add more.
- Dynamic exports match the preprocessed declarations exactly. No old `rpr_*`
  exports or opposite-dimension exports remain. Both 2D and 3D link and run in
  one executable, tested with shared/static linkage and f32/f64 separately.
- Inline math headers: all 8 combinations of C11/C++17, 2D/3D, and f32/f64 passed
  strict compilation and execution (`-pedantic -Wall -Wextra -Werror`).
- `cargo check` with `parallel,fem,enhanced-determinism` passed for all four FFI
  crates. This checks feature compatibility, not solver numerical behavior.
- Release 3D/f32 parallel + SIMD4: all 7 CTests passed, including ImGui input,
  example-owned loops, pause/step, worker changes, snapshots, and error reporting.
- Release 2D/f32 parallel + SIMD4, headless: all 6 CTests passed.
- The installed shared CMake package was consumed by a separate C++ application.
- `examples/RapierNative.cs` compiled under .NET 8 and ran through the renamed
  P/Invoke entry points against the rebuilt 3D/f32 library; y=0.074563645 after
  60 default steps. A previous native library beside the application had to be
  replaced with the ABI 2 build, as required for renamed entry points.

The C behavioral tests also cover explicit pipeline construction, ownership,
collision/force events, hooks, contact inspection, ray/shape queries, filtering,
heightfields, mass properties, buffer sizing, invalid inputs, snapshots, joints,
soft-body particles and meshes, cutting/tear events, character movement,
3D vehicles, debug lines, removal cascades, and stale handles.

## Earlier execution and packaging checks

The following checks predate the ABI 2 naming change:

- Installed shared/static CMake packages, including relocation of the shared
  package before consumption, passed on macOS with `@rpath` loading.
- Debug 2D/f32 serial + SIMD4: all 6 CTests passed.
- Debug 3D/f32 parallel + SIMD8, headless: all 5 CTests passed.
- CMake rejected unsupported SIMD widths, SIMD8/f64, and SIMD8 with enhanced
  determinism before invoking Cargo.
- All 140 existing demo ports preserved their initial/final physics state when
  moved to direct API calls and example-owned loops. Animated scenes ran for
  260 steps; the other scenes ran for 3 steps.
- The Keva viewer's per-step timing uses the same native counter as the Rust UI.
  See the [C/Rust comparison](../testbed/tools/timing-results.md) for measurements,
  raw runs, and reproduction instructions.

The workflow covers Linux, macOS, and Windows builds plus installed-package
consumers. Remote CI has not been run for this change. Unity editor/IL2CPP,
Unreal, mobile targets, and consoles need tests in their target projects.

## Value initialization helpers (2026-09-19)

The C11 and C++17 initialization tests pass with shared and static libraries for all four dimension/precision variants. Release CMake initializer tests also pass in both dimensions with FEM enabled. These exercise descriptor insertion, configuration application, invalid handles, and query defaults.

## Typed array views (2026-09-19)

- C11 and C++17 tests pass for all four dimension/precision variants with shared
  and static linkage, export checks (771 in 2D, 796 in 3D), and dimension coexistence.
- View tests exercise edge/triangle/cell counts, soft surface/skin insertion,
  copied input lifetimes, null/alignment/length errors, unchanged descriptions on
  failure, and topology validation at insertion.
- All 62 Rust binding tests and strict FFI Clippy pass. Header regeneration is
  reproducible; the existing ABI and descriptor layouts are unchanged.
- Both release viewers rebuild with FEM, parallelism and SIMD4 and pass all
  14 CTests per dimension. The updated polyline2 and debug_trimesh3 examples each
  pass 120 steps with sleeping disabled and one worker.
