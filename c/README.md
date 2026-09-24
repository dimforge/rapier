# Rapier C bindings

C11 ABI for Rapier, with optional C++17 ownership helpers. A world owns all
simulation components. Live bodies, colliders, joints, and soft bodies are accessed
through generational handles that contain their owning world pointer. Construction descriptions and query
options are plain C values; no builders or component pointers need cleanup.

This implementation targets Rapier’s `master` branch. ABI version **1** uses
explicit operation names, collection counts, and `TimeStep` accessors. Entity
handles embed a borrowed world pointer, avoiding redundant world parameters. Newly
produced values return directly. A single world owns the simulation;
callback-scoped read access and runtime borrow checks protect access to it.
There are no compatibility aliases. Rebuild consumers using the matching header,
library, dimension, precision, and feature configuration.

See [API reference](#api-reference) for searchable Doxygen documentation and build instructions.

## C names

Functions use a dimension prefix and camelCase: `r2NewWorld` in 2D and
`r3NewWorld` in 3D. Instance methods separate the receiver type and method with
an underscore: `r3RigidBody_Position`, `r3Collider_SetFriction`, and `r3JointDesc_SetMotorPosition`.
Creation and destruction are lifecycle exceptions: `r3NewWorld()` and
`r3FreeWorld(world)`. Named constructors put the qualifier before the type, such as
`r3DynamicRigidBodyDesc()`, `r3CuboidColliderDesc(halfExtents)`, and
`r3DefaultQueryOptions()`. Loading and conversion constructors retain their
`TypeFromSource` spelling, for example `r3UrdfRobotFromFile(path, &options)`.
Static math helpers and whole-world operations omit the receiver separator,
for example `r3VectorAdd`, `r3Step`, and `r3InsertRigidBody`. Documentation below
uses 3D names unless noted;
2D uses the same suffix with `r2`. Types use PascalCase with the corresponding
`R2` or `R3` prefix, for example `R2Vector` and `R3ColliderDesc`. Constants use
`R2_` or `R3_`, for example `R2_OK` and `R3_DYNAMIC`. There are no legacy type
or constant aliases. New construction/configuration POD fields
use camelCase. Existing ABI fields retain their original Rust spelling.

Property setters include `Set`, for example `r3RigidBody_SetTranslation` and
`r3Collider_SetFriction`. Entity operations take a handle that identifies its world. Whole-world operations
and standalone insertion still take an explicit world. Operations combining entities
reject handles from different worlds. `World` appears only in lifecycle operations:
`r3NewWorld`, `r3FreeWorld`. Simulation uses `r3Step`, queries use `r3CastRay`, and
insertion uses `r3InsertRigidBody` followed by `r3InsertCollider` to attach a collider. Descriptions expose fields directly and use setters for compound operations,
such as `JointDesc_SetMotorPosition` and `ShapeDesc_SetTrimesh`.

Dimension-specific examples call the concrete functions directly. Shared C/C++
code compiled for either dimension can use `RAPIER_FN(NewWorld)`,
`RAPIER_FN(RigidBody_SetTranslation)`,
`RAPIER_TYPE(World)`, and `RAPIER_CONST(OK)`. These select the corresponding
function, type, and constant using `RAPIER_DIM2`/`RAPIER_DIM3`, with no wrapper
function or extra call. Dimension-neutral import and calling-convention macros
are named `RAPIER_API` and `RAPIER_CALL`. Inline math functions use
the same convention, for example `r2Vector`, `r3Vector`, and `r3TranslationPose`.

Rust implementations use snake_case identifiers. The
`#[rapier_export]` attribute in `rapier-c-macros` chooses the exported C symbol.
Instance methods specify the receiver, for example `#[rapier_export(rigid_body)]`;
constructors, destructors, and static helpers leave the attribute empty. Callback read methods
retain their `Read` prefix, for example `r3ReadRigidBody_Position`. The macro
has no third-party dependencies. The header generator translates the same
function names and receiver annotations, and maps the shared Rust `Rpr`/`RPR_` type and constant names to
each C dimension. This keeps the Rust implementation shared. ABI tests compare
every generated declaration with the binary exports. The dimension-specific type and constant prefixes do not affect binary layouts
or function symbols.

## Build

From the repository root:

```sh
cargo build --release -p rapier3d-ffi
# Also available: rapier2d-ffi, rapier3d-f64-ffi, rapier2d-f64-ffi.
```

Each crate produces a shared library and static library in `target/release`.
Library names replace hyphens with underscores, e.g. `librapier3d_ffi.so`,
`librapier3d_ffi.dylib`, or `rapier3d_ffi.dll`. On Windows use the MSVC Rust target
with MSVC C/C++ consumers. The dynamic import library is `rapier3d_ffi.dll.lib`;
the static library is `rapier3d_ffi.lib`.

The checked-in `include/rapier.h` needs no generator when consumed. Define one
of `RAPIER_DIM2` / `RAPIER_DIM3` and one of `RAPIER_F32` / `RAPIER_F64` before
including it. Defaults are 3D and f32. For static linkage define `RAPIER_STATIC`.
2D and 3D can be linked together, with each dimension's header configuration in
separate translation units. Select one scalar precision per dimension: f32 and
f64 of the same dimension share symbol names. Objects must only be passed to
the dimension and precision that created them.

The bindings are unreleased and use ABI version 1 throughout initial development.
Headers and libraries must come from the same revision; the ABI number does not
distinguish development revisions. After the first release, incompatible releases
will increment the ABI version.

Before any other call that passes vectors or poses, check the build:

```c
r3CheckAbi(R3_ABI_VERSION, R3_DIMENSION,
              sizeof(R3Real), sizeof(R3Vector), sizeof(R3Pose));
```

Check its return status. `r3Version()` returns the loaded C library's release
version, currently `"0.35.3+c.2"` (`r2Version()` in 2D). The returned string is
borrowed and must not be freed. `c/VERSION` is the source of this version for Rust
and CMake builds: keep the Rust crate version as the base and increment `c.N` for
C bindings releases against that version. The build rejects mismatched Rust versions.
This release identifier is separate from ABI version 1. SemVer treats `+c.N` as
build metadata, so it does not establish dependency upgrade ordering. CMake uses
the numeric base for `find_package` comparisons and exposes the full string as
`Rapier_BINDINGS_VERSION` in the installed package.

`r3BuildInfo` reports dimension, scalar width, pointer
width, and ABI version without using dimension-dependent arguments.
`r3BuildProfile()` returns a borrowed, static string containing the loaded
library's Cargo profile category (`"release"` or `"debug"`). It is independent of the
C/C++ consumer's build mode. Custom Cargo profiles report their inherited category;
per-package optimization overrides do not change that profile name.
`r3BuildFeatures` reports the solver SIMD lane count and whether parallel
execution and profiling are available through the loaded C library.

### CMake and installation

```sh
cmake -S c -B build/c -DRAPIER_DIMENSION=3 -DRAPIER_PRECISION=32
cmake --build build/c --config Release
ctest --test-dir build/c -C Release --output-on-failure
cmake --install build/c --prefix /your/sdk/rapier
```

Use `-DRAPIER_SHARED=OFF` for static linkage, `-DRAPIER_PROFILE=debug` for a debug
Cargo build, and `-DRAPIER_FEATURES=fem,enhanced-determinism` for additional Rust
features. Select execution features explicitly:

- `-DRAPIER_ENABLE_PARALLEL=ON` or `OFF`: enables Rayon and thread-pool control.
  Defaults to ON when building the testbed, OFF for bindings alone.
- `-DRAPIER_SIMD_LANES=4` (default) or `8`: this branch always uses SIMD and has no
  scalar solver build. Eight lanes require f32 and exclude `enhanced-determinism`.
  Hardware instruction width depends on the target CPU; eight lanes do not imply
  native eight-lane instructions or better performance on every machine.

For direct Cargo builds, use `--features parallel` and optionally `simd8` on the
f32 crates. The older `RAPIER_FEATURES=parallel,simd8` spelling initializes the
explicit options on the first CMake configuration; the explicit cache options
control subsequent configurations.

`r3SetNumThreads(world, count)` sets a dedicated pool per
world: 0 selects Rayon's automatic count, 1 selects one worker. Changes take effect
on the next step and must be made between steps. `r3NumThreads`
reports its actual size (0 means no dedicated pool; 1 in a build without parallel
support). `r3ClearThreadPool` returns to the calling/global Rayon
pool; it does not force serial execution. Configuration APIs return
`R3_UNSUPPORTED` when parallel support is compiled out. Pools are not serialized;
configure them again after restoring a snapshot.

`r3SetCountersEnabled` enables native profiling, and
`r3StepTimeMs` reports the last engine step using the same
counter as the Rust testbed. This requires `--features profiler` (or
`RAPIER_FEATURES=profiler`); CMake enables it automatically for the testbed.
Counters are disabled by default in newly created worlds. The timer excludes
rendering, callbacks outside the physics step, and dispatch into the dedicated pool.

`RAPIER_TARGET` accepts a Rust target
triple for cross builds; install that target and configure CMake's matching
compiler/toolchain. Rust's linker must also be configured for the target. Cross
builds are supported by the build configuration, not locally verified for every
target. Disable tests/examples for targets that cannot run on the build host.

CMake builds Cargo automatically and provides `Rapier::rapier`, with the selected
ABI definitions and native system libraries. It supports `add_subdirectory(c)`
or an installed package:

```cmake
find_package(Rapier CONFIG REQUIRED)
target_link_libraries(your_game PRIVATE Rapier::rapier)
```

Install each dimension/precision/configuration to a separate prefix. CMake builds
into its own `cargo` directory by default; `RAPIER_CARGO_TARGET_DIR` can reuse an
existing Cargo target directory. On macOS the shared library has an `@rpath`
install name; configure your application's runtime library search path for
redistribution. On Windows deploy the DLL beside the executable or plugin.

## Construction with caller-owned descriptions

Prefer caller-owned descriptions for construction. Initialize them using the API,
edit their fields, then insert directly into a world:

```c
R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
R3ColliderDesc collider = r3BallColliderDesc(0.5f);
body.position.translation.y = 5.0f;
body.canSleep = 0;
R3RigidBodyHandle handle = r3InsertRigidBody(world, &body);
// Check r3LastStatus() before using handle unless a fail-fast error handler is installed.
R3ColliderHandle colliderHandle = r3InsertCollider(handle, &collider);
// Check r3LastStatus() again.
// No builder, body, or collider temporary needs freeing.
```

Insert the rigid body first, then pass its handle by value to
`r3InsertCollider(body, &desc)`. The function uses the world stored in that handle.
Repeat collider insertion to attach multiple colliders to the same body.
For a collider without a parent, use `r3InsertColliderWithoutParent(world, &desc)`. Each call validates its own input;
if collider insertion fails, the rigid body remains in the world and may be
reused or removed with `r3RemoveRigidBody`.

Description constructors and copying descriptions need no heap allocation.
Constructors return plain values and defer validation to build/insert (or a
soft-geometry preview). They do not report errors or substitute valid defaults
for invalid arguments. You can edit a description before passing it to a
fallible operation. Invalid joint axes produce invalid frames that are
rejected during description validation. Insertion still
allocates the native simulation objects and geometry it needs.

These are ordinary C values. Assignment copies them; they can live on the stack,
in an engine component, or in a language's blittable struct. Always call the
matching initializer: `{0}` does not produce Rapier defaults (in particular,
rotations, collision groups, and query exclusions need initialization).

| Value | Initialization and use |
| --- | --- |
| `R3RigidBodyDesc` | `DynamicRigidBodyDesc`, `FixedRigidBodyDesc`, `KinematicPositionBasedRigidBodyDesc`, `KinematicVelocityBasedRigidBodyDesc`; `InsertRigidBody` |
| `R3ColliderDesc` | `DefaultColliderDesc`, `BallColliderDesc`, `CuboidColliderDesc`; world insertion |
| `R3ShapeDesc` | Inline in a collider; primitives, borrowed mesh/heightfield arrays, compound children, or a borrowed shared-shape reference |
| `R3JointDesc` | `DefaultJointDesc` or `FixedJointDesc`, `RevoluteJointDesc`, `PrismaticJointDesc`, `RopeJointDesc`, `SpringJointDesc` (plus dimension-specific joints); world insertion |
| `R3SoftBodyDesc` | `DefaultSoftBodyDesc`; particles, procedural generators, or borrowed surface/volume meshes; world insertion |
| `R3SoftBodyMaterial` | `DefaultSoftBodyMaterial`; inline in a soft recipe, or live `SoftBody_Material` / `SoftBody_SetMaterial` |
| `R3SoftMeshBindingDesc` | `DefaultSoftMeshBindingDesc`; `InsertDeformableCollider` |
| `R3IntegrationParameters` | `DefaultIntegrationParameters` or `IntegrationParameters`; edit then `SetIntegrationParameters` |
| `R3QueryOptions` | `DefaultQueryOptions`; reusable filter/predicate settings passed alongside the world to ray, point, shape, and intersection queries |

`R3ShapeDesc` and `R3SoftBodyDesc` borrow their array views until the
build/insertion call returns. Insertion copies arrays and retains shared geometry;
you can then release or reuse input buffers. Copying a description alone does
**not** extend the lifetime of its arrays or shared-shape handle. Geometry view counts always count elements: vectors, edges, triangles, or
tetrahedra. `R3ShapeDesc.triangles` and `.edges` replace flattened indices.

Soft recipes retain generator-specific radius and shape-matching defaults.
Override them explicitly, for example `soft.particleRadius = (R3OptionalReal){1, 0.05f}`
or `soft.shapeMatching = (R3OptionalBool){1, 0}`. Other material options use the
same `{enabled, value}` representation. Nonempty topology arrays override generated
topology. Procedural recipes include ropes, grids, disks, cloth, cloth tubes,
cuboids, spheres, and volumetric meshes. `SoftBodyDesc_ParticlePositions` and
`SoftBodyDesc_CellIndices` copy generated geometry into caller storage for editing;
each preview regenerates the recipe. For counts after insertion, query the soft
body by handle instead of generating it again.

Query options hold a filter, predicate, and user pointer. They have no fixed world owner
and need no destructor. Filters that exclude specific bodies or colliders must use
handles from the queried world; rebind those after snapshot restoration. Keep predicate data
alive during each query. Pass NULL options to use the defaults. Queries observe the
broad phase from the latest `Step` or `DetectCollisions`; the latter updates
collision detection without advancing time. Some queries allocate internal scratch
buffers.

`ImpulseJoint_Desc` / `ImpulseJoint_SetDesc` copy and apply joint configuration. They exclude
solver impulses; applying a description resets cached limit and motor impulses. Configuration
apply functions validate the complete value before replacing live settings.

POD layouts depend on dimension/precision and, for integration settings, FEM.
`PodLayout` reports sizes for language-wrapper checks; use matching headers and
feature definitions. C++ value factories live alongside the RAII owners in
`rapier.hpp`. The C# example uses a blittable body description and only disposes
the world.

## Typed array views

Use typed views to keep pointers and counts together and express mesh topology
with named element types. A view's count always means **elements**: two triangles
have a count of two, irrespective of their six vertex indices.

```c
#include "rapier_helpers.h"

R3Vector vertices[] = {{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {1, 0, 1}};
R3Triangle triangles[] = {{0, 2, 1}, {1, 2, 3}};
R3ColliderDesc collider = r3DefaultColliderDesc();
R3VectorView points = {vertices, 4};
R3TriangleView faces = {triangles, 2};
R3Status status = r3ShapeDesc_SetTrimesh(&collider.shape, points, faces, 0);
if (status == R3_OK) {
    R3ColliderHandle handle = r3InsertColliderWithoutParent(world, &collider);
    status = r3LastStatus();
}
// After insertion returns, the arrays may be freed or leave scope.
```

`ShapeDesc_SetPolyline` accepts `R3EdgeView`; `ShapeDesc_SetConvexHull` accepts
`R3VectorView`. Shape setters replace the complete shape description with the
selected geometry and defaults; the surrounding collider configuration is preserved.

Soft descriptions have setters for particles, surface meshes, edges, bend edges,
cells, surface elements, skin, masses, pinned particles, and tension-only edge
indices; 3D also exposes dihedrals and wire edges. `R3CellView` means triangle
cells in 2D and tetrahedra in 3D. `R3SurfaceElementView` means edges in 2D and
triangles in 3D. `R3RealView` and `R3IndexView` represent scalar arrays.
Soft setters preserve other fields. `SetParticles` and `SetSurfaceMesh` also select
the corresponding recipe kind. Zero topology counts retain generated topology,
just as the underlying description fields do.

Views and descriptors own no arrays and need no destructor. Setters store pointers
without allocating or copying elements. **Keep the arrays alive and unmodified
until build/insert returns.** Copying a view or description does not extend that
lifetime. Insertion copies the required data into Rapier-owned storage. Setters
reject null nonempty views, misalignment, and unrepresentable lengths without
modifying the description. Element values, topology bounds, and geometry flags
are validated during build/insert; check `LastStatus()` too. As with all C pointer
inputs, the caller must supply valid storage for the declared extent.

Description geometry fields are typed views too. Shared-shape mesh, compound, and heightfield constructors
also accept views; the former pointer/count overloads have been removed.

## Value initialization helpers

Include `rapier_helpers.h` for single-expression initialization in C11 or C++17:

```c
R3RigidBodyDesc body = r3DynamicRigidBodyDesc();
R3ColliderDesc collider = r3DefaultColliderDesc();
R3SoftBodyDesc soft = r3DefaultSoftBodyDesc();
R3QueryFilter filter = r3DefaultQueryFilter();
R3RigidBodyHandle handle = R3_INVALID_RIGID_BODY_HANDLE;
```

Body helpers cover dynamic, fixed, and both kinematic types. Other descriptions
and configuration data have `Default...()` helpers; an unconstrained joint uses
`DefaultJointDesc()`. `DefaultShapeCastOptions()` initializes cast options. They
are exported native value-returning functions, usable from C and other language
bindings without compiling an inline shim. The same applies to parameterized
constructors such as `CuboidColliderDesc`, `SpringJointDesc`, and `RopeSoftBodyDesc`.
They require no cleanup; build/insert validates their contents and reports failures through `LastStatus()`.
`r2RevoluteJointDesc()` takes no axis, while `r3RevoluteJointDesc(axis)` does.
The replaced output-pointer constructors are removed.

Explicit invalid constants are provided for body, collider, impulse-joint,
multibody-joint, and soft-body handles. Use them for local initialization and
assignment; none retains an owner. Check the ABI and match FEM/dimension/precision
configuration as with other calls. The helpers are installed with the SDK and
included by `rapier.hpp`.

## Handle-based access

For runtime element access, pass the generational handle; it includes the world pointer.
Each call resolves the element internally; no borrowed element pointer escapes.
For example, `r3RigidBody_SetTranslation(body, position, 1)` and
`position = r3RigidBody_Translation(body)` work across steps and storage growth.
`RigidBodyReadStates` copies an ordered batch into caller-owned storage, validates
all handles before writing, and leaves the buffer untouched on failure.

Removed or stale handles return `R3_INVALID_HANDLE`, including after slot reuse.
Handles identify their original world but do not retain it. Each contains a `world`
pointer, `index`, and `generation` (16 bytes on 64-bit targets). Handle equality
compares all three fields. Joint creation and other operations involving multiple
entities reject mixed-world handles with `R3_INVALID_HANDLE` before mutation. The invalid
sentinel is `{NULL, UINT32_MAX, UINT32_MAX}`, not a zero-initialized handle.

The world pointer is process-local and must not be persisted as part of a handle.
Do not fabricate or change it except when deliberately rebinding indices from a matching
snapshot. Stale entity generations are checked while the world is alive; a freed world
cannot be detected safely. Language wrappers must keep their owning world object alive
through every handle-based call (for example, `GC.KeepAlive(world)` for a C# SafeHandle).

## Loader options

URDF and MJCF loading uses copyable configuration values. Initialize defaults and
edit fields directly; options own no resources and need no setters or destructor:

```c
R3UrdfLoaderOptions options = r3DefaultUrdfLoaderOptions();
options.makeRootsFixed = 1;
options.rigidBodyBlueprint.canSleep = 0;
R3UrdfRobot *robot = r3UrdfRobotFromFile(path, &options);
// Check r3LastStatus(), use robot, then r3FreeUrdfRobot(robot).
```

`r3DefaultMjcfLoaderOptions()` works the same way. Blueprints are embedded
`RigidBodyDesc` and `ColliderDesc` values; geometry referenced by a collider
blueprint is borrowed until loading returns. Copying options does not retain
that geometry. Invalid fields fail during loading, before file I/O.
The defaults preserve native behavior, including zero collider density and
dynamic body blueprints. `PodLayout` reports both option sizes when robotics is
enabled, and zero otherwise. Robotics requires 3D f32.

## Ownership and borrowing

- Descriptions, configuration data, and `R3QueryOptions` are caller-owned values
  and need no destructor. Description arrays must remain valid through insertion;
  insertion copies them and retains any shared geometry it needs.
- `Collider_CloneShape`, `ReadCollider_CloneShape`, and `MjcfVisualMesh_CloneShape`
  return owned wrappers sharing geometry. Release them with `FreeSharedShape`.
  Ordinary value getters such as `SoftBody_Material` and `ImpulseJoint_Desc`
  return POD copies that need no destructor.
- Worlds, controllers, shared shapes, mesh assets, event collectors, and snapshots
  are owned resources. Use the matching `Free`, never C `free` or C++ `delete`.
  `Free(NULL)` succeeds. Each owned pointer must be freed exactly once.
- A world owns its sets, pipelines, and integration settings. There are no public
  component pointers, independent component constructors, or component destructors.
  Configuration is accessed through world functions or copied POD snapshots.
- Ordinary world reads may overlap, including nested reads from a query predicate.
  Mutation and stepping require exclusive access. Conflicting calls report
  `R3_WORLD_BUSY` before borrowing native simulation state; they do not block.
- A physics hook receives a borrowed `R3ReadContext`. Use `ReadRigidBody*` and
  `ReadCollider*` to inspect the callback-visible state. Ordinary calls on the
  stepping world report `R3_WORLD_BUSY`. Contact context setters remain available.
  Never retain either context after the callback returns.
- Perform additions, removals, and body changes after the active step/query returns.
  The event collector supports this workflow. Applications may also record their
  own commands during callbacks and apply them afterward; there is no implicit queue.
- Synchronize world destruction externally: no other thread may start an operation
  during or after `FreeWorld`. Every entity handle becomes dangling when its world
  is freed; even a `Contains` call is then invalid. Copying handles never retains a world. The access gate rejects freeing from an active
  callback, but cannot make a dangling pointer safe. Controllers and other separately
  owned objects still require caller synchronization.
- Mutating or removing a soft body's hidden rigid root/proxy through ordinary body
  APIs is rejected. Use soft-body and cluster operations instead.
- Output buffers belong to the caller. Passing `(NULL, 0)` returns the element count.
  Insufficient capacity reports `R3_BUFFER_TOO_SMALL`, returns the required count,
  and leaves the buffer untouched. Counts are elements unless specified as bytes.
- Snapshot bytes belong to `R3Bytes`. Serialization copies state; restoration
  creates an independent world with the serialized entity indices and generations.
  Enumerate handles from the restored world to obtain its new pointers. If preserving
  application references to a matching snapshot, rebind their `world` field explicitly
  to the restored world; do not reuse pointers from the source world. Handles returned
  by queries, events, callbacks, and controller results already carry their owner.

C pointers must refer to live, aligned allocations of the documented type and
extent. Output buffers must not alias inputs or one another. Null/alignment checks
cannot establish allocation validity. `rapier.hpp` provides `unique_ptr` aliases
for owned objects and a `check` helper that converts statuses to C++ exceptions.

## Errors, threads, and callbacks

Operations that produce values return them directly: pointers for owned resources,
handles for inserted objects, PODs for getters, and counts for buffer fills. Related
outputs are grouped into structs such as `R3OptionalRayHit`,
`R3VelocityCorrection`, and `R3ByteView`. Returned structs need no cleanup, but
owned pointers inside or returned separately retain their documented ownership.
Setters, stepping, and operations without a produced value return `R3Status`.

Every fallible call records its status and diagnostic on the calling thread.
Read `r3LastStatus()` immediately after the call when recovering from errors;
`R3_OK` means success. Infallible constructors and reads of `LastStatus`/`LastError`
do not change the recorded status. A successful fallible call clears it.

On failure, value-returning calls return a default value: null owned pointers,
invalid handles, zero scalars/vectors, or the POD type's default configuration.
These are placeholders, not a substitute for checking the status. Array APIs keep
caller-provided buffers and return the count directly. A size query uses a null
buffer and zero capacity; `BUFFER_TOO_SMALL` returns the required count without
partially writing the buffer. Other errors return zero and preserve the buffer.
`NOT_FOUND` reports a query miss; `TryCastRay` instead returns a successful value
with `found == 0` for misses.

```c
R3Vector position = r3RigidBody_Translation(body);
if (r3LastStatus() != R3_OK) {
    fprintf(stderr, "%s\n", r3LastError());
}
```

`r3LastError()` returns a thread-local UTF-8 message valid until the next fallible
call on the same thread. Copy it before another call. After an error handler makes
nested calls, the original operation's status and diagnostic are restored.
Rust panics are caught at the boundary when built with unwinding and reported as
`R3_PANIC`; discard objects mutated by that call because rollback is not promised.
Do not build these bindings with `panic=abort` if you depend on panic containment.
Allocation failure and invalid/dangling C pointers are not recoverable statuses.

`r3SetErrorHandler` optionally installs a handler on the calling thread and
returns the previous handler for scoped restoration. The default handler is null;
both status-returning and value-returning calls report failures to the handler. A reporting handler may return normally,
or a fail-fast application may print the diagnostic and terminate the process.
It must never throw or `longjmp` through Rust frames. The testbed installs a
fail-fast handler around each example so its physics calls need no checking
macros. Handlers also receive `R3_NOT_FOUND` query misses, so applications using
expected misses should handle that status accordingly. The diagnostic is borrowed
only for the callback; the callback and its user data must remain alive until the
handler is replaced. Handler state is thread-local, not inherited by workers.

Independent worlds may run on independent threads. Shared reads on one world
are allowed; conflicting access reports `R3_WORLD_BUSY`. Callbacks use the C
calling convention and must never throw, unwind, or longjmp across Rust frames.
With `parallel`, callbacks and their user data must support concurrent invocation.
Keep callbacks and their data alive until the synchronous operation returns.

Pass NULL hooks/events for default behavior. Enable `activeEvents` and
`activeHooks` on colliders to request the corresponding callbacks/collections.
The event collector accumulates collision, force, and tear events until `clear`;
copying events does not consume them. Tear-event getters return owned copies.
`ContactForceEvent.started` preserves Rapier's threshold-crossing semantics.
The contact modification callback currently edits rigid-manifold material,
normal, user data, and enabled status; soft-contact candidate editing is not
exposed. Invalid callback material/normal values leave the manifold unchanged.

`modify_solver_contacts_context` additionally receives a borrowed native contact
context. Its accessors support `update_as_oneway_platform` and tangent velocity;
the context is only valid during that callback. Query predicates also receive a scoped read context. They may
perform nested world reads/queries but cannot mutate the world during traversal.

## Math and configuration

`R3Real` is float or double; vectors contain two or three packed scalar fields,
without Rust SIMD alignment. A 2D rotation is one angle in radians. A 3D rotation
is an `(x,y,z,w)` quaternion, normalized on input; initialize identity with w=1.
All boolean inputs/outputs are `uint32_t` values 0 or 1. Enum inputs and flags are
integers validated before conversion into Rust values. User data preserves all
128 bits as `{low, high}`. `size_t` is pointer-sized, including in language bindings.
There are no C varargs, C++ types, Rust references, slices, strings, or enums in
the binary interface. Callback function pointers are explicitly `cdecl` on Windows.

Defaults come from the native Rust constructors. Gravity, integration parameters,
soft-body recovery settings, joint motor models, collision/solver groups, and
material combine rules retain their Rust semantics. Generic joints cover the
standard fixed, revolute, prismatic, rope, spring, 2D pin-slot, and 3D spherical families.
3D heightfield samples are **column-major**, matching Parry `Array2`.

## Coverage and examples

- [testbed/README.md](testbed/README.md): raylib/Dear ImGui viewer and headless C scenes.
- [examples/falling_ball.c](examples/falling_ball.c): complete C simulation.
- [include/rapier.hpp](include/rapier.hpp): optional C++ ownership helpers.
- [examples/RapierNative.cs](examples/RapierNative.cs): executable P/Invoke
  example using a SafeHandle for the world, also suitable for a Unity wrapper.
- [tests/integration.c](tests/integration.c): world ownership and collision-only updates,
  events/hooks, queries, joints, snapshots, controllers, soft-body meshes and cuts.

The engine examples are integration starting points, not complete Unity or Unreal
plugins. Physics, game-object synchronization, editor tooling, and deployment
policies belong in those engine-specific layers.

## Development and verification

```sh
cargo test -p rapier-c-macros -p rapier2d-ffi -p rapier3d-ffi -p rapier2d-f64-ffi -p rapier3d-f64-ffi
python3 c/tools/test-native.py  # Unix: all four variants, shared and static, C and C++
# Header generation requires Python 3 and cbindgen 0.29.4:
cargo install cbindgen --version 0.29.4 --locked
sh c/tools/generate-header.sh
```

The generator uses Rust declarations and makes the transparent Rust wrapper types
opaque to C. The symbol test links every function declared for each selected ABI.
A CI workflow additionally builds the CMake tests on Linux, macOS, and Windows and
checks the generated header. The shared source is in `src/`; the four tiny crates
select Rapier's matching dimension and scalar precision.

The optional `robotics` feature (3D/f32 only, matching the native importer crates)
adds URDF/MJCF loading, insertion, keyframes, actuator controls, and visual mesh
access. Enable it with `-DRAPIER_FEATURES=robotics` or Cargo `--features robotics`.
It uses the workspace Rust importers and their mesh readers; no additional C
libraries are required. See the [robot examples](testbed/README.md#optional-examples).

Snapshots are limited to 256 MiB. Their 16-byte header contains `RPRS` followed
by the ABI version, dimension, and scalar byte size as little-endian `uint32_t`
values. Older six-byte headers are rejected. Load
**trusted snapshots from the identical Rapier build only**. The Rust serialized
world representation is neither an untrusted asset format nor a versioned storage
format; the tag cannot establish source-revision compatibility or data integrity.

### Direct builder and world APIs

`r3BallColliderDesc`, `r3CuboidColliderDesc`, and other shape constructors return
ordinary collider descriptions by value. `r3InsertRigidBody`,
`r3InsertCollider`, and `r3InsertSoftBody` insert descriptions into the world
and return handles directly. `r3InsertCollider` takes a parent handle by value;
`r3InsertColliderWithoutParent` takes the world explicitly for standalone colliders.

The optional `rapier_math.h` header supplies C11/C++17 value constructors and
arithmetic for vectors, rotations, and poses. It has no third-party dependencies;
on Unix, manual consumers using its trigonometric operations should link `libm`.
The CMake target supplies that system library automatically.

### C++ ownership helpers

`rapier.hpp` provides movable `std::unique_ptr` owners for all 14 owned resource
types: `World`, `Shape`, `EventCollector`, `Snapshot`, `SoftBodyTearEvent`,
`ShapeMesh`, `KinematicCharacterController`, `PidController`, and, in 3D,
`DynamicRayCastVehicleController` and `TriMeshData`. Robotics adds `UrdfRobot`,
`UrdfRobotHandles`, `MjcfRobot`, and `MjcfRobotHandles`.

```cpp
rapier::Shape shape(r3Collider_CloneShape(collider));
rapier::check(r3LastStatus());
rapier::ShapeMesh mesh(r3SharedShape_Tessellate(shape.get(), 16));
rapier::check(r3LastStatus());
// Each owner frees its resource when it leaves scope, including during exceptions.
```

Check errors after each fallible call. These owners wrap only owned pointers;
borrowed callback contexts and MJCF visual meshes must not be wrapped or freed.
Owners of resources referencing a world must be destroyed before that world.

## API reference

Generate searchable Doxygen documentation for 2D/3D and f32/f64:

```sh
cmake -S c/doxygen -B build/c-docs
cmake --build build/c-docs --target rapier_docs --parallel
```

Open `build/c-docs/html/index.html`. This needs Doxygen 1.9.4+, Python 3, CMake,
and a C compiler; it does not build physics or fetch testbed dependencies.
Alternatively, configure the normal C build with `-DRAPIER_BUILD_DOCS=ON` and build
`rapier_docs`; the entry page is then under `doxygen/html/index.html` in that build.
Normal builds do not require documentation tools.

The reference includes all optional APIs, with robotics limited to 3D/f32.
Descriptions of ownership, errors, arrays, callbacks, and snapshots accompany the
API groups. CI checks all four variants for Doxygen warnings and missing functions,
then uploads the HTML as the `rapier-c-api-docs` artifact.

Edit API comments in `c/src/*.rs` and regenerate `include/rapier.h`; edit inline
math/C++ helper comments in their headers. Keep contracts specific: units,
coordinate frames, ownership, callback lifetime, and special failure cases.
The concise usage pages live in `doxygen/reference.dox`.
