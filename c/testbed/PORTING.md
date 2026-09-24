# Writing C examples

Treat the matching Rust file as the structure of the C example. Preserve its
construction order, variable meanings, parameters, comments, and distinct examples.
Rapier instance methods separate the receiver and method with an underscore,
for example `r3RigidBody_SetTranslation`. Constructors such as
`r3DynamicRigidBodyDesc` put the qualifier first. Creation and destruction use
`r3NewWorld` and `r3FreeWorld`; simple math values use `r3Vector` and `r3Pose`.

Use camelCase for C functions, fields, parameters, and local variables, including
the testbed helpers. Keep types in PascalCase and macros in UPPER_SNAKE_CASE.
Keep the corresponding file and scene names. Do not combine separate Rust blocks
into loops or conditional expressions merely to shorten the C code.

Use the public Rapier C API for world creation, descriptions, insertion, queries,
joints, soft bodies, and simulation callbacks. If a Rust operation is missing
from the bindings, add its native counterpart to the bindings instead of hiding
it in a testbed helper. Translate Rust's `PhysicsWorld::insert(body, collider)`
into `r3InsertRigidBody(world, &body)`, then
`r3InsertCollider(bodyHandle, &collider)`. Keep these calls directly in
the example. The `r3*ColliderDesc` functions correspond to `ColliderBuilder`
constructors. Description constructors return POD values directly.

The testbed provides one-frame rendering, camera, colors, input, UI settings,
and run/pause controls. Each example owns its world and its simulation loop:

```c
tbSetWorld(testbed, world);
while (tbRenderFrame(testbed, &world)) {
    if (tbSimulating(testbed)) {
        r3Step(world, NULL, NULL);
    }
}
r3FreeWorld(world);
```

`tbSetWorld` borrows the world. `tbRenderFrame` renders and processes input;
it never advances physics. It returns false on close, restart, or scene switch.
The world pointer is passed by address so restoring a snapshot updates the
example's local pointer. Code outside `tbSimulating` runs on paused frames too.
Keep per-frame logic in the same place relative to stepping as in the Rust source.
Sensor examples own their event collectors and process events immediately after
stepping. Animation state stays in ordinary local variables; there are no
before/after-step callbacks or heap-allocated callback state. Examples with local
state that snapshots cannot restore set `snapshotSupported` to zero.

Show ownership in the example. POD descriptions need no explicit cleanup; their
array views borrow data that must remain valid through the build or insertion call.
Inserted objects belong to their world and are accessed through handles.
Pass the world directly; do not cache component aliases or add physics wrappers. Shared
shapes retained by the example must be freed when no longer needed. The optional
`--no-sleep` override is explicit in the example's description setup.

Call the dimension-specific `r2*` or `r3*` functions directly without wrapping them
in testbed checking macros.
The example dispatcher installs a thread-local error handler for the duration of
the example. Unexpected API errors print the scene and diagnostic and terminate
the process before invalid outputs can be used. Viewer operations handle their
recoverable errors separately. The C API reports failures through status returns
or `LastStatus()` by default; applications may choose their own error policy. Never longjmp or throw
from an error handler through Rust frames.

Use named locals for positions, handles, and material parameters. Separate
construction, configuration, insertion, and release. Use `rapier_math.h` for the
public math types' value constructors and arithmetic. Keep dimension-specific
source files dimension-specific. Helpers implementing a particular example's
algorithm are appropriate when the Rust example has the same helper; general
physics convenience wrappers do not belong in the viewer.

Format authored C and headers with the adjacent `.clang-format`; do not reformat
vendored code. `examples3d/primitives3.c`, `examples3d/compound3.c`,
`examples3d/soft_bodies3.c`, and `examples2d/add_remove2.c` show the intended layout.
