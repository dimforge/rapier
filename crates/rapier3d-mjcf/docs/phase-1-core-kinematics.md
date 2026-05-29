# Phase 1 — Core Kinematics

**Goal:** Load a self-contained MJCF file (no `<include>`, no `<default>`,
no meshes) into rapier rigid-bodies, primitive-shape colliders, and joints.

After this phase, simple test files (a cartpole, a chain of capsules, a
box-on-a-plane) load and simulate identically to a hand-built rapier scene.

## Scope

### `mjcf-rs` (parser)

- Single-file XML parsing via `quick-xml`.
- AST coverage for: `<mujoco>`, `<compiler>` (only `angle`, `eulerseq`,
  `coordinate`), `<option>` (record only), `<worldbody>`, `<body>`, `<inertial>`,
  `<joint>`, `<freejoint>`, `<geom>`, `<site>` (record only).
- Pose attributes: `pos`, `quat`, `axisangle`, `euler`, `xyaxes`, `zaxis`.
- Angle normalization: convert all angle attributes to radians based on
  `compiler/angle`. Apply `compiler/eulerseq` for every `euler="…"`.
- **No** `<default>` / `<include>` resolution yet — fail with a clear error
  if the file uses them.
- **No** `<asset>` resolution — fail with a clear error if `<geom mesh="…">`
  / `<geom hfield="…">` is used.

### `rapier3d-mjcf` (loader)

- `MjcfLoaderOptions` struct (mirrors `UrdfLoaderOptions`).
- `MjcfRobot::from_str` / `from_file` / `from_model`.
- Body/joint conversion:
  - `<body>` → `RigidBody`.
  - 0 joints, parent = world → fixed body (when `make_roots_fixed` or `squeeze_empty_fixed_bodies`).
  - 0 joints, non-world parent → fixed joint to parent.
  - 1 joint of type `hinge` / `slide` / `ball` → `revolute` / `prismatic` / `spherical`.
  - 1 joint of type `free` (or `<freejoint>`) → no joint emitted; rigid-body is dynamic + free.
  - **Multiple joints in the same body**: error in Phase 1, supported in Phase 1.5
    (insert massless intermediate bodies). Loader options gain a
    `forbid_multi_joint_bodies: bool` (default `false`) so users get a clear
    error in v1 instead of silently broken kinematics.
- Geom conversion:
  - `box`, `sphere`, `capsule`, `cylinder`, `plane`, `ellipsoid` (approximated).
  - `fromto` form for capsule / cylinder / box.
  - Multiple geoms per body → multiple colliders.
- Inertial conversion:
  - `<inertial pos quat mass diaginertia>` → `MassProperties::with_principal_inertia`.
  - `<inertial fullinertia>` → `MassProperties::with_inertia_matrix`.
  - When `<inertial>` is missing in Phase 1: dynamic body with no extra mass,
    same as URDF; the user is expected to add `<inertial>` until Phase 2 ships
    `inertiafromgeom`.
- `make_roots_fixed`, `enable_joint_collisions`, `shift`, `scale`,
  `collider_blueprint`, `rigid_body_blueprint`: same semantics as URDF.
- `insert_using_impulse_joints` and `insert_using_multibody_joints`.

## Non-goals (deferred)

- `<default>` / `<include>` (Phase 2)
- Mesh / hfield assets (Phase 2)
- `<contact>`, `<equality>`, contact filtering (Phase 3)
- Joint stiffness / damping / armature / ref / springref (Phase 4)
- `<keyframe>`, `<actuator>`, `<sensor>` (Phase 5)
- Multi-joint-per-body bodies — error out for now; supported in Phase 1.5.

## Phase 1.5 — Multi-joint bodies

Sub-phase, before Phase 2. MJCF allows `<body>` to declare multiple
`<joint>` children, which together form a serial DoF chain between the
parent body and the body itself. Rapier's joint model is one-joint-per-body,
so we synthesize intermediate massless `RigidBody`s.

### Algorithm

For a body B with N joints `j_1 … j_N` and parent P:

```
P
└── intermediate body I_1   ← j_1 connects P to I_1 (uses j_1's pos/axis)
    └── intermediate body I_2   ← j_2 connects I_1 to I_2
        └── …
            └── B    ← j_N connects I_{N-1} to B
```

- The Nth (last) joint is the one that ends in B. Its `local_anchor2` and
  the body's full `<body pos quat>` agree at rest.
- Intermediate bodies have zero mass and inertia. Rapier accepts this in
  multibody chains (treated as kinematic spacers).
- `<inertial>` lives entirely on B.
- Collider attachment: every collider stays on B; the intermediate bodies
  carry no colliders.

Insert the intermediate bodies into `MjcfRobot::bodies` with synthesized names
(`<body name>__intermediate_<i>`) so the user can resolve them if needed.
Original-body indices stay 1:1 with the parser AST.

## Tests

- A flat cartpole MJCF: 3 bodies, 2 hinges, 1 prismatic. Simulate 1 second
  and compare end-state to a known reference.
- A pure free body (no joints) with a sphere collider; gravity drops it.
- A chain of N capsules connected by hinges; smoke test that link count
  matches expectations.
- (Phase 1.5) A planar body with two slides + one hinge ("body sliding in a
  plane that can rotate"); verify the multibody chain has 4 entries (3
  intermediate + 1 final) and that the kinematics work out.

## Acceptance criteria

- All Phase-1 example tests pass.
- A user can load a hand-written MJCF with a tree of bodies + primitive geoms +
  hinge/slide/ball/free joints + no defaults / no includes / no meshes, and
  see the same simulation behavior they'd get hand-coding it in rapier.
- Loader emits a clear, actionable error when it hits an MJCF feature
  scheduled for a later phase.
