# Phase 3 — Contact Filtering and Equality Constraints

**Goal:** Faithfully reproduce MJCF's collision-filtering rules and add
support for the `<equality>` constraints that map cleanly to rapier joints.

## Scope

### `mjcf-rs` (parser)

- `<contact>` with children `<pair>` and `<exclude>`.
- `<equality>` with children `<connect>` and `<weld>` (and the unsupported
  `<joint>` / `<tendon>` / `<flex*>` parsed but flagged).
- Per-geom `contype`, `conaffinity`, `condim`, `priority`, `margin`, `gap`.
- Existing `<default>` resolution extended to `<pair>` and `<equality>`
  prototypes (already in Phase 2 for the AST machinery; here we exercise it).

### `rapier3d-mjcf` (loader)

#### Bitmask-based filtering (`contype`/`conaffinity`)

MuJoCo's filter rule for two geoms `A` and `B`:

```
A.contype & B.conaffinity != 0  OR  A.conaffinity & B.contype != 0
```

This is symmetric to rapier's `InteractionGroups::test`:

```
membership_a & filter_b != 0  AND  membership_b & filter_a != 0
```

Because rapier uses **AND** while MuJoCo uses **OR**, the most faithful
encoding is:

- `Collider::collision_groups`:
  - `memberships`: `geom.contype | geom.conaffinity` (so the geom is "in"
    every group it can match against).
  - `filter`: `geom.contype | geom.conaffinity` (likewise).

That gives MuJoCo's OR semantics: pair `(A, B)` collides iff
`(A.ct|A.ca) & (B.ct|B.ca) != 0`, which is true iff at least one of
the two MuJoCo conditions holds.

For users who need the pure MuJoCo behavior in edge cases, expose a
`MjcfLoaderOptions::contact_filter_mode` enum:

```rust
pub enum ContactFilterMode {
    /// Symmetric encoding (default). Matches MuJoCo for the common case
    /// of `contype == conaffinity` per geom.
    Symmetric,
    /// One-way encoding: `memberships = contype`, `filter = conaffinity`.
    /// Matches MuJoCo only when the user is careful about which side they
    /// place each geom in a pair.
    Asymmetric,
}
```

#### `<contact><exclude>`

Disable contacts between two named bodies. Implemented by stripping shared
bits from the involved colliders' `InteractionGroups` *or* by using
rapier's `PhysicsHooks` with a `filter_contact_pair` callback. We default
to a hooks-based exclusion (no bit budget pressure).

`MjcfLoaderOptions` gains an opt-in
`exclude_via_groups: bool` for users who can't run hooks.

#### `<contact><pair>`

A `<pair geom1=… geom2=…>` overrides friction / margin / `condim` /
`solref` / `solimp` for that specific pair. We emit a per-pair contact-mod
hook:

```rust
struct MjcfPairOverride {
    geom1: ColliderHandle,
    geom2: ColliderHandle,
    friction: Real,
    restitution: Real,
    margin: Real,
}
```

The user installs `MjcfRobot::physics_hooks()` if they want pair overrides
to fire; otherwise the per-collider averages apply.

#### `<equality><connect>`

Maps to a `GenericJoint` with all linear axes locked at the chosen anchors,
and all angular axes free — i.e. a spherical joint at the anchor.

```rust
GenericJointBuilder::new(JointAxesMask::LIN_AXES)
    .local_anchor1(anchor1)
    .local_anchor2(anchor2)
    .build()
```

#### `<equality><weld>`

Maps to a `GenericJoint` with all six axes locked, anchored and oriented to
match the `relpose` attribute when present.

```rust
GenericJointBuilder::new(JointAxesMask::LOCKED_FIXED_AXES)
    .local_frame1(frame1)
    .local_frame2(frame2)
    .build()
```

`<equality>` `active="false"` → emit the joint with `set_enabled(false)`.

These extra constraints land in `MjcfRobot::equality: Vec<MjcfEqualityJoint>`,
inserted as **impulse joints only** (multibodies don't support extra
loop-closure joints; we log a warning and still insert as impulse joints
when the user picks `insert_using_multibody_joints`).

## Non-goals (deferred)

- `<equality joint>` polynomial coupling, `<equality tendon>`, `<equality flex*>` — out of scope.
- Soft-contact `solref`/`solimp` translation — recorded only.
- `condim` 1 (frictionless) and 6 (full torsional) — Phase 4 may revisit
  via contact-modification hooks.

## Tests

- A four-bar linkage modeled with a `<connect>` equality. Verify the loop
  closure works using impulse joints.
- A cube glued to a moving platform via `<weld>`; verify the relative pose
  stays constant.
- A pair of overlapping geoms with `<exclude>` — verify they fall through
  one another instead of bouncing.
- `<pair>` with custom friction = 2.0, on top of a baseline `<geom friction>`
  of 0.3 — verify the contact uses the override.

## Acceptance criteria

- All Phase-3 example tests pass.
- `mujoco_menagerie` files that depend on `<exclude>` (most quadruped
  models do, to disable foot self-collisions) load without manual
  `<exclude>` translation.
- Loader documents the contact-filter encoding clearly enough that users
  can reason about edge cases.
