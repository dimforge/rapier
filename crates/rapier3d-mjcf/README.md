## MJCF loader for the Rapier physics engine

> **Disclaimer.** Most of this crate — source, tests, and documentation — was
> produced by an AI coding assistant working iteratively from MJCF reference
> scenes (primarily the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)),
> under human direction and review.

Rapier is a set of 2D and 3D physics engines for games, animation, and
robotics. The `rapier3d-mjcf` crate lets you convert a [MuJoCo MJCF XML
file](https://mujoco.readthedocs.io/en/stable/XMLreference.html) into a
set of rigid-bodies, colliders, and joints, for use with the `rapier3d`
physics engine.

The parser ([`mjcf-rs`](../mjcf-rs)) is a sibling crate. Both are
**pure Rust** — no `libmujoco` / FFI bindings.

## Quick start

```rust,no_run
use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfRobot};

let mut bodies = RigidBodySet::new();
let mut colliders = ColliderSet::new();
let mut impulse_joints = ImpulseJointSet::new();

let (robot, _model) =
    MjcfRobot::from_file("robot.xml", MjcfLoaderOptions::default()).unwrap();
robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
```

## Optional cargo features

| Feature      | Effect                                                              |
| ------------ | ------------------------------------------------------------------- |
| `stl`        | Load `.stl` meshes referenced by `<mesh>` assets.                   |
| `wavefront`  | Load `.obj` meshes referenced by `<mesh>` assets.                   |
| `msh`        | Parse MuJoCo's custom binary `.msh` mesh format (in `mjcf-rs`).     |

`.dae` (Collada) is intentionally unsupported because MJCF doesn't use it.

## Feature matrix

This crate is rolled out in phases. The table tracks what each phase
delivers and the current implementation status.

Legend: ✅ supported · ⚠️ partial / approximated · 📦 preserved as
metadata · ❌ out of scope.

### Phase 1 — Core kinematics ✅

- ✅ `<mujoco>` root, `<compiler>` (`angle`, `eulerseq`, `coordinate=local`),
  `<option>` (`timestep`, `gravity` recorded)
- ✅ `<worldbody>` and nested `<body>` trees
- ✅ Body poses via `pos`, `quat`, `axisangle`, `euler`, `xyaxes`, `zaxis`
- ✅ Angle-unit conversion for `compiler/angle="degree"` (default)
- ✅ `<joint>` types: `hinge`, `slide`, `ball`, `free`
- ✅ `<freejoint>`
- ✅ `<inertial>` with `mass` + `diaginertia` or `fullinertia`
- ✅ `<geom>` types: `plane`, `sphere`, `capsule`, `cylinder`, `box`, `ellipsoid`*
- ✅ `<geom fromto>` form for capsule / cylinder / box
- ✅ Multi-joint bodies: rapier intermediates synthesized so each rapier
  body has at most one parent joint, just like MJCF would expand.
- ✅ Welded bodies (no joint) — fixed joint to parent, force-fixed when the
  parent is the world.
- ✅ `make_roots_fixed`, `enable_joint_collisions`, `shift`, `scale`,
  `collider_blueprint`, `rigid_body_blueprint`, `skip_plane_geoms`
  loader options. (`skip_plane_geoms` drops `<geom type="plane">`
  elements at load time so callers can supply their own ground.)

`*` ellipsoid is approximated as the convex hull of an icosphere scaled
per-axis (rapier has no ellipsoid primitive).

### Phase 2 — Defaults, includes, mesh assets ✅

- ✅ `<default>` class inheritance, with nested classes and `childclass`
  propagation. The parser bakes resolved attributes into every element so
  the loader never revisits the class hierarchy.
- ✅ `<include file="…"/>` recursive inlining (with cycle detection).
- ✅ `<frame>` pose grouping (poses are folded into children's local poses).
- ✅ `<asset><mesh>` and `<asset><hfield>` (collider build behind the
  `stl`/`wavefront`/`msh` cargo features; inline-elevation hfields work
  without a feature).
- ✅ `compiler/inertiafromgeom` (`true`/`auto`/`false`) — bodies without
  `<inertial>` derive their mass properties from the union of their geoms.
- ✅ `compiler/autolimits` — `range` implies `limited="true"` when set.
- ✅ `compiler/discardvisual` — drops visual-only geoms (those with
  `contype=conaffinity=0`) at load time.
- ✅ `compiler/convexhull` — when `true`, `<geom type="mesh">` becomes a
  convex hull instead of a triangle mesh.
- ✅ `compiler/strippath` — strips directory components from `file=`
  references (legacy compatibility).
- ✅ `compiler/meshdir` / `assetdir` / `texturedir` for asset resolution.

### Phase 3 — Contact filtering and equality constraints ✅

- ✅ `contype` / `conaffinity` → `InteractionGroups` (the loader option
  `ContactFilterMode::{Symmetric,Asymmetric}` chooses how MJCF's "OR" rule
  maps onto rapier's "AND" rule; symmetric is the default).
- ✅ `<contact><exclude>` between bodies, via the user-installable
  `MjcfContactHooks` that the handles object exposes.
- ✅ `<contact><pair>` per-pair friction / margin overrides through the
  same hook (uses rapier's contact-modification path).
- ✅ `<equality><connect>` — point-to-point constraint as an impulse joint
  with all linear axes locked at the chosen anchor.
- ✅ `<equality><weld>` — rigid attachment as a fixed impulse joint with
  optional `relpose`.

Equality joints are inserted as **impulse joints** even when the rest of
the model uses multibody joints (rapier multibodies don't support
loop-closure joints).

### Phase 4 — Joint dynamics ✅

- ✅ Joint `damping` and `stiffness` (mapped to a rapier `JointMotor`
  using the `ForceBased` motor model — the motor acts as a PD spring at
  the rest position).
- ✅ `springref` — drives the spring's reference position (in radians for
  hinges).
- ✅ `springdamper="(timeconst, dampratio)"` — converted to
  `(stiffness, damping)` via the standard MuJoCo formula.
- ✅ `armature` — adds rotor inertia along the joint axis. For axes
  aligned with the body's principal axes the result is exact; for arbitrary
  axes the contribution is approximated by adding `armature · outer(axis,
  axis)` to the inertia tensor before re-diagonalizing.
- ✅ `frictionloss` — approximated by a velocity-based motor with
  capped force (lossy; documented under "Limitations").
- ✅ `<body gravcomp>` — `gravcomp ∈ {0, 1}` maps directly to
  `gravity_scale = 1 - gravcomp`. Fractional values are honored to first
  order, and an opt-in
  `MjcfRobotHandles::apply_gravity_compensation` helper applies an
  exact per-step external force when configuration / mass changes
  matter.
- ✅ `compiler/boundmass`, `compiler/boundinertia`,
  `compiler/balanceinertia`, `compiler/settotalmass`.

`ref` (joint reference) is recorded on the AST and used by the spring
target, but not yet baked into the joint's local frames — the joint's
reported angle is therefore offset by `ref` compared to MuJoCo's
convention. (Tracked for a future polish pass.)

### Phase 5 — Actuators, sensors, keyframes ✅

- ✅ `<actuator>` pass-through. The handles object exposes per-actuator
  joint handles, and a built-in
  `MjcfRobotHandles::apply_controls(impulse_joints, ctrl)` drives the
  rapier joint motors for `<motor>`, `<position>`, `<velocity>`, and
  `<damper>` actuator types. `<general>` and `<intvelocity>` are
  recorded but left to the user to wire up. The `*_scaled` variants
  (`apply_controls_scaled` / `apply_controls_multibody_scaled`) take a
  `gain_scale` that uniformly softens (`< 1`) or stiffens (`> 1`) every
  actuator's gains and force limits — handy to ease a servo-driven move
  that would otherwise saturate and snap.
- ✅ `<keyframe>` state — `MjcfRobotHandles::apply_keyframe` applies a
  keyframe's full state: `qpos` / `qvel` joint coordinates (using
  `MjcfRobot::qpos_dofs`, a precomputed map from MuJoCo's
  generalized-coordinate order onto the rapier joints/bodies), a
  floating base's world pose, and `mpos` / `mquat` mocap state. It works
  on both insertion paths — the multibody path writes the multibody's
  generalized coordinates (then runs forward-kinematics), the
  impulse-joint path runs forward-kinematics over the joint tree and
  writes each body's world pose. `apply_mocap_keyframe` (mocap only) and
  `MjcfRobot::keyframe_by_name` remain available. (Per-joint `qvel` is
  applied on the multibody path; the impulse path applies only a floating
  base's `qvel`.)
- ✅ `<sensor>` reading via `MjcfRobot::read_sensor` for the
  state-derivable subset: `framepos`, `framequat`, `framelinvel`,
  `frameangvel`, `velocimeter`, `gyro`, `subtreemass`, `subtreecom`,
  `clock`. Sensors that need contact-area integration (`touch`,
  `rangefinder`, `geomdist`, `camprojection`) are out of scope.
- ✅ `<tendon><fixed>` (linear joint-coordinate tendons). An actuator with
  `tendon="…"` is bound to the tendon's first joint, and the tendon's other
  joints are coupled to it (`q_k = (coef_k/coef_0)·q_0`, a multibody DoF
  coupling — exact for equal-inertia joints, which covers every menagerie
  fixed tendon) so they move as one. This is what makes e.g. the shadow hand's
  tendon-driven middle+distal finger segments curl together. Pairs already
  coupled by an explicit `<equality><joint>` (franka, robotiq) are left alone.
  `<tendon><spatial>` (site-routed cables with wrapping) and `<equality><tendon>`
  length constraints remain out of scope.

### Out of scope ❌

- `<extension>` / MuJoCo plugins.
- `<flexcomp>` / `<flex>` / `<skin>` (deformables).
- `<composite>` (procedural body clusters).
- `<tendon><spatial>` and `<equality>` of tendons / flex (fixed-tendon joint
  coupling and actuation *are* supported — see above).
- Fixed-tendon passive `stiffness`/`damping`/`range`/`frictionloss`.
- `<actuator>` types: `cylinder`, `muscle`, `adhesion`.
- `<sensor>` types that need contact-area integration: `touch`, `rangefinder`,
  `geomdist`, `camprojection`.
- `<geom type="sdf">`.
- `<compiler coordinate="global">` (deprecated MJCF feature).
- MJCF write-back (the parser is read-only).

## Limitations

The mappings noted with ⚠️ above are deliberate trade-offs. In particular:

- Ellipsoid shapes are approximated by an icosphere convex hull.
- `frictionloss` is approximated by a velocity-capped motor (Phase 4).
- `solref` / `solimp` softness parameters are not translated — rapier has
  its own joint-softness controls.

When inserting joints as multibody joints, the bodies are reset to their
neutral configuration (all generalized coordinates = 0), matching the
behavior of `rapier3d-urdf`.

## Resources and discussions

- [Dimforge](https://dimforge.com): See all the open-source projects we are
  working on! Follow our announcements on our [blog](https://www.dimforge.com/blog).
- [User guide](https://www.rapier.rs/docs/): Learn to use Rapier in your
  project by reading the official User Guides.
- [Discord](https://discord.gg/vt9DJSW): Come chat with us, get help, suggest
  features, on Discord!

Please make sure to familiarize yourself with our
[Code of Conduct](../../CODE_OF_CONDUCT.md) and our
[Contribution Guidelines](../../CONTRIBUTING.md) before contributing or
participating in discussions with the community.
