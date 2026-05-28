# 02 — Full MJCF Coverage and Rapier Mapping

This document enumerates every MJCF top-level element, its children and
attributes, and how each maps (or doesn't) to a rapier construct. It is the
authoritative reference for what the parser and loader must support.

Legend for the **Status** column:

| Symbol | Meaning                                                          |
| ------ | ---------------------------------------------------------------- |
| ✅      | Fully mapped to a rapier feature.                                |
| ⚠️      | Partially mapped or approximated (lossy).                        |
| 📦     | Preserved in the AST as metadata; no runtime effect in rapier.   |
| ❌      | Out of scope. See [`04-out-of-scope.md`](04-out-of-scope.md).    |

Each row also notes the **Phase** in which it lands.

---

## Top-level structure

```xml
<mujoco model="...">
  <compiler/>           ← simulation-input parsing knobs
  <option/>             ← simulation runtime knobs
  <size/>               ← memory pre-allocation
  <statistic/>          ← purely informational
  <visual/>             ← rendering only
  <asset>...</asset>
  <default>...</default>
  <custom>...</custom>
  <extension>...</extension>
  <worldbody>...</worldbody>
  <contact>...</contact>
  <equality>...</equality>
  <tendon>...</tendon>
  <actuator>...</actuator>
  <sensor>...</sensor>
  <keyframe>...</keyframe>
</mujoco>
```

| Element       | Status | Phase | Notes |
| ------------- | ------ | ----- | ----- |
| `<mujoco>`    | ✅      | 1     | Root. `model` attribute → `MjcfRobot::name`. |
| `<compiler>`  | ⚠️      | 1/2   | See dedicated table below. |
| `<option>`    | 📦     | —     | Pass-through. Rapier already has its own integrator config; we record `gravity`, `timestep` for the caller. |
| `<size>`      | 📦     | —     | Memory pre-allocation hint; ignored at simulation time. |
| `<statistic>` | 📦     | —     | Informational only. |
| `<visual>`    | 📦     | —     | Pure rendering knobs; not used by physics. |
| `<asset>`     | ⚠️      | 2     | See dedicated table. |
| `<default>`   | ✅      | 2     | Class-based attribute inheritance, applied by the parser. |
| `<custom>`    | 📦     | —     | User-defined data; preserved verbatim. |
| `<extension>` | ❌      | —     | Plugin instances — out of scope (would need MuJoCo plugins). |
| `<worldbody>` | ✅      | 1     | Root of the kinematic tree. Children: `<body>`, `<geom>`, `<site>`, `<light>`, `<camera>`, `<frame>`. |
| `<contact>`   | ✅      | 3     | Friction/exclusion pairs → rapier `InteractionGroups` + per-pair material. |
| `<equality>`  | ⚠️      | 3     | `connect`/`weld` → impulse joints. `joint`/`tendon`/`flex` are out of scope. |
| `<tendon>`    | ❌      | —     | No first-class rapier equivalent. |
| `<actuator>`  | 📦     | 5     | Preserved as metadata; rapier's joint motors are exposed but the user wires them up manually. |
| `<sensor>`    | 📦     | 5     | Preserved as metadata. |
| `<keyframe>`  | ✅      | 5     | Optional initial state to apply at insertion time. |

---

## `<compiler>`

| Attribute             | Status | Phase | Notes |
| --------------------- | ------ | ----- | ----- |
| `angle`               | ✅      | 1     | `"degree"` (default) / `"radian"`. Convert to rad before passing values to rapier. |
| `eulerseq`            | ✅      | 1     | Default `"xyz"`. Used for **all** `euler=` attributes in the file. Map to `glamx::EulerRot`. |
| `coordinate`          | ⚠️      | 1     | Modern MJCF only supports `"local"`. We reject `"global"` with an explicit error (deprecated, rare in practice). |
| `autolimits`          | ✅      | 2     | When `true` (default since v3), specifying `range` implies `limited="true"`. |
| `boundmass`           | ⚠️      | 2     | Lower bound on body mass; we clamp during inertia compute. |
| `boundinertia`        | ⚠️      | 2     | Lower bound on diagonal inertia; clamp during inertia compute. |
| `settotalmass`        | ⚠️      | 2     | Rescales all masses so the model totals this value. |
| `inertiafromgeom`     | ✅      | 2     | `"true"` / `"false"` / `"auto"` — derive inertial from geom density when `<inertial>` missing. |
| `inertiagrouprange`   | ⚠️      | 2     | Restricts which geom groups contribute to derived inertia. |
| `meshdir`             | ✅      | 2     | Resolves mesh `file=` paths; combined with the URL given to `from_file`. |
| `texturedir`          | 📦     | 2     | We don't load textures; record for downstream use. |
| `assetdir`            | ✅      | 2     | Default for both meshdir and texturedir if they aren't set. |
| `strippath`           | ✅      | 2     | If `true`, strip directory components from `file=` references (compat behavior with old MJCFs). |
| `discardvisual`       | ⚠️      | 2     | When `true`, drop visual-only geoms during compile. We can honor this at load time. |
| `convexhull`          | ⚠️      | 2     | When `true`, mesh geoms are converted to their convex hull (great fit for `MeshConverter::ConvexHull`). |
| `usethread`           | 📦     | —     | Compile-time threading; irrelevant. |
| `fitaabb`             | 📦     | —     | Used for `<geom fitscale=>`; we don't support fit-scale. |
| `balanceinertia`      | ⚠️      | 4     | Adjusts inertia diagonal to satisfy triangle inequality. |
| `fusestatic`          | 📦     | —     | MuJoCo runtime optimization. |
| `exactmeshinertia`    | ⚠️      | 2     | When inertia is derived from a mesh, use the exact tetrahedral integration vs. the AABB shortcut. |
| `inertiadensity`      | ✅      | 2     | Default density when geoms specify neither `mass` nor `density`. |
| `alignfree`           | 📦     | —     | Affects free-joint pose convention; we use the canonical convention. |

---

## `<asset>`

| Element       | Status | Phase | Notes |
| ------------- | ------ | ----- | ----- |
| `<mesh>`      | ✅      | 2     | See dedicated table. |
| `<hfield>`    | ✅      | 2     | Maps to `SharedShape::heightfield`. |
| `<texture>`   | 📦     | —     | Preserved (texture filename + procedural params); we don't load images. |
| `<material>`  | ⚠️      | 2     | Preserved metadata. `friction` etc. on materials affects the *visual* defaults only; physical friction lives on `<geom>`. |
| `<skin>`      | ❌      | —     | Skinned-mesh visualisation only. |
| `<model>`     | ❌      | —     | Cross-file model composition (rare, replaced by `<include>` in practice). |

### `<mesh>` attributes

| Attribute       | Status | Notes |
| --------------- | ------ | ----- |
| `name`          | ✅      | Used for `<geom mesh="...">` lookup. |
| `file`          | ✅      | Resolved against `meshdir` / `assetdir` / containing-file directory. |
| `scale`         | ✅      | Per-axis scale, applied to vertices before building the rapier shape. |
| `vertex`        | ✅      | Inline vertex list (alternative to `file`). |
| `face`          | ✅      | Inline face list. |
| `normal`        | 📦     | Visualisation only; ignored for collision. |
| `texcoord`      | 📦     | Visualisation only. |
| `refpos`        | ✅      | Pre-applied to the mesh's local pose. |
| `refquat`       | ✅      | Pre-applied to the mesh's local pose. |
| `smoothnormal`  | 📦     | Visualisation only. |
| `maxhullvert`   | ⚠️      | Used when convex-hull conversion is requested. |
| `inertia`       | ⚠️      | `"shell"` / `"convex"` / `"exact"` — drives derived inertia computation. |
| `builtin`       | ⚠️      | Procedural meshes (`box`, `sphere`, `wedge`, etc.). Map to the equivalent rapier primitive. |
| `params`        | ⚠️      | Built-in mesh parameters. |

#### Mesh file formats

| Format | Loader                      | Phase |
| ------ | --------------------------- | ----- |
| `.stl` | `rapier3d-meshloader` (`stl` feature) | 2     |
| `.obj` | `rapier3d-meshloader` (`wavefront`)   | 2     |
| `.msh` | Custom parser in `mjcf-rs` (`msh` feature). MuJoCo's binary mesh format: 4×i32 header `(nvertex, nnormal, ntexcoord, nface)` followed by `nvertex×3 f32` positions, etc. Spec is documented in `mujoco/src/cc/engine/engine_io.cc` and the user manual ("Meshes" section). | 2     |

### `<hfield>` attributes

| Attribute  | Status | Notes |
| ---------- | ------ | ----- |
| `name`     | ✅      | Used by `<geom hfield="...">`. |
| `nrow`/`ncol` | ✅   | Grid size. |
| `size`     | ✅      | `(radius_x, radius_y, elevation_z, base_z)`. |
| `file`     | ⚠️      | PNG / custom `.hfield` binary. PNG via the `image` crate is the simplest path; `.hfield` (4-byte header + `nrow*ncol` floats) is trivial to parse. |
| `elevation` | ✅     | Inline elevation array. |

---

## `<worldbody>` / `<body>`

### `<body>` attributes

| Attribute     | Status | Phase | Notes |
| ------------- | ------ | ----- | ----- |
| `name`        | ✅      | 1     | |
| `pos`         | ✅      | 1     | Position relative to parent. |
| `quat`        | ✅      | 1     | (`w x y z` in MJCF, **note** the order). |
| `axisangle`   | ✅      | 1     | Mutually exclusive with quat/euler/xyaxes/zaxis. |
| `xyaxes`      | ✅      | 1     | |
| `zaxis`       | ✅      | 1     | |
| `euler`       | ✅      | 1     | Uses `compiler/eulerseq` order. |
| `childclass`  | ✅      | 2     | Default class for child elements. |
| `mocap`       | ⚠️      | 1     | Map to `RigidBodyType::KinematicPositionBased`. |
| `gravcomp`    | ⚠️      | 4     | 1.0 = perfect gravity compensation; we apply it as a per-body inverted gravity force or via the rigid-body's `gravity_scale`. |
| `sleep`       | ⚠️      | 1     | Per-body sleep allowed flag → rapier's `can_sleep`. |
| `user`        | 📦     | —     | Custom data, preserved. |

### Children of `<body>`

| Child             | Status | Phase | Notes |
| ----------------- | ------ | ----- | ----- |
| `<inertial>`      | ✅      | 1     | Mass + inertia frame. |
| `<joint>`         | ✅      | 1     | Connects this body to its parent. Multiple = serial chain (Phase 1 supports the single-joint and zero-joint cases; multi-joint handled in 1.5). |
| `<freejoint>`     | ✅      | 1     | Equivalent to `<joint type="free"/>` but cannot coexist with other joints. |
| `<geom>`          | ✅      | 1     | See dedicated table. |
| `<site>`          | 📦     | 1     | Named reference frame; no rapier object. Stored on the body so equality/actuator references can resolve them. |
| `<camera>`        | 📦     | —     | Visualisation only. |
| `<light>`         | 📦     | —     | Visualisation only. |
| `<body>`          | ✅      | 1     | Recurse. |
| `<frame>`         | ✅      | 2     | Pure pose-grouping — fold its pose into the children's local poses. |
| `<composite>`     | ❌      | —     | Macro that expands to many bodies (rope, cloth, etc.). |
| `<flexcomp>`      | ❌      | —     | Deformable / soft body — out of scope. |
| `<attach>`        | ❌      | —     | Cross-file body grafting; replaced in 99% of files by `<include>`. |
| `<plugin>`        | ❌      | —     | MuJoCo plugins. |

### `<inertial>` attributes

| Attribute      | Status | Phase | Notes |
| -------------- | ------ | ----- | ----- |
| `pos`          | ✅      | 1     | Position of inertial frame in the body frame. |
| `quat`/`euler`/`axisangle`/`xyaxes`/`zaxis` | ✅ | 1 | Orientation of the inertial frame. |
| `mass`         | ✅      | 1     | |
| `diaginertia`  | ✅      | 1     | Maps to `MassProperties::with_principal_inertia`. |
| `fullinertia`  | ✅      | 1     | `(ixx iyy izz ixy ixz iyz)` → `MassProperties::with_inertia_matrix`. |

### `<joint>` attributes

| Attribute             | Status | Phase | Notes |
| --------------------- | ------ | ----- | ----- |
| `name`                | ✅      | 1     | |
| `type`                | ✅      | 1     | hinge → revolute, slide → prismatic, ball → spherical, free → no joint (free body). |
| `pos`                 | ✅      | 1     | Joint anchor in the **body** frame (after `<body pos/quat>`). Drives `local_anchor1/2`. |
| `axis`                | ✅      | 1     | Joint axis in the body frame; ignored for `ball`/`free`. |
| `range`               | ✅      | 1     | Joint limits (degrees by default — convert per `compiler/angle`). |
| `limited`             | ✅      | 1     | `"true"` / `"false"` / `"auto"` (autolimits). |
| `class`               | ✅      | 2     | Default-class lookup. |
| `group`               | 📦     | —     | Visualization grouping. |
| `springdamper`        | ✅      | 4     | `(timeconst, dampratio)` — derive `(stiffness, damping)`. |
| `stiffness`           | ✅      | 4     | → `JointMotor::motor_position(0, stiffness, damping, …)`. |
| `damping`             | ✅      | 4     | |
| `springref`           | ✅      | 4     | Reference position for the spring. |
| `armature`            | ⚠️      | 4     | Adds rotor inertia along the joint axis. Best modeled by adding `armature` to the link's inertia tensor at the joint axis. |
| `frictionloss`        | ⚠️      | 4     | Coulomb friction on the joint. Approximated via a velocity-based motor with capped force. |
| `actuatorfrclimited`/`actuatorfrcrange` | 📦 | 5 | Pass-through. |
| `ref`                 | ⚠️      | 4     | Reference value; the joint reads zero at this configuration. We bake it into the local frames. |
| `solreflimit`/`solimplimit`/`solreffriction`/`solimpfriction` | 📦 | — | MuJoCo's softness parameters; we don't try to translate them. |
| `margin`              | 📦     | —     | |
| `user`                | 📦     | —     | |

### `<freejoint>` attributes

| Attribute | Status | Phase | Notes |
| --------- | ------ | ----- | ----- |
| `name`    | ✅      | 1     | |
| `group`   | 📦     | —     | |
| `align`   | 📦     | —     | Pose convention switch; we always use the natural one. |

### `<geom>` attributes

| Attribute               | Status | Phase | Notes |
| ----------------------- | ------ | ----- | ----- |
| `name`                  | ✅      | 1     | |
| `type`                  | ✅      | 1     | See shape table below. |
| `size`                  | ✅      | 1     | Interpretation depends on `type`. |
| `pos`/`quat`/`euler`/`axisangle`/`xyaxes`/`zaxis` | ✅ | 1 | Local pose in the body frame. |
| `class`                 | ✅      | 2     | |
| `material`              | 📦     | 2     | Visual; preserved. |
| `rgba`                  | 📦     | —     | |
| `friction`              | ✅      | 1     | `(slide, spin, roll)` — we use the slide value as rapier's friction; spin/roll are recorded but not simulated. |
| `mass`                  | ✅      | 1     | Direct mass override. Wins over `density`. |
| `density`               | ✅      | 1     | Default 1000.0 (water). |
| `solref`/`solimp`       | 📦     | —     | |
| `margin`                | ⚠️      | 1     | → `Collider::contact_skin`. |
| `gap`                   | 📦     | —     | |
| `contype`/`conaffinity` | ✅      | 3     | Contact-bitmask filtering — map to `InteractionGroups`. |
| `condim`                | ⚠️      | 4     | Contact dimensionality (1/3/4/6); rapier always uses 3 in 3D, so we just record it. |
| `priority`              | 📦     | —     | Friction-combination priority. |
| `group`                 | 📦     | —     | Visibility group (also drives `discardvisual`). |
| `mesh`                  | ✅      | 2     | References an `<asset><mesh>`. |
| `hfield`                | ✅      | 2     | References an `<asset><hfield>`. |
| `fitscale`              | ⚠️      | 2     | Rescales mesh to fit AABB. |
| `fluidshape`/`fluidcoef`| ❌      | —     | Fluid-drag; not in rapier. |
| `user`                  | 📦     | —     | |

#### Geom-type → rapier shape mapping

| MJCF type | rapier shape                                | Notes |
| --------- | ------------------------------------------- | ----- |
| `plane`   | `SharedShape::halfspace(Y axis)`            | `size = (x_extent, y_extent, grid_spacing)` — only the half-space is used; extent + grid are visualization. |
| `hfield`  | `SharedShape::heightfield`                  | Heights from the asset. |
| `sphere`  | `SharedShape::ball(size[0])`                | |
| `capsule` | `SharedShape::capsule_z(size[1], size[0])`  | size[0] = radius, size[1] = half-length along Z. From-to form: build from the `fromto` attribute. |
| `cylinder`| `SharedShape::cylinder(size[1], size[0])`   | size[0] = radius, size[1] = half-length along Z. |
| `box`     | `SharedShape::cuboid(size[0..3])`           | size = half-extents (matches rapier convention). |
| `ellipsoid` | ⚠️ approximated as `SharedShape::convex_hull` of an icosphere scaled by `size`, OR a ball if `size = (r,r,r)`. | Rapier has no ellipsoid primitive. Documented as a lossy approximation. |
| `mesh`    | `SharedShape::trimesh(_with_flags)` or `convex_hull` (driven by `MeshConverter` and `compiler/convexhull`) | |
| `sdf`     | ❌ skipped with a warning                    | Rapier has no SDF shape. |

`fromto` — capsules / cylinders / boxes can be specified as `fromto="x1 y1 z1 x2 y2 z2"`,
which determines orientation, length, and (for box) half-extents from two endpoints.
Phase 1 supports `fromto` for capsule/cylinder; Phase 1 also for box.

### `<site>` attributes

| Attribute  | Status | Phase | Notes |
| ---------- | ------ | ----- | ----- |
| `name`/`pos`/`quat`/`euler`/… | 📦 | 1 | Stored on the parent body for reference. |
| `type`/`size`/`rgba` | 📦 | — | Visual only. |

---

## `<default>`

Class definitions that supply per-element defaults. The parser resolves them
**before** the loader runs, so the loader sees fully-populated elements.

| Behavior                                   | Status | Phase |
| ------------------------------------------ | ------ | ----- |
| Top-level `<default>` (the `"main"` class) | ✅      | 2     |
| Named classes via `<default class="…">`    | ✅      | 2     |
| Nested `<default>` (class inheritance)     | ✅      | 2     |
| `class="…"` on an element                  | ✅      | 2     |
| `childclass="…"` on `<body>`               | ✅      | 2     |

Defaults can be supplied for: `joint`, `geom`, `site`, `camera`, `light`,
`mesh`, `material`, `pair`, `equality`, `tendon`, `general` (= actuator),
`motor`/`position`/`velocity`/etc. The parser stores the defaults verbatim
and applies them to instances in pre-order.

---

## `<include>`

| Behavior                                  | Status | Phase |
| ----------------------------------------- | ------ | ----- |
| `<include file="...">` inlining            | ✅      | 2     |
| Recursive inclusion                       | ✅      | 2     |
| Cycle detection                           | ✅      | 2     |

Resolved relative to the **including** file's directory. The parser inlines
included files before any other processing.

---

## `<contact>`

| Element       | Status | Phase | Notes |
| ------------- | ------ | ----- | ----- |
| `<pair>`      | ✅      | 3     | Override friction / solref / solimp / condim / margin / gap on a specific geom pair. Maps to a custom contact-modification path or a per-collider material clone in rapier. |
| `<exclude>`   | ✅      | 3     | Disable contacts between two bodies → `InteractionGroups`. |

`<geom>` `contype` / `conaffinity` are also part of contact filtering and are
covered in the geom table above.

---

## `<equality>`

| Element       | Status | Phase | Notes |
| ------------- | ------ | ----- | ----- |
| `<connect>`   | ✅      | 3     | Point-to-point constraint between two bodies. Maps to a `GenericJoint` with all linear axes locked at the chosen anchors and all angular axes free → spherical at the anchor. |
| `<weld>`      | ✅      | 3     | Rigid attachment between two bodies. Maps to a fixed joint (or generic joint with all six axes locked). |
| `<joint>`     | ❌      | —     | Polynomial coupling between two joint coordinates. Not natively supported by rapier. |
| `<tendon>`    | ❌      | —     | Tendon length coupling. |
| `<flex>`/`<flexvert>`/`<flexstrain>` | ❌ | — | Soft-body / FEM constraints. |

`<equality>` elements share `solref`/`solimp` softness controls; we ignore those.

---

## `<tendon>`

| Element       | Status | Phase | Notes |
| ------------- | ------ | ----- | ----- |
| `<spatial>`   | ❌      | —     | Routed cable that wraps around geoms. No first-class rapier object. |
| `<fixed>`     | ❌      | —     | Linear combination of joint coordinates; would require a custom constraint. |

---

## `<actuator>`

Pass-through only. We preserve actuator definitions on the AST and expose
helpers that, given an actuator name + a computed control value, find the
corresponding rapier joint and call `set_motor_velocity` /
`set_motor_position` /etc. The rapier joint motor is not auto-driven.

| Element         | Status | Phase | Notes |
| --------------- | ------ | ----- | ----- |
| `<motor>`       | 📦     | 5     | Force/torque control. |
| `<position>`    | 📦     | 5     | PD position controller. |
| `<velocity>`    | 📦     | 5     | PD velocity controller. |
| `<intvelocity>` | 📦     | 5     | Integrated velocity. |
| `<damper>`      | 📦     | 5     | |
| `<cylinder>`    | ❌      | —     | Pneumatic; not on the roadmap. |
| `<muscle>`      | ❌      | —     | |
| `<adhesion>`    | ❌      | —     | |
| `<general>`     | 📦     | 5     | Most general form; we record `gainprm`/`biasprm`/`dyntype`. |
| `<plugin>`      | ❌      | —     | |

---

## `<sensor>`

All sensors are preserved verbatim. None of them affect the simulation —
they query state. We optionally provide helpers that, given a sensor and the
inserted rapier handles, return the sensor's reading on demand.

Reading-implementable from rapier state (Phase 5): `accelerometer`, `velocimeter`,
`gyro`, `force`, `torque`, `jointpos`, `jointvel`, `framepos`, `framequat`,
`framelinvel`, `frameangvel`, `subtreecom`, `subtreemass`, `subtreelinvel`,
`subtreeangmom`, `clock`.

Not implementable in rapier (📦 only): `touch` (requires contact-area
integration), `rangefinder`, `camprojection`, `geomdist` etc.

---

## `<keyframe>`

| Attribute   | Status | Phase | Notes |
| ----------- | ------ | ----- | ----- |
| `<key qpos>`| ✅      | 5     | Initial joint configuration. Apply at insertion time. |
| `<key qvel>`| ✅      | 5     | Initial joint velocities. |
| `<key act>` | 📦     | 5     | Actuator activation state. |
| `<key time>`| 📦     | 5     | |
| `<key mpos>`/`<key mquat>` | ✅ | 5 | Mocap-body initial poses. |
| `<key ctrl>`| 📦     | 5     | Controller setpoints. |

---

## Miscellaneous

| Element         | Status | Notes |
| --------------- | ------ | ----- |
| `<custom>`/`<numeric>`/`<text>`/`<tuple>` | 📦 | Stored on the AST, never touched by the loader. |
| `<size>` attrs (`memory`, `nuserdata`, …) | 📦 | Memory hints. |
| `<option flag>` | 📦     | Per-feature simulator switches; we record them but rapier already has its own switches. |
