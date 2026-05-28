# Phase 2 — Defaults, Includes, and Mesh Assets

**Goal:** Load the vast majority of real-world MJCF files (humanoid,
quadrupeds, manipulators) end-to-end. After this phase, the
`mujoco_menagerie` benchmark suite — the standard collection of robot
models — should load without manual editing.

## Scope

### `mjcf-rs` (parser)

#### `<include>`

- Syntax: `<include file="..."/>` anywhere a sibling element could go.
- Resolution: relative to the **including** file's directory.
- Recurse, with cycle detection (`HashSet<canonical_path>`).
- Inline children of the included `<mujoco>` root in place. Top-level tags
  (`<asset>`, `<worldbody>`, etc.) merge with the parent file's same tags.

#### `<default>`

- Top-level `<default>` is the implicit `"main"` class.
- `<default class="…">` declares a named class.
- Nested `<default>`: the outer class is parent; the inner inherits.
- A `<default>` element holds prototype elements (`<joint>`, `<geom>`,
  `<site>`, `<mesh>`, `<material>`, `<pair>`, `<equality>`, `<tendon>`,
  `<general>` and the actuator subtypes).
- Resolution algorithm:
  1. For every concrete element (`<joint>`, `<geom>`, …) walk the class
     chain (`element.class` → … → `body.childclass` → "main").
  2. For each class, look up the prototype of the matching element type and
     overlay its attributes on a fresh "default-filled" instance.
  3. The element's own attributes overlay last (highest priority).
- After resolution, the AST contains fully-populated elements; the loader
  never has to revisit the class hierarchy.

#### `<asset>`

- `<mesh>`: store name, file, scale, refpos, refquat, inline vertex/face,
  builtin/params, maxhullvert, inertia, smoothnormal.
- `<hfield>`: name, nrow, ncol, size, file, elevation.
- `<texture>` / `<material>` / `<skin>` / `<model>`: stored verbatim (the
  loader doesn't look at them in phase 2 except `<material>` when a geom
  references it for friction defaults — and even that's metadata).

#### `<frame>` (worldbody / body child)

- A `<frame>` is a pure pose grouping: it carries `pos`/`quat`/etc. and a
  list of children, but does not produce a body.
- The parser folds `<frame>` poses into the children's local poses.

#### Compiler additions

- `meshdir`, `assetdir`, `texturedir`, `strippath`.
- `inertiafromgeom` (`true`/`false`/`auto`) — informs the loader whether to
  derive `<inertial>` from geoms.
- `inertiagrouprange`.
- `discardvisual`.
- `convexhull` — when `true`, mesh geoms are turned into their convex hull at
  load time.
- `autolimits` — when `true` (MJCF default in v3+), specifying `range`
  implies `limited="true"`.
- `inertiadensity` (default 1000.0) for derived inertia.
- `exactmeshinertia`.

### `rapier3d-mjcf` (loader)

#### Mesh & hfield colliders

- `<geom type="mesh" mesh="X">` resolves to the named asset, then:
  - If `compiler/convexhull="true"` (or the user passes
    `MeshConverter::ConvexHull`): build a convex hull collider.
  - Otherwise: build a `TriMesh` via the same path used by `rapier3d-urdf`
    (delegating to `rapier3d-meshloader`).
- `<geom type="hfield" hfield="X">` builds `SharedShape::heightfield`.
- `MjcfLoaderOptions::mesh_converter` (same field as URDF) overrides the
  default. When the user picks an OBB / ConvexHull converter and the source
  is a mesh, populate `MjcfVisual { shape: trimesh, local_pose }` so the
  rendering side can still draw the original triangles. (Direct port of the
  `UrdfVisual` mechanism.)
- Cargo features that gate mesh formats: `stl`, `wavefront`, `msh`.

#### `.msh` parser (in `mjcf-rs`, behind the crate's `msh` feature)

MuJoCo's binary `.msh`:

```
struct MshHeader {
    nvertex: i32,
    nnormal: i32,
    ntexcoord: i32,
    nface: i32,
}
// followed by:
//   f32[nvertex * 3]   positions
//   f32[nnormal * 3]   normals
//   f32[ntexcoord * 2] texcoords
//   i32[nface * 3]     indices
```

Trivial pure-Rust parse with `byteorder`. (`mjcf-rs` adds `byteorder` only
behind the `msh` feature.)

#### Inertia derivation (`compiler/inertiafromgeom`)

When `<inertial>` is absent (or `inertiafromgeom="auto"` and the original
file omitted the inertial), compute the body's inertia from the union of its
geoms:

- For each geom in the inertia-eligible group:
  - Determine effective mass: `mass` if set, else `density * volume`,
    else `compiler/inertiadensity * volume`.
  - Compute the geom's `MassProperties` in the body frame using parry3d's
    primitive-shape mass-properties helpers.
- Sum the mass properties.
- Apply `boundmass` / `boundinertia` clamps.
- Apply `settotalmass` rescaling at the model level (after all bodies have
  their inertias).
- For mesh geoms, `compiler/exactmeshinertia` switches between
  `tetrahedral_integration` (exact) and the AABB shortcut.

This makes `<inertial>`-free MJCFs (very common in the wild) load with
sensible mass properties out of the box.

#### Auto-limits

When `compiler/autolimits` is true (default for modern files):

- A joint with `range` set but no explicit `limited` defaults to
  `limited="true"`.

## Non-goals (deferred)

- `<contact>`, `<equality>`, `contype`/`conaffinity` filtering (Phase 3)
- `solref`/`solimp`, `springdamper`, `armature` translation (Phase 4)
- Keyframes (Phase 5)
- `<actuator>` / `<sensor>` semantic mapping (Phase 5)
- `<geom type="sdf">`, `<flex*>`, `<composite>`, `<skin>` are out of scope.

## Tests

- `humanoid.xml` from `mujoco_menagerie/standalone`.
- `cartpole.xml` (with `<default>`).
- `pendulum.xml` (with `<include>` of a shared `<asset>` file).
- A model with `<geom mesh>` of a `.stl` and a `.obj`, both flat and via
  `<default>`.
- A model with `compiler/inertiafromgeom="true"` and no `<inertial>` —
  verify the resulting body has the same total mass and centre of mass as
  computed by hand.
- A model that exercises `<geom fromto>` for capsules and cylinders.

## Acceptance criteria

- Roughly 80% of `mujoco_menagerie` standalone files load without
  warnings (the remainder typically use tendons / actuators / sensors that
  we cover in Phase 4–5).
- Resolved-default behavior is observable: a `<joint>` inside a body with
  `childclass="legjoint"` ends up with the legjoint's `damping`, `range`,
  etc. on the AST, with no extra work in the loader.
- Mesh colliders simulate at the same step rate as URDF mesh colliders of
  the same size (no surprise overhead).
