# 01 — Overview

## Goals

1. Let users load a MuJoCo XML model and turn it into a set of rapier
   `RigidBody`, `Collider`, and `GenericJoint`/multibody-joint instances.
2. Match the API shape of `rapier3d-urdf` (`Robot::from_file`, `from_str`,
   `from_robot`, `insert_using_impulse_joints`, `insert_using_multibody_joints`)
   so users moving between the two crates feel at home.
3. Preserve the original MJCF AST alongside the converted rapier objects, so
   downstream code can inspect MJCF-only metadata (sites, actuators, sensors,
   names, classes, …) that rapier doesn't simulate.
4. Pure Rust. No `libmujoco`, no C bindings.

## Non-goals

- We do **not** aim to **simulate** MJCF — only to load the kinematic / dynamic
  description into rapier. Anything MJCF-specific that rapier cannot represent
  is either (a) approximated, (b) preserved as metadata for the caller, or
  (c) explicitly skipped. See [`04-out-of-scope.md`](04-out-of-scope.md).
- We do not aim to round-trip MJCF (load + edit + write). The parser may grow a
  serializer later, but it isn't on the roadmap.

## Two-crate split

Mirrors the URDF stack:

```
crates/
├── mjcf-rs/           ← XML → typed MJCF AST (no rapier deps)
└── rapier3d-mjcf/     ← MJCF AST → rapier rigid bodies / colliders / joints
```

Why split:

- The parser has zero rapier dependency, so it can be reused (e.g. by other
  Dimforge crates, by users who want to read MJCF without using rapier).
- It keeps the rapier-side conversion logic small and focused.
- It mirrors `urdf-rs` / `rapier3d-urdf`, so the codebase stays consistent.

## Why MJCF differs structurally from URDF

The two formats look superficially similar but model the world differently:

| Concept                | URDF                                     | MJCF                                                  |
| ---------------------- | ---------------------------------------- | ----------------------------------------------------- |
| Topology               | Flat list of `<link>` + `<joint>` (joint names parent + child by name) | Nested `<body>` tree (joint sits **inside** the child body and connects it to its parent) |
| Default angle units    | radians                                  | **degrees** (configurable via `<compiler angle>`)      |
| Default geom type      | n/a (each shape has its own tag)         | `sphere` (with `size` interpretation depending on `type`) |
| Joints per link        | exactly 1                                | **0..N** (multiple joints in one body = serial DoFs)   |
| Default class system   | none                                     | `<default>` with nested classes and `class` / `childclass` attributes |
| File composition       | flat, single file                        | `<include file="..."/>` recursively inlines other files |
| Geom origin            | shape-local                              | shape-local, but the body's **inertial frame may differ from the body frame** |
| Coordinate convention  | right-handed, X-forward                  | right-handed, **Z-up**                                |
| Mesh formats           | STL/OBJ/Collada via robot description    | STL / OBJ / `.msh` (custom MuJoCo binary)              |

Concrete consequences for the loader:

- **Multiple joints per body**: rapier's multibody / impulse-joint model
  permits at most one joint per body pair. The loader has to insert massless
  intermediate rigid-bodies for the joints "between" the parent and the
  declared body. (Phase 1 covers the single-joint case; Phase 1.5 / Phase 2
  handles multi-joint bodies.)
- **`<freejoint>`**: the body's parent is the world and there is no joint —
  emit a free dynamic rigid-body. No constraint to insert.
- **0 joints, parent = world**: the body is welded to the world; force-fixed
  rigid-body, just like the URDF "empty-fixed-link → squeeze" fix-up.
- **0 joints, non-world parent**: the body is welded to its parent → emit a
  fixed joint between the two.
- **`compiler/angle="degree"`** is the default, so we must convert to
  radians before handing values to rapier (URDF doesn't have this trap).
- **`compiler/eulerseq`** changes the convention used for `<… euler="…">`
  attributes (default `"xyz"`). The default differs from `glamx`'s default
  parser convention, so we handle each case explicitly.

## API shape (target)

```rust
pub struct MjcfLoaderOptions {
    pub create_colliders_from_collision_shapes: bool, // geom group/contype-aware (see Phase 3)
    pub create_colliders_from_visual_shapes: bool,    // geoms with contype=conaffinity=0
    pub apply_imported_mass_props: bool,
    pub enable_joint_collisions: bool,
    pub make_roots_fixed: bool,
    pub trimesh_flags: TriMeshFlags,
    pub mesh_converter: Option<MeshConverter>,
    pub shift: Pose,
    pub scale: Real,
    pub collider_blueprint: ColliderBuilder,
    pub rigid_body_blueprint: RigidBodyBuilder,
    pub squeeze_empty_fixed_bodies: bool, // analogue of squeeze_empty_fixed_links
}

pub struct MjcfRobot {
    pub bodies: Vec<MjcfBody>,    // one entry per <body>, plus implicit world at index 0
    pub joints: Vec<MjcfJoint>,   // one entry per joint reachable in the kinematic tree
    pub equality: Vec<MjcfEqualityJoint>, // <equality> mapped to extra impulse joints
}

impl MjcfRobot {
    pub fn from_file(path, options, asset_dir) -> anyhow::Result<(Self, mjcf_rs::Model)>;
    pub fn from_str(str, options, asset_dir) -> anyhow::Result<(Self, mjcf_rs::Model)>;
    pub fn from_model(model, options, asset_dir) -> Self;
    pub fn insert_using_impulse_joints(self, ...) -> MjcfRobotHandles<ImpulseJointHandle>;
    pub fn insert_using_multibody_joints(self, ..., MjcfMultibodyOptions) -> MjcfRobotHandles<Option<MultibodyJointHandle>>;
    pub fn append_transform(&mut self, transform: &Pose);
}
```

Same `MjcfMultibodyOptions::JOINTS_ARE_KINEMATIC | DISABLE_SELF_CONTACTS`
flags as the URDF loader.

## Cargo features

Same shape as `rapier3d-urdf`:

- `stl` — load `.stl` meshes via `rapier3d-meshloader`
- `wavefront` — load `.obj` meshes via `rapier3d-meshloader`
- `msh` — parse MuJoCo's custom `.msh` mesh format (pure Rust, written
  in `mjcf-rs` since the format is MJCF-specific). See Phase 2.

(Collada is intentionally absent — MJCF doesn't use it.)
