# Rust API correspondence

The header is generated from Rust. The C API preserves Rapier's operation and
handle semantics while making `R3World` the sole owner of simulation components.
It is a broad runtime binding, not a binding of every Rust public item, solver
implementation detail, or Parry re-export.

| Rust area | C surface |
| --- | --- |
| Math | Dimension-specific vectors, angular vectors, rotations, poses; optional C11/C++17 math constructors and arithmetic; AABBs, mass properties, groups, 128-bit user data, spring coefficients |
| `PhysicsWorld` | Construction, insertion from reusable descriptions, gravity, stepping, collision-only updates, snapshots, queries, debug lines |
| `PhysicsPipeline`, `CollisionPipeline` | World-owned workspaces; `Step` and `DetectCollisions`, hooks/events, dedicated parallel thread pools, native step timing and counter enablement |
| `IntegrationParameters` | Timestep, CCD, solver iterations, contact softness, length scale, warmstarting, clustering, recycling, friction bias; soft-body recovery and optional FEM scalar settings |
| `RigidBodyBuilder`, `RigidBody` | Types, transforms, kinematic targets, velocities, forces, impulses, torque, damping, gravity, mass/inertia, locks, sleep/wake, CCD, gyroscopic forces, dominance, iteration counts, user data, collider handles |
| `RigidBodySet`, `ColliderSet` | World-based insertion, handle lookup, enumeration, containment, coordinated removal, body-to-collider pose propagation |
| `SharedShape` | Balls, boxes, rounded boxes, capsules, segments, triangles, halfspaces, 3D cylinders/cones, convex hulls, convex decomposition, polylines, triangle meshes, compounds, heightfields with flags, triangle-mesh flags, voxels from points/meshes and voxel editing; bounds, point containment, mass properties |
| `ColliderBuilder`, `Collider` | Direct shape constructors, pose/local pose, density/mass, friction/restitution and combine rules, sensor/enabled, groups, collision types, events/hooks, contact skin, force threshold, parent, user data, bounds |
| Joints | Generic joints; fixed/revolute/prismatic/rope/spring/spherical/pin-slot constructors; frames, axes, limits, motors, motor models/force, contacts, softness, user data |
| `ImpulseJointSet` | Insertion/removal, handles, connected bodies, copied joint descriptions and setters with wake control |
| `MultibodyJointSet` | Insertion/removal, handles, joint data, tuning updates preserving degrees of freedom, articulation velocity read/write, inverse kinematics, generalized displacements |
| `QueryPipeline` | World queries with reusable POD options, filters and scoped C predicates, ray and linear shape casts, point projection, point/shape/AABB intersections |
| `NarrowPhase` | Contact pair lookup/enumeration, aggregate impulses, rigid geometric contact points, sensor intersection pairs |
| Events/hooks | Thread-safe event collection for collisions, contact forces and tears; contact/intersection filtering; rigid manifold material/normal/enabled modification, native one-way-platform context and tangent velocity |
| `SoftBodyBuilder`, `SoftBodySet` | Custom particles/edges/cells/surfaces/masses; rope, cloth/tube, cuboid/grid, triangle-mesh/polyline and volumetric generators; particle settings, material, cell model, shape matching, self-contact, surface collider, insertion/removal |
| `SoftBody` | Particle positions/velocities/targets/pinning, forces/impulses, rigid attachments, material, enable/wake, topology, volume, clusters/proxies, piece handles, deformed mesh vertices/indices by stable mesh ID, including collider-free skins |
| Soft topology | Immediate cuts/tears, owned tear events, particle destinations, torn/removed/inserted elements, pieces, cluster splits and moved joints; adding/removing/configuring clusters; direct/by-position/skinned deformable collider bindings |
| `KinematicCharacterController` | Up/offset/sliding/slopes/autostep/ground snapping; shape movement, collision output and collision impulse solving |
| `DynamicRayCastVehicleController` | 3D chassis, wheel creation/tuning, controls, vehicle axes, stepping, speed and wheel/contact state |
| PID controller | Native gains, controlled axes, rigid-body velocity corrections |
| URDF/MJCF | Optional 3D/f32 importers, options, both joint insertion paths, body handles, keyframes, scaled actuator controls, visual shapes/UVs/normals/textures/materials |
| Debug rendering | Caller-owned HSLA line buffers using Rapier debug mode flags |

## Deliberate adaptations

- Descriptions and configuration snapshots are POD values. Constructors return them
  directly; insertion copies them into the world. Owned builders and independent
  sets/pipelines are removed, with no compatibility aliases.
- Insertions clone inputs and removals discard removed objects. This avoids ambiguous
  ownership transfer. A shape clone shares its immutable geometry through an Arc.
- Runtime Rust references become world-and-handle operations. Iterators/slices become
  caller-owned buffers or explicit accessor calls. Error/Option results become
  statuses, invalid handles, or documented nullable outputs.
- Standard joint constructors return `R3JointDesc`, preserving the Rust
  `Into<GenericJoint>` relationship without an additional C allocation layer for
  each joint-builder family.
- The character controller retains the last movement's collisions for buffer access.
  Its impulse helper and the vehicle's update helper accept `PhysicsWorld` to avoid
  exporting a mutable query view with difficult cross-language lifetime rules.
- Physics callbacks receive a scoped read context, plus their user pointer. World
  reentrancy conflicts return `R3_WORLD_BUSY`; contact edits use the contact context.
  Events are collected for polling and mutations after stepping.

## Explicit gaps

These are not exposed in this initial ABI:

- Every specialized shape parameter/flag, direct editable heightfield data,
  detailed shape-type introspection, standalone Parry pairwise queries, nonlinear
  shape casts.
- Per-contact anchor edits, soft-contact candidate modification callbacks,
  and full solver-manifold/soft-contact internal views. The exposed contact points
  are geometric manifolds; contact-pair totals handle clustering correctly.
- Link/Jacobian internals, standalone passive-spring configuration, every robotics
  loader/sensor option, and detailed profiler data.
- Every soft-body meshing/generator option, all per-element material overrides, custom
  skin-mapping internals, and the full graph of diagnostic soft-body recovery data.
  Arbitrary particle/cell/surface inputs and common mesh/cluster workflows are covered.
- Individual serialization APIs for each set/type, a stable cross-version snapshot
  format, simultaneous f32/f64 variants of the same dimension, and engine editor
  plugins or full generated language wrappers. 2D and 3D already have distinct
  exported namespaces.

New wrappers can be added without changing existing object layouts. Incompatible
changes to released exports, public POD layouts, enum meanings, or signatures
require an ABI version change and coordinated consumer updates. During initial
unreleased development, the ABI stays at version 1 and consumers must use matching
header and library revisions.
