## v0.5.0
In this release we are dropping `ncollide` and use our new crate [`parry`](https://parry.rs)
instead! This comes with a lot of new features, as well as two new crates: `rapier2d-f64` and
`rapier3d-f64` for physics simulation with 64-bits floats.

### Added
- Added a `RAPIER.version()` function at the root of the package to retrieve the version of Rapier
  as a string.

Several geometric queries have been added to the `QueryPipeline`:
- `QueryPipeline::intersections_with_ray`: get all colliders intersecting a ray.
- `QueryPipeline::intersection_with_shape`: get one collider intersecting a shape.
- `QueryPipeline::project_point`: get the projection of a point on the closest collider.
- `QueryPipeline::intersections_with_point`: get all the colliders containing a point.
- `QueryPipeline::cast_shape`: get the first collider intersecting a shape moving linearly
  (aka. sweep test).
- `QueryPipeline::intersections_with_shape`: get all the colliders intersecting a shape.

Several new shape types are now supported:
- `RoundCuboid`, `Segment`, `Triangle`, `RoundTriangle`, `Polyline`, `ConvexPolygon` (2D only),
  `RoundConvexPolygon` (2D only), `ConvexPolyhedron` (3D only), `RoundConvexPolyhedron` (3D only),
  `RoundCone` (3D only).

It is possible to build `ColliderDesc` using these new shapes:
- `ColliderBuilder::round_cuboid`, `ColliderBuilder::segment`, `ColliderBuilder::triangle`, `ColliderBuilder::round_triangle`,
  `ColliderBuilder::convex_hull`, `ColliderBuilder::round_convex_hull`, `ColliderBuilder::polyline`,
  `ColliderBuilder::convex_decomposition`, `ColliderBuilder::round_convex_decomposition`,
  `ColliderBuilder::convex_polyline` (2D only), `ColliderBuilder::round_convex_polyline` (2D only),
  `ColliderBuilder::convex_mesh` (3D only),`ColliderBuilder::round_convex_mesh` (3D only), `ColliderBuilder::round_cone` (3D only).

It is possible to specify different rules for combining friction and restitution coefficients
of the two colliders involved in a contact with:
- `ColliderDesc::friction_combine_rule`, and `ColliderDesc::restitution_combine_rule`.

Various RigidBody-related getter and setters have been added:
- `RigidBodyBuilder::gravity_scale`, `RigidBody::gravity_scale`, `RigidBody::set_gravity_scale` to get/set the scale
  factor applied to the gravity affecting a rigid-body. Setting this to 0.0 will make the rigid-body ignore gravity.
- `RigidBody::set_linear_damping` and `RigidBody::set_angular_damping` to set the linear and angular damping of
  the rigid-body.
- `RigidBodyBuilder::restrict_rotations` to prevent rotations along specific coordinate axes. This replaces the three
  boolean arguments previously passed to `.set_principal_angular_inertia`.
  
### Breaking changes
Breaking changes related to contacts:
- The way contacts are represented changed. Refer to the documentation of `parry::query::ContactManifold`, `parry::query::TrackedContact`
  and `rapier::geometry::ContactManifoldData` and `rapier::geometry::ContactData` for details.

Breaking changes related to rigid-bodies:
- The `RigidBodyDesc.setMass` takes only one argument now. Use `RigidBodyDesc.lockTranslations` to lock the translational
  motion of the rigid-body.
- The `RigidBodyDesc.setPrincipalAngularInertia` no longer have boolean parameters to lock rotations.
  Use `RigidBodyDesc.lockRotations` or `RigidBodyDesc.restrictRotations` to lock the rotational motion of the rigid-body.

Breaking changes related to colliders:
- The collider shape type has been renamed from `ColliderShape` to `SharedShape` (now part of the Parry crate).
- The `Polygon` shape no longer exists. For a 2D convex polygon, use a `ConvexPolygon` instead.
- All occurrences of `Trimesh` have been replaced by `TriMesh` (note the change in case).

Breaking changes related to events:
- Rename all occurrences of `Proximity` to `Intersection`.
- The `Proximity` enum has been removed, it's replaced by a boolean.

## v0.4.2
- Fix a bug in angular inertia tensor computation that could cause rotations not to
  work properly.
- Add `RigidBody::set_mass_properties` to set the mass properties of an already-constructed
  rigid-body.

## v0.4.1
- The `RigidBodyBuilder::principal_inertia` method has been deprecated and renamed to
  `principal_angular_inertia` for clarity.

## v0.4.0
- The rigid-body `linvel`, `angvel`, and `position` fields are no longer public. Access using
  their corresponding getters/setters. For example: `rb.linvel()`, `rb.set_linvel(vel, true)`.
- Add `RigidBodyBuilder::sleeping(true)` to allow the creation of a rigid-body that is asleep
  at initialization-time.

#### Locking translation and rotations of a rigid-body
- Add `RigidBodyBuilder::lock_rotations` to prevent a rigid-body from rotating because of forces.
- Add `RigidBodyBuilder::lock_translations` to prevent a rigid-body from translating because of forces.
- Add `RigidBodyBuilder::principal_inertia` for setting the principal inertia of a rigid-body, and/or
  preventing the rigid-body from rotating along a specific axis.
- Change `RigidBodyBuilder::mass` by adding a bool parameter indicating whether or not the collider
  contributions should be taken into account in the future too.

#### Reading contact and proximity information
- Add `NarrowPhase::contacts_with` and `NarrowPhase::proximities_with` to retrieve all the contact
  pairs and proximity pairs involving a specific collider.
- Add `NarrowPhase::contact_pair` and `NarrowPhase::proximity_pair` to retrieve one specific contact
  pair or proximity pair if it exists.
- Add `NarrowPhase::contact_pairs`, and `NarrowPhase::proximity_pairs` to retrieve all the contact or
  proximity pairs detected by the narrow-phase.

## v0.3.2
- Add linear and angular damping. The damping factor can be set with `RigidBodyBuilder::linear_damping` and
  `RigidBodyBuilder::angular_damping`.
- Implement `Clone` for almost everything that can be worth cloning.
- Allow setting the initial mass and mass properties of a rigid-bodies using `RigidBodyBuilder::mass` and
  `RigidBodyBuilder::mass_properties`.
- The restitution coefficient of colliders is now taken into account by the physics solver.

## v0.3.1
- Fix non-determinism problem when using triangle-meshes, cone, cylinders, or capsules.
- Add `JointSet::remove(...)` to remove a joint from the `JointSet`.

## v0.3.0
- Collider shapes are now trait-objects instead of a `Shape` enum.
- Add a user-defined `u128` to each colliders and rigid-bodies for storing user data.
- Add the support for `Cylinder`, `RoundCylinder`, and `Cone` shapes.
- Added the support for collision filtering based on bit masks (often known as collision groups, collision masks, or
  collision layers in other physics engines). Each collider has two groups. Their `collision_groups` is used for filtering
  what pair of colliders should have their contacts computed by the narrow-phase. Their `solver_groups` is used for filtering
  what pair of colliders should have their contact forces computed by the constraints solver.
- Collision groups can also be used to filter what collider should be hit by a ray-cast performed by the `QueryPipeline`.
- Added collision filters based on user-defined trait-objects. This adds two traits `ContactPairFilter` and
  `ProximityPairFilter` that allows user-defined logic for determining if two colliders/sensors are allowed to interact.
- The `PhysicsPipeline::step` method now takes two additional arguments: the optional `&ContactPairFilter` and `&ProximityPairFilter`
for filtering contact and proximity pairs.

## v0.2.1
- Fix panic in TriMesh construction and QueryPipeline update caused by a stack overflow or a subtraction underflow.

## v0.2.0
The most significant change on this version is the addition of the `QueryPipeline` responsible for performing
scene-wide queries. So far only ray-casting has been implemented.

- Add `ColliderSet::remove(...)` to remove a collider from the `ColliderSet`.
- Replace `PhysicsPipeline::remove_rigid_body` by `RigidBodySet::remove`.
- The `JointSet.iter()` now returns an iterator yielding `(JointHandle, &Joint)` instead of just `&Joint`.
- Add `ColliderDesc::translation(...)` to set the translation of a collider relative to the rigid-body it is attached to.
- Add `ColliderDesc::rotation(...)` to set the rotation of a collider relative to the rigid-body it is attached to.
- Add `ColliderDesc::position(...)` to set the position of a collider relative to the rigid-body it is attached to.
- Add `Collider::position_wrt_parent()` to get the position of a collider relative to the rigid-body it is attached to.
- Modify `RigidBody::set_position(...)` so it also resets the next kinematic position to the same value.
- Deprecate `Collider::delta()` in favor of the new `Collider::position_wrt_parent()`.
- Fix multiple issues occurring when having colliders resulting in a non-zero center-of-mass.
- Fix a crash happening when removing a rigid-body with a collider, stepping the simulation, adding another rigid-body
 with a collider, and stepping the simulation again.
- Fix NaN when detection contacts between two polygonal faces where one has a normal perfectly perpendicular to the
separating vector.
- Fix bug collision detection between trimeshes and other shapes. The bug appeared depending on whether the trimesh
collider was added before the other shape's collider or after.