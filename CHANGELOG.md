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