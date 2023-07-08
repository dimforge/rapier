## Unlereased
### Added
- Add `SphericalJoint::local_frame1/2`, `::set_local_frame1/2`, and `SphericalJointBuilder::local_frame1/2` to set both
  the joint’s anchor and reference orientation.
- Add `EffectiveCharacterMovement::is_sliding_down_slope` to indicate if the character controlled by the kinematic
  character controller is sliding on a slope that is too steep.
- Add `Wheel::side_friction_stiffness` to customize the side friction applied to the vehicle controller’s wheel.

### Modified
- Make `Wheel::friction_slip` public to customize the front friction applied to the vehicle controller’s wheels.
- Add the `DebugRenderBackend::filter_object` predicate that can be implemented to apply custom filtering rules
  on the objects being rendered.

## v0.17.2 (26 Feb. 2023)
### Fix
- Fix issue with convex polyhedron jitter due to missing contacts.
- Fix character controller getting stuck against vertical walls.
- Fix character controller’s snapping to ground not triggering sometimes.
- Fix character controller’s horizontal offset being mostly ignored and some instances of vertical offset being ignored.

## v0.17.1 (22 Jan. 2023)
### Fix
- Fix bug resulting in dynamic rigid-bodies acting as kinematic bodies after being disabled and then re-enabled.


## v0.17.0 (15 Jan. 2023)
### Added
- Add `RigidBody::set_enabled`, `RigidBody::is_enabled`, `RigidBodyBuilder::enabled` to enable/disable a rigid-body
  without having to delete it. Disabling a rigid-body attached to a multibody joint isn’t supported yet.
- Add `Collider::set_enabled`, `Collider::is_enabled`, `ColliderBuilder::enabled` to enable/disable a collider
  without having to delete it.
- Add `GenericJoint::set_enabled`, `GenericJoint::is_enabled` to enable/disable a joint without having to delete it.
  Disabling a multibody joint isn’t supported yet.
- Add `DynamicRayCastVehicleController`, a vehicle controller based on ray-casting and dynamic rigid-bodies (mostly
  a port of the vehicle controller from Bullet physics).
- Add `RigidBody::user_force` and `RigidBody::user_torque` to read the forces or torques added by the user to a
  dynamic rigid-body.
- Add `RigidBody::locked_axes` to get the rigid-body axes that were locked by the user.

### Modified
- Add the `QueryPipeline` as an optional argument to `PhysicsPipeline::step` and `CollisionPipeline::step`. If this
  argument is specified, then the query pipeline will be incrementally (i.e. more efficiently) update at the same time as
  these other pipelines. In that case, calling `QueryPipeline::update` a `PhysicsPipeline::step` isn’t needed.
- `RigidBody::set_body_type` now takes an extra boolean argument indicating if the rigid-body should be woken-up
  (if it becomes dynamic).
- `RigidBody::mass_properties` now also returns the world-space mass-properties of the rigid-body.

### Fix
- Fix bug resulting in rigid-bodies being awakened after they are created, even if they are created sleeping.

## v0.16.1 (10 Nov. 2022)
### Fix
- Fixed docs build on `docs.rs`.

## v0.16.0 (30 Oct. 2022)
### Added
- Implement `Copy` for `CharacterCollision`.
- Implement conversion (`From` trait) between `Group` and `u32`.
- Add `ColliderBuilder::trimesh_with_flags` to build a triangle mesh with specific flags controlling
  its initialization.

### Modified
- Rename `AABB` to `Aabb` to comply with Rust’s style guide.
- Switch to `parry 0.11`.

### Fix
- Fix internal edges of 3D triangle meshes or 3D heightfields generating invalid contacts preventing
  balls from moving straight.

## v0.15.0 (02 Oct. 2022)
### Added
- Add a **kinematic character** controller implementation. See the `control` module. The character controller currently
  supports the following features:
    - Slide on uneven terrains
    - Interaction with dynamic bodies.
    - Climb stairs automatically.
    - Automatically snap the body to the floor when going downstairs.
    - Prevent sliding up slopes that are too steep
    - Prevent sliding down slopes that are not steep enough
    - Interactions with moving platforms.
    - Report information on the obstacles it hit on its path.
- Implement `serde` serialization/deserialization for `CollisionEvents` when the `serde-serialize` feature is enabled


### Modified
- The methods `Collider::set_rotation`, `RigidBody::set_rotation`, and `RigidBody::set_next_kinematic_rotation` now
  take a rotation (`UnitQuaternion` or `UnitComplex`) instead of a vector/angle.
- The method `QueryFilter::exclude_dynamic` is now a static method (the `self` argument was removed).
- The `QueryPipeline::cast_shape` method has a new argument `stop_at_penertation`. If set to `false`, the linear
  shape-cast won’t immediately stop if the shape is penetrating another shape at its starting point **and** its
  trajectory is such that it’s on a path to exist that penetration state.
- The `InteractionGroups` is now a set of explicit bit flags instead of a raw `u32`.
- The world-space mass properties of rigid-bodies are now updated automatically whenever the user changes their
  position.

## v0.14.0 (09 July 2022)
### Fixed
- Fix unpredictable broad-phase panic when using small colliders in the simulation.
- Fix collision events being incorrectly generated for any shape that produces multiple
  contact manifolds (like triangle meshes).
- Fix panic in the `CollisionPipeline` if a collider is both added and removed before a call
  to `CollisionPipeline::step`.

### Modified
- The `RigidBodyBuilder::additional_mass` method will now result in the additional angular inertia
  being automatically computed based on the shapes of the colliders attached to the rigid-body.
- Remove the deprecated methods `RigidBodyBuilder::mass`, `::principal_angular_inertia`, `::principal_inertia`.
- Remove the methods `RigidBodyBuilder::additional_principal_angular_inertia`. Use
  `RigidBodyBuilder::additional_mass_properties` instead.
- The `Collider::density` method now always returns a `Real` (instead of an `Option<Real>`).
- Rename `RigidBody::restrict_rotations` and `RigidBody::restrict_translations` to
  `RigidBody::set_enabled_rotations` and `RigidBody::set_enabled_translations`.
- Rename `RigidBodyBuilder::restrict_rotations` and `RigidBodyBuilder::restrict_translations` to
  `RigidBodyBuilder::enabled_rotations` and `RigidBodyBuilder::enabled_translations`.

### Added
- Add `RigidBody::recompute_mass_properties_from_colliders` to force the immediate computation
  of a rigid-body’s mass properties (instead of waiting for them to be recomputed during the next
  timestep). This is useful to be able to read immediately the result of a change of a rigid-body
  additional mass-properties or a change of one of its collider’s mass-properties.
- Add `RigidBody::set_additional_mass` to set the additional mass for the collider. The additional
  angular inertia is automatically computed based on the attached colliders shapes.
- Add `Collider::set_density`, `::set_mass`, `set_mass_properties`, to alter a collider’s mass
  properties. Note that `::set_mass` will result in the collider’s angular inertia being automatically
  computed based on this mass and on its shape.
- Add `ColliderBuilder::mass` to set the mass of the collider instead of its density. Its angular
  inertia tensor will be automatically computed based on this mass and its shape.
- Add `Collider::mass` and `Collider::volume` to retrieve the mass or volume of a collider.
- Add the `QueryFilter` that is now used by all the scene queries instead of the `CollisionGroups` and `Fn(ColliderHandle) -> bool`
  closure. This `QueryFilter` provides easy access to most common filtering strategies (e.g. dynamic bodies only,
  excluding one particular collider, etc.) for scene queries.
- Add force reporting based on contact force events. The `EventHandler` trait has been modified to include
  the method `EventHandler::handle_contact_force_event`. Contact force events are generated whenever the sum of the
  magnitudes of all the forces between two colliders is greater than any of their
  `Collider::contact_force_event_threshold` values (only the colliders wit the  `ActiveEvents::CONTACT_FORCE_EVENT` flag
  set are taken into account for this threshold).
- Add the `ContactForceEvent` struct that is generated by the `ChannelEventCollector` to report
  contact force events.

## v0.13.0 (31 May 2022)
### Fixed
- Fix incorrect sensor events being generated after collider removal.
- Fix bug where the CCD thickness wasn’t initialized properly.
- Fix bug where the contact compliance would result in undesired tunneling, despite CCD being enabled.

### Modified
- Add a `wake_up: bool` argument to the `ImpulseJointSet::insert` and `MultibodyJointSet::insert` to
  automatically wake-up the rigid-bodies attached to the inserted joint.
- The methods `ImpulseJointSet::remove/remove_joints_attached_to_rigid_body`,
  `MultibodyJointSet::remove/remove_joints_attached_to_rigid_body` and
  `MultibodyjointSet::remove_multibody_articulations` no longer require the `bodies`
  and `islands` arguments.
- Make the `instant` dependency optional, behind a `profiler` cargo feature.
- Rename STATIC to FIXED in the `ActiveCollisionTypes` flags.
- Rename `ImpulseJointSet::joints_with` to `::attached_joints`. Add the joint’s handle to the closure arguments.
- Make the default debug-render less noisy out-of-the-box by only rendering joints, rigid-bodies, and colliders
  by default.

### Added
- Debug-renderer: add rendering of contacts, solver contacts, and collider Aabbs
- Add `MultibodyJointSet::attached_joints` to return all the multibody joints attached to a given rigid-body.

## v0.12.0 (30 Apr. 2022)
### Fixed
- Fix the simulation when the `parallel` feature is enabled.
- Fix bug where damping would not be applied properly to some bodies.
- Fix panics caused by various situations (contact or joints) involving rigid-bodies with locked translations/rotations.
- Fix bug where collider modifications (like changes of collision groups, or shape) would not wake-up their attached
  rigid-body, or would not have any effect on pre-existing contacts.
- Fix the automatic update of a rigid-body’s mass properties after changing one of its attached colliders.
- Fix the broad-phase becoming potentially invalid after a change of collision groups.

### Modified
- Switch to `nalgebra` 0.31.
- Switch to `parry` 0.9.
- Rename `JointHandle` to `ImpulseJointHandle`.
- Rename `RigidBodyMassPropsFlags` to `LockedAxes`.
- Rename `RigidBody::apply_force`, `::apply_torque`, `::apply_force_at_point` to `::add_force`,
  `::add_torque`, and `::add_force_at_point` to better reflect the fact that they are not cleared at the end
  of the timestep.
- Rename `RigidBodyType::Static` to `RigidBodyType::Fixed` to avoid confusion with the `static` keyword.
- All method referring to `static` rigid-bodies now use `fixed` instead of `static`.
- Rename `RigidBodyBuilder::new_static, new_kinematic_velocity_based, new_kinematic_velocity_based` to
  `RigidBodyBuilder::fixed, kinematic_velocity_based, kinematic_velocity_based`.
- The `ContactEvent` and `IntersectionEvent` have been replaced by a single enum `CollisionEvent` in order
  to simplify the user’s event handling.
- The `ActiveEvents::CONTACT_EVENTS` and `ActiveEvents::INTERSECTION_EVENTS` flags have been replaced by a single
  flag `ActiveEvents::COLLISION_EVENTS`.
- Joint motors no longer have a `VelocityBased` model. The new choices are `AccelerationBased` and `ForceBased`
  which are more stable.  
- Calling the `.build()` function from builders (`RigidBodyBuilder`, `ColliderBuilder`, etc.) is no longer necessary
  whan adding them to sets. It is automatically called thanks to `Into<_>` implementations.  
- The `ComponentSet` abstractions (and related `_generic` methods like `PhysicsPipeline::step_generic`) have been
  removed. Custom storage for colliders and rigid-bodies are no longer possible: use the built-in `RigidBodySet` and
  `ColliderSet` instead.

### Semantic modifications
These are changes in the behavior of the physics engine that are not necessarily
reflected by an API change. See [#304](https://github.com/dimforge/rapier/pull/304) for extensive details.
- `RigidBody::set_linvel` and `RigidBody::set_angvel` no longer modify the velocity of static bodies.
- `RigidBody::set_body_type` will reset the velocity of a rigid-body to zero if it is static.
- Don’t automatically clear forces at the end of a timestep.
- Don’t reset the velocity of kinematic bodies to zero at the end of the timestep.
- Events `CollisionEvent::Stopped` are now generated after a collider is removed. 

### Added
- Significantly improve the API of joints by adding:
  * Builders based on the builder pattern.
  * Getters and setters for all joints.
  * Method to convert a `GenericJoint` to one of the more specific joint type.
- Improve stability of joint motors.
- Adds a `bool` argument to `RigidBodySet::remove`. If set to `false`, the colliders attached to the rigid-body
  won’t be automatically deleted (they will only be detached from the deleted rigid-body instead).
- Add `RigidBody::reset_forces` and `RigidBody::reset_torques` to reset all the forces and torques added to the
  rigid-body by the user.
- Add the `debug-render` cargo feature that enables the `DebugRenderPipeline`: a line-based backend-agnostic
  renderer to debug the state of the physics engine.

## v0.12.0-alpha.0 (2 Jan. 2022)
### Fixed
- Fixed `RigidBody::restrict_rotations` to properly take into account the axes to lock.
### Modified
- All the impulse-based joints have been replaced by a single generic 6-Dofs joint in 3D
  (or 3-Dofs joint in 2D) named `ImpulseJoint`. The `RevoluteJoint, PrismaticJoint, FixedJoint`,
  and `SphericalJoint` (formely named `BallJoint`) structures still exist but are just convenient
  ways to initialize the generic `ImpulseJoint`.
- Our constraints solver has been modified. Before we used one velocity-based resolution followed
  by one position-based resolution. We are now using two velocity-based resolution: the first one
  includes constraints regularization whereas the second one doesn’t. This simplifies the resolution
  code significantly while offering stiffer results.

### Added
- Added multibody joints: joints based on the reduced-coordinates modeling. These joints can’t 
  violate their positional constraint.
- Implement `Default` for most of the struct that supports it.

## v0.11.1
### Fixed
- Fix a bug causing large moving colliders to miss some collisions after some time.
- Fix invalid forces generated by contacts with position-based kinematic bodies.
- Fix a bug where two colliders without parent would not have their collision computed even if the
  appropriate flags were set.

## v0.11.0
Check out the user-guide for the JS/Typescript bindings for rapier. It has been fully rewritten and is now exhaustive!
Check it out on [rapier.rs](https://www.rapier.rs/docs/user_guides/javascript/getting_started_js)

### Added
- Joint limits are now implemented for all joints that can support them (prismatic, revolute, and ball joints).

### Modified
- Switch to  `nalgebra 0.29`.

### Fixed
- Fix the build of Rapier when targeting emscripten.

## v0.10.1
### Added
- Add `Collider::set_translation_wrt_parent` to change the translation of a collider wrt. its parent rigid-body.
- Add `Collider::set_rotation_wrt_parent` to change the translation of a collider wrt. its parent rigid-body.


## v0.10.0
### Added
- Implement `Clone` for `IslandManager`.

### Modified
- `JointSet::insert` no longer takes the rigid-body set in its arguments.
- Modify the testbed's plugin system to let plugins interact with the rendering.
- Implement `PartialEq` for most collider and rigid-body components.

## v0.9.2
### Added
- Make the method JointSet::remove_joints_attached_to_rigid_body public so that it can can be called externally for
  letting component-based Rapier integration call it to cleanup joints after a rigid-body removal.

### Fixed
- Fix a panic that could happen when the same collider is listed twice in the removed_colliders array.


## v0.9.1
### Added
- Add `rapier::prelude::nalgebra` so that the `vector!` and `point!` macros work out-of-the-box after importing
  the prelude: `use rapier::prelude::*`

## v0.9.0
The user-guide has been fully rewritten and is now exhaustive! Check it out on [rapier.rs](https://rapier.rs/)

### Added
- A prelude has been added in order to simplify the most common imports. For example: `use rapier3d::prelude::*`
- Add `RigidBody::set_translation` and `RigidBody.translation()`.
- Add `RigidBody::set_rotation` and `RigidBody.rotation()`.
- Add `RigidBody::set_next_translation` for setting the next translation of a position-based kinematic body.
- Add `RigidBody::set_next_rotation` for setting the next rotation of a position-based kinematic body.
- Add kinematic bodies controlled at the velocity level: use `RigidBodyBuilder::new_kinematic_velocity_based` or
  `RigidBodyType::KinematicVelocityBased`.
- Add the cargo feature `debug-disable-legitimate-fe-exceptions` that can be enabled for debugging purpose. This will
  disable floating point exceptions whenever they happen at places where we do expect them to happen (for example
  some SIMD code do generate NaNs which are filtered out by lane-wise selection).

### Modified
The use of `RigidBodySet, ColliderSet, RigidBody, Collider` is no longer mandatory. Rigid-bodies and colliders have
been split into multiple components that can be stored in a user-defined set. This is useful for integrating Rapier
with other engines (for example this allows us to use Bevy's Query as our rigid-body/collider sets).

The `RigidBodySet, ColliderSet, RigidBody, Collider` are still the best option for whoever doesn't want to
provide their own component sets.

#### Rigid-bodies
- Renamed `BodyStatus` to `RigidBodyType`.
- `RigidBodyBuilder::translation` now takes a vector instead of individual components.
- `RigidBodyBuilder::linvel` now takes a vector instead of individual components.
- The `RigidBodyBuilder::new_kinematic` has be replaced by the `RigidBodyBuilder::new_kinematic_position_based` and
  `RigidBodyBuilder::new_kinematic_velocity_based` constructors.
- The `RigidBodyType::Kinematic` variant has been replaced by two variants: `RigidBodyType::KinematicVelocityBased` and
  `RigidBodyType::KinematicPositionBased`.
  
#### Colliders
- `Colliderbuilder::translation` now takes a vector instead of individual components.
- The way `PhysicsHooks` are enabled changed. Now, a physics hooks is executed if any of the two
  colliders involved in the contact/intersection pair contains the related `PhysicsHooksFlag`.
  These flags are configured on each collider with `ColliderBuilder::active_hooks`. As a result,
  there is no `PhysicsHooks::active_hooks` method any more.
- All events are now disabled for all colliders by default. Enable events for specific colliders by setting its
  `active_events` bit mask to `ActiveEvents::CONTACT_EVENTS` and/or `ActiveEvents::PROXIMITY_EVENTS`.
- Add a simpler way of enabling collision-detection between colliders attached to two non-dynamic rigid-bodies: see
  `ColliderBuilder::active_collision_types`.
- The `InteractionGroups` is now a structures with two `u32` integers: one integers for the groups
  membership and one for the group filter mask. (Before, both were only 16-bits wide, and were
  packed into a single `u32`).
- Before, sensor colliders had a default density set to 0.0 whereas non-sensor colliders had a
  default density of 1.0. This has been unified by setting the default density to 1.0 for both
  sensor and non-sensor colliders.
- Colliders are no longer required to be attached to a rigid-body. Therefore, `ColliderSet::insert`
  only takes the collider as argument now. In order to attach the collider to a rigid-body,
  (i.e., the old behavior of `ColliderSet::insert`), use `ColliderSet::insert_with_parent`.
- Fixed a bug where collision groups were ignored by CCD.

#### Joints
- The fields `FixedJoint::local_anchor1` and `FixedJoint::local_anchor2` have been renamed to
  `FixedJoint::local_frame1` and `FixedJoint::local_frame2`.
  
#### Pipelines and others
- The field `ContactPair::pair` (which contained two collider handles) has been replaced by two
  fields: `ContactPair::collider1` and `ContactPair::collider2`.
- The list of active dynamic bodies is now retrieved with `IslandManager::active_dynamic_bodies`
  instead of `RigidBodySet::iter_active_dynamic`.
- The list of active kinematic bodies is now retrieved with `IslandManager::active_kinematic_bodies`
  instead of `RigidBodySet::iter_active_kinematic`.
- `NarrowPhase::contacts_with` now returns an `impl Iterator<Item = &ContactPair>` instead of 
  an `Option<impl Iterator<Item = (ColliderHandle, ColliderHandle, &ContactPair)>>`. The colliders
  handles can be read from the contact-pair itself.
- `NarrowPhase::intersections_with` now returns an iterator directly instead of an `Option<impl Iterator>`.
- Rename `PhysicsHooksFlags` to `ActiveHooks`.
- Add the contact pair as an argument to `EventHandler::handle_contact_event`


## v0.8.0
### Modified
- Switch to nalgebra 0.26.

## v0.7.2
### Added
- Implement `Serialize` and `Deserialize` for the `CCDSolver`.

### Fixed
- Fix a crash that could happen after adding and then removing a collider right away,
before stepping the simulation.

## v0.7.1
### Fixed
- Fixed a bug in the broad-phase that could cause non-determinism after snapshot restoration.

## v0.7.0
### Added
- Add the support of **Continuous Collision Detection** (CCD) to
make sure that some fast-moving objects (chosen by the user) don't miss any contacts.
This is done by using motion-clamping, i.e., each fast-moving rigid-body with CCD enabled will
be stopped at the time where their first contact happen. This will result in some "time loss" for that
rigid-body. This loss of time can be reduced by increasing the maximum number  of CCD substeps executed
(the default being 1).
- Add the support of **collider modification**. Now, most of the characteristics of a collider can be
modified after the collider has been created.
- We now use an **implicit friction cone** for handling friction, instead of a pyramidal approximation
of the friction cone. This means that friction will now  behave in a more isotropic way (i.e. more realistic
Coulomb friction).
- Add the support of **custom filters** for the `QueryPipeline`. So far, interaction groups (bit masks)
had to be used to exclude from colliders from a query made with the `QueryPipeline`. Now it is also
possible to provide a custom closures to apply arbitrary user-defined filters.
- It is now possible to solve penetrations using the velocity solver instead of (or alongside) the
position solver (this is disabled by default, set `IntegrationParameters::velocity_based_erp` to
  a value `> 0.0` to enable.).

Added the methods:
- `ColliderBuilder::halfspace` to create a collider with an unbounded plane shape.
- `Collider::shape_mut` to get a mutable reference to its shape.
- `Collider::set_shape`, `::set_restitution_combine_rule`, `::set_position_wrt_parent`, `::set_collision_groups`
`::set_solver_groups` to change various properties of a collider after its creation.
- `RigidBodyBuilder::ccd_enabled` to enable CCD for a rigid-body.

### Modified
- The `target_dist` argument of `QueryPipeline::cast_shape` was removed.
- `RigidBodyBuilder::mass_properties` has been deprecated, replaced by `::additional_mass_properties`.
- `RigidBodyBuilder::mass` has been deprecated, replaced by `::additional_mass`.
- `RigidBodyBuilder::principal_angular_inertia` has been deprecated, replaced by `::additional_principal_angular_inertia`.
- The field `SolveContact::data` has been replaced by the fields `SolverContact::warmstart_impulse`, 
  `SolverContact::warmstart_tangent_impulse`, and `SolverContact::prev_rhs`.
- All the fields of `IntegrationParameters` that we don't use have been removed.
- `NarrowPhase::maintain` has been renamed to `NarrowPhase::handle_user_changes`.
- `BroadPhase::maintain` has been removed. Use ` BroadPhase::update` directly.

### Fixed
- The Broad-Phase algorithm has been completely reworked to support large colliders properly (until now
they could result in very large memory and CPU usage).

## v0.6.1
### Fixed
- Fix a determinism problem that may happen after snapshot restoration, if a rigid-body is sleeping at
  the time the snapshot is taken.

## v0.6.0
### Added
- The support of **dominance groups** have been added. Each rigid-body is part of a dominance group in [-127; 127]
(the default is 0). If two rigid-body are in contact, the one with the highest dominance will act as if it has
an infinite mass, making it immune to the forces the other body would apply on it. See [#122](https://github.com/dimforge/rapier/pull/122)
for further details.
- The support for **contact modification** has been added. This can bee used to simulate conveyor belts,
one-way platforms and other non-physical effects. It can also be used to simulate materials with
variable friction and restitution coefficient on a single collider. See [#120](https://github.com/dimforge/rapier/pull/120)
for further details.
- The support for **joint motors** have been added. This can be used to control the position and/or
velocity of a joint based on a spring-like equation. See [#119](https://github.com/dimforge/rapier/pull/119)
for further details.

### Removed
- The `ContactPairFilter` and `IntersectionPairFilter` traits have been removed. They are both
  combined in a single new trait: `PhysicsHooks`.

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
