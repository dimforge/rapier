# Changelog

## Unreleased

### Modified

- Update from rapier `0.33.0-alpha` to rapier `0.35.0-glamx0.2`.
  See [rapier's changelog](https://github.com/dimforge/rapier/blob/master/CHANGELOG.md) for details.
  - Removed the `simd-stable` and `simd-nightly` features: SIMD is now always enabled in rapier.
    The new `simd8` feature widens the solver’s SIMD from 4 to 8 lanes (f32 only, incompatible
    with `enhanced-determinism`).
  - `SolverContactView::{friction, restitution}` now read from the contact manifold data (rapier
    stores them per-manifold), and `SolverContactView::point` is reconstructed from the body-local
    contact anchors; the public API is preserved.
  - Custom event handlers assigned to `RapierContextSimulation` must now be `Send + Sync`.
- Known issue: the `enhanced-determinism` feature currently fails to compile with bevy, because
  parry enables `glam/scalar-math` which removes the serde impls of `glam::BVec3A`/`BVec4A` that
  `bevy_reflect` requires unconditionally.

## v0.35.0 (12 July 2026)

### Modified

- Update to bevy 0.19.

## v0.34.0 (14 May 2026)

### Added

- New `change_contexts3` 3D example demonstrating how to move an entity between physics contexts.
  [#684](https://github.com/dimforge/bevy_rapier/pull/684)

### Modified

- Update from rapier `0.31` to rapier `0.32`. [#692](https://github.com/dimforge/bevy_rapier/pull/692)
  See [rapier's changelog](https://github.com/dimforge/rapier/blob/master/CHANGELOG.md) for details.
  - Most `Collider`, joint, and query-pipeline APIs now take `glam` vectors/rotations directly instead of
    nalgebra `Point`s/`Isometry`s. For example, `Collider::capsule`, `Collider::segment`,
    `Collider::triangle`, `Collider::voxels`, `RapierQueryPipeline::cast_ray`, `project_point`,
    `intersect_shape`, `cast_shape`, etc. no longer require `.into()` conversions on input vectors.
  - `TransformInterpolation::{start, end}` are now `Option<Pose>` instead of `Option<Isometry<f32>>`.
  - `iso_to_transform` now takes a `&Pose` instead of `&Isometry<Real>`.
  - `SolverContactView::is_new` now compares an internal float; the public boolean API is preserved.
  - `ContactPairView::has_any_active_contact` is now a method call on the underlying contact pair.
- Renamed `Velocity` fields: `linvel` → `linear` and `angvel` → `angular`.
  [#690](https://github.com/dimforge/bevy_rapier/pull/690)

### Fix

- Fix context swapping failing when physics runs in a schedule other than `PostUpdate`: a stray run
  of `sync_removals` was removing handles that had just been added.
  [#684](https://github.com/dimforge/bevy_rapier/pull/684)

## v0.33.0 (06 March 2026)

### Modified

- Update Bevy to `0.18`. [#686](https://github.com/dimforge/bevy_rapier/pull/686)
  - Examples now require the `bevy_gizmos_render` feature.
  - The testbed examples moved their UI systems to the new `EguiPrimaryContextPass` schedule.

## v0.32.0 (24 February 2026)

### Added

- Reflection support for `SpringCoefficients` (used by the new contact softness parameters) and, in 3D,
  for `FrictionModel` (`Simplified` / `Coulomb`). [#680](https://github.com/dimforge/bevy_rapier/pull/680)
- New `voxels2_no_collider` 2D example demonstrating `Voxels` shapes without a collider.
  [#680](https://github.com/dimforge/bevy_rapier/pull/680)

### Modified

- Update Bevy to `0.17.3`. [#680](https://github.com/dimforge/bevy_rapier/pull/680)
  - Migrated from Bevy's `Event`/`Events`/`EventReader`/`EventWriter` to the new
    `Message`/`Messages`/`MessageReader`/`MessageWriter` API. `CollisionEvent`, `ContactForceEvent`,
    and `MassModifiedEvent` now derive `Message` instead of `Event`.
  - Renamed `TransformSystem::TransformPropagate` usages to `TransformSystems::Propagate`.
  - `PickSet` → `PickingSystems` for the picking backend.
- Update from rapier `0.27` to rapier `0.31`. [#680](https://github.com/dimforge/bevy_rapier/pull/680)
  See [rapier's changelog](https://github.com/dimforge/rapier/blob/master/CHANGELOG.md) for details.
  - `IntegrationParametersWrapper` reflection now exposes `contact_softness` (a `SpringCoefficients`),
    `warmstart_coefficient`, and `length_unit`, replacing the previous `contact_damping_ratio` and
    `contact_natural_frequency` fields.

## v0.31.0 (04 August 2025)

### Added

- Expose `RapierBevyComponentApply`, to help with creating your own schedules when you set `default_system_setup` to `false`.
- Add `set_local_axis1` and `set_local_axis2` to `RevoluteJoint` and `RevoluteJointBuilder`. [#666](https://github.com/dimforge/bevy_rapier/pull/666)

### Modified

- Update from rapier `0.25` to rapier `0.27`,
  see [rapier's changelog](https://github.com/dimforge/rapier/blob/master/CHANGELOG.md).
  - `RapierQueryPipeline` is no longer a component.
    - Migration: Use `RapierContext` or retrieve the needed components to pass to `RapierQueryPipeline::new_scoped` and make your logic in a scoped function. This function allows capturing and returning information.
  - a new `QueryPipelineMut`  to provide the same API as rapier. It's currently used for the character controller.

### Fix

- Fix scale being applied with a frame delay. [#659](https://github.com/dimforge/bevy_rapier/pull/659)

## v0.30.0 (15 May 2025)

### Added

- Added a serialization `serialization2` example for `bevy_rapier2d`.
- Added reflection for `Default` in addition to `Component`. [#649](https://github.com/dimforge/bevy_rapier/pull/649)

### Modified

- Update Bevy to `0.16`.
- Update from rapier `0.23` to rapier `0.25`,
  see [rapier's changelog](https://github.com/dimforge/rapier/blob/master/CHANGELOG.md).
  - Notably, support and examples for parry's new `Voxels` shape have been added.
- `RapierContextInitialization::InitializeDefaultRapierContext` now has more fields for better control over default physics context.
- `ContactPairView::collider1` and `ContactPairView::collider2` now return an `Option`.

### Fix

- Fix position being incorrect when a rigidbody bevy entity has a scaled parent. [#646](https://github.com/dimforge/bevy_rapier/pull/646)

## v0.29.0 (18 February 2025)

### Added

- Added optional feature `picking-backend` to support bevy_picking.
  - See `picking_backend` module documentation for more details.
- Added `geometry::to_bevy_mesh` module behind the feature `to-bevy-mesh` to help with converting parry shapes into bevy meshes (#628).
  - Lines, round and custom shapes are not implemented.

### Modified

- Update from rapier `0.22` to rapier `0.23`,
  see [rapier's changelog](https://github.com/dimforge/rapier/blob/master/CHANGELOG.md).
- `Collider::trimesh` and `Collider::trimesh_with_flags` now return a `Result`.
- Under the feature `async-collider`, The `RapierPhysicsPlugin` now adds
  `AssetPlugin`, `MeshPlugin` and `ScenePlugin` if these were not added, to circumvent a runtime crash
  over missing required resources.
- `RapierPhysicsPlugin` can be customized through `with_physics_sets_systems` to opt out of default systems from `PhysicsSet`.
- `RapierContext` has been split in multiple `Component`s:
  - `RapierContextColliders`
  - `RapierContextJoints`
  - `RapierContextSimulation`
  - `RapierRigidBodySet`
- Renamed `DefaultReadRapierContext` to `ReadRapierContext` and `DefaultWriteRapierContext` to `WriteRapierContext`.
  They have a new `bevy::QueryFilter` type parameter, defaulting to `With<DefaultRapierContext>`.

## v0.28.0 (09 December 2024)

### Modified

- Update from rapier `0.21` to rapier `0.22`,
  see [rapier's changelog](https://github.com/dimforge/rapier/blob/master/CHANGELOG.md).
- Update bevy to 0.15.
- `RapierContext`, `RapierConfiguration` and `SimulationToRenderTime` are now a `Component` instead of resources.
  - Rapier now supports multiple independent physics worlds, see example `multi_world3` for usage details.
  - Migration guide:
    - `ResMut<mut RapierContext>` -> `WriteDefaultRapierContext`
    - `Res<RapierContext>` -> `ReadDefaultRapierContext`
    - Access to `RapierConfiguration` and `SimulationToRenderTime` should query for it
on the responsible entity owning the `RenderContext`.
  - If you are building a library on top of `bevy_rapier` and would want to support multiple independent physics worlds too,
you can check out the details of [#545](https://github.com/dimforge/bevy_rapier/pull/545)
to get more context and information.
- `colliders_with_aabb_intersecting_aabb` now takes `bevy::math::bounding::Aabb3d` (or `[..]::Aabb2d` in 2D) as parameter.
  - it is now accessible with `headless` feature enabled.

### Fix

- Fix a crash when using `TimestepMode::Interpolated` and removing colliders
during a frame which would not run a simulation step.

### Added

- Added a `TriMeshFlags` parameter for `ComputedColliderShape`,
its default value is `TriMeshFlags::MERGE_DUPLICATE_VERTICES`,
which was its hardcoded behaviour.
- Added a way to configure which colliders should be debug rendered: `global` parameter for both 
  `RapierDebugColliderPlugin` and `DebugRenderContext`, as well as individual collider setup via
  a `ColliderDebug` component.

## v0.27.0 (07 July 2024)

**This is an update from rapier 0.19 to Rapier 0.21 which includes several stability improvements
and new features. Please have a look at the
[0.20 and 0.21 changelogs](https://github.com/dimforge/rapier/blob/master/CHANGELOG.md) of Rapier.**

### Modified

- Update from rapier `0.19` to rapier `0.21`.
- Update to nalgebra `0.33`.
- Update to bevy `0.14`.
- Renamed `has_any_active_contacts` to `has_any_active_contact` for better consistency with rapier.
- `ColliderDebugColor`'s property is now a `bevy::color::Hsla`.
- `ImpulseJoint::data` and `MultibodyJoint::data` are now a more detailed enum `TypedJoint` instead of a `GenericJoint`.
You can still access its inner `GenericJoint` with `.as_ref()` or `as_mut()`.
- `data` fields from all joints (`FixedJoint`, …) are now public, and their getters removed.

### Added

- Derive `Debug` for `LockedAxes`.
- Expose `is_sliding_down_slope` to both `MoveShapeOutput` and `KinematicCharacterControllerOutput`.
- Added a First Person Shooter `character_controller` example for `bevy_rapier3d`.
- Added serialization support for `CollisionGroups`, `SolverGroups`, `ContactForceEventThreshold`, `ContactSkin`.
- Added `RapierContext::context.impulse_revolute_joint_angle` to compute the angle along a revolute joint’s principal axis.

### Fix

- Fix rigidbodies never going to sleep when a scale was applied to their `Transform`.
- Fix losing information about hit details when converting from `ShapeCastHit` in parry to `ShapeCastHit` in bevy_rapier

## v0.26.0 (05 May 2024)

**This is an update to Rapier 0.19 which includes several stability improvements
and character-controller fix. Please have a look at the
[0.19 changelog](https://github.com/dimforge/rapier/blob/master/CHANGELOG.md) of Rapier.**

### Modified

- Renamed `Toi/ToiDetails` to `ShapeCastHit/ShapeCastHitDetails`.
- Switch to rapier’s built-in `length_unit` instead of explicitly scaling shapes with `physics_scale`.
- Linear shape-casting functions now take a `ShapeCastOptions` parameter that describes how the shape-cast should
  behave on special-cases (like toi == 0).
- Internal edge correction is now opt-in with the `TriMeshFlags::FIX_INTERNAL_EDGES` and
  `HeightFieldFlags::FIX_INTERNAL_EDGES` flags.
- Rename `RayIntersection::toi` to `RayIntersection::time_of_impact`.

### Fix

- Fix character controller occasionally getting stuck against vertical walls.

### Added

- Add the `SoftCcd` (for rigid-bodies) and `ContactSkin` (for colliders) components. See
  [rapier#625](https://github.com/dimforge/rapier/pull/625) for details on the features they enable.

## v0.25.0 (19 Feb. 2024)

### Modified

- Update to bevy `0.13`.

## v0.24.0 (27 Jan. 2024)

The main highlight of this release is the implementation of a new non-linear constraints solver for better stability
and increased convergence rates. See [#579](https://github.com/dimforge/rapier/pull/579) for additional information.

In order to adjust the number of iterations of the new solver, simply
adjust `IntegrationParameters::num_solver_iterations`.
If recovering the old solver behavior is useful to you, call `IntegrationParameters::switch_to_standard_pgs_solver()`.

It is now possible to specify some additional solver iteration for specific rigid-bodies (and everything interacting
with it directly or indirectly through contacts and joints) by adding the `AdditionalSolverIterations` component to the
same entity as the rigid-body. This allows for higher-accuracy on subsets of the physics scene without affecting
performance of the other parts of the simulation.

### Fix

- Fix bug causing angular joint limits and motor to sometimes only take into account half of the angles specified by the
  user.
- Fix bug where collisions would not be re-computed after a collider was re-enabled.

### Added

- Add a `SpringJoint` and `SpringJointBuilder` for simulating springs with customizable stiffness and damping
  coefficients.
- Fix incorrect update of angular degrees-of-freedoms on spherical multibody joints.
- Fix debug-renderer showing moved kinematic rigid-bodies only at their initial position.

### Modified

- Rename `RapierContext::contacts_with` to `RapierContext::contact_pairs_with`.
- Rename `RapierContext::intersections_with` to `RapierContext::intersection_pairs_with`.
- Collisions between the character controller and sensors are now disabled by default.

## 0.23.0

### Modified

- Update to Bevy 0.12

### Added

- `ColliderView::as_typed_shape` and `::to_shared_shape` to convert a `ColliderView` to a parry’s
  `TypedShape` or `SharedShape`. The `From` trait has also been implemented accordingly.
- Implement `Copy` for `ColliderView` and all the other non-mut shape views.
- Add `RapierContext::rigid_body_colliders` to retrieve all collider entities attached to this rigid-body.
- Add `RapierPhysicsPlugin::in_fixed_schedule`/`::in_schedude` to add rapier’s systems to a fixed/custom
  schedule.
- Re-export `JointAxesMask`, `JointAxis`, `MotorModel`.
- Implement `Deref` for `ReadMassProperties`.
- Expose the `stop_at_penetration` parameter of shape-casting, to enable or ignore overlaps at the initial position
  of the shape.

### Fix

- Fix `RapierContext::integration_parameters::dt` not being updated on non-fixed timestep modes.
- Fix debug-renderer lagging one frame behind.
- Fix Collider `Transform` rotation change not being taken into account by the physics engine.
- Fix automatic update of `ReadMassProperties`.

## 0.22.0 (10 July 2023)

### Modified

- Update to Bevy 0.11.
- Disabled rigid-bodies are no longer synchronized with the rapier backend.
- Switch to bevy’s gizmo system for the debug-renderer. This removes the vendored debug lines plugin.

### Added

- Add a joint for simulating ropes: the `RopeJoint`.
- Add `Velocity::linear_velocity_at_point` to calculate the linear velocity at the given world-space point.
- Add the `ComputedColliderShape::ConvexHull` variant to automatically calculate the convex-hull of an imported mesh.
- Implement `Reflect` for the debug-renderer.

### Fix

- Fix broken interpolation for rigid-bodies with the `TransformInterpolation` component.
- Fix compilation when `bevy_rapier` is being used with headless bevy.
- Improved performance of the writeback system by not iterating on non-rigid-body entities.
- Fix typo by renaming `CuboidViewMut::sed_half_extents` to `set_half_extents`.
- Properly scale parented collider’s offset based on changes on its `ColliderScale`.

## 0.21.0  (07 March 2023)

### Modified

- Update to Bevy 0.10.
- The `PhysicsHooksWithQuery` trait has been renamed to by the `BevyPhysicsHooks`.
- Bevy resources and queries accessed by the physics hook are now specified by the implementer of `BevyPhysicsHooks`
  which must derive Bevy’s `SystemParam` trait. This implies that the physics hook’s `filter_contact_pair` (and
  all its other methods) no longer take the Bevy `Query` as argument. Queries and resources are accessed through
  `self`.
- Rename `PhysicsStages` to `PhysicsSet`.

## 0.20.0 (15 Jan. 2023)

### Added

- Add the `RigidBodyDisabled` and `ColliderDisabled` component that can be inserted to disable a rigid-body
  or collider without removing it from the scene.

### Fix

- Fix spawn position of colliders without rigid bodies.
- Fix overriding enabled flag in debug render.

### Modified

- Make debug-rendering enabled by default when inserting the `RapierDebugRenderPlugin` plugin with its default
  configuration.
- The `debug-render` feature has been replaced by two features: `debug-render-2d` and `debug-render-3d`. For example,
  using `debug-render-2d` with `bevy_rapier3d`, the debug-render will work with 2D cameras (useful, e.g., for top-down
  games
  with 3D graphics).
- In order to facilitate the use of `bevy_rapier` in headless mode, the `AsyncCollider` and `AsyncSceneCollider`
  components were moved behind the `async-collider` feature (enabled by default). Disabling that feature will
  make `bevy_rapier` work even with the `MinimalPlugins` inserted instead of the `DefaultPlugins`.
- Corrected an API inconsistency where `bevy_rapier` components would sometimes require an `InteracitonGroup` type
  defined in
  `rapier`. It has been replaced by the `CollisionGroup` type (defined in `bevy_rapier`).
- `Velocity::zero,linear,angular` are now const-fn.

## 0.19.0 (18 Nov. 2022)

### Modified

- Update to Bevy 0.9

## 0.18.0 (30 Oct. 2022)

### Added

- Add the accessor `RapierContext::physics_scale()` to read the physics scale
  that was set when initializing the plugin.
- Add `RapierConfiguration::force_update_from_transform_changes` to force the transform
  updates even if it is equal to the transform that was previously set. Useful for
  rollback in networked applications described in [#261](https://github.com/dimforge/bevy_rapier/pull/261).
- Add `Collider::trimesh_with_flags` to create a triangle mesh collider with custom pre-processing
  flags.

### Fix

- Reset the `ExternalImpulse` component after each step automatically.
- Fix `transform_to_iso` to preserve identical rotations instead of
  converting to an intermediate axis-angle representation.
- Fix **internal edges** of 3D triangle meshes or 3D heightfields generating invalid contacts
  preventing balls from moving straight. Be sure to set the triangle mesh flag
  `TriMeshFlags::MERGE_DUPLICATE_VERTICES` when creating the collider if your mesh have duplicated
  vertices.

### Modified

- Rename `AABB` to `Aabb` to comply with Rust’s style guide.

## 0.17.0 (02 Oct. 2022)

### Added

- Add a **kinematic character controller** implementation. This feature is accessible in two different ways:
    1. The first approach is to insert the `KinematicCharacterController` component to an entity. If the
       `KinematicCharacterController::custom_shape` field is set, then this shape is used for the character control.
       If this field is `None` then the `Collider` attached to the same entity as the character controller is used.
       The character controller will be automatically updated when the `KinematicCharacterController::movement` is set.
       The result position is written to the `Transform` of the character controller’s entity.
    2. The second, lower level, approach, is to call `RapierContext::move_shape` to compute the possible movement
       of a shape, taking obstacle and sliding into account.
- Add implementations of `Add`, `AddAssign`, `Sub`, `SubAssign` to `ExternalForce` and `ExternalImpulse`.
- Add `ExternalForce::at_point` and `ExternalImpulse::at_point` to apply a force/impulse at a specific point
  of a rigid-body.

### Fix

- Fix shapes quickly switching between scaled and non-scaled versions due to rounding errors in the scaling extraction
  from bevy’s global affine transform.

## 0.16.2 (23 August 2022)

### Added

- Implement `Debug` for `Collider` and `ColliderView`.
- Add the missing `ActiveEvent::CONTACT_FORCE_EVENTS` to enable contact force events on a collider.

## 0.16.1 (19 August 2022)

### Fixed

- Fix crash of the 2D debug-render on certain platforms (including Metal/MacOS).
- Fix bug where collision events and contact force events were not cleared automatically.
- Implement `Reflect` for `AsyncCollider`.

## 0.16.0 (31 July 2022)

### Modified

- Switch to Bevy 0.8.

## 0.15.0 (10 July 2022)

### Fixed

- Fix unpredictable broad-phase panic when using small colliders in the simulation.
- Fix collision events being incorrectly generated for any shape that produces multiple
  contact manifolds (like triangle meshes).
- Fix transform hierarchies not being properly taken into account for colliders with a
  parent rigid-body.
- Fix force and impulse application when the `ExternalImpulse` or `ExternalForce` components were added
  at the same time as the rigid-body creation.
- Fix sleeping threshold application when these thresholds are set at the same time as the rigid-body
  creation.

### Added

- Add the `ColliderMassProperties::Mass` variant to let the user specify a collider’s mass directly (instead of its
  density).
  As a result the collider’s angular inertia tensor will be automatically be computed based on this mass and its shape.
- Add the `ContactForceEvent` event. It can be read by a bevy system with the `EventReader<ContactForceEvent>`. This
  event is useful to read contact forces. A `ContactForceEvent` is generated whenever the sum of the magnitudes of the
  forces applied by contacts between two colliders exceeds the value specified by the `ContactForceEventThreshold`
  component.
- Add the `QueryFilter` struct that is now used by all the scene queries instead of the `CollisionGroups` and
  `Fn(Entity) -> bool` closure. This `QueryFilter` provides easy access to most common filtering strategies
  (e.g. dynamic bodies only, excluding one particular entity, etc.) for scene queries.
- Added some missing serialization of joints.
- Implement `Default` for `Collider`. It defaults to a `Ball` with radius 0.5.
- Added a `contacts_enabled` flag to all the joints. If this flag is set to `false` for a joint, no contact will be
  computed between two colliders attached to rigid-bodies liked by that joint.

### Modified

- The `MassProperties` struct is no longer a `Component`. A common mistake was to assume that `MassProperties` could
  be used to initialize the mass/angular inertia tensor of a rigid-body. It is not the case. Instead, the user should
  use the `AdditionalMassProperties` component. The `ReadMassProperties` component has been added to read the mass
  properties of a rigid-body.
- The `Sensor` component is now a marker component: if it exists the related collider is a sensor, otherwise it is
  a solid collider.

## 0.14.1 (01 June 2022)

### Fixed

- Add the missing `init_async_scene_colliders` to the list of the plugin systems.

## 0.14.0 (31 May 2022)

### Added

- Add the `AsyncSceneCollider` component to generate collision for scene meshes similar to `AsyncCollider`.
- Add calls to `App::register_type` for the types implementing `Reflect`.

### Modified

- `Collider::bevy_mesh`, `Collider::bevy_mesh_convex_decomposition`
  and `Collider::bevy_mesh_convex_decomposition_with_params` was replaced with single `Collider::from_bevy_mesh`
  function which accepts `ComputedColliderShape`.
- `AsyncCollider` is now a struct which contains a mesh handle and `ComputedColliderShape`.
- The physics systems are now running after `CoreStage::Update` but before `CoreStage::PostUpdate`.
- The collider and rigid-body positions are read from the `GlobalTransform` instead of `Transform` (the
  transforms modified by Rapier are written back to the `Transform` component). It is therefore important
  to insert both a `Transform` and `GlobalTransform` component (or the `TransformBundle` bundle).
- It is now possible to prevent the plugin from registering its system thanks
  to `RapierPhysicsPlugin::with_default_system_setup(false)`.
  If that’s the case, the `RapierPhysicsPlugin::get_systems` method can be called to retrieve the relevant `SystemSet`
  that can be added to your own stages in order to apply your own scheduling.

### Fixed

- Fixed issues where contact regularization (using compliance) would result in tunnelling despite tunnelling
  being enabled.
- Don’t overwrite the user’s `RapierConfiguration` if one already exists before initializing the plugin.

## 0.13.2 (5 May 2022)

### Modified

- The `TimestepMode` and `SimulationToRenderTime` structures are now public.

### Fixed

- Fixed colors rendered by the debug-renderer.
- Fix issue where the debug-renderer would sometimes render lines behind the user’s meshes/sprites.

## 0.13.1 (1 May 2022)

### Added

- Add the `CollidingEntities` components which tracks the set of entities colliding
  with a given entity.
- Add constructors `Velocity::linear`, `Velocity::angular`, `Ccd::enabled()`, `Ccd::disabled()`,
  `Dominance::group`, `Friction::coefficient`, `Restitution::coefficient`, `CollisionGroups::new`,
  `SolverGroups::new`.
- Add `RapierContext::collider_parent` that returns the entity containing the parent `RigidBody`
  of a collider.

### Modified

- Switched to linear ordering to order our systems.
- Make the `plugin::systems` module public.

## 0.13.0 (30 Apr. 2022)

This is a **complete rewrite of the plugin** (mostly likely the last large redesign this plugin
will be subject to). It switches to `rapier` 0.12 and `bevy` 0.7. The focus of this rewrite was
to significantly improve ergonomics while simplifying the codebase and adding new features.

Refer to [#138](https://github.com/dimforge/bevy_rapier/pull/138) for extensive details
on this change. See the folders `bevy_rapier2d/examples` and `bevy_rapier3d/examples`
for some examples.

## 0.12.0

### Modified

- Switch to `rapier` 0.12.0-alpha.0 and `nalgebra` 0.30.
- Switch to `bevy` 0.6.
- All the Rapier components have been wrapped into wrapper types (for example `ColliderPosition`
  has been wrapped into `ColliderPositionComponent`). These wrapper types are the ones that need
  to be used as bevy components. To convert a Rapier component to it’s corresponding Bevy component,
  simply use `.into()`. See the examples in `bevy_rapier2d/examples` and `bevy_rapier3d/examples`
  for details.

## 0.11.0

### Modified

- Switch to `Rapier` 0.11 and `nalgebra` 0.29.
- Add labels to each system from `bevy-rapier`.

### Fixed

- Fix panics when despawning joints or colliders.
- Fix a panic where adding a collider.
- Don’t let the plugin overwrite the user’s `PhysicsHooksWithQueryObject` if it was already
  present before inserting the plugin.

## 0.10.2

### Fixed

- Fix build when targeting WASM.

## 0.10.1

### Fixed

- Fix joint removal when despawning its entity.

## 0.10.0

A new, exhaustive, user-guide for bevy_rapier has been uploaded to rapier.rs.

This version is a complete rewrite of the plugin. Rigid-bodies and
colliders are now split into components that can be queried like any other
components. They are created by inserting a `RigidBodyBundle` and/or a `ColliderBundle`.

In addition, the `Entity` type and `ColliderHandle/RigidBodyHandle` type can now be
converted directly using `entity.handle()` or `handle.entity()`.

Finally, there is now a prelude: `use bevy_rapier2d::prelude::*`.

## 0.9.0

### Added

- The `ColliderDebugRender` component must be added to an entity
  containing a collider shape in order to render it.

## 0.8.0

### Changed

- Use the version 0.5.0 of Rapier.

### Added

- The debug rendering of 3D trimesh now work.
- The debug rendering won't crash any more if a not-yet-supported shape
  is used. It will silently ignore the shape instead.
- The crate has now a `render` feature that allows building it without any
  rendering support (avoiding some dependencies that may not compile when
  targeting WASM).

## 0.7.0

### Changed

- Use the version 0.4.0 of Bevy.

## 0.6.2

### Changed

- Fix the rendering of colliders attached to an entity children of another
  entity containing the rigid-body it is attached to.

## 0.6.1

### Changed

- The adaptive change of number of timesteps executed during each render loop
  (introduced in the version 0.4.0 with position interpolation) is
  now disabled by default. It needs to be enabled explicitly by setting
  `RapierConfiguration.time_dependent_number_of_timesteps` to `true`.

## 0.6.0

### Added

- It is now possible to attach multiple colliders to a single
  rigid-body by using Bevy hierarchy: an entity contains
  the `RigidBodyBuilder` whereas its children contain the `ColliderBuilder`.

### Changed

- We now use the latest version of Rapier: 0.4.0. See the
  [Rapier changelog](https://github.com/dimforge/rapier/blob/master/CHANGELOG.md#v040)
  for details. In particular, this includes the ability to lock the rotations of a rigid-body.

## 0.5.0

### Changed

- We now use the latest version of Bevy: 0.3.0

### Added

- Rigid-bodies, colliders, and joints, will automatically removed from the Rapier
  sets when their corresponding Bevy components are removed.

## 0.4.0

This release update the plugin to the latest version of Rapier, which itself
includes lots of new features. Refer to the Rapier changelogs for details.

### Added

- A `InteractionPairFilters` resource where you can your own filters for contact pair
  and proximity pair filtering. Before considering the use of a custom filter, consider
  using collision groups instead (as it is faster, but less versatile).

### Changed

- The stepping system now applies interpolation between two physics state at render time.
  Refer to [that issue](https://github.com/dimforge/bevy_rapier/pull/19) and the following
  blog post article for details: https://www.gafferongames.com/post/fix_your_timestep/

## 0.3.1

### Changed

- Rapier configuration
    - Replaced `Gravity` and `RapierPhysicsScale` resources with a unique `RapierConfiguration` resource.
    - Added an `physics_pipeline_active` attribute to `RapierConfiguration` allowing to pause the physic simulation.
    - Added a `query_pipeline_active` attribute to `RapierConfiguration` allowing to pause the query pipeline update.

