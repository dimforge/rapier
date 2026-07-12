## 0.19.3 (05 Nov. 2025)

- Significantly improve performances of `combineVoxelStates`.

### 0.19.2 (17 Oct. 2025)

- Fix bug where kinematic bodies would not wake up when setting its velocity.
- Fix bug where slow-moving kinematic bodies would fall asleep.
- Fix point-projection on voxels shapes.

### 0.19.1 (03 Oct. 2025)

### Modified

- Update to Rapier 0.30.0. The only change is a [switch to a sparse storage](https://github.com/dimforge/parry/pull/380)
  for the Voxels shapes. This allows support for orders of magnitudes larger maps without reaching the 4GB WASM memory
  limit.

### 0.19.0 (05 Sept. 2025)

### Modified

- Update to Rapier 0.29.0 which includes performance improvements for scenes involving a lot of contact constraints.
  See https://github.com/dimforge/rapier/pull/876 for details.
- Renamed the `RigidBody.invPrincipalInertiaSqrt` and `.effectiveWorldInvInertiaSqrt` methods to
  `RigidBody.invPrincipalInertia` and `.effectiveWorldInvInertia` (removed the `Sqrt` suffix). These methods will now
  return the actual inverse angular inertia matrix rather than its square root.
- Removed methods related to the legacy PGS solver: `World.numAdditionalFrictionIterations`,
  `switchToStandardPgsSolver`, `switchToSmallStepsPgsSolver`, `switchToSmallStepsPgsSolverWithoutWarmstart`.

### 0.18.2 (13 August 2025)

### Fixed

- Fix rollup configuration adding `types: "./rapier.d.ts"` to the export config.

### 0.18.1 (8 August 2025)

### Modified

- Update to Rapier 0.28.0 which includes performance improvements when CCD is active and when
  the user applies modification to a collider or rigid-body.

### Fix

- Another attempt to fix bundlerless module import with rapier-compat.

### 0.18.0 (24 July 2025)

### Added

- Add `World.timing*` functions to access the internal performances measurements if the internal
  profiler is enabled with `World.profilerEnabled = true`.
- Add `World.maxCcdSubsteps` to get/set the max number of CCD substeps run by the engine.

### Fix

- Fixed crash that would happen when removing colliders in a particular order (e.g. in the same order
  as their insertion).

### 0.18.0-beta.0 (12 July 2025)

#### Modified

- Update to Rapier 0.27.0-beta.1 which includes a fully reworked broad-phase tha supports scene queries.
  This implies a performance gain on large scenes by avoiding the need to re-build the underlying acceleration
  structure at each frame.
- Un-deprecate methods for reading shape properties (for example `collider.radius()`). It turned out that these
  methods are more convenient as they are guaranteed to always be in sync with rapier’s state on wasm.
- Add `collider.translationWrtParent()` and `collider.rotationWrtParent()` to get the collider’s translation/rotation
  relative to its parent rigid-body.

#### Fix

- rapier-compat top level javascript files extensions have been changed from `.cjs.js` and `.es.js` to `.cjs` and `mjs`
  respectively. This results in better compatibility with NPM.

### 0.17.3 (30 May 2025)

#### Fix

- The published package for 0.17.2 had a broken package.json. It is fixed on this release.

### 0.17.2 (30 May 2025)

#### Added

- Added the function `RAPIER.reserveMemory` to instruct the internal allocator to pre-allocate more memory in preparation
  for future operations. This typically called only once after initializing the WASM module.

### 0.17.1 (23 May 2025)

#### Added

- Added optional arguments to `World.debugRender(filterFlags, filterPredicate)` to prevent some colliders from being
  rendered.
- Added `Collider.combineVoxelStates` to ensure two adjacent voxels colliders don’t suffer from the internal edges
  problem, and `Collider.propagateVoxelChange` to maintain that coupling after modifications with `.setVoxel`.

### 0.17.0 (16 May 2025)

#### Fixed

- Fix sensor events not triggering when hitting a voxels collider.

#### Added

- Added support for voxels colliders attached to dynamic rigid-bodies.
- Added force calculation between colliding voxels/voxels and voxels/compound shapes.

### 0.16.2 (5 May 2025)

#### Fixed

- Fixed infinite loop in `collider.setVoxel`.

### 0.16.1 (2 May 2025)

#### Added

- Added `Collider.clearShapeCache` to release the reference to the JS shape stored in the collider, oor too force its
  recomputation the next time the collider shape is accessed.
- Added support for shape-casting involving Voxels colliders.
- Added support for CCD involving Voxels colliders.

### 0.16.0 (24 April 2025)

#### Added

- Added `ColliderDesc.voxels` to create a collider with a shape optimized for voxels.
- Added `Collider.setVoxel` for adding or removing a voxel from a collider with a voxel shape.
- Added the `Voxels` shape class.

The support or voxels is still experimental. In particular the following features will currently **not** work on
colliders with voxel shapes:

- Voxels colliders attached to dynamic rigid-bodies will not run the automatic mass/angular inertia calculation.
- Shape-casting on voxel shapes/colliders.
- Collision-detection between two-voxels colliders, or a voxels collider and a mesh, polyline, or heightfield.

See [parry#336](https://github.com/dimforge/parry/pull/336) for additional information.

### 0.15.1 (10 April 2025)

#### Added

- Added `RigidBody.velocityAtPoint` function to retrieve the velocity of a given world-space point on a rigid-body.

#### Modified

- Update to Rapier 0.25. The main notable change is that the `TriMeshFlags.FIX_INTERNAL_EDGES` flag will no longer
  imply that the `TriMeshFlags.ORIENTED` flag is set, avoiding edge-cases where enabling `FIX_INTERNAL_EDGES`
  results in unexpected collisions between open meshes and balls.

#### Fixed

- Fixed `*-simd-compat` builds not actually emitting SIMD instructions.

### 0.15.0 (06 March 2025)

#### Added

- Added `PidController`, `World.createPidController`, `World.removePidController` to create a Position-Integral-Derivative
  controller; a building block for building velocity-based dynamic character controllers.

#### Modified

- Update to Rapier 0.24.
- Package tree shaking has been disabled to avoid a crash on certain build configuration (#289).
- Multiple packages with different feature sets are now released, and their build process has been automated (#309)
  - Released packages are:
    - rapier2d
    - rapier2d-compat
    - rapier2d-simd
    - rapier2d-simd-compat
    - rapier2d-deterministic
    - rapier2d-deterministic-compat
    - rapier3d
    - rapier3d-compat
    - rapier2d
    - rapier2d-compat
    - rapier3d-simd
    - rapier3d-simd-compat
    - rapier3d-deterministic
    - rapier3d-deterministic-compat
  - :warning: To this occasion, existing packages `rapier2d` and `rapier3d` are now built without `enhanced-determinism` feature enabled,
    so if you rely on that feature, you should migrate to the new `-deterministic` flavor.


### 0.14.0 (20 July 2024)

#### Modified

- Update from the rust library of rapier 0.19 to 0.22, see [rapier's changelog](https://github.com/dimforge/rapier/blob/master/CHANGELOG.md#v0210-23-june-2024) for more context.

#### Added

- Added `RigidBody.userForce` function to retrieve the constant force(s) the user added to a rigid-body
- Added `RigidBody.userTorque` function to retrieve the constant torque(s) the user added to a rigid-body

### 0.13.1 (2024-05-08)

#### Fixed

- Fix regression that could cause missed contact between a ball and another shape type.

### 0.13.0 (2024-05-05)

Several stability improvements are added as part of this release.
See [rapier#625](https://github.com/dimforge/rapier/pull/625) for overviews of the most important improvements.

#### Modified

- The `castShape` and `castCollider` functions have been modified to add a `targetDistance` parameter. This parameter
  indicates the distance below which a hit is detected.
- Rename `RayIntersection.toi` to `.timeOfImpact` for better clarity.
- Rename `RayColliderIntersection.toi` to `.timeOfImpact` for better clarity.
- Rename `RayColliderToi` to `RayColliderHit`.
- Rename `RayColliderHit.toi` to `.timeOfImpact` for better clarity.
- Rename `ShapeTOI` to `ShapeCastHit`.
- Rename `ShapeColliderTOI` to `ColliderShapeCastHit`.
- Rename `ShapeCastHit.toi` to `.timeOfImpact`.

#### Added

- Fix the kinematic character controller getting stuck against vertical walls.
- Add `KinematicCharacterController.normalNudgeFactor` and `.setNormalNudgeFactor` so set a coefficient used for
  avoiding having the character controller get stuck on penetrations.
- Add `RigidBody.softCcdPrediction`, `.setSoftCcdPrediction`, and `RigidBodyDesc.setSoftCcdPrediction` for configuring
  soft-ccd on the rigid-body. See [rapier#625](https://github.com/dimforge/rapier/pull/625) for additional details on
  soft-ccd.
- 3D version only: add `TriMeshFlags::FIX_INTERNAL_EDGES` and `HeightFieldFlags::FIX_INTERNAL_EDGES` for enabling
  internal edges correction (which is no longer enabled by default). The flags have been added as an optional parameter
  when building the shapes.
- Add `Collider.contactSkin`, `.setContactSkin`, and `ColliderDesc.setContactSkin` for configuring the collider’s
  contact skin. See [rapier#625](https://github.com/dimforge/rapier/pull/625) for additional details on contact skins.
- Add `World.lengthUnit` which can be used to indicate the typical size of dynamic objects (e.g. 100 pixels instead of
  1 meter). This helps the physics engine adjust internal thresholds for better results.

#### Fixed

- Fix an issue where the reported contact force are lower than their actual value.

### 0.12.0 (2024-01-28)

The main highlight of this release is the implementation of a new non-linear constraints solver for better stability
and increased convergence rates. See [#579](https://github.com/dimforge/rapier/pull/579) for additional information.

In order to adjust the number of iterations of the new solver, simply adjust `World.numSolverIterations`.
If recovering the old solver behavior is useful to you, call `World.switchToStandardPgsSolver()`.

It is now possible to specify some additional solver iteration for specific rigid-bodies (and everything interacting
with it directly or indirectly through contacts and joints): `RigidBodyDesc.additionalSolverIterations` and
`RigidBody::setAdditionalSolverIterations`. This allows for higher-accuracy on subsets of the physics scene
without affecting performance of the other parts of the simulation.

#### Modified

- Renamed `CharacterController.translationApplied`, `.translationRemaining` and the `desiredTranslation`
  method argument to `CharacterController.translationDeltaApplied`, `.translationDeltaRemaining` and the
  `desiredTranslationDelta` to avoid confusion with the usage of the `translation` world in `RigidBody.translation()`.

#### Added

- Added `DynamicRayCastVehicleController` to simulate vehicles based on ray-casting.
- Added `JointData.generic` (3D only) to create a generic 6-dof joint and manually specify the locked axes.

### 0.11.2

#### Fixed

- Fix bug that made dynamic rigid-bodies behave like kinematic bodies after being disabled and then re-enabled.
- Fix issue with convex polyhedron jitter due to missing contacts.
- Fix character controller getting stuck against vertical walls.
- Fix character controller’s snapping to ground not triggering sometimes.
- Fix character controller’s horizontal offset being mostly ignored and some instances of vertical offset being ignored.

#### Added

- Add `JointData.spring` and `JointData.rope` joints.
- Add access to the mass-properties of a rigid-body: `RigidBody.effectiveInvMass`, `.invMass()`, `.localCom()`,
  `.worldCom()`, `.invPrincipalInertiaSqrt()`, `.principalInertia()`, `.principalInertiaLocalFrame()`,
  `.effectiveWorldInvInertiaSqrt()`, `.effectiveAngularInertia()`.
- Add `DynamicRayCastVehicleController.siteFrictionStiffness` to customize the side friction applied to the vehicle
  controller’s wheel.

#### Modified

- Rename `World.contactsWith` to `World.contactPairsWith`.
- Rename `World.intersectionsWith` to `World.intersectionPairsWith`.

### 0.11.1 (2023-01-16)

#### Fixed

- Fix bug that disabled all colliders at construction time.

### 0.11.0 (2023-01-15)

#### Added

- Add `World.propagateModifiedBodyPositionsToColliders` to propagate rigid-bodies position changes to their attached
  colliders.
- Add `World.updateSceneQueries` to update the scene queries data structures without stepping the whole simulation.
- Add `RigidBody.isEnabled, RigidBody.setEnabled, RigidBodyDesc.setEnabled` to disable a rigid-body (and all its
  attached colliders) without removing it from the physics world.
- Add `Collider.isEnabled, Collider.setEnabled, ColliderDesc.setEnabled` to disable a collider without removing it
  from the physics world.
- Add shape-specific methods to modify a collider’s
  size: `Collider.setRadius, setHalfExtents, setRoundRadius, setHalfHeight`.

#### Modified

- Add a boolean argument to `RigidBody.setBodyType` to indicate if the rigid-body should awaken after changing
  its type.

#### Fixed

- Fix rigid-bodies automatically waking up at creation even if they were explicitly created sleeping.

### 0.10.0 (2022-11-06)

#### Added

- Add `World.createCharacterController`, `World.removeCharacterController` to create/remove a kinematic character
  controller.
- Add a character-controller implementation with the `KinematicCharacterController` class and its method
  `KinematicCharacterController.computeColliderMovement`. The character controller features currently supported are:
    - Slide on uneven terrains
    - Interaction with dynamic bodies.
    - Climb stairs automatically.
    - Automatically snap the body to the floor when going downstairs.
    - Prevent sliding up slopes that are too steep
    - Prevent sliding down slopes that are not steep enough
    - Interactions with moving platforms.
    - Report information on the obstacles it hit on its path.
- Add the `HalfSpace` (infinite plane) shape. Colliders with this shape can be built using `ColliderDesc.halfspace`.

#### Modified

- Change the signature of `Collider.castShape` and World.castShape by adding a boolean argument `stop_at_penetration`
  before the filter-related arguments. Set this argument to `true` to get the same result as before. If this is set to
  `false` and the shape being cast starts it path already intersecting another shape, then a hit won’t be returned
  with that intersecting shape unless the casting movement would result in more penetrations.
- Reduce rounding errors in 3D when setting the rotation of a rigid-body or collider.

#### Fixed

- Fix incorrect application of torque if the torque is applies right after setting the rigid-body’s
  position, but before calling `World.step`.

### 0.9.0 (2022-10-07)

#### Fixed

- Fix unpredictable broad-phase panic when using small colliders in the simulation.
- Fix collision events being incorrectly generated for any shape that produces multiple
  contact manifolds (like triangle meshes).

#### Modified

- The `RigidBodyDesc.setAdditionalMass` method will now result in the additional angular inertia
  being automatically computed based on the shapes of the colliders attached to the rigid-body.
- Removed the method `RigidBodyDesc.setAdditionalPrincipalAngularInertia`. Use
  `RigidBodyDesc.setAdditionalMassProperties` instead.
- The methods of `RigidBodyDesc` and `ColliderDesc` will now always copy the object provided by
  the user instead of storing the object itself.
- The following method will now copy the user’s objects instead of storing it: `ColliderDesc.setRotation`,
  `ColliderDesc.setMassProperties`, `RigidBodyDesc.setRotation`, `RigidBodyDesc.setAdditionalMassProperties`,
  `RigidBodyDesc.setAngvel`.
- Rename `RigidBody.restrictRotations` and `RigidBody.restrictTranslations` to
  `RigidBody.setEnabledRotations` and `RigidBody.setEnabledTranslations`.
- Rename `RigidBodyDesc.restrictRotations` and `RigidBodyDesc.restrictTranslations` to
  `RigidBodyDesc.enabledRotations` and `RigidBodyDesc.enabledTranslations`.

#### Added

- Add `ImpulseJoint.setContactsEnabled`, and `MultibodyJoint.setContactsEnabled` to set whether
  contacts are enabled between colliders attached to rigid-bodies linked by this joint.
- Add `UnitImpulseJoint.setLimits` to set the limits of a unit joint.
- Add `RigidBody.recomputeMassPropertiesFromColliders` to force the immediate computation
  of a rigid-body’s mass properties (instead of waiting for them to be recomputed during the next
  timestep). This is useful to be able to read immediately the result of a change of a rigid-body
  additional mass-properties or a change of one of its collider’s mass-properties.
- Add `RigidBody::setAdditionalMass` to set the additional mass for the rigid-body. The additional
  angular inertia is automatically computed based on the attached colliders shapes. If this automatic
  angular inertia computation isn’t desired, use `RigidBody::setAdditionalMassProperties` instead.
- Add `Collider.setDensity`, `.setMass`, `.setMassProperties`, to alter a collider’s mass
  properties. Note that `.setMass` will result in the collider’s angular inertia being automatically
  computed based on this mass and on its shape.
- Add `ColliderDesc.setMass` to set the mass of the collider instead of its density. Its angular
  inertia tensor will be automatically computed based on this mass and its shape.
- Add more filtering options for scene-queries. All the scene query methods now take additional optional
  arguments to indicate if one particular collider, rigid-body, or type of colliders/rigid-bodies have to
  be ignored by the query.
- Add force reporting based on contact force events. The `EventHandler` trait has been modified to include
  the method `EventQueue.drainContactForceEvents`. Contact force events are generated whenever the sum of the
  magnitudes of all the forces between two colliders is greater than any of their
  `Collider.contactForceEventThreshold` values (only the colliders with the `ActiveEvents.CONTACT_FORCE_EVENT`
  flag set are taken into account for this threshold).

### 0.8.1 (2022-04-06)

Starting with this release, all the examples in `testbed2d` and `testbed3d` have been updated to `webpack 5`,
and are written in Typescript. In addition, canary `0.0.0` releases will be generated automatically after each merge
to the `master` branch.

#### Fixed

- Fix bug causing `World.intersectionPair` to always return `false`.

### 0.8.0 (2022-03-31)

#### Breaking changes

- Most APIs that relied on rigid-body handles or collider handles have been modified to rely on `RigidBody`
  or `Collider` objects instead. Handles are now only needed for events and hooks.
- Rename STATIC to FIXED in the `ActiveCollisionTypes` flags.
- The `RigidBody.applyForce` and `.applyTorque` methods have been replaced by `.addForce` and `.addTorque`. These
  force/torques are no longer automatically zeroed after a timestep. To zero them manually, call `.resetForce` or
  `.resetTorque`.
- The `EventQueue.drainContactEvents` and `EventQueue.drainIntersectionEvents` have been merged into a single
  method: `EventQueue:drainCollisionEvents`.

#### Added

- Add a lines-based debug-renderer. See [#119](https://github.com/dimforge/rapier.js/pull/119) for examples of
  integration of the debug-renderer with `THREE.js` and `PIXI.js`.
- Each rigid-body, collider, impulse joint, and multibody joint now have a unique object instance. This instance
  in only valid as long as the corresponding objects is inserted to the `World`.
- Add a `wakeUp: bool` argument to the `World.createImpulseJoint` and `MultibodyJointSet::createMultibodyJoint` to
  automatically wake-up the rigid-bodies attached to the inserted joint.
- Add a `filter` callback to all the scene queries. Use this for filtering more fine-grained than collision groups.
- Add `collider.shape` that represents the shape of a collider. This is a more convenient way of reading the collider’s
  shape properties. Modifying this shape will have no effect unless `collider.setShape` is called with the modified
  shape.
- Add `Collider.containsPoint`, `.projectPoint`, `.intersectsRay`, `.castShape`, `.castCollider`, `.intersectsShape`,
  `.contactShape`, `.contactCollider`, `.castRay`, `.castRayAndGetNormal`.
- Add `Shape.containsPoint`, `.projectPoint`, `.intersectsRay`, `.castShape`, `.intersectsShape`,
  `.contactShape`, `.castRay`, `.castRayAndGetNormal`.
- Add `World.projectPointAndGetFeature` which includes the feature Id the projected point lies on.
- Add `RigidBodyDesc.sleeping` to initialize the rigid-body in a sleeping state.

### 0.8.0-alpha.2 (2022-03-20)

The changelog hasn’t been updated yet.
For an overview of the changes, refer to the changelog for the unreleased Rust version:
https://github.com/dimforge/rapier/blob/master/CHANGELOG.md#unreleased

### 0.8.0-alpha.1 (2022-01-28)

#### Fixed

- Fix a crash when calling `collider.setShape`.
- Fix a crash when calling `world.collidersWithAabbIntersectingAabb`.
- Fix damping not being applied properly to some rigid-bodies.

### 0.8.0-alpha.0 (2022-01-16)

This release updates to Rapier 0.12.0-alpha.0 which contains:

- A **complete rewrite** of the joint and contact constraint solver.
- The support for locking individual translation axes.
- The support for multibody joint.

This is an **alpha** release because the new solver still needs some tuning,
and the spherical joint motors/limits is currently not working.

#### Breaking changes

- In 3D: renamed `BallJoint` to `SphericalJoint`.
- In 2D: renamed `BallJoint` to `RevoluteJoint`.
- Remove the joint motors and limits for the spherical joint (this is a temporary removal until with leave alpha).
- All the joint-related structures and methods (`RevoluteJoint`, `PrismaticJoint`, `createJoint`, etc.) have renamed to
  include `impulse` in the names: `RevoluteImpulseJoint`, `PrismaticImpulseJoint`, `createImpulseJoint`, etc. This is
  to differentiate them from the new multibody joints.
- Remove from the integration parameters all the parameters that are no longer meaningful (`maxPositionIterations`,
  `jointErp`, `warmstartCoeff`, `allowedAngularError`, `maxLinearCorrection`, `maxAngularCorrection`).

#### Added

- Add multibody joints. They are created the same way as impulse joints, except that they are created
  by `world.createMultibodyJoint` instead of `world.createImpulseJoint`.
- Add the ability to lock individual translation axes. Use `rigidBody.restrictTranslation`.

#### Fixed

- Fixed an issue with velocity-based kinematic bodies applying kinematic rotation improperly.
- Fixed the second witness points and normals returned by shape-casts.
- Fixed the second local contact normal and points returned by contact manifolds.

### 0.7.6

This release updates to Rapier 0.11.1 which contains several bug fixes.

#### Fixed

- Fix a bug causing large moving colliders to miss some collisions after some time.
- Fix invalid forces generated by contacts with position-based kinematic bodies.
- Fix a bug where two colliders without parent would not have their collision computed even if the
  appropriate flags were set.

### 0.7.5

#### Fixed

- Fixed an issue where a collider with no parent attached would not be created
  under some conditions.

### 0.7.4

This release was broken and has been unpublished.

### 0.7.3

#### Fixed

- The `collider.halfExtents()` methods now returns a valid value for round cuboids.

### 0.7.2 (rapier-compat only)

#### Modified

- The `rapier-compat` packages don’t need the `Buffer` polyfill anymore.

### 0.7.1

#### Modified

- Update to use Rapier 0.11.0.
- The `rapier-compat` packages are now generated by rollup.

### v0.7.0

The typescripts bindings for Rapier have
a [brand new user-guide](https://rapier.rs/docs/user_guides/javascript/getting_started_js)
covering all the features of the physics engine!

### Breaking change

- The `World.castRay` and `World.castRayAndGetNormal` methods have a different signature now, making them
  more flexible.
- Rename `ActiveHooks::FILTER_CONTACT_PAIR` to `ActiveHooks.FILTER_CONTACT_PAIRS`.
- Rename `ActiveHooks::FILTER_INTERSECTION_PAIR` to `ActiveHooks.FILTER_INTERSECTION_PAIRS`.
- Rename `BodyStatus` to `RigidBodyType`.
- Rename `RigidBody.bodyStatus()` to `RigidBody.bodyType()`.
- Rename `RigidBodyDesc.setMassProperties` to `RigidBodyDesc.setAdditionalMassProperties`.
- Rename `RigidBodyDesc.setPrincipalAngularInertia` to `RigidBodyDesc.setAdditionalPrincipalAngularInertia`.
- Rename `ColliderDesc.setIsSensor` to `ColliderDesc.setSensor.

#### Added

- Add `Ray.pointAt(t)` that conveniently computes `ray.origin + ray.dir * t`.
- Add access to joint motors by defining each joint with its own class deriving from `Joint`. Each joint now
  have its relevant motor configuration
  methods: `configurMotorModel, configureMotorVelocity, configureMotorPosition, configureMotor`.
- Add `World.collidersWithAabbIntersectingAabb` for retrieving the handles of all the colliders intersecting the given
  AABB.
- Add many missing methods for reading/modifying a rigid-body state after its creation:
    - `RigidBody.lockTranslations`
    - `RigidBody.lockRotations`
    - `RigidBody.restrictRotations`
    - `RigidBody.dominanceGroup`
    - `RigidBody.setDominanceGroup`
    - `RigidBody.enableCcd`
- Add `RigidBodyDesc.setDominanceGroup` for setting the dominance group of the rigid-body being built.
- Add many missing methods for reading/modifying a collider state after its creation:
    - `Collider.setSendor`
    - `Collider.setShape`
    - `Collider.setRestitution`
    - `Collider.setFriction`
    - `Collider.frictionCombineRule`
    - `Collider.setFrictionCombineRule`
    - `Collider.restitutionCombineRule`
    - `Collider.setRestitutionCombineRule`
    - `Collider.setCollisionGroups`
    - `Collider.setSolverGroups`
    - `Collider.activeHooks`
    - `Collider.setActiveHooks`
    - `Collider.activeEvents`
    - `Collider.setActiveEvents`
    - `Collider.activeCollisionTypes`
    - `Collider.setTranslation`
    - `Collider.setTranslationWrtParent`
    - `Collider.setRotation`
    - `Collider.setRotationWrtParent`
- Add `ColliderDesc.setMassProperties` for setting explicitly the mass properties of the collider being built (instead
  of relying on density).

#### Modified

- Colliders are no longer required to be attached to a rigid-body. Therefore, the second argument
  of `World.createCollider`
  is now optional.

### v0.6.0

#### Breaking changes

- The `BodyStatus::Kinematic` variant has been replaced by `BodyStatus::KinematicPositionBased` and
  `BodyStatus::KinematicVelocityBased`. Position-based kinematic bodies are controlled by setting (at each frame) the
  next
  kinematic position of the rigid-body (just like before), and the velocity-based kinematic bodies are controlled
  by setting (at each frame) the velocity of the rigid-body.
- The `RigidBodyDesc.newKinematic` has been replaced by `RigidBodyDesc.newKinematicPositionBased` and
  `RigidBodyDesc.newKinematicVelocityBased` in order to build a position-based or velocity-based kinematic body.
- All contact and intersection events are now disabled by default. The can be enabled for each collider individually
  by setting
  its `ActiveEvents`: `ColliderDesc.setActiveEvents(ActiveEvents.PROXIMITY_EVENTS | ActiveEvents.ContactEvents)`.

#### Added

- It is now possible to read contact information from the narrow-phase using:
    - `world.narrowPhase.contactsWith(collider1, callback)`
    - `world.narrowPhase.intersectionsWith(collider1, callback)`
    - `world.narrowPhase.contactPair(collider1, collider2, callback)`
    - `world.narrowPhase.intersectionPair(collider1, collider2, callback)`
- It is now possible to apply custom collision-detection filtering rules (more flexible than collision groups)
  based on a JS object by doing the following:
    - Enable physics hooks for the colliders you want the custom rules to apply to:
      `ColliderDesc.setActiveHooks(ActiveHooks.FILTER_CONTACT_PAIR | ActiveHooks.FILTER_CONTACT_PAIR)`
    - Define an object that implements the `PhysicsHooks` interface.
    - Pass and instance of your physics hooks object as the second argument of the `World.step` method.
- It is now possible to enable collision-detection between two non-dynamic bodies (e.g. between a kinematic
  body and a fixed body) by setting the active collision types of a collider:
  `ColliderDesc.setActiveCollisionTypes(ActiveCollisionTypes.ALL)`

### v0.5.3

- Fix a crash when loading the WASM file on iOS safari.

### v0.5.2

- Fix a crash that could happen after adding and then removing a collider right away,
  before stepping the simulation.

### v0.5.1

- Fix a determinism issue after snapshot restoration.

### v0.5.0

- Significantly improve the broad-phase performances when very large colliders are used.
- Add `RigidBodyDesc::setCcdEnabled` to enable Continuous Collision Detection (CCD) for this rigid-body. CCD ensures
  that a fast-moving rigid-body doesn't miss a collision (the tunneling problem).

#### Breaking changes:

- Removed multiple fields from `IntegrationParameters`. These were unused parameters.

### v0.4.0

- Fix a bug in ball/convex shape collision detection causing the ball to ignore penetrations with depths greater than
  its radius.

Breaking changes:

- Removed `IntegrationParameters.restitutionVelocityThreshold`
  and `IntegrationParameters.set_restitutionVelocityThreshold`.

### v0.3.1

- Fix crash happening when creating a collider with a `ColliderDesc.convexHull` shape.
- Actually remove the second argument of `RigidBodyDesc.setMass` as mentioned in the 0.3.0 changelog.
- Improve snapshotting performances.

### v0.3.0

#### Added

- Added a `RAPIER.version()` function at the root of the package to retrieve the version of Rapier as a string.

Several geometric queries have been added (the same methods have been added to the
`QueryPipeline` too):

- `World.intersectionsWithRay`: get all colliders intersecting a ray.
- `World.intersectionWithShape`: get one collider intersecting a shape.
- `World.projectPoint`: get the projection of a point on the closest collider.
- `World.intersectionsWithPoint`: get all the colliders containing a point.
- `World.castShape`: get the first collider intersecting a shape moving linearly
  (aka. sweep test).
- `World.intersectionsWithShape`: get all the colliders intersecting a shape.

Several new shape types are now supported:

- `RoundCuboid`, `Segment`, `Triangle`, `RoundTriangle`, `Polyline`, `ConvexPolygon` (2D only),
  `RoundConvexPolygon` (2D only), `ConvexPolyhedron` (3D only), `RoundConvexPolyhedron` (3D only),
  `RoundCone` (3D only).

It is possible to build `ColliderDesc` using these new shapes:

- `ColliderDesc.roundCuboid`, `ColliderDesc.segment`, `ColliderDesc.triangle`, `ColliderDesc.roundTriangle`,
  `ColliderDesc.convexHull`, `ColliderDesc.roundConvexHull`, `ColliderDesc.Polyline`,
  `ColliderDesc.convexPolyline` (2D only), `ColliderDesc.roundConvexPolyline` (2D only),
  `ColliderDesc.convexMesh` (3D only),`ColliderDesc.roundConvexMesh` (3D only), `ColliderDesc.roundCone` (3D only).

It is possible to specify different rules for combining friction and restitution coefficients of the two colliders
involved in a contact with:

- `ColliderDesc.frictionCombineRule`, and `ColliderDesc.restitutionCombineRule`.

Various RigidBody-related getter and setters have been added:

- `RigidBodyDesc.newStatic`, `RigidBodyDesc.newDynamic`, and `RigidBodyDesc.newKinematic` are new static method, short
  equivalent to `new RigidBodyDesc(BodyStatus.Static)`, etc.
- `RigidBodyDesc.setGravityScale`, `RigidBody.gravityScale`, `RigidBody.setGravityScale` to get/set the scale factor
  applied to the gravity affecting a rigid-body. Setting this to 0.0 will make the rigid-body ignore gravity.
- `RigidBody.setLinearDamping` and `RigidBody.setAngularDamping` to set the linear and angular damping of the
  rigid-body.
- `RigidBodyDesc.restrictRotations` to prevent rotations along specific coordinate axes. This replaces the three boolean
  arguments previously passed to `.setPrincipalAngularInertia`.

#### Breaking changes

Breaking changes related to rigid-bodies:

- The `RigidBodyDesc.setTranslation` and `RigidBodyDesc.setLinvel` methods now take the components of the translation
  directly as arguments instead of a single `Vector`.
- The `RigidBodyDesc.setMass` takes only one argument now. Use `RigidBodyDesc.lockTranslations` to lock the
  translational motion of the rigid-body.
- The `RigidBodyDesc.setPrincipalAngularInertia` no longer have boolean parameters to lock rotations.
  Use `RigidBodyDesc.lockRotations` or `RigidBodyDesc.restrictRotations` to lock the rotational motion of the
  rigid-body.

Breaking changes related to colliders:

- The `ColliderDesc.setTranslation` method now take the components of the translation directly as arguments instead of a
  single `Vector`.
- The `roundRadius` fields have been renamed `borderRadius`.
- The `RawShapeType.Polygon` no longer exists. For a 2D convex polygon, use `RawShapeType.ConvexPolygon`
  instead.
- All occurrences of `Trimesh` have been replaced by `TriMesh` (note the change in case).

Breaking changes related to events:

- Rename all occurrences of `Proximity` to `Intersection`.
- The `Proximity` enum has been removed.
- The `drainIntersectionEvents` (previously called `drainProximityEvent`) will now call a callback with
  arguments `(number, number, boolean)`: two collider handles, and a boolean indicating if they are intersecting.

Breaking changes related to scene queries:

- The `QueryPipeline.castRay` method now takes two additional parameters: a boolean indicating if the ray should not
  propagate if it starts inside of a shape, and a bit mask indicating the group the ray is part of and these it
  interacts with.
- The `World.castRay` and `QueryPipeline.castRay` now return a struct of type `RayColliderHit`
  which no longer contains the normal at the hit point. Use the new methods `World.castRayAndGetNormal`
  or `QueryPipeline.castRayAndGetNormal` in order to retrieve the normal too.

### v0.2.13

- Fix a bug where `RigidBodyDesc.setMass(m)` with `m != 0.0` would cause the rotations of the created rigid-body to be
  locked.

### v0.2.12

- Add a boolean argument to `RigidBodyDesc.setMass` to indicate if the mass contribution of colliders should be enabled
  for this rigid-body or not.
- Add a `RigidBody.lockRotations` method to lock all the rigid-body rotations resulting from forces.
- Add a `RigidBody.lockTranslations` method to lock all the rigid-body translations resulting from forces.
- Add a `RigidBody.setPrincipalAngularInertia` method to set the principal inertia of the rigid-body. This gives the
  ability to lock individual rotation axes of one rigid-body.

### v0.2.11

- Fix a bug where removing when immediately adding a collider would cause collisions to fail with the new collider.
- Fix a regression causing some colliders added after a few timesteps not to be properly taken into account.

### v0.2.10

- Fix a bug where removing a collider would result in a rigid-body being removed instead.
- Fix a determinism issue when running simulation on the Apple M1 processor.
- Add `JointData.prismatic` and `JointData.fixed` for creating prismatic joint or fixed joints.

### v0.2.9

- Add `RigidBody.setLinvel` to set the linear velocity of a rigid-body.
- Add `RigidBody.setAngvel` to set the angular velocity of a rigid-body.

### v0.2.8

- Add `RigidBodySet.remove` to remove a rigid-body from the set.
- Add `ColliderSet.remove` to remove a collider from the set.
- Add `JointSet.remove` to remove a joint from the set.
- Add `RigidBodyDesc.setLinearDamping` and `RigidBodyDesc.setAngularDamping` for setting the linear and angular damping
  coefficient of the rigid-body to create.
- Add `RigidBodyDesc.setMass`, and `RigidBodyDesc.setMassProperties` for setting the initial mass properties of a
  rigid-body.
- Add `ColliderDesc.setCollisionGroups` to use bit-masks for collision filtering between some pairs of colliders.
- Add `ColliderDesc.setSolverGroups` to use bit-masks for making the constraints solver ignore contacts between some
  pairs of colliders.
- Add `ColliderDesc.heightfield` to build a collider with an heightfield shape.
- Add `ColliderDesc.trimesh` to build a collider with a triangle mesh shape.

### v0.2.7

- Reduce snapshot size and computation times.

### v0.2.6

- Fix bug causing an unbounded memory usage when an objects falls indefinitely.

### v0.2.5

- Fix wrong result given by `RigidBody.isKinematic()` and `RigidBody.isDynamic()`.

### v0.2.4

- Add the support for round cylinder colliders (i.e. cylinders with round edges).

### v0.2.3

- Add the support for cone, cylinder, and capsule colliders.

### v0.2.2

- Fix regression causing the density and `isSensor` properties of `ColliderDesc` to not be taken into account.
- Throw an exception when the parent handle passed to `world.createCollider` is not a number.

### v0.2.1

This is a significant rewrite of the JavaScript bindings for rapier. The objective of this rewrite is to make the API
closer to Rapier's and remove most need for manual memory management.

- Calling `.free()` is now required only for objects that live for the whole duration of the simulation. This means that
  it is no longer necessary to `.free()` vectors, rays, ray intersections, colliders, rigid-bodies, etc. Object that
  continue to require an explicit `.free()` are:
    - `World` and `EventQueue`.
    - Or, if you are not using the `World` directly:
      `RigidBodySet`, `ColliderSet`, `JointSet`, `IntegrationParameters`, `PhysicsPipeline`, `QueryPipeline`
      , `SerializationPipeline`, and `EventQueue`.
- Collider.parent() now returns the `RigidBodyHandle` of its parent (instead of the `RigidBody` directly).
- Colliders are now built with `world.createCollider`, i.e., `body.createCollider(colliderDesc)`
  becomes `world.createCollider(colliderDesc, bodyHandle)`.
- Shape types are not an enumeration instead of strings: `ShapeType.Ball` and `ShapeType.Cuboid` instead of `"ball"`
  and `"cuboid"`.
- `collider.handle` is now a field instead of a function.
- `body.handle` is now a field instead of a function.
- The world's gravity is now a `Vector` instead of individual components, i.e., `let world = new RAPIER.World(x, y, z);`
  becomes `let world = new RAPIER.World(new RAPIER.Vector3(x, y, z))`.
- Most methods that took individual components as argument (`setPosition`, `setKinematicPosition`, `setRotation`, etc.)
  now take a `Vector` or `Rotation` structure instead. For example `rigid_body.setKinematicPosition(x, y, z)`
  becomes `rigid_body.setKinematicPosition(new RAPIER.Vector3(x, y, z))`.
- `world.stepWithEvents` becomes `world.step` (the event queue is the last optional argument).
- `RigidBodyDesc` and `ColliderDesc` now use the builder pattern. For example
  `let bodyDesc = new RAPIER.RigidBodyDesc("dynamic"); bodyDesc.setTranslation(x, y, z)` becomes
  `new RigidBodyDesc(BodyStatus.Dynamic).setTranslation(new Vector(x, y, z))`.
- `ray.dir` and `ray.origin` are now fields instead of methods.
- 2D rotations are now just a `number` instead of a `Rotation` struct. So instead of doing `rotation.angle`, single use
  the number as the rotation angle.
- 3D rotations are now represented by the interface `Rotation` (with fields `{x,y,z,w}`) or the class `Quaternion`. Any
  object with these `{x, y, z, w}` fields can be used wherever a `Rotation` is required.
- 2D vectors are now represented by the interface `Vector` (with fields `{x,y}`) or the class `Vector2`). Any object
  with these `{x,y}` fields can be used wherever a `Vector` is required.
- 3D vectors are now represented by the interface `Vector` (with fields `{x,y,z}`) or the class `Vector3`). Any object
  with these `{x,y,z}` fields can be used wherever a `Vector` is required.

### v0.2.0

See changelogs for v0.2.1 instead. The NPM package for v0.2.0 were missing some files.

### v0.1.17

- Fix bug when ghost forces and/or crashes could be observed when a kinematic body touches a fixed body.

### v0.1.16

- Fix kinematic rigid-body not waking up dynamic bodies it touches.
- Added `new Ray(origin, direction)` constructor instead of `Ray.new(origin, direction)`.

### v0.1.15

- Fix crash when removing a kinematic rigid-body from the World.

### v0.1.14

- Fix issues where force application functions took ownership of the JS vector, preventing the user from freeing
  with `Vector.free()` afterwards.

### v0.1.13

- Added `rigidBody.setNextKinematicTranslation` to set the translation of a kinematic rigid-body at the next timestep.
- Added `rigidBody.setNextKinematicRotation` to set the rotation of a kinematic rigid-body at the next timestep.
- Added `rigidBody.predictedTranslation` to get the translation of a kinematic rigid-body at the next timestep.
- Added `rigidBody.predictedRotation` to set the rotation of a kinematic rigid-body at the next timestep.
- Added `Ray` and `RayIntersection` structures for ray-casting.
- Added `world.castRay` to compute the first hit of a ray with the physics scene.
- Fix a bug causing a kinematic rigid-body not to teleport as expected after a `rigidBody.setPosition`.

### v0.1.12

- Added `world.removeCollider(collider)` to remove a collider from the physics world.
- Added `colliderDesc.setTranslation(...)` to set the relative translation of the collider to build wrt. the rigid-body
  it is attached to.
- Added `colliderDesc.setRotation(...)` to set the relative rotation of the collider to build wrt. the rigid-body it is
  attached to.

### v0.1.11

- Fix a bug causing a crash when the broad-phase proxy handles were recycled.

### v0.1.10

- Fix a determinism problem that could cause rigid-body handle allocation to be non-deterministic after a snapshot
  restoration.

### v0.1.9

- Added `world.getCollider(handle)` that retrieves a collider from its integer handle.
- Added `joint.handle()` that returns the integer handle of the joint.

### v0.1.8

- Added `world.forEachRigidBodyHandle(f)` to apply a closure on the integer handle of each rigid-body on the world.
- Added `world.forEachActiveRigidBody(f)` to apply a closure on each rigid-body on the world.
- Added `world.forEachActiveRigidBodyHandle(f)` to apply a closure on the integer handle of each rigid-body on the
  world.
- Added `rigidBody.applyForce`, `.applyTorque`, `.applyImpulse`, `.applyTorqueImpulse`, `.applyForceAtPoint`, and
  `.applyImpulseAtPoint` to apply a manual force or torque to a rigid-body.
- Added the `EventQueue` structure that can be used to collect and iterate through physics events.
- Added the `Proximity` enum that represents the proximity state of a sensor collider and another collider.
- Added the `world.stepWithEvents(eventQueue)` which executes a physics timestep and collects the physics events into
  the given event queue.

### v0.1.7

- Added `world.getRigidBody(handle)` to retrieve a rigid-body from its handle.
- Added `world.getJoint(handle)` to retrieve a joint from its handle.
- Added `rigidBody.rotation()` to retrieve its world-space orientation as a quaternion.
- Added `rigidBody.setTranslation(...)` to set the translation of a rigid-body.
- Added `rigidBody.setRotation(...)` to set the orientation of a rigid-body.
- Added `rigidBody.wakeUp()` to manually wake up a rigid-body.
- Added `rigidBody_desc.setRotation(...)` to set tho orientation of the rigid-body to be created.

### v0.1.6

- Added `world.removeRigidBody(...)` to remove a rigid-body from the world.
