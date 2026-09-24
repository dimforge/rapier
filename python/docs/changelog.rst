Changelog
=========

The Python bindings track the underlying Rust crates and share their
release cadence. The authoritative changelog is the Cargo
``CHANGELOG.md`` at the root of the
`Rapier repository
<https://github.com/dimforge/rapier/blob/master/CHANGELOG.md>`_.

Unreleased
----------

**Small fixes.** ``DebugRenderStyle.copy`` returns a detached style;
``BroadPhasePairEvent.is_added`` / ``is_removed`` give the event kind (the
``added`` constructor shadows the ``added`` flag on instances); the
``QueryPipeline.project_point`` stub lists ``max_dist``.

**Soft bodies.** ``PhysicsWorld.add_soft_body`` inserts a deformable body
(``SoftBody.rope`` / ``cloth`` / ``cuboid`` / ``sphere`` / ``trimesh`` /
``volumetric`` generators, ``SoftBodyBuilder`` setters, ``SoftBodyMaterial``),
simulated together with the rigid bodies, contacts and joints; ``world.soft_bodies``
is the ``SoftBodySet``. Particles can be pinned, attached to rigid bodies, driven
kinematically or pushed by forces and impulses; clusters
(``add_soft_body_cluster``) give a set of particles a rigid proxy that joints and
colliders attach to; deformable colliders (``insert_deformable``) bind a triangle
mesh to a cluster. Tearing and cutting (``tear_soft_body``, ``cut_soft_body``,
material thresholds) split pieces off into soft bodies of their own and report a
``SoftBodyTearEvent`` (``EventHandler.handle_soft_body_tear_event``,
``ChannelEventCollector.drain_soft_body_tear_events``). See :doc:`api/soft_bodies`.

Related changes: ``RigidBodyType.SOFT_FRAME`` marks the proxies of soft-body
clusters (``RigidBody.is_soft_frame`` / ``soft_body`` / ``soft_cluster``);
``RigidBody.additional_pgs_iterations``; ``IntegrationParameters.soft_bodies``
(``SoftBodiesSettings``); ``RigidBodySet.remove``, ``ColliderSet.remove``,
``PhysicsPipeline.step`` and ``DebugRenderPipeline.render`` / ``render_to_arrays``
take an optional ``soft_bodies`` argument; ``DebugRenderMode.SOFT_BODIES`` and
related modes; snapshots include the soft bodies; ``SolverFlags.COMPUTE_RIGID_IMPULSES``
is the engine's new name for ``COMPUTE_IMPULSES``.

**Rigid bodies and colliders.** ``RigidBody.set_next_kinematic_translation`` /
``set_next_kinematic_rotation`` / ``set_next_kinematic_position`` drive position-based
kinematic bodies; ``RigidBody.allow_fast_rotation`` (and the matching builder method and
factory keyword) lifts the angular speed cap. ``Collider.position_wrt_parent`` /
``translation_wrt_parent`` / ``rotation_wrt_parent`` read and move a collider on its body.
New shapes: ``Collider.segment`` / ``polyline`` / ``voxelized_mesh`` and their
``SharedShape`` siblings, with ``FillMode`` selecting a hollow or solid voxelization. Every
``Collider`` shape factory now accepts the ``ColliderBuilder`` keyword arguments, and the
mesh-based ones accept nested lists as well as NumPy arrays.

Fixes: ``SharedShape.heightfield`` / ``Collider.heightfield`` read row-major (C-ordered)
arrays correctly (rows along Z, columns along X; previously the heights were scrambled) and
accept ``float64`` arrays and nested lists. ``InteractionGroups.test_mode`` reads back as
``AND`` instead of ``DEFAULT`` (``DEFAULT`` and ``ONLY_DYNAMIC`` are now deprecated aliases
of ``AND``). The enums are hashable, and their stubs no longer claim they are
``enum.IntEnum`` subclasses. ``ColliderSet.remove`` takes ``wake_up`` (``wake_parent`` still
works but is deprecated) and ``RigidBody.add_gravitational_force`` takes ``wake_up``. The
force and torque docstrings now state that user forces persist until
``reset_forces`` / ``reset_torques``.

Soft-body additions: the FEM solver is always built in (``SoftBodySolver``,
``SoftBodyBuilder.solver`` / ``solver=``, ``SoftBody.solver``, ``SoftFemParameters`` on
``SoftBodiesSettings.fem``); ``SoftBody.volumetric_with`` with ``VolumeMeshParameters`` and
``MeshEnclosure``; ``to_trimesh()`` on ``Ball``, ``Cuboid``, ``Capsule``, ``Cylinder``,
``Cone``, ``ConvexPolyhedron``, ``HeightField`` and ``Voxels``;
``SoftBody.set_cluster_shape_matching_target``, ``set_edge_tear_resistance``,
``set_cell_tear_resistance`` and ``set_particle_damaged``; ``SoftBody.particle_positions`` and
``particle_velocities`` are writable. ``SoftBody.material`` and
``IntegrationParameters.soft_bodies`` (with ``recovery`` and ``fem``) are now live views, with a
``copy()`` for a detached copy; assigning them still works. A view of a removed soft body raises
``InvalidHandle``; ``RigidBodySet.remove`` and ``ColliderSet.remove`` raise ``ValueError`` for a
soft-body proxy or collision mesh when ``soft_bodies`` is left out; the soft-body index arrays
(pinned particles, elements, cluster particles, torn edges and cells) accept any memory layout and
integer dtype, where a sliced array used to be silently ignored.

**Breaking — ``SpringCoefficients`` keywords and defaults.** Its constructor takes ``natural_frequency`` and
``damping_ratio`` (``stiffness`` and ``damping`` remain as deprecated aliases, for the keywords and
the properties), and its defaults are now those of ``SpringCoefficients.contact_defaults()``
(``30.0`` Hz and ``10.0``, instead of a damping ratio of ``5.0``).

**Breaking — repackaged as ``rapier3d``.** The single ``rapier`` umbrella
package (with ``rapier.dim3`` submodules) has been replaced by the
``rapier3d`` package (3D / f32):

.. list-table::
   :header-rows: 1

   * - Before
     - After
   * - ``import rapier`` (3D f32 default)
     - ``import rapier3d``

The package is a standard ``abi3`` maturin wheel, so installs and platform
support (manylinux/musllinux, macOS, Windows) now go through the normal
wheel pipeline. The Panda3D testbed moved to a separate ``rapier-testbed``
package.

**Multi-threaded by default.** The wheels are now built with the engine's
``parallel`` feature always enabled, and the worker count is a runtime
setting: :meth:`~rapier3d.PhysicsWorld.set_num_threads` (also on
:class:`~rapier3d.PhysicsPipeline`) picks how many workers a world's
parallel stages run on, and :attr:`~rapier3d.PhysicsWorld.num_threads`
reports it. A worker count gives the world its own pool; ``None`` goes back
to rayon's global pool (one worker per logical CPU, shared by every world)
and ``1`` runs everything inline on the calling thread. Results are
bit-identical whatever the worker count.

**Controllers.** ``Wheel.center`` / ``suspension`` / ``axle`` give the
world-space state of a vehicle wheel after the last update.
``PdController`` and ``PidController`` expose their gains as ``lin_kp`` /
``ang_kp`` / ``lin_kd`` / ``ang_kd`` (plus ``lin_ki`` / ``ang_ki`` and the
read-only ``lin_integral`` / ``ang_integral`` on the PID), and
``rigid_body_correction`` takes an optional ``target_vels``
(``RigidBodyVelocity``). ``CharacterCollision.hit`` aliases ``toi``.
Fixes: scalar gains (``PidController(Kp=60.0)``) are applied to every axis
instead of raising ``TypeError``; the ``PidController.axes`` setter was
exposed as ``axes_attr``; ``update_vehicle`` and
``solve_character_collision_impulses`` now honor the ``QueryFilter``
predicate (evaluated once per collider before the update) and
``update_vehicle`` always excludes the chassis colliders;
``KinematicCharacterController(snap_to_ground=None)`` (and
``autostep=None``) disables the feature; out-of-range wheel indices raise
``IndexError`` instead of ``TypeError``. The unused ``bodies`` /
``colliders`` arguments of ``move_shape`` and ``character_pos`` of
``solve_character_collision_impulses`` accept ``None``; passing sets other
than the query pipeline's now raises ``ValueError``.

**Joints.** ``multibody_joints[handle]`` returns a live
:class:`~rapier3d.MultibodyJoint` (``data``, ``kinematic``, ``coords``,
``link_id``, ``multibody``); modifying its ``data`` wakes its bodies up and
changing its locked axes raises ``ValueError``.
:meth:`~rapier3d.Multibody.apply_displacements` applies the result of
``inverse_kinematics_for_link``, which now takes a ``joint_can_move``
callback. ``PrismaticJointBuilder.motor``, the ``RopeJointBuilder`` motor
methods and ``RopeJoint.motor`` / ``set_motor_*``. The joint builders accept
``softness=`` as a keyword argument. Fixed: ``len(multibody_joints)`` now
counts joints (like iteration) instead of multibodies;
``InverseKinematicsOption`` shows its ``constrained_axes`` in its repr.

**Loaders.** MJCF actuators, keyframes and contact rules:
``MjcfRobotHandles.actuators`` (:class:`~rapier3d.loaders.mjcf.MjcfActuatorHandle`),
``apply_controls``, ``apply_keyframe``, ``keyframe_controls``,
``keyframe_names`` and ``contact_hooks`` (a
:class:`~rapier3d.loaders.mjcf.MjcfContactHooks` that runs natively when
assigned to ``PhysicsWorld.physics_hooks``); ``MjcfRobot.gravity`` and
``keyframe_names``; ``MjcfMultibodyOptions.SKIP_JOINT_SPRINGS``. The
``Mjcf``/``UrdfMultibodyOptions`` flags support ``in``. Fixed: the repr of
``MjcfModel`` no longer shows ``Some(...)``, the URDF handles have a repr,
and the ``load_from_path`` docs list the supported formats.

**Scene queries, contacts and hooks.** ``QueryPipeline.project_point`` takes a
``max_dist``. ``NarrowPhase.contact_pairs_with`` / ``intersection_pairs_with``
list the pairs of one collider. ``ContactManifoldData.solver_contacts`` and
``solver_contact_world_points`` expose the solver contacts;
``SolverContact.tangent_velocity`` and
``ContactModificationContext.set_solver_contact`` / ``set_tangent_velocity``
modify them from a hook. ``PairFilterContext`` / ``ContactModificationContext``
expose the ``colliders`` / ``bodies`` being stepped, and event handlers receive
them (and the ``SoftBodySet``) instead of ``None``: during a callback, they and
the views into them can be read (not modified). ``QueryFilter``'s current groups
and predicate are read through ``interaction_groups`` / ``predicate_fn`` (the
former getters were shadowed by the builder methods).
``DebugRenderPipeline.style`` is now the same live object on every access.

Fixes: ``PhysicsWorld.update_query_pipeline`` / ``QueryPipeline.update`` no
longer consume the collider changes, which made the next ``step()`` panic or
miss them; the query pipeline is up to date after every step, so
``auto_update_query`` now has no effect. Reading the world from a hook or event
handler no longer raises a ``PanicException`` (running a query or stepping from
there raises ``RuntimeError``). A hooks or event-handler object may omit any of its
methods. ``DebugLineCollector.lines()`` returns the documented ``(N, 2, 3)``
array. ``ContactData.contact_id`` is the index of the point in its manifold
(it was always 0). ``ShapeCastHit`` documents its frames (``witness1`` /
``normal1`` on the hit collider in world space, ``witness2`` / ``normal2`` on
the cast shape in its local space). ``ContactPair``, ``ContactForceEvent``,
``ContactManifoldData`` and ``SolverContact`` have a ``repr``.

**World and pipelines.** ``PhysicsWorld.quarantine`` / ``PhysicsPipeline.quarantine``
report the objects neutralized by the last step because their state became non-finite
(:class:`~rapier3d.Quarantine`). ``PhysicsWorld.detect_collisions()`` updates the contacts
and the scene queries without stepping, with the world's new ``collision_pipeline``.
``PhysicsPipeline.counters`` is now a live view (``counters.disable()`` / ``enable()``
act on the pipeline), and the wheels are built with rapier's ``profiler`` feature so
its timings are measured. ``build_features()`` (:class:`~rapier3d.BuildFeatures`)
reports the Cargo profile and the optional features of the loaded extension.
``IntegrationParameters.warmstart_joints`` and ``friction_in_bias_pass``;
``FrictionModel.SIMPLIFIED`` is the new name of ``COEFFICIENT`` (kept as a deprecated
alias). ``rapier3d.math`` gains ``sin``, ``cos``, ``tan``, ``asin``, ``acos``, ``atan``,
``atan2``, ``exp``, ``log``, ``pow`` and ``sqrt``, computed by Rapier's math backend
(cross-platform deterministic with the ``determinism`` feature). ``SoftBody.is_enabled``
is writable. The testbed's ``set_world`` accepts a ``PhysicsWorld``.

**Fixes.** ``PhysicsWorld.clear()`` also removes the soft bodies and resets the
pipelines. Using a rigid-body, collider, impulse-joint or multibody view after its object
was removed raises ``InvalidHandle`` instead of a ``PanicException``. A world and its sets
can be used from any thread: using them from another thread no longer panics, and
modifying them (or reading them outside of its callbacks) while another thread steps the
world raises ``RuntimeError``. The type stubs now
cover ``PhysicsWorld``, the pipelines, the counters and the whole package
(``__init__.pyi``), and ``Vec3`` / ``Point3`` coordinates are read-only.
