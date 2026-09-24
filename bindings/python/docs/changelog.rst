Changelog
=========

The Python bindings track the underlying Rust crates and share their
release cadence. The authoritative changelog is the Cargo
``CHANGELOG.md`` at the root of the
`Rapier repository
<https://github.com/dimforge/rapier/blob/master/CHANGELOG.md>`_.

0.36.0 (24 September 2026)
--------------------------

**Added**

- Soft bodies: ``PhysicsWorld.add_soft_body``, ``SoftBodySet``, ``SoftBody`` generators
  (``rope``, ``cloth``, ``trimesh``, ``volumetric``…), ``SoftBodyBuilder``, ``SoftBodyMaterial``.
- Soft-body tearing and cutting (``tear_soft_body``, ``cut_soft_body``), reported as
  ``SoftBodyTearEvent`` (``EventHandler.handle_soft_body_tear_event``).
- Soft-body clusters (``add_soft_body_cluster``) and deformable colliders (``insert_deformable``).
- The FEM soft-body solver is always built in (``SoftBodySolver``, ``SoftFemParameters``).
- ``RigidBodyType.SOFT_FRAME``, ``RigidBody.additional_pgs_iterations``,
  ``IntegrationParameters.soft_bodies``; snapshots include the soft bodies.
- ``PhysicsWorld.detect_collisions()``, ``PhysicsWorld.quarantine`` and ``build_features()``.
- ``RigidBody.set_next_kinematic_*`` and ``allow_fast_rotation``; ``Collider.*_wrt_parent``.
- ``Collider.segment`` / ``polyline`` / ``voxelized_mesh`` (with ``FillMode``), and ``to_trimesh()``
  on the primitive shapes.
- ``NarrowPhase.contact_pairs_with``; solver contacts readable and editable from hooks
  (``set_solver_contact``, ``set_tangent_velocity``); ``QueryPipeline.project_point(max_dist=)``.
- PD/PID controller gain properties and the world-space ``Wheel`` state.
- Live ``MultibodyJoint`` views and ``Multibody.apply_displacements``.
- MJCF actuators, keyframes and contact rules (``MjcfRobotHandles.contact_hooks``).
- ``IntegrationParameters.warmstart_joints`` / ``friction_in_bias_pass``; deterministic
  ``sin``, ``cos``, ``exp``, ``sqrt``… in ``rapier3d.math``.
- The wheels are built with rapier's ``profiler`` feature, so the ``Counters`` timings are measured.
- Testbed: ``set_world`` accepts a ``PhysicsWorld``, soft-body rendering and examples.

**Modified**

- **Breaking:** ``SpringCoefficients`` takes ``natural_frequency`` / ``damping_ratio`` (``stiffness``
  / ``damping`` are deprecated aliases) and its default damping ratio is ``10.0`` instead of ``5.0``.
- ``RigidBodySet.remove``, ``ColliderSet.remove``, ``PhysicsPipeline.step`` and
  ``DebugRenderPipeline.render`` take an optional ``soft_bodies`` argument.
- ``SolverFlags.COMPUTE_IMPULSES`` is renamed ``COMPUTE_RIGID_IMPULSES``.
- Deprecated: ``FrictionModel.COEFFICIENT`` (now ``SIMPLIFIED``), ``ColliderSet.remove(wake_parent=)``
  (now ``wake_up``), and the ``InteractionGroups`` test modes ``DEFAULT`` / ``ONLY_DYNAMIC``.
- The enums are hashable; the type stubs cover the whole package and ``Vec3`` / ``Point3`` are read-only.

**Fixed**

- Heightfields read row-major arrays correctly (the heights were scrambled).
- ``update_query_pipeline`` no longer consumes the collider changes, which broke the next ``step()``.
- Using a view of a removed object raises ``InvalidHandle`` instead of a ``PanicException``.
- Reading the world from a hook, or using it from another thread, no longer panics.
- ``len(multibody_joints)`` counts joints, scalar PID gains apply to every axis, and
  ``ContactData.contact_id`` is the point index (it was always 0).
- ``update_vehicle`` and ``solve_character_collision_impulses`` honor the ``QueryFilter`` predicate.
- ``PhysicsWorld.clear()`` also removes the soft bodies.

0.35.1 (08 August 2026)
-----------------------

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
