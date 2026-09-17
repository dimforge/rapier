Changelog
=========

The Python bindings track the underlying Rust crates and share their
release cadence. The authoritative changelog is the Cargo
``CHANGELOG.md`` at the root of the
`Rapier repository
<https://github.com/dimforge/rapier/blob/master/CHANGELOG.md>`_.

Unreleased
----------

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
reports it. Each world owns its pool; ``None`` restores the default of one
worker per logical CPU and ``1`` runs everything inline on the calling
thread. Results are bit-identical whatever the worker count.
