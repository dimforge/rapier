Soft bodies
===========

Deformable bodies made of particles linked by elastic constraints (edges, bending
constraints and tetrahedral cells), simulated together with the rigid bodies, contacts
and joints of a world.

.. currentmodule:: rapier3d

A soft body starts from a :class:`SoftBodyBuilder`, made by one of the generators on
:class:`SoftBody` and inserted with :meth:`PhysicsWorld.add_soft_body`::

    import rapier3d as rp

    world = rp.PhysicsWorld(gravity=(0, -9.81, 0))
    world.colliders.insert(rp.Collider.cuboid(10, 0.1, 10).build())

    # A cloth pinned by its four corners.
    n = 20
    cloth = world.add_soft_body(
        rp.SoftBody.cloth((-1, 2, -1), (0.1, 0, 0), (0, 0, 0.1), n, n)
        .pinned_particles([0, n - 1, n * (n - 1), n * n - 1])
        .softness((30.0, 1.0))
        .particle_mass(0.05)
    )
    # A jelly cube with corotational elastic cells.
    jelly = world.add_soft_body(
        rp.SoftBody.cuboid((2, 1, 0), (0.5, 0.5, 0.5), 4, 4, 4,
                           cell_model=rp.SoftBodyCellModel.COROTATIONAL,
                           material=rp.SoftBodyMaterial(young_modulus=2e3, poisson_ratio=0.35),
                           particle_mass=0.2)
    )

    for _ in range(120):
        world.step()

    print(world.soft_bodies[cloth].particle_positions.shape)  # (400, 3)

Bodies and sets
---------------

.. autoclass:: SoftBody
.. autoclass:: SoftBodyBuilder
.. autoclass:: SoftBodySet
.. autoclass:: SoftBodyHandle

Material and settings
---------------------

.. autoclass:: SoftBodyMaterial
.. autoclass:: SoftBodyCellModel
.. autoclass:: SoftEdgePlasticFlow
.. autoclass:: SoftBodyParticleSettings
.. autoclass:: SoftBodiesSettings
.. autoclass:: SoftRecoverySettings
.. autoclass:: SoftPatchConstraints

Elements
--------

Snapshots of a body's particles and elements, read through :meth:`SoftBody.particle`,
:meth:`SoftBody.edge`, :meth:`SoftBody.cell` and :meth:`SoftBody.dihedral`.

.. autoclass:: SoftBodyParticle
.. autoclass:: SoftBodyEdge
.. autoclass:: SoftBodyEdgeKind
.. autoclass:: SoftBodyCell
.. autoclass:: SoftBodyDihedral
.. autoclass:: SoftParticleAttachment
.. autoclass:: SoftVolumePiece

Clusters and deformable colliders
---------------------------------

A cluster (:meth:`PhysicsWorld.add_soft_body_cluster`) gives a set of particles a rigid proxy
that joints and colliders attach to. A deformable collider
(:meth:`PhysicsWorld.insert_deformable`) is a triangle mesh built with
``TriMeshFlags.DEFORMABLE`` whose vertices follow a cluster through a :class:`SoftMeshBinding`.

.. autoclass:: SoftBodyCluster
.. autoclass:: SoftMeshBinding
.. autoclass:: SoftCollisionMesh
.. autoclass:: SoftMeshId
.. autoclass:: SoftMeshRef

Tearing and cutting
-------------------

Elements tear on their own past the material's ``tear_strain`` / ``tear_force``, on request
(:meth:`SoftBody.tear_edge`, :meth:`SoftBody.tear_cell`), or at once with
:meth:`PhysicsWorld.tear_soft_body` and :meth:`PhysicsWorld.cut_soft_body`. Pieces a tear
disconnects become soft bodies of their own; every tear is reported as a
:class:`SoftBodyTearEvent` (through the event handler, or returned by the immediate calls).

.. autoclass:: SoftBodyTearEvent
.. autoclass:: SoftBodyPiece
.. autoclass:: SoftClusterSplit
.. autoclass:: SoftJointMove
