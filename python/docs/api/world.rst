PhysicsWorld — the main entry point
===================================

.. currentmodule:: rapier3d

:class:`PhysicsWorld` is the recommended starting point for almost every
program. It bundles **all** of rapier's sub-state — the rigid-body,
collider, and joint sets, the broad/narrow phase, island manager, CCD
solver, integration parameters, and the physics + query pipelines — into
a single object, and drives the whole simulation with one
:meth:`~PhysicsWorld.step` call.

Reach for the individual pieces (documented under
:doc:`dynamics`, :doc:`geometry`, :doc:`joints`, and :doc:`pipeline`)
only when you need finer control than the world wrapper offers; for
everything else, start here.

.. code-block:: python

    import rapier3d as rp

    world = rp.PhysicsWorld(gravity=(0, -9.81, 0))

    # Static ground + a dynamic ball 5 m above it.
    world.colliders.insert(rp.Collider.cuboid(50, 0.1, 50).build())
    ball = world.add_body(
        rp.RigidBody.dynamic(translation=(0, 5, 0)),
        colliders=[rp.Collider.ball(0.5)],
    )

    for _ in range(240):
        world.step()

    print(world.rigid_bodies[ball].translation)

The sub-sets are exposed as **stable** properties — every access returns
the same Python object, so mutations through them persist
(``world.rigid_bodies is world.rigid_bodies``). Insert bodies and
colliders either directly on those sets or via the
:meth:`~PhysicsWorld.add_body` / :meth:`~PhysicsWorld.add_collider`
helpers.

.. autoclass:: PhysicsWorld
   :members:
   :undoc-members:
