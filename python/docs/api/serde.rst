Serialization & snapshot
========================

The :class:`~rapier3d.PhysicsWorld` umbrella supports binary snapshots
of a full world via :meth:`~rapier3d.PhysicsWorld.snapshot` and
:meth:`~rapier3d.PhysicsWorld.restore`. Every value-style pyclass
(``RigidBody``, ``Collider``, ``Vec3``, …) also supports
``to_bytes()`` / ``from_bytes(...)`` and Python's ``pickle`` protocol.

Pickle is wired up via the ``__reduce__`` protocol on each pyclass so
that round-tripping through ``pickle.dumps`` / ``pickle.loads`` (or
``copy.deepcopy``) reconstructs equivalent objects.

.. currentmodule:: rapier3d

Snapshot API
------------

The snapshot methods live on :class:`PhysicsWorld` itself — see
:doc:`pipeline` for the full class:

.. autoclass:: PhysicsWorld
   :no-index:
   :members: snapshot, restore

Errors
------

.. autoclass:: rapier3d.SerializationError

Flavor-tagging caveat
---------------------

Snapshots are **not** tagged with the ``(dim, scalar)`` flavor that
produced them. Restoring an f32 snapshot through
``rapier3d_f64.PhysicsWorld.restore(...)`` will silently misinterpret
the floats. See :doc:`../limitations` for the long version.
