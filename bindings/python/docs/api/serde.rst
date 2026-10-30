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
:doc:`world` for the full class:

.. autoclass:: PhysicsWorld
   :no-index:
   :members: snapshot, restore

Snapshot deltas
---------------

To replicate a stepping world (e.g. from a server to its clients), send
the full state once, then only what changed since the state each receiver
acknowledged. A receiver that applied every delta from its baseline has
the same :meth:`~rapier3d.PhysicsWorld.snapshot` bytes as the sender and
keeps simulating identically.

.. code-block:: python

   server.enable_change_tracking()

   # Bootstrap an empty receiver with the full state, as a delta from nothing.
   full, acked = server.delta_since(None)
   client = PhysicsWorld()
   acked = client.apply_delta(full)

   # Each frame: only the changes since the acknowledged tick.
   server.step()
   delta, _ = server.delta_since(acked)
   acked = client.apply_delta(delta)  # the receiver sends this tick back

Deltas and ticks are opaque ``bytes``, so they can travel over the
network. A receiver can also start from a world restored from a
:meth:`~rapier3d.PhysicsWorld.snapshot`: take ``delta_since(None)[1]``
at the same instant as the snapshot as its first acknowledged tick.

A delta can be produced against any tick a receiver acknowledged, not
only the last one: a receiver that missed some deltas catches up with a
single delta from its own tick. A delta's size follows what changed:
roughly the awake bodies, their colliders and contacts.

Calling :meth:`~rapier3d.PhysicsWorld.enable_change_tracking` again
restarts the record and invalidates every tick issued before. Event
handlers, physics hooks and the other non-serialized settings listed for
:meth:`~rapier3d.PhysicsWorld.snapshot` are not replicated either.

.. autoclass:: PhysicsWorld
   :no-index:
   :members: enable_change_tracking, disable_change_tracking, delta_since, apply_delta

Errors
------

:meth:`~rapier3d.PhysicsWorld.restore`, ``from_bytes(...)``, and the
``pickle.loads`` / ``copy.deepcopy`` paths raise
:class:`~rapier3d.SerializationError` (a subclass of
:class:`~rapier3d.RapierError`) on malformed or truncated input. See
:doc:`errors`.

:meth:`~rapier3d.PhysicsWorld.delta_since` and
:meth:`~rapier3d.PhysicsWorld.apply_delta` raise
:class:`~rapier3d.SerializationError` on malformed or truncated blobs, and
its subclass :class:`~rapier3d.DeltaError` when the delta cannot be
produced or applied. Its ``kind`` attribute names the cause:

- ``"tracking_disabled"``: ``delta_since`` was called without change tracking.
- ``"unknown_baseline"``: the tick was not issued by this world since change
  tracking was last enabled; send a full state (``delta_since(None)``).
- ``"baseline_mismatch"``: the delta was produced against another baseline
  than the last delta the receiver applied.
- ``"malformed"`` / ``"missing_shape"``: the delta is inconsistent with the
  receiver.

A failed ``apply_delta`` may leave the receiver partially patched: apply a
full delta to recover.
