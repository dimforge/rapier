"""Python `Protocol` shims for the event-handler / physics-hooks APIs.

These are duck-typing protocols (`typing.Protocol`); user classes don't need to
inherit from them, they just need to implement the right method names.

Both protocols are dim/scalar-agnostic (they are exposed under both
`rapier.dim2` and `rapier.dim3`).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Optional, Protocol, runtime_checkable

if TYPE_CHECKING:
    # The exact types vary by (dim, scalar). We keep them as `Any` in the
    # protocol so that user code doesn't lock itself to a single variant.
    pass


@runtime_checkable
class EventHandler(Protocol):
    """A receiver of collision, contact-force and soft-body tear events.

    Assign an instance of any class implementing this protocol to
    ``world.event_handler``. The methods are called from the physics solver
    (with the GIL re-acquired) during ``world.step()``.

    Exceptions raised inside the callbacks are deferred by default and
    re-raised after ``step()`` returns. Set ``world.event_error_policy = 'strict'``
    to short-circuit subsequent callbacks within the same step (the solver
    still runs to completion since rapier does not support mid-step aborts,
    but no further user callbacks are invoked).

    ``bodies`` and ``colliders`` are the ``RigidBodySet`` and ``ColliderSet``
    being stepped (``world.rigid_bodies`` / ``world.colliders``). During the
    callback, they and the bodies / colliders read from them can be read but
    not modified; the rest of the world (queries, joints, ...) can't be used
    until ``step()`` returns. Every method is optional: a handler without one
    of them doesn't receive the corresponding events.
    """

    def handle_collision_event(
        self,
        bodies: Any,
        colliders: Any,
        event: Any,
        contact_pair: Optional[Any],
    ) -> None:
        """Called when two colliders start or stop touching.

        ``event`` is a ``CollisionEvent`` (with ``.started`` / ``.stopped``
        flags). ``contact_pair`` is ``None`` for sensors.
        """
        ...

    def handle_contact_force_event(
        self,
        dt: float,
        bodies: Any,
        colliders: Any,
        contact_pair: Any,
        total_force_magnitude: float,
    ) -> None:
        """Called when the contact-force magnitude exceeds the
        ``contact_force_event_threshold`` of either collider in the pair.

        At least one of the involved colliders must have
        ``ActiveEvents.CONTACT_FORCE_EVENTS`` set.
        """
        ...

    def handle_soft_body_tear_event(self, soft_bodies: Any, event: Any) -> None:
        """Called at the end of a step for every soft body that tore during it,
        once the topology change is applied.

        ``event`` is a ``SoftBodyTearEvent``: the torn elements, the split
        particles and the pieces that became soft bodies of their own.
        ``soft_bodies`` is the ``SoftBodySet`` being stepped, readable like
        ``bodies`` above (``None`` with a ``PhysicsPipeline`` stepped without
        soft bodies). Immediate ``SoftBodySet.tear`` / ``cut`` calls return
        their event instead.
        """
        ...


@runtime_checkable
class PhysicsHooks(Protocol):
    """Custom collision / solver hooks invoked by the physics step.

    Assign an instance of any class implementing this protocol to
    ``world.physics_hooks``. To receive callbacks, the relevant colliders must
    have the appropriate ``ActiveHooks`` flags set (e.g.
    ``ActiveHooks.FILTER_CONTACT_PAIRS``).

    All three methods are optional: only define the ones you care about, a
    missing one behaves like the default hook (the pair is kept, the contacts
    are left unmodified). (But ``Protocol`` formally requires all three for type
    checkers.)

    The contexts expose the ``bodies`` / ``colliders`` sets being stepped. During
    the callback, they and the bodies / colliders read from them can be read but
    not modified. Read them through the context rather than through the world:
    the hooks may run on the engine's worker threads, from which the
    ``PhysicsWorld`` object itself can't be used.
    """

    def filter_contact_pair(self, ctx: Any) -> Any:
        """Return ``SolverFlags`` to allow contact computation (``None`` to
        completely discard the pair). The default behavior corresponds to
        returning ``SolverFlags.COMPUTE_IMPULSES``.

        ``ctx`` is a ``PairFilterContext`` view (read-only), e.g.
        ``ctx.colliders[ctx.collider1].user_data``.
        """
        ...

    def filter_intersection_pair(self, ctx: Any) -> bool:
        """Return ``True`` to allow intersection computation between the two
        colliders in ``ctx`` (a ``PairFilterContext``)."""
        ...

    def modify_solver_contacts(self, ctx: Any) -> None:
        """Mutate ``ctx`` (a ``ContactModificationContext``) in place to
        change the solver contacts before they're consumed by the solver.

        The context is **only valid for the duration of this call**: do not
        keep references to it after the method returns. Doing so will raise
        ``RuntimeError`` on access.
        """
        ...


__all__ = ["EventHandler", "PhysicsHooks"]
