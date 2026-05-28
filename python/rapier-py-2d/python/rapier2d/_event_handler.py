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
    """A receiver of collision / contact-force events.

    Assign an instance of any class implementing this protocol to
    ``world.event_handler``. The methods are called from the physics solver
    (with the GIL re-acquired) during ``world.step()``.

    Exceptions raised inside the callbacks are deferred by default and
    re-raised after ``step()`` returns. Set ``world.event_error_policy = 'strict'``
    to short-circuit subsequent callbacks within the same step (the solver
    still runs to completion since rapier does not support mid-step aborts,
    but no further user callbacks are invoked).

    Note: ``bodies`` and ``colliders`` are passed as ``None`` to avoid borrow
    conflicts with the in-flight ``step()`` — handles can still be obtained
    from ``event`` / ``contact_pair`` and looked up against the world after
    ``step()`` returns.
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


@runtime_checkable
class PhysicsHooks(Protocol):
    """Custom collision / solver hooks invoked by the physics step.

    Assign an instance of any class implementing this protocol to
    ``world.physics_hooks``. To receive callbacks, the relevant colliders must
    have the appropriate ``ActiveHooks`` flags set (e.g.
    ``ActiveHooks.FILTER_CONTACT_PAIRS``).

    All three methods are optional in the duck-typed sense — only define the
    ones you care about. (But ``Protocol`` formally requires all three for type
    checkers.)
    """

    def filter_contact_pair(self, ctx: Any) -> Any:
        """Return ``SolverFlags`` to allow contact computation (``None`` to
        completely discard the pair). The default behavior corresponds to
        returning ``SolverFlags.COMPUTE_IMPULSES``.

        ``ctx`` is a ``PairFilterContext`` view (read-only).
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
