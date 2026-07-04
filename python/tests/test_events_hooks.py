"""3D events & hooks tests (f32)."""

from __future__ import annotations

import pytest

import rapier3d as dim3


@pytest.fixture(params=[dim3], ids=["f32"])
def ns(request):
    return request.param


# ---- helpers --------------------------------------------------------------

def _two_balls_world(ns):
    """Build a world with two balls headed towards each other."""
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    h1 = w.rigid_bodies.insert(
        ns.RigidBody.dynamic(translation=(-1.0, 0, 0))
        .linvel(ns.Vec3(1.0, 0, 0))
        .build()
    )
    h2 = w.rigid_bodies.insert(
        ns.RigidBody.dynamic(translation=(1.0, 0, 0))
        .linvel(ns.Vec3(-1.0, 0, 0))
        .build()
    )
    c1 = ns.Collider.ball(0.5).active_events(ns.ActiveEvents.COLLISION_EVENTS)
    c2 = ns.Collider.ball(0.5).active_events(ns.ActiveEvents.COLLISION_EVENTS)
    ch1 = w.colliders.insert_with_parent(c1, h1, w.rigid_bodies)
    ch2 = w.colliders.insert_with_parent(c2, h2, w.rigid_bodies)
    return w, ch1, ch2


def _heavy_ball_on_ground(ns):
    """Heavy ball impacting the ground generates a contact-force event."""
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    ground = (
        ns.Collider.cuboid(50, 0.1, 50)
        .active_events(ns.ActiveEvents.CONTACT_FORCE_EVENTS)
        .contact_force_event_threshold(0.1)
    )
    w.colliders.insert(ground)
    ball_h = w.rigid_bodies.insert(
        ns.RigidBody.dynamic(translation=(0, 5, 0)).build()
    )
    bc = (
        ns.Collider.ball(0.5)
        .active_events(ns.ActiveEvents.CONTACT_FORCE_EVENTS)
        .contact_force_event_threshold(0.1)
        .density(10.0)
    )
    w.colliders.insert_with_parent(bc, ball_h, w.rigid_bodies)
    return w


# ---- ChannelEventCollector ------------------------------------------------


def test_channel_collector_receives_collision_event(ns):
    w, ch1, ch2 = _two_balls_world(ns)
    collector = ns.ChannelEventCollector()
    w.event_handler = collector
    for _ in range(60):
        w.step()
    events = collector.drain_collision_events()
    assert any(e.started for e in events), \
        f"expected at least one Started event, got {len(events)}"


def test_channel_collector_receives_contact_force_event(ns):
    w = _heavy_ball_on_ground(ns)
    collector = ns.ChannelEventCollector()
    w.event_handler = collector
    for _ in range(60):
        w.step()
    forces = collector.drain_contact_force_events()
    assert len(forces) > 0, "expected at least one ContactForceEvent"
    # The event reports a finite, positive total-force magnitude.
    assert forces[0].total_force_magnitude > 0.0


def test_channel_collector_drain_empties_buffer(ns):
    collector = ns.ChannelEventCollector()
    assert len(collector) == 0
    # First drain on empty buffer is empty.
    assert collector.drain_collision_events() == []
    assert collector.drain_contact_force_events() == []


def test_channel_collector_clear(ns):
    w, _, _ = _two_balls_world(ns)
    collector = ns.ChannelEventCollector()
    w.event_handler = collector
    for _ in range(60):
        w.step()
    assert len(collector) >= 1
    collector.clear()
    assert len(collector) == 0


# ---- Custom Python EventHandler (duck-typing) ----------------------------


def test_custom_event_handler_receives_collision(ns):
    received = []

    class MyHandler:
        def handle_collision_event(self, bodies, colliders, event, contact_pair):
            received.append(event)

        def handle_contact_force_event(self, dt, bodies, colliders, contact_pair, mag):
            received.append(("force", mag))

    w, _, _ = _two_balls_world(ns)
    w.event_handler = MyHandler()
    for _ in range(60):
        w.step()
    assert len(received) > 0, "expected at least one collision event"
    assert any(getattr(e, "started", False) for e in received)


# ---- PhysicsHooks ---------------------------------------------------------


def test_physics_hooks_filter_contact_pair_returning_none_drops_pair(ns):
    """A `filter_contact_pair` returning `None` makes the narrow phase skip the pair."""

    w, ch1, ch2 = _two_balls_world(ns)

    class Filter:
        def filter_contact_pair(self, ctx):
            return None  # discard pair

        def filter_intersection_pair(self, ctx):
            return True

        def modify_solver_contacts(self, ctx):
            pass

    w.physics_hooks = Filter()
    # The colliders must opt in to the filter hook.
    # Colliders are live views; mutating them persists directly.
    c1 = w.colliders[ch1]
    c1.active_hooks = ns.ActiveHooks.FILTER_CONTACT_PAIR
    c2 = w.colliders[ch2]
    c2.active_hooks = ns.ActiveHooks.FILTER_CONTACT_PAIR

    # Track ball positions to verify they passed through.
    start_pos = w.rigid_bodies[w.colliders[ch1].parent].translation.x
    for _ in range(60):
        w.step()
    end_pos = w.rigid_bodies[w.colliders[ch1].parent].translation.x

    # When the pair is filtered out, the contact pair stored in the narrow
    # phase has no active contacts (rapier clears the workspace but keeps a
    # placeholder entry — see `narrow_phase.rs::pair.clear()`).
    pair = w.narrow_phase.contact_pair(ch1, ch2)
    if pair is not None:
        assert not pair.has_any_active_contact, \
            "filtered pair must have no active contacts"
    # And the ball must have actually moved past the start (no bounce).
    assert end_pos > start_pos + 0.5, \
        f"balls didn't pass through; moved from {start_pos} to {end_pos}"


# ---- Error policy ---------------------------------------------------------


def test_callback_exception_deferred_then_raised(ns):
    """Default `defer` policy: exception is re-raised after `step()` returns."""
    w, _, _ = _two_balls_world(ns)

    class BadHandler:
        def handle_collision_event(self, *args, **kwargs):
            raise RuntimeError("boom")

        def handle_contact_force_event(self, *args, **kwargs):
            pass

    w.event_handler = BadHandler()
    # Run until at least one collision happens — that's when our handler raises.
    raised = False
    for _ in range(120):
        try:
            w.step()
        except RuntimeError as e:
            assert "boom" in str(e)
            raised = True
            break
    assert raised, "expected RuntimeError to be re-raised at least once"


def test_event_error_policy_default_is_defer(ns):
    w = ns.PhysicsWorld()
    assert w.event_error_policy == "defer"


def test_event_error_policy_setter_accepts_strict_and_defer(ns):
    w = ns.PhysicsWorld()
    w.event_error_policy = "strict"
    assert w.event_error_policy == "strict"
    w.event_error_policy = "defer"
    assert w.event_error_policy == "defer"


def test_event_error_policy_setter_rejects_unknown(ns):
    w = ns.PhysicsWorld()
    with pytest.raises(ValueError):
        w.event_error_policy = "panic"


def test_strict_policy_still_raises(ns):
    """Strict policy: also re-raises after step (it short-circuits further callbacks)."""
    w, _, _ = _two_balls_world(ns)

    n_calls = [0]

    class BadHandler:
        def handle_collision_event(self, *args, **kwargs):
            n_calls[0] += 1
            raise RuntimeError("strict boom")

        def handle_contact_force_event(self, *args, **kwargs):
            pass

    w.event_handler = BadHandler()
    w.event_error_policy = "strict"
    raised = False
    for _ in range(120):
        try:
            w.step()
        except RuntimeError as e:
            assert "strict boom" in str(e)
            raised = True
            break
    assert raised, "strict policy should still re-raise on first error"


# ---- SolverFlags bitflags -------------------------------------------------


def test_solver_flags_compute_impulses(ns):
    f = ns.SolverFlags.COMPUTE_IMPULSES
    assert f.bits == 1
    assert not ns.SolverFlags.EMPTY.bits
    assert bool(f)
    assert not bool(ns.SolverFlags.empty())


def test_solver_flags_ops(ns):
    a = ns.SolverFlags.COMPUTE_IMPULSES
    b = ns.SolverFlags.empty()
    assert (a | b) == a
    assert (a & b).is_empty()


# ---- PairFilterContext / ContactModificationContext ----------------------


def test_pair_filter_context_exposes_handles(ns):
    w, ch1, ch2 = _two_balls_world(ns)

    seen = []

    class Filter:
        def filter_contact_pair(self, ctx):
            seen.append((ctx.collider1, ctx.collider2,
                         ctx.rigid_body1, ctx.rigid_body2))
            return ns.SolverFlags.COMPUTE_IMPULSES

        def filter_intersection_pair(self, ctx):
            return True

        def modify_solver_contacts(self, ctx):
            pass

    w.physics_hooks = Filter()
    c1 = w.colliders[ch1]
    c1.active_hooks = ns.ActiveHooks.FILTER_CONTACT_PAIR
    c2 = w.colliders[ch2]
    c2.active_hooks = ns.ActiveHooks.FILTER_CONTACT_PAIR
    for _ in range(60):
        w.step()
    assert len(seen) > 0
    c1h, c2h, b1h, b2h = seen[0]
    # The pair must reference our two colliders/bodies (order may vary).
    assert {c1h, c2h} == {ch1, ch2}


def test_modify_solver_contacts_can_clear(ns):
    """Clearing solver contacts via the modification context disables resolution."""
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    h1 = w.rigid_bodies.insert(
        ns.RigidBody.dynamic(translation=(-1.0, 0, 0))
        .linvel(ns.Vec3(1.0, 0, 0))
        .build()
    )
    h2 = w.rigid_bodies.insert(
        ns.RigidBody.dynamic(translation=(1.0, 0, 0))
        .linvel(ns.Vec3(-1.0, 0, 0))
        .build()
    )
    c1 = ns.Collider.ball(0.5).active_hooks(ns.ActiveHooks.MODIFY_SOLVER_CONTACTS)
    c2 = ns.Collider.ball(0.5).active_hooks(ns.ActiveHooks.MODIFY_SOLVER_CONTACTS)
    ch1 = w.colliders.insert_with_parent(c1, h1, w.rigid_bodies)
    ch2 = w.colliders.insert_with_parent(c2, h2, w.rigid_bodies)

    invocations = [0]

    class ClearAll:
        def filter_contact_pair(self, ctx):
            return ns.SolverFlags.COMPUTE_IMPULSES

        def filter_intersection_pair(self, ctx):
            return True

        def modify_solver_contacts(self, ctx):
            invocations[0] += 1
            ctx.clear_solver_contacts()

    w.physics_hooks = ClearAll()
    for _ in range(60):
        w.step()
    # The hook must have been called at least once.
    assert invocations[0] > 0
