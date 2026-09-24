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


# ---- reading the world from the callbacks ----------------------------------


def _ball_on_ground_with_events(ns, events=None, hooks=None):
    events = events if events is not None else ns.ActiveEvents.COLLISION_EVENTS
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    builder = ns.Collider.cuboid(10, 0.1, 10).active_events(events).user_data(7)
    if hooks is not None:
        builder = builder.active_hooks(hooks)
    ground = w.add_collider(builder)
    body = w.add_body(
        ns.RigidBody.dynamic(translation=(0, 0.7, 0)),
        colliders=[ns.Collider.ball(0.5).user_data(8).contact_force_event_threshold(0.0)],
    )
    ball = w.rigid_bodies[body].colliders[0]
    return w, ground, body, ball


def test_event_handler_receives_readable_sets(ns):
    w, ground, body, ball = _ball_on_ground_with_events(ns)
    stored_view = w.rigid_bodies[body]
    seen = []

    class Handler:
        def handle_collision_event(self, bodies, colliders, event, contact_pair):
            assert bodies is w.rigid_bodies and colliders is w.colliders
            seen.append((
                colliders[ground].user_data,
                colliders.get(ball).user_data,
                bodies[body].translation.y,
                stored_view.translation.y,
                w.colliders[ball].user_data,
                len(colliders),
                ball in colliders,
                body in bodies,
                sorted(h.index for h, _ in colliders),
                len(list(bodies.handles())),
            ))

    w.event_handler = Handler()
    for _ in range(30):
        w.step()
    assert seen
    ud_ground, ud_ball, y, y_view, ud_world, n, has_ball, has_body, handles, nbodies = seen[0]
    assert (ud_ground, ud_ball, ud_world) == (7, 8, 8)
    assert y == y_view and 0.5 < y < 0.7
    assert (n, has_ball, has_body, handles, nbodies) == (2, True, True, [0, 1], 1)


def test_contact_force_handler_receives_readable_sets(ns):
    w, ground, body, ball = _ball_on_ground_with_events(ns, ns.ActiveEvents.CONTACT_FORCE_EVENTS)
    seen = []

    class Handler:
        def handle_contact_force_event(self, dt, bodies, colliders, pair, magnitude):
            other = pair.collider2 if pair.collider1 == ground else pair.collider1
            seen.append((colliders[other].user_data, bodies[colliders[other].parent].is_dynamic))

    w.event_handler = Handler()
    for _ in range(30):
        w.step()
    assert seen and seen[0] == (8, True)


def test_modifying_or_querying_the_world_from_a_callback_raises(ns):
    w, ground, body, ball = _ball_on_ground_with_events(ns)

    class Handler:
        def __init__(self, action):
            self.action = action

        def handle_collision_event(self, bodies, colliders, event, contact_pair):
            self.action()

    actions = [
        lambda: w.colliders.insert(ns.Collider.ball(1.0)),
        lambda: w.query_pipeline.cast_ray(ns.Ray((0, 5, 0), (0, -1, 0)), 10.0, True),
        lambda: w.update_query_pipeline(),
        lambda: w.step(),
    ]
    for action in actions:
        w, ground, body, ball = _ball_on_ground_with_events(ns)
        w.event_handler = Handler(action)
        with pytest.raises(RuntimeError):
            for _ in range(30):
                w.step()


def test_sets_are_not_lent_outside_callbacks(ns):
    w, ground, body, ball = _ball_on_ground_with_events(ns)
    kept = []

    class Handler:
        def handle_collision_event(self, bodies, colliders, event, contact_pair):
            kept.append(colliders)

    w.event_handler = Handler()
    for _ in range(30):
        w.step()
    assert kept
    # After the step, the sets are ordinary (mutable) sets again.
    kept[0][ball].user_data = 3
    assert w.colliders[ball].user_data == 3


def test_pair_filter_context_sets(ns):
    w, ground, body, ball = _ball_on_ground_with_events(
        ns, hooks=ns.ActiveHooks.FILTER_CONTACT_PAIRS
    )
    seen = []

    class Hooks:
        def filter_contact_pair(self, ctx):
            ud1 = ctx.colliders[ctx.collider1].user_data
            ud2 = ctx.colliders[ctx.collider2].user_data
            seen.append((ud1, ud2, ctx.bodies is w.rigid_bodies))
            return None  # Discard the pair.

    w.physics_hooks = Hooks()
    for _ in range(60):
        w.step()
    assert seen and set(seen[0][:2]) == {7, 8} and seen[0][2]
    # The ball fell through the ground.
    assert w.rigid_bodies[body].translation.y < 0.0


def test_hooks_missing_optional_methods(ns):
    w, ground, body, ball = _ball_on_ground_with_events(
        ns,
        ns.ActiveEvents.COLLISION_EVENTS | ns.ActiveEvents.CONTACT_FORCE_EVENTS,
        ns.ActiveHooks.FILTER_CONTACT_PAIRS | ns.ActiveHooks.MODIFY_SOLVER_CONTACTS,
    )
    modified = [0]

    class OnlyModify:
        def modify_solver_contacts(self, ctx):
            modified[0] += 1

    class NoMethods:
        pass

    w.physics_hooks = OnlyModify()
    w.event_handler = NoMethods()
    for _ in range(30):
        w.step()
    assert modified[0] > 0
    # The pair wasn't filtered out: the ball rests on the ground.
    assert w.rigid_bodies[body].translation.y > 0.5


# ---- contact modification ----------------------------------------------------


def test_set_solver_contact(ns):
    w, ground, body, ball = _ball_on_ground_with_events(
        ns, hooks=ns.ActiveHooks.MODIFY_SOLVER_CONTACTS
    )
    results = []
    contexts = []

    class Hooks:
        def modify_solver_contacts(self, ctx):
            contexts.append(ctx)
            n = ctx.num_solver_contacts()
            with pytest.raises(IndexError):
                ctx.set_solver_contact(n, dist=0.0)
            if n == 0:
                return
            before = ctx.solver_contacts[0]
            ctx.set_solver_contact(0, tangent_velocity=(1.0, 2.0, 3.0), dist=before.dist + 0.25)
            after = ctx.solver_contacts[0]
            # Unset arguments are kept.
            assert after.point == before.point and after.point2 == before.point2
            ctx.set_solver_contact(0, point=(1, 2, 3), point2=(4, 5, 6))
            moved = ctx.solver_contacts[0]
            results.append((after.dist - before.dist, after.tangent_velocity, moved.point, moved.point2))

    w.physics_hooks = Hooks()
    for _ in range(10):
        w.step()
    assert results
    ddist, tv, p1, p2 = results[0]
    assert ddist == pytest.approx(0.25)
    assert (tv.x, tv.y, tv.z) == pytest.approx((1, 2, 3))
    assert (p1.x, p1.y, p1.z) == pytest.approx((1, 2, 3))
    assert (p2.x, p2.y, p2.z) == pytest.approx((4, 5, 6))
    with pytest.raises(RuntimeError):
        contexts[0].set_solver_contact(0, dist=0.0)
    with pytest.raises(RuntimeError):
        contexts[0].set_tangent_velocity((0, 0, 1))


@pytest.mark.parametrize("belt_first", [True, False])
def test_set_tangent_velocity_conveyor(ns, belt_first):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))

    def add_belt():
        return w.add_collider(
            ns.Collider.cuboid(10, 0.1, 10).active_hooks(ns.ActiveHooks.MODIFY_SOLVER_CONTACTS)
        )

    def add_box():
        return w.add_body(
            ns.RigidBody.dynamic(translation=(0, 0.4, 0)),
            colliders=[ns.Collider.cuboid(0.25, 0.25, 0.25)],
        )

    if belt_first:
        belt, box = add_belt(), add_box()
    else:
        box, belt = add_box(), add_belt()
    seen = []

    class Conveyor:
        def modify_solver_contacts(self, ctx):
            # The velocity is the one of collider2's surface relative to collider1's.
            vz = 2.0 if ctx.collider1 == belt else -2.0
            ctx.set_tangent_velocity((0.0, 0.0, vz))
            seen.extend(c.tangent_velocity.z == vz for c in ctx.solver_contacts)

    w.physics_hooks = Conveyor()
    for _ in range(60):
        w.step()
    assert seen and all(seen)
    assert w.rigid_bodies[box].linvel.z > 1.0


# ---- contact graph -------------------------------------------------------------


def test_manifold_solver_contacts_and_world_points(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    ground = w.add_collider(ns.Collider.cuboid(10, 0.1, 10))
    body = w.add_body(
        ns.RigidBody.dynamic(translation=(0.3, 1.0, 0.2)), colliders=[ns.Collider.ball(0.5)]
    )
    ball = w.rigid_bodies[body].colliders[0]
    for _ in range(120):
        w.step()
    pair = w.narrow_phase.contact_pair(ground, ball)
    assert pair is not None and pair.has_any_active_contact
    y = w.rigid_bodies[body].translation.y
    n = 0
    for manifold in pair.manifolds:
        assert len(manifold.data.solver_contacts) == manifold.data.num_active_contacts
        for i, point in enumerate(manifold.points):
            assert point.contact_id == i
        for sc in manifold.data.solver_contacts:
            n += 1
            assert sc.contact_id < len(manifold.points)
            assert (sc.tangent_velocity.x, sc.tangent_velocity.y, sc.tangent_velocity.z) == (0, 0, 0)
            p1, p2 = manifold.data.solver_contact_world_points(sc, w.rigid_bodies)
            # On the ground's top face and on the bottom of the ball.
            assert (p1.x, p1.y, p1.z) == pytest.approx((0.3, 0.1, 0.2), abs=1e-3)
            assert (p2.x, p2.y, p2.z) == pytest.approx((0.3, y - 0.5, 0.2), abs=1e-3)
    assert n > 0
    assert pair.find_deepest_contact().contact_id == 0
    assert "ContactPair(" in repr(pair)
    assert "ContactManifoldData(" in repr(pair.manifolds[0].data)
    assert "SolverContact(" in repr(pair.manifolds[0].data.solver_contacts[0])


def test_contact_and_intersection_pairs_with(ns):
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    ground = w.add_collider(ns.Collider.cuboid(10, 0.1, 10))

    def ball_at(x, y, z, radius=0.5, sensor=False):
        body = w.add_body(
            ns.RigidBody.dynamic(translation=(x, y, z)),
            colliders=[ns.Collider.ball(radius).sensor(sensor)],
        )
        return w.rigid_bodies[body].colliders[0]

    a = ball_at(0, 0.55, 0)
    b = ball_at(5, 0.55, 0)
    sensor = ball_at(0, 0.55, 0, radius=1.0, sensor=True)
    far = ball_at(50, 50, 50)
    w.step()
    pairs = w.narrow_phase.contact_pairs_with(ground)
    others = {p.collider2 if p.collider1 == ground else p.collider1 for p in pairs}
    assert others == {a, b}
    assert w.narrow_phase.contact_pairs_with(far) == []
    inter = w.narrow_phase.intersection_pairs_with(sensor)
    assert any(i and {c1, c2} == {sensor, a} for c1, c2, i in inter)
    assert all(sensor in (c1, c2) for c1, c2, _ in inter)
    assert w.narrow_phase.intersection_pairs_with(far) == []


def test_contact_force_event_repr(ns):
    w = _heavy_ball_on_ground(ns)
    collector = ns.ChannelEventCollector()
    w.event_handler = collector
    for _ in range(60):
        w.step()
    forces = collector.drain_contact_force_events()
    assert forces and repr(forces[0]).startswith("ContactForceEvent(")


def test_hooks_read_sets_from_worker_threads(ns):
    # With several workers, the hooks run on the engine's threads.
    import gc

    # Collect the previous tests' garbage now: unsendable objects freed by a GC pass
    # running on a worker thread would be reported as unraisable errors.
    gc.collect()
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    w.set_num_threads(4)
    w.add_collider(
        ns.Collider.cuboid(50, 0.1, 50)
        .active_hooks(ns.ActiveHooks.FILTER_CONTACT_PAIRS | ns.ActiveHooks.MODIFY_SOLVER_CONTACTS)
        .user_data(1000)
    )
    for i in range(400):
        w.add_body(
            ns.RigidBody.dynamic(translation=((i % 20) * 1.1 - 10, 0.6 + (i // 20) * 1.1, 0)),
            colliders=[ns.Collider.ball(0.5).user_data(i)],
        )
    reads = []

    class Hooks:
        def filter_contact_pair(self, ctx):
            body = ctx.rigid_body1 if ctx.rigid_body1 is not None else ctx.rigid_body2
            reads.append((ctx.colliders[ctx.collider1].user_data, ctx.bodies[body].is_dynamic))
            return ns.SolverFlags.COMPUTE_IMPULSES

        def modify_solver_contacts(self, ctx):
            reads.append((ctx.colliders[ctx.collider2].user_data, True))

    w.physics_hooks = Hooks()
    for _ in range(20):
        w.step()
    assert reads and all(0 <= ud <= 1000 and dynamic for ud, dynamic in reads)


def test_broad_phase_pair_event_flags():
    pair = dim3.ColliderPair(dim3.ColliderHandle.from_raw_parts(0, 0), dim3.ColliderHandle.from_raw_parts(1, 0))
    added = dim3.BroadPhasePairEvent.added(pair)
    removed = dim3.BroadPhasePairEvent.removed(pair)
    assert added.is_added and not added.is_removed
    assert removed.is_removed and not removed.is_added
