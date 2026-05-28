"""2D events & hooks tests (abbreviated)."""

from __future__ import annotations

import pytest

import rapier2d as dim2
import rapier2d_f64 as dim2_f64


@pytest.fixture(params=[dim2, dim2_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


def _two_balls_world(ns):
    w = ns.PhysicsWorld(gravity=(0, 0))
    h1 = w.rigid_bodies.insert(
        ns.RigidBody.dynamic(translation=(-1.0, 0))
        .linvel(ns.Vec2(1.0, 0))
        .build()
    )
    h2 = w.rigid_bodies.insert(
        ns.RigidBody.dynamic(translation=(1.0, 0))
        .linvel(ns.Vec2(-1.0, 0))
        .build()
    )
    c1 = ns.Collider.ball(0.5).active_events(ns.ActiveEvents.COLLISION_EVENTS)
    c2 = ns.Collider.ball(0.5).active_events(ns.ActiveEvents.COLLISION_EVENTS)
    ch1 = w.colliders.insert_with_parent(c1, h1, w.rigid_bodies)
    ch2 = w.colliders.insert_with_parent(c2, h2, w.rigid_bodies)
    return w, ch1, ch2


def test_channel_collector_2d(ns):
    w, ch1, ch2 = _two_balls_world(ns)
    collector = ns.ChannelEventCollector()
    w.event_handler = collector
    for _ in range(60):
        w.step()
    events = collector.drain_collision_events()
    assert any(e.started for e in events)


def test_custom_event_handler_2d(ns):
    received = []

    class MyHandler:
        def handle_collision_event(self, bodies, colliders, event, contact_pair):
            received.append(event)

        def handle_contact_force_event(self, *_a, **_k):
            pass

    w, _, _ = _two_balls_world(ns)
    w.event_handler = MyHandler()
    for _ in range(60):
        w.step()
    assert len(received) > 0


def test_solver_flags_2d(ns):
    assert ns.SolverFlags.COMPUTE_IMPULSES.bits == 1
    assert ns.SolverFlags.empty().is_empty()


def test_physics_hooks_filter_2d(ns):
    w, ch1, ch2 = _two_balls_world(ns)

    class Filter:
        def filter_contact_pair(self, ctx):
            return None

        def filter_intersection_pair(self, ctx):
            return True

        def modify_solver_contacts(self, ctx):
            pass

    w.physics_hooks = Filter()
    c1 = w.colliders[ch1]
    c1.active_hooks = ns.ActiveHooks.FILTER_CONTACT_PAIR
    w.colliders.replace(ch1, c1)
    c2 = w.colliders[ch2]
    c2.active_hooks = ns.ActiveHooks.FILTER_CONTACT_PAIR
    w.colliders.replace(ch2, c2)

    start = w.rigid_bodies[w.colliders[ch1].parent].translation.x
    for _ in range(60):
        w.step()
    end = w.rigid_bodies[w.colliders[ch1].parent].translation.x
    assert end > start + 0.5


def test_event_callback_exception_deferred_2d(ns):
    w, _, _ = _two_balls_world(ns)

    class Bad:
        def handle_collision_event(self, *args, **kwargs):
            raise RuntimeError("boom2d")

        def handle_contact_force_event(self, *args, **kwargs):
            pass

    w.event_handler = Bad()
    raised = False
    for _ in range(120):
        try:
            w.step()
        except RuntimeError as e:
            assert "boom2d" in str(e)
            raised = True
            break
    assert raised
