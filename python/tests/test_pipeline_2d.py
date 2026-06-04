"""2D analogue of `test_pipeline.py` / `test_queries.py`."""

from __future__ import annotations

import threading

import pytest

import rapier2d as dim2
import rapier2d_f64 as dim2_f64


@pytest.fixture(params=[dim2, dim2_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


def test_world_default(ns):
    w = ns.PhysicsWorld()
    assert len(w.rigid_bodies) == 0
    assert w.gravity == ns.Vec2(0, 0)


def test_world_shared_subsets(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81))
    assert w.rigid_bodies is w.rigid_bodies
    assert w.colliders is w.colliders


def test_world_step_drops_ball_2d(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81))
    w.colliders.insert(ns.Collider.cuboid(50, 0.1).build())
    h = w.add_body(
        ns.RigidBody.dynamic(translation=(0, 5)),
        colliders=[ns.Collider.ball(0.5)],
    )
    for _ in range(120):
        w.step()
    body = w.rigid_bodies[h]
    assert body.translation.y < 5.0
    assert body.translation.y > -1.0


def test_world_gil_release_2d(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81))
    w.colliders.insert(ns.Collider.cuboid(50, 0.1).build())
    h = w.add_body(
        ns.RigidBody.dynamic(translation=(0, 5)),
        colliders=[ns.Collider.ball(0.5)],
    )

    counter = [0]

    def bump():
        for _ in range(50):
            counter[0] += 1

    t = threading.Thread(target=bump)
    t.start()
    for _ in range(60):
        w.step()
    t.join()
    assert counter[0] == 50


def test_cast_ray_2d(ns):
    w = ns.PhysicsWorld()
    w.add_body(
        ns.RigidBody.fixed(translation=(0, 0)),
        colliders=[ns.Collider.cuboid(10, 0.5)],
    )
    w.update_query_pipeline()
    ray = ns.Ray(ns.Point2(0, 10), (0, -1))
    hit = w.query_pipeline.cast_ray(ray, 100.0, True)
    assert hit is not None
    _, toi = hit
    # Ground top at y = 0.5 → toi = 9.5
    assert abs(toi - 9.5) < 0.05


def test_intersect_shape_2d(ns):
    w = ns.PhysicsWorld()
    w.add_collider(ns.Collider.cuboid(1, 1).translation((0, 0)))
    w.update_query_pipeline()
    qp = w.query_pipeline
    found = []
    qp.intersect_shape(
        ns.Isometry2.from_translation(0, 0),
        ns.SharedShape.cuboid(0.5, 0.5),
        lambda h: found.append(h) or True,
    )
    assert len(found) == 1


def test_test_aabb_2d(ns):
    w = ns.PhysicsWorld()
    w.add_collider(ns.Collider.cuboid(1, 1).translation((0, 0)))
    w.update_query_pipeline()
    qp = w.query_pipeline
    assert qp.test_aabb(ns.Aabb((-2, -2), (2, 2))) is True
    assert qp.test_aabb(ns.Aabb((100, 100), (101, 101))) is False


def test_query_filter_2d(ns):
    f = ns.QueryFilter.exclude_dynamic()
    assert f is not None


def test_counters_2d(ns):
    c = ns.Counters()
    c.enable()
    assert c.enabled
