"""2D geometry tests."""

from __future__ import annotations

import numpy as np
import pytest

import rapier2d as dim2
import rapier2d_f64 as dim2_f64


@pytest.fixture(params=[dim2, dim2_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


def test_collider_ball_2d(ns):
    cs = ns.ColliderSet()
    coll = ns.Collider.ball(0.5).build()
    h = cs.insert(coll)
    fetched = cs[h]
    assert fetched.shape.as_ball().radius == pytest.approx(0.5)


def test_collider_cuboid_2d(ns):
    coll = ns.Collider.cuboid(1.0, 2.0).build()
    cu = coll.shape.as_cuboid()
    assert cu is not None
    assert cu.half_extents.x == pytest.approx(1.0)
    assert cu.half_extents.y == pytest.approx(2.0)


def test_collider_cuboid_2d_kwargs(ns):
    coll = ns.Collider.cuboid(1.0, 2.0, density=3.0, sensor=True).build()
    assert coll.density == 3.0
    assert coll.is_sensor is True


def test_collider_segment_2d(ns):
    coll = ns.Collider.segment((0.0, 0.0), (1.0, 0.0)).build()
    assert coll.shape.shape_type == ns.ShapeType.SEGMENT
    seg = coll.shape.as_segment()
    assert seg is not None


def test_collider_polyline_2d(ns):
    verts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]], dtype=np.float32)
    coll = ns.Collider.polyline(verts).build()
    assert coll.shape.shape_type == ns.ShapeType.POLYLINE
    pl = coll.shape.as_polyline()
    assert pl is not None
    assert pl.num_segments() == 2  # connected pairs by default


def test_collider_convex_polygon_2d(ns):
    pts = np.array(
        [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
        dtype=np.float32,
    )
    coll = ns.Collider.convex_polygon(pts).build()
    cp = coll.shape.as_convex_polygon()
    assert cp is not None
    assert cp.num_points() >= 3


def test_collider_capsule_2d(ns):
    coll = ns.Collider.capsule(0.5, 0.2).build()
    cap = coll.shape.as_capsule()
    assert cap is not None
    assert cap.radius == pytest.approx(0.2)


def test_collider_set_2d(ns):
    cs = ns.ColliderSet()
    h1 = cs.insert(ns.Collider.ball(0.5).build())
    h2 = cs.insert(ns.Collider.cuboid(1, 1).build())
    assert len(cs) == 2
    assert h1 in cs and h2 in cs


def test_interaction_groups_2d(ns):
    g = ns.InteractionGroups(ns.Group.GROUP_0, ns.Group.ALL)
    coll = ns.Collider.ball(0.5).collision_groups(g).build()
    assert coll.collision_groups.memberships == ns.Group.GROUP_0


def test_collider_translation_2d(ns):
    coll = ns.Collider.ball(0.5).translation((3.0, 4.0)).build()
    t = coll.translation
    assert t.x == pytest.approx(3.0) and t.y == pytest.approx(4.0)


def test_shared_shape_cuboid_2d(ns):
    s = ns.SharedShape.cuboid(0.5, 0.5)
    assert s.shape_type == ns.ShapeType.CUBOID


def test_shared_shape_compute_aabb_2d(ns):
    s = ns.SharedShape.ball(1.0)
    aabb = s.compute_aabb(ns.Isometry2.identity())
    assert aabb.mins.x == pytest.approx(-1.0)


def test_active_events_2d(ns):
    e = ns.ActiveEvents.COLLISION_EVENTS
    coll = ns.Collider.ball(0.5).active_events(e).build()
    assert coll.active_events.contains(ns.ActiveEvents.COLLISION_EVENTS)


def test_insert_with_parent_2d(ns):
    bodies = ns.RigidBodySet()
    cs = ns.ColliderSet()
    rb = ns.RigidBody.dynamic(translation=(0.0, 0.0)).build()
    rb_h = bodies.insert(rb)
    ch = cs.insert_with_parent(ns.Collider.ball(0.5).build(), rb_h, bodies)
    assert cs[ch].parent == rb_h
