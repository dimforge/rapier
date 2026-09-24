"""scene query tests (`QueryPipeline`, `QueryFilter`, parry types)."""

from __future__ import annotations

import math
import pytest

import rapier3d as dim3


@pytest.fixture(params=[dim3], ids=["f32"])
def ns(request):
    return request.param


def _stack_world(ns, n=3, gap=2.5):
    """Build a world with `n` static cubes stacked along +Y."""
    w = ns.PhysicsWorld()
    handles = []
    for i in range(n):
        y = (i + 1) * gap
        h = w.add_body(
            ns.RigidBody.fixed(translation=(0, y, 0)),
            colliders=[ns.Collider.cuboid(1, 1, 1)],
        )
        handles.append(h)
    w.update_query_pipeline()
    return w, handles


# ---- Raycasts -------------------------------------------------------------


def test_cast_ray_hits_top_cube(ns):
    w, _ = _stack_world(ns, n=3, gap=3.0)
    origin = ns.Point3(0, 20, 0)
    ray = ns.Ray(origin, (0, -1, 0))
    qp = w.query_pipeline
    hit = qp.cast_ray(ray, 100.0, True)
    assert hit is not None
    handle, toi = hit
    # Top cube top is at y = 9 + 1 = 10 → toi = 20 - 10 = 10
    assert abs(toi - 10.0) < 0.05


def test_cast_ray_misses(ns):
    w, _ = _stack_world(ns)
    ray = ns.Ray(ns.Point3(100, 100, 100), (1, 0, 0))
    hit = w.query_pipeline.cast_ray(ray, 5.0, True)
    assert hit is None


def test_cast_ray_and_get_normal(ns):
    w, _ = _stack_world(ns, n=1)
    ray = ns.Ray(ns.Point3(0, 5, 0), (0, -1, 0))
    hit = w.query_pipeline.cast_ray_and_get_normal(ray, 10.0, True)
    assert hit is not None
    _, ri = hit
    assert hasattr(ri, "toi")
    assert hasattr(ri, "normal")
    assert hasattr(ri, "feature")
    # cube top normal points +Y
    assert ri.normal.y > 0.9


def test_intersect_ray_visits_all_cubes(ns):
    w, handles = _stack_world(ns, n=3, gap=3.0)
    visited = []
    ray = ns.Ray(ns.Point3(0, 20, 0), (0, -1, 0))

    def cb(h, ri):
        visited.append(h)
        return True  # continue

    w.query_pipeline.intersect_ray(ray, 100.0, True, cb)
    assert len(visited) == 3
    for h in handles:
        assert any(w.rigid_bodies[h].colliders[0] == v for v in visited)


def test_intersect_ray_callback_stop(ns):
    w, _ = _stack_world(ns, n=3, gap=3.0)
    visited = []
    ray = ns.Ray(ns.Point3(0, 20, 0), (0, -1, 0))

    def cb(h, ri):
        visited.append(h)
        return False  # stop after first

    w.query_pipeline.intersect_ray(ray, 100.0, True, cb)
    assert len(visited) == 1


# ---- QueryFilter ----------------------------------------------------------


def test_exclude_dynamic_skips_dynamic_bodies(ns):
    w = ns.PhysicsWorld()
    # static ground
    ground = w.add_body(
        ns.RigidBody.fixed(translation=(0, 0, 0)),
        colliders=[ns.Collider.cuboid(10, 0.1, 10)],
    )
    # dynamic above
    w.add_body(
        ns.RigidBody.dynamic(translation=(0, 3, 0)),
        colliders=[ns.Collider.cuboid(1, 1, 1)],
    )
    w.update_query_pipeline()
    qp = w.query_pipeline
    ray = ns.Ray(ns.Point3(0, 10, 0), (0, -1, 0))

    # Without filter: hits the dynamic cube first.
    hit_default = qp.cast_ray(ray, 100.0, True)
    assert hit_default is not None
    _, toi_default = hit_default

    # exclude dynamic → must hit the ground (toi ≈ 9.9).
    f = ns.QueryFilter.exclude_dynamic()
    hit_filtered = qp.cast_ray(ray, 100.0, True, f)
    assert hit_filtered is not None
    _, toi_filtered = hit_filtered
    assert toi_filtered > toi_default + 1.0  # hit the ground much lower


def test_exclude_collider_skips_handle(ns):
    w, handles = _stack_world(ns, n=2, gap=3.0)
    # top cube collider
    top_body = w.rigid_bodies[handles[-1]]
    top_ch = top_body.colliders[0]

    qp = w.query_pipeline
    ray = ns.Ray(ns.Point3(0, 20, 0), (0, -1, 0))

    hit_default = qp.cast_ray(ray, 100.0, True)
    assert hit_default is not None
    handle_default, _ = hit_default
    assert handle_default == top_ch

    f = ns.QueryFilter().exclude_collider(top_ch)
    hit_filtered = qp.cast_ray(ray, 100.0, True, f)
    assert hit_filtered is not None
    handle_filtered, _ = hit_filtered
    assert handle_filtered != top_ch


def test_predicate_filter(ns):
    w, handles = _stack_world(ns, n=2, gap=3.0)
    top_body = w.rigid_bodies[handles[-1]]
    top_ch = top_body.colliders[0]

    qp = w.query_pipeline
    ray = ns.Ray(ns.Point3(0, 20, 0), (0, -1, 0))

    # Predicate returns False for the top collider — should hit the other.
    def keep(h, _co):
        return h != top_ch

    f = ns.QueryFilter().predicate(keep)
    hit = qp.cast_ray(ray, 100.0, True, f)
    assert hit is not None
    handle, _ = hit
    assert handle != top_ch


# ---- Intersection queries -------------------------------------------------


def test_intersect_shape_returns_overlapping(ns):
    w, handles = _stack_world(ns, n=1, gap=2.5)
    # The single cube is at y=2.5 with extents ±1 → AABB y in [1.5, 3.5].
    # A unit-cube probe at y=2.5 must overlap it.
    qp = w.query_pipeline
    found = []

    def cb(h):
        found.append(h)
        return True

    pose = ns.Isometry3.from_translation(0, 2.5, 0)
    qp.intersect_shape(pose, ns.SharedShape.cuboid(0.5, 0.5, 0.5), cb)
    assert len(found) == 1


def test_intersect_point_returns_containing(ns):
    w, handles = _stack_world(ns, n=1, gap=2.5)
    qp = w.query_pipeline
    found = []

    def cb(h):
        found.append(h)
        return True

    qp.intersect_point((0, 2.5, 0), cb)
    assert len(found) == 1


def test_test_aabb(ns):
    w, _ = _stack_world(ns, n=1, gap=2.5)
    qp = w.query_pipeline
    # Probe AABB overlapping the cube (y in [1.5, 3.5])
    overlap = ns.Aabb((-2, 2, -2), (2, 3, 2))
    assert qp.test_aabb(overlap) is True
    # Probe AABB far away
    far = ns.Aabb((100, 100, 100), (101, 101, 101))
    assert qp.test_aabb(far) is False


def test_project_point(ns):
    w, _ = _stack_world(ns, n=1, gap=2.5)
    qp = w.query_pipeline
    # Project a point above the cube (y=5) onto the world.
    res = qp.project_point((0, 5, 0), False)
    assert res is not None
    _, proj = res
    # Closest point on the cube top center is (0, 3.5, 0).
    assert abs(proj.point.y - 3.5) < 0.05


# ---- Shape casts ----------------------------------------------------------


def test_cast_shape_hits_cube(ns):
    w, handles = _stack_world(ns, n=1, gap=2.5)
    qp = w.query_pipeline
    shape = ns.SharedShape.ball(0.5)
    start = ns.Isometry3.from_translation(0, 10, 0)
    vel = (0, -1, 0)
    opts = ns.ShapeCastOptions(max_time_of_impact=20.0)
    hit = qp.cast_shape(start, vel, shape, opts)
    assert hit is not None
    _, sc = hit
    # cube top at y = 3.5; ball radius 0.5 → contact at y_ball = 4.0 → toi = 6
    assert abs(sc.time_of_impact - 6.0) < 0.1


# ---- Ray helper -----------------------------------------------------------


def test_ray_point_at(ns):
    ray = ns.Ray(ns.Point3(1, 2, 3), (1, 0, 0))
    p = ray.point_at(4.0)
    assert p == ns.Point3(5, 2, 3)
    assert ray.origin == ns.Point3(1, 2, 3)
    assert ray.dir == ns.Vec3(1, 0, 0)


# ---- ShapeCastOptions & ShapeCastStatus ----------------------------------


def test_shape_cast_options_kwargs(ns):
    o = ns.ShapeCastOptions(max_time_of_impact=42.0, target_distance=0.1)
    assert o.max_time_of_impact == 42.0
    assert abs(o.target_distance - 0.1) < 1e-6


def test_shape_cast_status_enum(ns):
    assert ns.ShapeCastStatus.CONVERGED == ns.ShapeCastStatus.CONVERGED
    assert ns.ShapeCastStatus.CONVERGED != ns.ShapeCastStatus.FAILED


# ---- QueryFilterFlags -----------------------------------------------------


def test_query_filter_flags(ns):
    f = ns.QueryFilterFlags.EXCLUDE_FIXED | ns.QueryFilterFlags.EXCLUDE_DYNAMIC
    assert ns.QueryFilterFlags.EXCLUDE_FIXED in f
    assert ns.QueryFilterFlags.EXCLUDE_KINEMATIC not in f


# ---- FeatureId ------------------------------------------------------------


def test_feature_id(ns):
    v = ns.FeatureId.Vertex(7)
    assert v.is_vertex
    assert v.id == 7
    f = ns.FeatureId.Face(2)
    assert f.is_face
    u = ns.FeatureId.Unknown()
    assert u.is_unknown


# ---- update / update_query_pipeline --------------------------------------


def test_update_query_pipeline_refreshes_bvh(ns):
    w = ns.PhysicsWorld()
    # Insert collider AFTER the world is built. Without `update_query_pipeline`,
    # the broad-phase BVH wouldn't see it.
    w.add_collider(ns.Collider.cuboid(1, 1, 1).translation((0, 0, 0)))
    w.update_query_pipeline()
    qp = w.query_pipeline
    ray = ns.Ray(ns.Point3(0, 5, 0), (0, -1, 0))
    hit = qp.cast_ray(ray, 100.0, True)
    assert hit is not None


def test_queries_are_fresh_after_step_without_manual_update(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    b = w.add_body(ns.RigidBody.dynamic(translation=(0, 10, 0)), colliders=[ns.Collider.ball(0.5)])
    for _ in range(60):
        w.step()
    y = w.rigid_bodies[b].translation.y
    assert y < 9.0
    hit = w.query_pipeline.cast_ray(ns.Ray((0, 20, 0), (0, -1, 0)), 100.0, True)
    assert hit is not None
    assert hit[1] == pytest.approx(20.0 - (y + 0.5), abs=1e-3)


def test_update_query_pipeline_then_step_after_removal(ns):
    # Consuming the removed colliders in `update_query_pipeline` made the next step panic.
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    w.add_collider(ns.Collider.cuboid(10, 0.1, 10))
    b = w.add_body(ns.RigidBody.dynamic(translation=(0, 0.55, 0)), colliders=[ns.Collider.ball(0.5)])
    for _ in range(5):
        w.step()
    assert len(w.narrow_phase.contact_pairs()) == 1
    w.remove_collider(w.rigid_bodies[b].colliders[0])
    w.update_query_pipeline()
    w.query_pipeline.update(w.rigid_bodies, w.colliders)
    w.step()
    assert len(w.narrow_phase.contact_pairs()) == 0
    hit = w.query_pipeline.cast_ray(ns.Ray((0, 5, 0), (0, -1, 0)), 100.0, True)
    assert hit is not None and hit[1] == pytest.approx(4.9, abs=1e-4)


def test_update_query_pipeline_keeps_changes_for_the_step(ns):
    # The step still sees the colliders added or moved before a manual refresh.
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    body = w.add_body(ns.RigidBody.dynamic(translation=(0, 0.55, 0)), colliders=[ns.Collider.ball(0.5)])
    ball = w.rigid_bodies[body].colliders[0]
    w.step()
    ground = w.add_collider(ns.Collider.cuboid(10, 0.1, 10).translation((20, 0, 0)))
    w.update_query_pipeline()
    hit = w.query_pipeline.cast_ray(ns.Ray((20, 5, 0), (0, -1, 0)), 100.0, True)
    assert hit is not None and hit[0] == ground
    w.colliders[ground].translation = (0, 0, 0)
    w.update_query_pipeline()
    hit = w.query_pipeline.cast_ray(ns.Ray((20, 5, 0), (0, -1, 0)), 100.0, True)
    assert hit is None
    w.step()
    assert w.narrow_phase.contact_pair(ground, ball) is not None


def test_update_query_pipeline_sees_moved_bodies(ns):
    w = ns.PhysicsWorld()
    b = w.add_body(ns.RigidBody.fixed(), colliders=[ns.Collider.cuboid(1, 1, 1)])
    w.step()
    w.rigid_bodies[b].translation = (10, 0, 0)
    w.update_query_pipeline()
    ray = ns.Ray((10, 5, 0), (0, -1, 0))
    assert w.query_pipeline.cast_ray(ray, 100.0, True) is not None
    w.step()
    assert w.query_pipeline.cast_ray(ray, 100.0, True) is not None


def test_auto_update_query_is_accepted(ns):
    w = ns.PhysicsWorld(auto_update_query=True)
    assert w.auto_update_query
    w.add_collider(ns.Collider.cuboid(1, 1, 1))
    w.step()
    assert w.query_pipeline.cast_ray(ns.Ray((0, 5, 0), (0, -1, 0)), 100.0, True) is not None


def test_project_point_max_dist(ns):
    w, handles = _stack_world(ns, n=1, gap=3.0)
    qp = w.query_pipeline
    # The cube spans y in [2, 4]: the origin is 2 away from it.
    hit = qp.project_point((0, 0, 0), True)
    assert hit is not None
    assert hit[1].point.y == pytest.approx(2.0)
    assert qp.project_point((0, 0, 0), True, max_dist=1.0) is None
    hit = qp.project_point((0, 0, 0), True, filter=ns.QueryFilter(), max_dist=2.5)
    assert hit is not None and not hit[1].is_inside


def test_query_filter_getters(ns):
    groups = ns.InteractionGroups(ns.Group.GROUP_1, ns.Group.GROUP_2)
    pred = lambda h, c: True  # noqa: E731
    f = ns.QueryFilter.exclude_dynamic().groups(groups).predicate(pred)
    assert f.interaction_groups.memberships == ns.Group.GROUP_1
    assert f.interaction_groups.filter == ns.Group.GROUP_2
    assert f.predicate_fn is pred
    assert ns.QueryFilterFlags.EXCLUDE_DYNAMIC in f.flags
    assert ns.QueryFilter().interaction_groups is None
    assert ns.QueryFilter().predicate_fn is None


def test_shape_cast_hit_frames(ns):
    # witness1/normal1: hit collider, world space; witness2/normal2: cast shape, local space.
    w, _ = _stack_world(ns, n=1, gap=3.0)
    shape = ns.SharedShape.ball(0.5)
    pos = ns.Isometry3(translation=(0.0, 10.0, 0.0))
    hit = w.query_pipeline.cast_shape(pos, (0, -1, 0), shape, ns.ShapeCastOptions())
    assert hit is not None
    _, hit = hit
    assert hit.time_of_impact == pytest.approx(5.5, abs=1e-3)
    assert (hit.witness1.x, hit.witness1.y, hit.witness1.z) == pytest.approx((0, 4, 0), abs=1e-3)
    assert (hit.normal1.x, hit.normal1.y, hit.normal1.z) == pytest.approx((0, 1, 0), abs=1e-3)
    assert (hit.witness2.x, hit.witness2.y, hit.witness2.z) == pytest.approx((0, -0.5, 0), abs=1e-3)
    assert (hit.normal2.x, hit.normal2.y, hit.normal2.z) == pytest.approx((0, -1, 0), abs=1e-3)
