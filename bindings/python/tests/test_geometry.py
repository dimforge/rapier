"""3D geometry tests (f32)."""

from __future__ import annotations

import math as pymath

import numpy as np
import pytest

import rapier3d as rapier
import rapier3d as dim3


@pytest.fixture(params=[dim3], ids=["f32"])
def ns(request):
    return request.param


# ---- ColliderHandle ------------------------------------------------------


def test_collider_handle_round_trip(ns):
    h1 = ns.ColliderHandle.from_raw_parts(3, 7)
    h2 = ns.ColliderHandle.from_raw_parts(3, 7)
    h3 = ns.ColliderHandle.from_raw_parts(3, 8)
    assert h1 == h2
    assert h1 != h3
    assert hash(h1) == hash(h2)
    assert h1.index == 3 and h1.generation == 7
    assert "index=3" in repr(h1)


def test_collider_handle_invalid(ns):
    inv = ns.ColliderHandle.invalid()
    # Two invalid handles compare equal.
    assert inv == ns.ColliderHandle.invalid()


# ---- Ball collider ------------------------------------------------------


def test_collider_ball_basic(ns):
    cs = ns.ColliderSet()
    builder = ns.Collider.ball(0.5)
    coll = builder.build()
    assert coll.shape.shape_type == ns.ShapeType.BALL
    h = cs.insert(coll)
    assert h in cs
    fetched = cs[h]
    ball = fetched.shape.as_ball()
    assert ball is not None
    assert abs(ball.radius - 0.5) < 1e-5


def test_collider_ball_kwargs(ns):
    coll = ns.Collider.ball(0.7, density=2.0, friction=0.3, sensor=True).build()
    assert coll.density == 2.0
    assert abs(coll.friction - 0.3) < 1e-5
    assert coll.is_sensor is True


# ---- Cuboid collider ----------------------------------------------------


def test_collider_cuboid_three_arg(ns):
    coll = ns.Collider.cuboid(1.0, 2.0, 3.0).build()
    cu = coll.shape.as_cuboid()
    assert cu is not None
    he = cu.half_extents
    assert abs(he.x - 1.0) < 1e-5
    assert abs(he.y - 2.0) < 1e-5
    assert abs(he.z - 3.0) < 1e-5


def test_collider_cuboid_kwargs(ns):
    coll = ns.Collider.cuboid(1, 2, 3, density=4.0, sensor=True).build()
    assert coll.density == 4.0
    assert coll.is_sensor is True


# ---- Capsule / Cylinder / Cone ------------------------------------------


def test_collider_capsule(ns):
    coll = ns.Collider.capsule(0.5, 0.2).build()
    cap = coll.shape.as_capsule()
    assert cap is not None
    assert abs(cap.radius - 0.2) < 1e-5
    assert abs(cap.half_height - 0.5) < 1e-5


def test_collider_cylinder(ns):
    coll = ns.Collider.cylinder(0.5, 0.3).build()
    cy = coll.shape.as_cylinder()
    assert cy is not None
    assert abs(cy.half_height - 0.5) < 1e-5
    assert abs(cy.radius - 0.3) < 1e-5


def test_collider_cone(ns):
    coll = ns.Collider.cone(0.5, 0.3).build()
    co = coll.shape.as_cone()
    assert co is not None
    assert abs(co.half_height - 0.5) < 1e-5


# ---- TriMesh / ConvexHull / Compound ------------------------------------


def test_collider_trimesh(ns):
    # Build a simple tetrahedron mesh.
    verts = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    idx = np.array(
        [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
        dtype=np.uint32,
    )
    coll = ns.Collider.trimesh(verts, idx).build()
    tm = coll.shape.as_trimesh()
    assert tm is not None
    assert tm.num_vertices() == 4
    assert tm.num_triangles() == 4


def test_collider_convex_hull(ns):
    verts = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    coll = ns.Collider.convex_hull(verts).build()
    assert coll.shape.shape_type == ns.ShapeType.CONVEX_POLYHEDRON


def test_collider_compound(ns):
    ball = ns.SharedShape.ball(0.5)
    iso = ns.Isometry3.identity()
    coll = ns.Collider.compound([(iso, ball), (iso, ball)]).build()
    cmp = coll.shape.as_compound()
    assert cmp is not None
    assert cmp.num_shapes() == 2


# ---- ColliderBuilder modifiers ------------------------------------------


def test_collider_builder_density_chained(ns):
    coll = ns.Collider.ball(1.0).density(2.0).build()
    assert coll.density == 2.0


def test_collider_builder_friction_restitution(ns):
    coll = (
        ns.Collider.ball(1.0)
        .friction(0.4)
        .restitution(0.9)
        .build()
    )
    assert abs(coll.friction - 0.4) < 1e-5
    assert abs(coll.restitution - 0.9) < 1e-5


def test_collider_builder_translation(ns):
    coll = ns.Collider.ball(1.0).translation((1.0, 2.0, 3.0)).build()
    t = coll.translation
    assert abs(t.x - 1.0) < 1e-5
    assert abs(t.y - 2.0) < 1e-5
    assert abs(t.z - 3.0) < 1e-5


# ---- ColliderSet --------------------------------------------------------


def test_collider_set_insert_get_iter(ns):
    cs = ns.ColliderSet()
    h1 = cs.insert(ns.Collider.ball(0.5).build())
    h2 = cs.insert(ns.Collider.cuboid(1, 1, 1).build())
    assert len(cs) == 2
    assert h1 in cs and h2 in cs
    pairs = list(cs)
    assert len(pairs) == 2
    handles = list(cs.handles())
    assert h1 in handles


def test_collider_set_invalid_handle_raises(ns):
    cs = ns.ColliderSet()
    inv = ns.ColliderHandle.invalid()
    with pytest.raises(ns.InvalidHandle):
        _ = cs[inv]
    assert cs.get(inv) is None


def test_collider_set_insert_with_parent(ns):
    bodies = ns.RigidBodySet()
    cs = ns.ColliderSet()
    rb = ns.RigidBody.dynamic(translation=(0.0, 0.0, 0.0)).build()
    rb_h = bodies.insert(rb)
    coll = ns.Collider.ball(0.5).build()
    ch = cs.insert_with_parent(coll, rb_h, bodies)
    assert ch in cs
    fetched = cs[ch]
    assert fetched.parent == rb_h
    # rigid body's collider tuple should contain the new handle
    body = bodies[rb_h]
    assert ch in body.colliders


# ---- InteractionGroups --------------------------------------------------


def test_interaction_groups_round_trip(ns):
    g = ns.InteractionGroups(ns.Group.GROUP_0, ns.Group.GROUP_0 | ns.Group.GROUP_1)
    coll = ns.Collider.ball(0.5).collision_groups(g).build()
    rt = coll.collision_groups
    assert rt.memberships == ns.Group.GROUP_0
    assert rt.filter == (ns.Group.GROUP_0 | ns.Group.GROUP_1)


def test_interaction_groups_all_none(ns):
    a = ns.InteractionGroups.all()
    n = ns.InteractionGroups.none()
    assert a != n
    assert a.test(a)


def test_group_ops(ns):
    g0 = ns.Group.GROUP_0
    g1 = ns.Group.GROUP_1
    combined = g0 | g1
    assert combined.contains(g0) and combined.contains(g1)
    inter = combined & g0
    assert inter == g0


# ---- MeshConverter ------------------------------------------------------


def test_mesh_converter_trimesh(ns):
    verts = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    idx = np.array(
        [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
        dtype=np.uint32,
    )
    conv = ns.MeshConverter.TRIMESH
    shape, _iso = conv.build(verts, idx)
    assert shape.shape_type == ns.ShapeType.TRIMESH


def test_mesh_converter_zero_vertices_raises(ns):
    verts = np.zeros((0, 3), dtype=np.float32)
    idx = np.zeros((0, 3), dtype=np.uint32)
    conv = ns.MeshConverter.TRIMESH
    with pytest.raises(ns.MeshConversionError):
        conv.build(verts, idx)


def test_mesh_converter_obb(ns):
    verts = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    idx = np.zeros((0, 3), dtype=np.uint32)
    conv = ns.MeshConverter.OBB
    shape, _iso = conv.build(verts, idx)
    assert shape.shape_type == ns.ShapeType.CUBOID


# ---- SharedShape direct constructors ------------------------------------


def test_shared_shape_ball(ns):
    s = ns.SharedShape.ball(0.7)
    assert s.shape_type == ns.ShapeType.BALL
    assert s.as_ball().radius == pytest.approx(0.7)


def test_shared_shape_cuboid(ns):
    s = ns.SharedShape.cuboid(1.0, 2.0, 3.0)
    cu = s.as_cuboid()
    assert cu is not None
    assert cu.half_extents.x == pytest.approx(1.0)
    assert cu.half_extents.z == pytest.approx(3.0)


def test_shared_shape_compute_aabb(ns):
    s = ns.SharedShape.ball(1.0)
    aabb = s.compute_aabb(ns.Isometry3.identity())
    assert aabb.mins.x == pytest.approx(-1.0)
    assert aabb.maxs.z == pytest.approx(1.0)


def test_shared_shape_compute_mass_properties(ns):
    s = ns.SharedShape.ball(1.0)
    mp = s.compute_mass_properties(1.0)
    # mass of a unit-radius unit-density ball ≈ 4/3*pi
    expected = (4.0 / 3.0) * pymath.pi
    assert abs(mp.mass - expected) < 0.05


# ---- Bitflags -----------------------------------------------------------


def test_active_events(ns):
    e = ns.ActiveEvents.COLLISION_EVENTS | ns.ActiveEvents.CONTACT_FORCE_EVENTS
    assert e.contains(ns.ActiveEvents.COLLISION_EVENTS)
    coll = ns.Collider.ball(0.5).active_events(e).build()
    assert coll.active_events.contains(ns.ActiveEvents.COLLISION_EVENTS)


def test_active_hooks(ns):
    h = ns.ActiveHooks.FILTER_CONTACT_PAIR | ns.ActiveHooks.MODIFY_SOLVER_CONTACTS
    coll = ns.Collider.ball(0.5).active_hooks(h).build()
    assert coll.active_hooks.contains(ns.ActiveHooks.FILTER_CONTACT_PAIR)


def test_trimesh_flags(ns):
    f = ns.TriMeshFlags.MERGE_DUPLICATE_VERTICES
    assert not f.is_empty()
    # round-trip into TriMesh via SharedShape
    verts = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    idx = np.array(
        [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
        dtype=np.uint32,
    )
    s = ns.SharedShape.trimesh(verts, idx, f)
    assert s.shape_type == ns.ShapeType.TRIMESH


# ---- BroadPhase / NarrowPhase smoke -------------------------------------


def test_broad_phase_constructible(ns):
    bp = ns.BroadPhaseBvh()
    assert bp is not None
    bp.clear()


def test_broad_phase_optimized(ns):
    bp = ns.BroadPhaseBvh.optimized_for(ns.BvhOptimizationStrategy.AUTO)
    assert bp is not None


def test_narrow_phase_empty(ns):
    np_ = ns.NarrowPhase()
    assert list(np_.contact_pairs()) == []
    h = ns.ColliderHandle.invalid()
    assert np_.contact_pair(h, h) is None


# ---- ColliderMaterial -------------------------------------------------


def test_collider_material(ns):
    mat = ns.ColliderMaterial(0.4, 0.7)
    assert abs(mat.friction - 0.4) < 1e-5
    assert abs(mat.restitution - 0.7) < 1e-5
    mat.friction = 0.5
    assert abs(mat.friction - 0.5) < 1e-5
