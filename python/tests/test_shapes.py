"""Shape view introspection tests.

Exercises the `SharedShape.as_*` casts across every concrete shape
constructed via `Collider.*` or `SharedShape.*`.
"""

from __future__ import annotations

import numpy as np
import pytest

import rapier3d as dim3
import rapier3d_f64 as dim3_f64


@pytest.fixture(params=[dim3, dim3_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


def test_as_ball(ns):
    s = ns.SharedShape.ball(0.42)
    assert s.shape_type == ns.ShapeType.BALL
    assert s.as_ball() is not None
    assert s.as_cuboid() is None
    assert s.as_ball().radius == pytest.approx(0.42)


def test_as_cuboid(ns):
    s = ns.SharedShape.cuboid(0.1, 0.2, 0.3)
    cu = s.as_cuboid()
    assert cu is not None
    he = cu.half_extents
    assert he.x == pytest.approx(0.1)
    assert he.y == pytest.approx(0.2)
    assert he.z == pytest.approx(0.3)


def test_as_capsule(ns):
    s = ns.SharedShape.capsule_y(0.5, 0.2)
    cap = s.as_capsule()
    assert cap is not None
    assert cap.radius == pytest.approx(0.2)
    assert cap.half_height == pytest.approx(0.5)


def test_as_cylinder(ns):
    s = ns.SharedShape.cylinder(0.5, 0.3)
    cy = s.as_cylinder()
    assert cy is not None
    assert cy.half_height == pytest.approx(0.5)
    assert cy.radius == pytest.approx(0.3)


def test_as_cone(ns):
    s = ns.SharedShape.cone(0.5, 0.3)
    co = s.as_cone()
    assert co is not None
    assert co.half_height == pytest.approx(0.5)
    assert co.radius == pytest.approx(0.3)


def test_as_trimesh(ns):
    verts = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    idx = np.array(
        [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
        dtype=np.uint32,
    )
    s = ns.SharedShape.trimesh(verts, idx)
    tm = s.as_trimesh()
    assert tm is not None
    assert tm.num_vertices() == 4
    assert tm.num_triangles() == 4


def test_as_convex_polyhedron(ns):
    verts = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.5, 0.5, 0.5],
        ],
        dtype=np.float32,
    )
    s = ns.SharedShape.convex_hull(verts)
    cp = s.as_convex_polyhedron()
    assert cp is not None
    assert cp.num_points() >= 4

    # Triangulated face topology (used by the testbed to build a real mesh
    # rather than falling back to a bounding box). Indices must form
    # triangles referencing valid points, with enough faces to enclose a
    # solid (a tetrahedron is the minimum: 4 triangles).
    pts = cp.points
    idx = cp.indices
    assert idx.ndim == 2 and idx.shape[1] == 3
    assert idx.shape[0] >= 4
    assert idx.min() >= 0
    assert idx.max() < pts.shape[0]


def test_as_compound(ns):
    ball = ns.SharedShape.ball(0.5)
    parts = [(ns.Isometry3.identity(), ball), (ns.Isometry3.identity(), ball)]
    s = ns.SharedShape.compound(parts)
    cmp = s.as_compound()
    assert cmp is not None
    assert cmp.num_shapes() == 2


def test_as_voxels(ns):
    # A flat 4x4 grid of points -> 16 filled voxels of size 1.
    pts = np.array(
        [[i, 0.0, j] for i in range(4) for j in range(4)], dtype=np.float32
    )
    s = ns.SharedShape.voxels_from_points((1.0, 1.0, 1.0), pts)
    assert s.shape_type == ns.ShapeType.VOXELS
    vx = s.as_voxels()
    assert vx is not None
    # voxel_size echoes what we passed.
    np.testing.assert_allclose(np.asarray(vx.voxel_size), [1.0, 1.0, 1.0])
    # 16 filled voxels, each center is (D,)-dimensional. The testbed renders
    # one cuboid per center, so these must be present and well-shaped.
    centers = vx.centers
    assert vx.num_voxels() == 16
    assert centers.shape == (16, 3)


def test_round_cuboid(ns):
    s = ns.SharedShape.round_cuboid(0.5, 0.5, 0.5, 0.1)
    assert s.shape_type == ns.ShapeType.ROUND_CUBOID


def test_round_cylinder(ns):
    s = ns.SharedShape.round_cylinder(0.5, 0.3, 0.05)
    assert s.shape_type == ns.ShapeType.ROUND_CYLINDER


def test_round_cone(ns):
    s = ns.SharedShape.round_cone(0.5, 0.3, 0.05)
    assert s.shape_type == ns.ShapeType.ROUND_CONE


def test_compute_mass_properties_cuboid(ns):
    s = ns.SharedShape.cuboid(0.5, 0.5, 0.5)
    mp = s.compute_mass_properties(1.0)
    # 1×1×1 cube with density 1 has mass 1.
    assert mp.mass == pytest.approx(1.0, rel=1e-3)


def test_compute_bounding_sphere(ns):
    s = ns.SharedShape.ball(0.5)
    bs = s.compute_bounding_sphere(ns.Isometry3.identity())
    assert bs.radius == pytest.approx(0.5)


def test_shape_type_repr(ns):
    s = ns.SharedShape.ball(1.0)
    r = repr(s)
    assert "SharedShape" in r
