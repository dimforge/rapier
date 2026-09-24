"""Rigid-body and collider API tests: kinematic targets, fast rotation, forces, parent-relative
collider poses, polylines, voxelized meshes, heightfield layout and input conversions."""

from __future__ import annotations

import math
import warnings

import numpy as np
import pytest

import rapier3d as rp


def _close(a, b, tol=1.0e-5):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def _sleeping_body_world():
    w = rp.PhysicsWorld()
    h = w.add_body(rp.RigidBody.dynamic(sleeping=True), colliders=[rp.Collider.ball(0.5)])
    assert w.rigid_bodies[h].is_sleeping
    return w, h


# ---- Kinematic targets ------------------------------------------------------


def test_set_next_kinematic_translation():
    w = rp.PhysicsWorld()
    h = w.add_body(rp.RigidBody.kinematic_position_based(translation=(0.0, 1.0, 0.0)))
    body = w.rigid_bodies[h]
    body.set_next_kinematic_translation((0.0, 1.5, 0.0))
    # Only the target changes until the next step.
    assert body.next_position.translation == rp.Vec3(0.0, 1.5, 0.0)
    assert body.translation == rp.Vec3(0.0, 1.0, 0.0)
    w.step()
    assert _close(body.translation, (0.0, 1.5, 0.0))
    # The velocity was deduced from the target.
    assert body.linvel.y > 0.0


def test_set_next_kinematic_rotation_and_position():
    w = rp.PhysicsWorld()
    h = w.add_body(rp.RigidBody.kinematic_position_based())
    body = w.rigid_bodies[h]
    rot = rp.Rotation3.from_scaled_axis((0.0, 0.3, 0.0))
    body.set_next_kinematic_rotation(rot)
    w.step()
    assert _close(body.rotation.to_tuple(), rot.to_tuple())

    pose = rp.Isometry3((1.0, 2.0, 3.0), rp.Rotation3.from_scaled_axis((0.2, 0.0, 0.0)))
    body.set_next_kinematic_position(pose)
    w.step()
    assert _close(body.translation, (1.0, 2.0, 3.0))
    assert _close(body.rotation.to_tuple(), pose.rotation.to_tuple())


def test_set_next_kinematic_translation_ignored_by_dynamic_bodies():
    w = rp.PhysicsWorld()
    h = w.add_body(rp.RigidBody.dynamic(), colliders=[rp.Collider.ball(0.5)])
    body = w.rigid_bodies[h]
    body.set_next_kinematic_translation((5.0, 0.0, 0.0))
    assert body.next_position.translation == rp.Vec3(0.0, 0.0, 0.0)


# ---- Fast rotation -----------------------------------------------------------


def test_allow_fast_rotation_builder_kwarg_and_property():
    assert not rp.RigidBody.dynamic().build().allow_fast_rotation
    assert rp.RigidBody.dynamic().allow_fast_rotation(True).build().allow_fast_rotation
    assert rp.RigidBody.dynamic(allow_fast_rotation=True).build().allow_fast_rotation

    w = rp.PhysicsWorld()
    h = w.add_body(rp.RigidBody.dynamic())
    w.rigid_bodies[h].allow_fast_rotation = True
    assert w.rigid_bodies[h].allow_fast_rotation


@pytest.mark.parametrize("allow", [False, True])
def test_allow_fast_rotation_lifts_the_angular_speed_cap(allow):
    w = rp.PhysicsWorld()
    h = w.add_body(
        rp.RigidBody.dynamic(angvel=(0.0, 1000.0, 0.0), allow_fast_rotation=allow),
        colliders=[rp.Collider.ball(0.5)],
    )
    w.step()
    spin = w.rigid_bodies[h].angvel.y
    if allow:
        assert spin == pytest.approx(1000.0, rel=1.0e-4)
    else:
        assert spin < 100.0


def test_builder_additional_pgs_iterations():
    body = rp.RigidBody.dynamic().additional_pgs_iterations(3).build()
    assert body.additional_pgs_iterations == 3
    assert rp.RigidBody.dynamic(additional_pgs_iterations=2).build().additional_pgs_iterations == 2


# ---- Forces -------------------------------------------------------------------


def test_user_forces_persist_until_reset():
    w = rp.PhysicsWorld()
    h = w.add_body(rp.RigidBody.dynamic(), colliders=[rp.Collider.ball(0.5)])
    body = w.rigid_bodies[h]
    body.add_force((body.mass, 0.0, 0.0))
    body.add_torque((0.0, 0.0, 1.0))
    w.step()
    v1 = body.linvel.x
    w.step()
    v2 = body.linvel.x
    assert v1 > 0.0
    assert v2 == pytest.approx(2.0 * v1, rel=1.0e-3)
    assert body.user_force.x == pytest.approx(body.mass)
    assert body.user_torque.z == pytest.approx(1.0)
    body.reset_forces()
    body.reset_torques()
    assert body.user_force == rp.Vec3(0.0, 0.0, 0.0)
    assert body.user_torque == rp.Vec3(0.0, 0.0, 0.0)


def test_add_gravitational_force_wake_up():
    w, h = _sleeping_body_world()
    body = w.rigid_bodies[h]
    body.add_gravitational_force((0.0, -9.81, 0.0), wake_up=False)
    assert body.is_sleeping
    assert body.user_force.y == pytest.approx(-9.81 * body.mass)
    body.reset_forces(wake_up=False)
    body.add_gravitational_force((0.0, -9.81, 0.0))
    assert not body.is_sleeping


# ---- Enums --------------------------------------------------------------------


@pytest.mark.parametrize(
    "member",
    [
        rp.RigidBodyType.KINEMATIC_POSITION_BASED,
        rp.CoefficientCombineRule.GEOMETRIC_MEAN,
        rp.FrictionModel.COULOMB,
        rp.InteractionTestMode.OR,
        rp.ShapeType.BALL,
    ],
)
def test_enum_members_are_hashable_ints(member):
    assert {member: 1}[member] == 1
    assert member == int(member)
    with pytest.raises(TypeError):
        iter(type(member))


def test_all_combine_rules_are_usable():
    for rule in [
        rp.CoefficientCombineRule.AVERAGE,
        rp.CoefficientCombineRule.MIN,
        rp.CoefficientCombineRule.MULTIPLY,
        rp.CoefficientCombineRule.MAX,
        rp.CoefficientCombineRule.CLAMPED_SUM,
        rp.CoefficientCombineRule.GEOMETRIC_MEAN,
    ]:
        c = rp.Collider.ball(0.5, friction_combine_rule=rule).build()
        assert c.friction_combine_rule == rule


def test_interaction_test_mode_round_trips():
    for mode in [rp.InteractionTestMode.AND, rp.InteractionTestMode.OR]:
        g = rp.InteractionGroups(test_mode=mode)
        assert g.test_mode == mode
    assert rp.InteractionGroups().test_mode == rp.InteractionTestMode.AND
    # Deprecated aliases.
    assert rp.InteractionTestMode.DEFAULT == rp.InteractionTestMode.AND
    assert rp.InteractionTestMode.ONLY_DYNAMIC == rp.InteractionTestMode.AND
    g = rp.InteractionGroups(test_mode=rp.InteractionTestMode.DEFAULT)
    assert g.test_mode == rp.InteractionTestMode.AND
    assert "test_mode=AND" in repr(g)


# ---- Collider poses relative to the parent -----------------------------------


def test_position_wrt_parent_without_parent():
    c = rp.Collider.ball(0.5).build()
    assert c.position_wrt_parent is None
    assert c.translation_wrt_parent is None
    assert c.rotation_wrt_parent is None
    # No parent: setting does nothing.
    c.position_wrt_parent = rp.Isometry3.from_translation(1.0, 2.0, 3.0)
    assert c.position_wrt_parent is None


def test_position_wrt_parent_with_parent():
    w = rp.PhysicsWorld()
    b = w.add_body(rp.RigidBody.dynamic(translation=(10.0, 0.0, 0.0)))
    h = w.add_collider(rp.Collider.ball(0.5).translation((0.0, 1.0, 0.0)), parent=b)
    c = w.colliders[h]
    assert c.position_wrt_parent.translation == rp.Vec3(0.0, 1.0, 0.0)

    c.position_wrt_parent = rp.Isometry3.from_translation(1.0, 2.0, 3.0)
    assert c.position_wrt_parent.translation == (1.0, 2.0, 3.0)
    assert c.translation_wrt_parent == rp.Vec3(1.0, 2.0, 3.0)

    c.translation_wrt_parent = (0.0, 0.0, 2.0)
    rot = rp.Rotation3.from_scaled_axis((0.0, 0.5, 0.0))
    c.rotation_wrt_parent = rot
    assert c.translation_wrt_parent == rp.Vec3(0.0, 0.0, 2.0)
    assert _close(c.rotation_wrt_parent.to_tuple(), rot.to_tuple())

    # The world-space pose follows at the next step.
    w.step()
    assert _close(c.translation, (10.0, 0.0, 2.0), tol=1.0e-3)


# ---- Shapes ---------------------------------------------------------------------


def test_segment():
    shape = rp.SharedShape.segment((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert shape.shape_type == rp.ShapeType.SEGMENT
    c = rp.Collider.segment((0.0, 0.0, 0.0), (0.0, 2.0, 0.0), friction=0.25).build()
    assert c.shape.shape_type == rp.ShapeType.SEGMENT
    assert c.friction == pytest.approx(0.25)


def test_polyline():
    vertices = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]])
    strip = rp.SharedShape.polyline(vertices)
    assert strip.shape_type == rp.ShapeType.POLYLINE
    closed = rp.Collider.polyline(
        vertices, np.array([[0, 1], [1, 2], [2, 3], [3, 0]], dtype=np.uint32), sensor=True
    ).build()
    assert closed.shape.shape_type == rp.ShapeType.POLYLINE
    assert closed.is_sensor
    # Nested lists are accepted too.
    rp.SharedShape.polyline(vertices.tolist(), [[0, 1], [1, 2]])
    with pytest.raises(ValueError):
        rp.SharedShape.polyline(vertices, [[0, 4]])
    with pytest.raises(ValueError):
        rp.SharedShape.polyline(vertices, [[0, 1, 2]])


def _cube_mesh(half=0.5):
    h = half
    vertices = np.array(
        [[-h, -h, -h], [h, -h, -h], [h, h, -h], [-h, h, -h], [-h, -h, h], [h, -h, h], [h, h, h], [-h, h, h]]
    )
    indices = np.array(
        [
            [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
            [2, 3, 7], [2, 7, 6], [1, 2, 6], [1, 6, 5], [0, 4, 7], [0, 7, 3],
        ],
        dtype=np.uint32,
    )
    return vertices, indices


def test_voxelized_mesh_fill_modes():
    vertices, indices = _cube_mesh(1.0)
    solid = rp.SharedShape.voxelized_mesh(vertices, indices, 0.2).as_voxels()
    flood = rp.SharedShape.voxelized_mesh(vertices, indices, 0.2, rp.FillMode.flood_fill()).as_voxels()
    shell = rp.SharedShape.voxelized_mesh(vertices, indices, 0.2, rp.FillMode.SURFACE_ONLY).as_voxels()
    assert solid.num_voxels() == flood.num_voxels()
    assert 0 < shell.num_voxels() < solid.num_voxels()
    assert np.allclose(solid.voxel_size, [0.2, 0.2, 0.2])

    c = rp.Collider.voxelized_mesh(vertices.tolist(), indices.tolist(), 0.2, density=3.0).build()
    assert c.shape.shape_type == rp.ShapeType.VOXELS
    assert c.density == pytest.approx(3.0)

    with pytest.raises(ValueError):
        rp.SharedShape.voxelized_mesh(vertices, indices, 0.0)
    with pytest.raises(ValueError):
        rp.SharedShape.voxelized_mesh(vertices, [[0, 1, 8]], 0.2)


def test_fill_mode():
    assert rp.FillMode.SURFACE_ONLY == rp.FillMode.SURFACE_ONLY
    assert rp.FillMode.flood_fill() == rp.FillMode.flood_fill(detect_cavities=False)
    assert rp.FillMode.flood_fill() != rp.FillMode.flood_fill(detect_cavities=True)
    assert rp.FillMode.flood_fill() != rp.FillMode.SURFACE_ONLY
    assert not rp.FillMode.SURFACE_ONLY.is_flood_fill
    assert rp.FillMode.flood_fill(True).is_flood_fill
    assert rp.FillMode.flood_fill(True).detect_cavities
    assert repr(rp.FillMode.SURFACE_ONLY) == "FillMode.SURFACE_ONLY"
    assert repr(rp.FillMode.flood_fill(True)) == "FillMode.flood_fill(detect_cavities=True)"


# ---- Heightfields -----------------------------------------------------------------


def _ground_height_at(heights, x, z, scale=(10.0, 1.0, 10.0)):
    w = rp.PhysicsWorld()
    w.add_collider(rp.Collider.heightfield(heights, scale))
    w.update_query_pipeline()
    hit = w.query_pipeline.cast_ray(rp.Ray(rp.Point3(x, 10.0, z), (0.0, -1.0, 0.0)), 100.0, True)
    assert hit is not None
    return 10.0 - hit[1]


def test_heightfield_rows_along_z_columns_along_x():
    # A single raised grid point at row 2 (z = +0.5) and column 0 (x = -0.5).
    heights = np.zeros((3, 3), dtype=np.float32)
    heights[2, 0] = 1.0
    assert _ground_height_at(heights, -4.9, 4.9) > 0.9
    assert _ground_height_at(heights, 4.9, -4.9) == pytest.approx(0.0, abs=1.0e-4)
    assert _ground_height_at(heights, 4.9, 4.9) == pytest.approx(0.0, abs=1.0e-4)
    assert _ground_height_at(heights, -4.9, -4.9) == pytest.approx(0.0, abs=1.0e-4)


@pytest.mark.parametrize(
    "convert",
    [
        lambda h: h,
        lambda h: h.astype(np.float64),
        lambda h: np.asfortranarray(h),
        lambda h: h.tolist(),
        lambda h: np.pad(h, ((0, 0), (0, 3)))[:, :4],
    ],
    ids=["float32", "float64", "fortran", "nested-lists", "non-contiguous"],
)
def test_heightfield_input_layouts(convert):
    heights = np.arange(12, dtype=np.float32).reshape(3, 4)
    shape = rp.SharedShape.heightfield(convert(heights), (1.0, 1.0, 1.0)).as_heightfield()
    assert shape.nrows == 3 and shape.ncols == 4
    assert np.array_equal(shape.heights, heights)
    hf = rp.Collider.heightfield(convert(heights), (1.0, 1.0, 1.0), friction=0.1).build()
    assert hf.friction == pytest.approx(0.1)


def test_heightfield_rejects_degenerate_grids():
    with pytest.raises(ValueError):
        rp.SharedShape.heightfield(np.zeros((1, 3)), (1.0, 1.0, 1.0))
    with pytest.raises(ValueError):
        rp.SharedShape.heightfield([[0.0, 1.0], [2.0]], (1.0, 1.0, 1.0))


# ---- Mesh inputs ------------------------------------------------------------------


def _tetrahedron():
    vertices = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    indices = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]]
    return vertices, indices


@pytest.mark.parametrize(
    "convert_vertices, convert_indices",
    [
        (lambda v: v, lambda i: i),
        (lambda v: [tuple(p) for p in v], lambda i: [tuple(t) for t in i]),
        (lambda v: [rp.Vec3(*p) for p in v], lambda i: np.array(i, dtype=np.int64)),
        (lambda v: np.array(v, dtype=np.float32), lambda i: np.array(i, dtype=np.int32)),
        (lambda v: np.array(v, dtype=np.float64)[::-1][::-1], lambda i: np.array(i, dtype=np.uint32).T.copy().T),
    ],
    ids=["lists", "tuples", "vec3-int64", "float32-int32", "non-contiguous"],
)
def test_mesh_constructors_accept_nested_sequences(convert_vertices, convert_indices):
    vertices, indices = _tetrahedron()
    v, i = convert_vertices(vertices), convert_indices(indices)
    assert rp.SharedShape.trimesh(v, i).as_trimesh().num_triangles() == 4
    assert rp.Collider.trimesh(v, i).build().shape.shape_type == rp.ShapeType.TRIMESH
    assert rp.SharedShape.convex_hull(v).shape_type == rp.ShapeType.CONVEX_POLYHEDRON
    assert rp.Collider.convex_hull(v).build().shape.shape_type == rp.ShapeType.CONVEX_POLYHEDRON
    assert rp.SharedShape.convex_mesh(v, i).shape_type == rp.ShapeType.CONVEX_POLYHEDRON
    assert rp.Collider.convex_mesh(v, i).build() is not None
    assert rp.SharedShape.convex_decomposition(v, i).as_compound() is not None
    assert rp.Collider.convex_decomposition(v, i).build() is not None


def test_mesh_index_errors():
    vertices, _ = _tetrahedron()
    with pytest.raises(ValueError):
        rp.SharedShape.trimesh(vertices, [[0, 1]])
    with pytest.raises(ValueError):
        rp.SharedShape.trimesh(vertices, np.array([[0, 1, -1]], dtype=np.int64))
    with pytest.raises(ValueError):
        rp.SharedShape.trimesh(np.zeros((4, 2)), [[0, 1, 2]])


def test_voxels_accept_nested_lists():
    shape = rp.SharedShape.voxels((1.0, 1.0, 1.0), [[0, 0, 0], [1, 0, 0]])
    assert shape.as_voxels().num_voxels() == 2
    assert rp.Collider.voxels((1.0, 1.0, 1.0), [(0, 0, 0)], friction=0.2).build().friction == pytest.approx(0.2)


# ---- Builder keyword arguments on every shape factory ------------------------------

_V, _I = _tetrahedron()
_FACTORIES = {
    "ball": lambda **kw: rp.Collider.ball(0.5, **kw),
    "cuboid": lambda **kw: rp.Collider.cuboid(0.5, 0.5, 0.5, **kw),
    "halfspace": lambda **kw: rp.Collider.halfspace((0.0, 1.0, 0.0), **kw),
    "capsule": lambda **kw: rp.Collider.capsule(0.5, 0.2, **kw),
    "capsule_x": lambda **kw: rp.Collider.capsule_x(0.5, 0.2, **kw),
    "capsule_y": lambda **kw: rp.Collider.capsule_y(0.5, 0.2, **kw),
    "capsule_z": lambda **kw: rp.Collider.capsule_z(0.5, 0.2, **kw),
    "capsule_from_endpoints": lambda **kw: rp.Collider.capsule_from_endpoints((0, 0, 0), (0, 1, 0), 0.2, **kw),
    "cylinder": lambda **kw: rp.Collider.cylinder(0.5, 0.2, **kw),
    "cone": lambda **kw: rp.Collider.cone(0.5, 0.2, **kw),
    "round_cuboid": lambda **kw: rp.Collider.round_cuboid(0.5, 0.5, 0.5, 0.1, **kw),
    "round_cylinder": lambda **kw: rp.Collider.round_cylinder(0.5, 0.2, 0.05, **kw),
    "round_cone": lambda **kw: rp.Collider.round_cone(0.5, 0.2, 0.05, **kw),
    "triangle": lambda **kw: rp.Collider.triangle((0, 0, 0), (1, 0, 0), (0, 1, 0), **kw),
    "round_triangle": lambda **kw: rp.Collider.round_triangle((0, 0, 0), (1, 0, 0), (0, 1, 0), 0.1, **kw),
    "segment": lambda **kw: rp.Collider.segment((0, 0, 0), (1, 0, 0), **kw),
    "polyline": lambda **kw: rp.Collider.polyline(_V, **kw),
    "trimesh": lambda **kw: rp.Collider.trimesh(_V, _I, **kw),
    "convex_hull": lambda **kw: rp.Collider.convex_hull(_V, **kw),
    "convex_polyhedron": lambda **kw: rp.Collider.convex_polyhedron(_V, **kw),
    "round_convex_hull": lambda **kw: rp.Collider.round_convex_hull(_V, 0.1, **kw),
    "convex_mesh": lambda **kw: rp.Collider.convex_mesh(_V, _I, **kw),
    "round_convex_mesh": lambda **kw: rp.Collider.round_convex_mesh(_V, _I, 0.1, **kw),
    "convex_decomposition": lambda **kw: rp.Collider.convex_decomposition(_V, _I, **kw),
    "converted_trimesh": lambda **kw: rp.Collider.converted_trimesh(_V, _I, rp.MeshConverter.CONVEX_HULL, **kw),
    "voxels": lambda **kw: rp.Collider.voxels((1.0, 1.0, 1.0), [(0, 0, 0)], **kw),
    "voxels_from_points": lambda **kw: rp.Collider.voxels_from_points((1.0, 1.0, 1.0), _V, **kw),
    "voxelized_mesh": lambda **kw: rp.Collider.voxelized_mesh(_V, _I, 0.2, **kw),
    "heightfield": lambda **kw: rp.Collider.heightfield(np.zeros((2, 2)), (1.0, 1.0, 1.0), **kw),
    "compound": lambda **kw: rp.Collider.compound([(rp.Isometry3(), rp.SharedShape.ball(0.5))], **kw),
    "new": lambda **kw: rp.Collider.new(rp.SharedShape.ball(0.5), **kw),
}


@pytest.mark.parametrize("name", sorted(_FACTORIES))
def test_every_shape_factory_accepts_builder_kwargs(name):
    c = _FACTORIES[name](friction=0.25, sensor=True, translation=(1.0, 2.0, 3.0)).build()
    assert c.friction == pytest.approx(0.25)
    assert c.is_sensor
    assert c.translation == rp.Vec3(1.0, 2.0, 3.0)
    with pytest.raises(TypeError):
        _FACTORIES[name](not_a_builder_kwarg=1)


def test_every_collider_shape_factory_is_covered():
    factories = {
        n
        for n in dir(rp.Collider)
        if not n.startswith("_") and isinstance(getattr(rp.Collider, n), type(rp.Collider.ball))
    }
    factories -= {"from_bytes"}
    assert factories <= set(_FACTORIES), factories - set(_FACTORIES)


# ---- ColliderSet.remove ---------------------------------------------------------------


def test_collider_set_remove_wake_up():
    w, h = _sleeping_body_world()
    c = w.rigid_bodies[h].colliders[0]
    w.colliders.remove(c, w.islands, w.rigid_bodies, wake_up=False)
    assert w.rigid_bodies[h].is_sleeping

    w, h = _sleeping_body_world()
    c = w.rigid_bodies[h].colliders[0]
    w.colliders.remove(c, w.islands, w.rigid_bodies)
    assert not w.rigid_bodies[h].is_sleeping


def test_collider_set_remove_deprecated_wake_parent():
    w, h = _sleeping_body_world()
    c = w.rigid_bodies[h].colliders[0]
    with pytest.warns(DeprecationWarning, match="wake_up"):
        removed = w.colliders.remove(c, w.islands, w.rigid_bodies, wake_parent=False)
    assert removed is not None
    assert w.rigid_bodies[h].is_sleeping
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert w.colliders.remove(c, w.islands, w.rigid_bodies, False) is None


def test_trimesh_flags_members():
    flags = rp.TriMeshFlags.DEFORMABLE | rp.TriMeshFlags.FIX_INTERNAL_EDGES_TWO_SIDED
    assert rp.TriMeshFlags.DEFORMABLE in flags
    assert not flags.is_empty()
    assert rp.TriMeshFlags.empty().is_empty()


def test_collider_mass_is_the_computed_mass():
    c = rp.Collider.ball(1.0, density=2.0).build()
    assert c.mass == pytest.approx(2.0 * 4.0 / 3.0 * math.pi, rel=1.0e-4)
    c.mass = 5.0
    assert c.mass == pytest.approx(5.0)
