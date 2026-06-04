"""Regression tests for newly-exposed Rust API surface.

Covers three additions that previously had no Python binding:

* ``SharedShape.halfspace`` / ``Collider.halfspace`` — infinite-plane shape.
* ``ActiveCollisionTypes`` — per-collider rigid-body-type collision mask.
* joint ``softness`` / builder ``.softness(...)`` — spring coefficients on
  every impulse-joint type (and the generic joint).

Each test is parametrized over the 3D f32/f64 flavors; a couple of 2D
smoke checks confirm the dimension-uniform pieces compile for 2D too.
"""

from __future__ import annotations

import pytest

import rapier2d as dim2
import rapier3d as dim3
import rapier2d_f64 as dim2_f64
import rapier3d_f64 as dim3_f64


@pytest.fixture(params=[dim3, dim3_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


@pytest.fixture(params=[dim2, dim2_f64], ids=["2d-f32", "2d-f64"])
def ns2(request):
    return request.param


# --------------------------------------------------------------------------
# halfspace
# --------------------------------------------------------------------------


def _up(ns):
    """Unit 'up' normal for the namespace's dimension."""
    return (0.0, 1.0, 0.0) if ns in (dim3, dim3_f64) else (0.0, 1.0)


def test_halfspace_shared_shape(ns):
    s = ns.SharedShape.halfspace(_up(ns))
    assert s.shape_type == ns.ShapeType.HALFSPACE


def test_halfspace_shared_shape_2d(ns2):
    s = ns2.SharedShape.halfspace(_up(ns2))
    assert s.shape_type == ns2.ShapeType.HALFSPACE


def test_halfspace_collider_builder_and_kwargs(ns):
    coll = ns.Collider.halfspace(_up(ns), friction=0.7).build()
    assert coll.friction == pytest.approx(0.7)
    assert coll.shape.shape_type == ns.ShapeType.HALFSPACE


def test_halfspace_normalizes_non_unit_normal(ns):
    # A non-unit normal must still produce a valid half-space (the binding
    # normalizes it) rather than panicking or degenerating.
    coll = ns.Collider.halfspace((0.0, 5.0, 0.0) if ns in (dim3, dim3_f64) else (0.0, 5.0)).build()
    assert coll.shape.shape_type == ns.ShapeType.HALFSPACE


def test_halfspace_as_immovable_ground(ns):
    # A dynamic ball should come to rest on a half-space ground.
    gravity = (0.0, -9.81, 0.0) if ns in (dim3, dim3_f64) else (0.0, -9.81)
    world = ns.PhysicsWorld(gravity=gravity)
    world.colliders.insert(ns.Collider.halfspace(_up(ns)).build())
    body = world.rigid_bodies.insert(
        ns.RigidBody.dynamic(translation=_up(ns)).build()
    )
    radius = 0.5
    world.colliders.insert_with_parent(
        ns.Collider.ball(radius), body, world.rigid_bodies
    )
    for _ in range(400):
        world.step()
    y = world.rigid_bodies[body].translation[1]
    assert y == pytest.approx(radius, abs=0.1)


# --------------------------------------------------------------------------
# ActiveCollisionTypes
# --------------------------------------------------------------------------


def test_active_collision_types_flags(ns):
    A = ns.ActiveCollisionTypes
    combined = A.DYNAMIC_DYNAMIC | A.KINEMATIC_FIXED
    assert A.KINEMATIC_FIXED in combined
    assert A.DYNAMIC_DYNAMIC in combined
    assert A.FIXED_FIXED not in combined
    assert A.empty().is_empty()
    assert not A.all().is_empty()
    # default = every pair involving at least one dynamic body
    default = A.default_types()
    assert A.DYNAMIC_DYNAMIC in default
    assert A.DYNAMIC_FIXED in default
    assert A.KINEMATIC_KINEMATIC not in default


def test_active_collision_types_flags_2d(ns2):
    A = ns2.ActiveCollisionTypes
    assert A.KINEMATIC_FIXED in (A.DYNAMIC_FIXED | A.KINEMATIC_FIXED)


def test_active_collision_types_collider_roundtrip(ns):
    A = ns.ActiveCollisionTypes
    # via kwarg
    coll = ns.Collider.ball(0.5, active_collision_types=A.all()).build()
    assert coll.active_collision_types == A.all()
    # via setter
    coll.active_collision_types = A.DYNAMIC_FIXED | A.KINEMATIC_FIXED
    assert ns.ActiveCollisionTypes.KINEMATIC_FIXED in coll.active_collision_types
    # via builder chain method
    coll2 = ns.Collider.ball(0.5).active_collision_types(A.KINEMATIC_KINEMATIC).build()
    assert coll2.active_collision_types == A.KINEMATIC_KINEMATIC


def test_active_collision_types_enables_kinematic_fixed_contact(ns):
    # By default a kinematic body resting on a fixed collider generates no
    # contacts. Enabling KINEMATIC_FIXED on the fixed collider should make
    # the narrow-phase report the intersection.
    A = ns.ActiveCollisionTypes
    world = ns.PhysicsWorld()

    ground = ns.Collider.cuboid(*([5.0] * (3 if ns in (dim3, dim3_f64) else 2)))
    ground = ground.active_collision_types(A.default_types() | A.KINEMATIC_FIXED)
    ground_handle = world.colliders.insert(ground.build())

    kin = world.rigid_bodies.insert(ns.RigidBody.kinematic_position_based().build())
    kin_coll = world.colliders.insert_with_parent(
        ns.Collider.ball(2.0), kin, world.rigid_bodies
    )
    world.step()

    found = world.narrow_phase.contact_pair(ground_handle, kin_coll) is not None
    assert found


# --------------------------------------------------------------------------
# joint softness
# --------------------------------------------------------------------------

_JOINT_BUILDERS_3D = [
    "FixedJointBuilder",
    "RevoluteJointBuilder",
    "PrismaticJointBuilder",
    "SphericalJointBuilder",
    "RopeJointBuilder",
    "GenericJointBuilder",
]


def test_softness_present_on_all_joint_types(ns):
    for nm in [
        "FixedJoint",
        "RevoluteJoint",
        "PrismaticJoint",
        "SphericalJoint",
        "RopeJoint",
        "GenericJoint",
    ]:
        assert hasattr(getattr(ns, nm), "softness"), nm
    for nm in _JOINT_BUILDERS_3D:
        assert hasattr(getattr(ns, nm), "softness"), nm
    # SpringJoint is already a spring; it has no separate softness knob.
    assert not hasattr(ns.SpringJoint, "softness")


def test_softness_present_on_2d_joints(ns2):
    assert hasattr(ns2.PinSlotJoint, "softness")
    assert hasattr(ns2.PinSlotJointBuilder, "softness")


def test_generic_joint_softness_roundtrip(ns):
    sc = ns.SpringCoefficients(stiffness=12.0, damping=0.8)
    j = ns.GenericJoint()
    j.softness = sc
    assert j.softness.stiffness == pytest.approx(12.0)
    assert j.softness.damping == pytest.approx(0.8)


def test_fixed_joint_builder_softness_roundtrip(ns):
    sc = ns.SpringCoefficients(stiffness=15.0, damping=0.6)
    j = ns.FixedJointBuilder().softness(sc).build()
    assert j.softness.stiffness == pytest.approx(15.0)
    assert j.softness.damping == pytest.approx(0.6)
    # mutate on the built joint
    j.softness = ns.SpringCoefficients(stiffness=25.0, damping=1.0)
    assert j.softness.stiffness == pytest.approx(25.0)


def test_generic_joint_builder_softness(ns):
    sc = ns.SpringCoefficients(stiffness=9.0, damping=0.5)
    j = ns.GenericJointBuilder().softness(sc).build()
    assert j.softness.stiffness == pytest.approx(9.0)
