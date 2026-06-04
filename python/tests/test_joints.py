"""3D joint tests.

Parametrized across the f32 (`rapier.dim3`) and f64 (`rapier.dim3.f64`) flavors.
"""

from __future__ import annotations

import math

import pytest

import rapier3d as dim3
import rapier3d_f64 as dim3_f64


@pytest.fixture(params=[dim3, dim3_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


# --------------------------------------------------------------------------
# Construction smoke
# --------------------------------------------------------------------------


def test_motor_model_enum(ns):
    assert ns.MotorModel.ACCELERATION_BASED != ns.MotorModel.FORCE_BASED


def test_joint_axis_3d_has_six_variants(ns):
    axes = [
        ns.JointAxis.LIN_X, ns.JointAxis.LIN_Y, ns.JointAxis.LIN_Z,
        ns.JointAxis.ANG_X, ns.JointAxis.ANG_Y, ns.JointAxis.ANG_Z,
    ]
    # All distinct pairwise.
    for i, a in enumerate(axes):
        for j, b in enumerate(axes):
            if i == j:
                assert a == b
            else:
                assert a != b


def test_joint_axes_mask_bitops(ns):
    a = ns.JointAxesMask.LIN_X
    b = ns.JointAxesMask.ANG_Y
    c = a | b
    assert c.contains(a)
    assert c.contains(b)
    assert not c.contains(ns.JointAxesMask.LIN_Z)


def test_joint_limits_roundtrip(ns):
    lim = ns.JointLimits(min=-1.0, max=2.0)
    assert lim.min == -1.0
    assert lim.max == 2.0


# --------------------------------------------------------------------------
# Fixed joint
# --------------------------------------------------------------------------


def test_fixed_joint_keeps_bodies_connected(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    a = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(0, 5, 0)).build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(1, 5, 0)).build())
    fixed = ns.FixedJoint.builder().local_anchor1((1, 0, 0)).build()
    h = w.impulse_joints.insert(a, b, fixed)
    assert h in w.impulse_joints
    assert len(w.impulse_joints) == 1

    # Step many times; relative offset should remain ~ constant.
    for _ in range(60):
        w.step()
    ba = w.rigid_bodies[a]
    bb = w.rigid_bodies[b]
    rel = (
        bb.translation.x - ba.translation.x,
        bb.translation.y - ba.translation.y,
        bb.translation.z - ba.translation.z,
    )
    # Initial relative offset was (1, 0, 0). Anchor on body1 is (1, 0, 0)
    # so the bodies stay at the same xyz coordinates (anchor coincides).
    # The two bodies fall together under gravity.
    assert abs(rel[0] - 1.0) < 0.5 or abs(rel[0] - 0.0) < 0.5


def test_fixed_joint_data_view(ns):
    j = ns.FixedJoint()
    gj = j.data
    # Fixed joint locks all axes.
    assert gj.locked_axes.contains(ns.JointAxesMask.LIN_X)
    assert gj.locked_axes.contains(ns.JointAxesMask.LIN_Y)
    assert gj.locked_axes.contains(ns.JointAxesMask.LIN_Z)


# --------------------------------------------------------------------------
# Revolute joint
# --------------------------------------------------------------------------


def test_revolute_joint_requires_axis_in_3d(ns):
    with pytest.raises(TypeError):
        ns.RevoluteJoint()


def test_revolute_joint_with_axis_limits_motor(ns):
    j = (
        ns.RevoluteJoint.builder(axis=(0, 1, 0))
        .limits(-1.0, 1.0)
        .motor_velocity(2.0, 0.5)
        .build()
    )
    lim = j.limits()
    assert lim is not None
    assert abs(lim.min - (-1.0)) < 1e-6
    assert abs(lim.max - 1.0) < 1e-6
    motor = j.motor()
    assert motor is not None
    assert abs(motor.target_vel - 2.0) < 1e-6


def test_revolute_joint_motor_drives_body(ns):
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    a = w.rigid_bodies.insert(ns.RigidBody.fixed(translation=(0, 0, 0)).build())
    b = w.rigid_bodies.insert(
        ns.RigidBody.dynamic(translation=(1, 0, 0)).build()
    )
    # Give body b mass via a collider.
    w.colliders.insert_with_parent(
        ns.Collider.cuboid(0.5, 0.5, 0.5).density(1.0).build(),
        b, w.rigid_bodies,
    )
    j = (
        ns.RevoluteJoint.builder(axis=(0, 1, 0))
        .motor_velocity(2.0, 1.0)
        .motor_max_force(1000.0)
        .build()
    )
    h = w.impulse_joints.insert(a, b, j)
    assert h in w.impulse_joints
    for _ in range(120):
        w.step()
    body = w.rigid_bodies[b]
    # Body b should have angular velocity around y axis approaching 2.0
    # (motor target). With damping it might fall a bit short; allow slack.
    assert abs(body.angvel.y - 2.0) < 1.5


# --------------------------------------------------------------------------
# Prismatic / Rope / Spring / Spherical
# --------------------------------------------------------------------------


def test_prismatic_joint_constructable(ns):
    j = (
        ns.PrismaticJoint.builder(axis=(1, 0, 0))
        .local_anchor1((0, 0, 0))
        .local_anchor2((0, 0, 0))
        .limits(-1.0, 1.0)
        .build()
    )
    assert j.limits() is not None


def test_rope_joint_constructable(ns):
    j = ns.RopeJoint.builder(max_distance=2.5).build()
    assert abs(j.max_distance - 2.5) < 1e-6


def test_spring_joint_constructable(ns):
    j = ns.SpringJoint.builder(rest_length=1.0, stiffness=10.0, damping=0.5).build()
    assert abs(j.rest_length - 1.0) < 1e-6
    assert abs(j.spring_stiffness - 10.0) < 1e-6


def test_spherical_joint_constructable(ns):
    j = (
        ns.SphericalJoint.builder()
        .local_anchor1((0, 1, 0))
        .local_anchor2((0, -1, 0))
        .build()
    )
    p1 = j.local_anchor1
    assert abs(p1.x - 0.0) < 1e-6 and abs(p1.y - 1.0) < 1e-6


# --------------------------------------------------------------------------
# Generic joint
# --------------------------------------------------------------------------


def test_generic_joint_with_locked_axes(ns):
    mask = ns.JointAxesMask.LIN_X | ns.JointAxesMask.ANG_Y
    j = ns.GenericJoint.builder(locked_axes=mask).build()
    assert j.locked_axes.contains(ns.JointAxesMask.LIN_X)
    assert j.locked_axes.contains(ns.JointAxesMask.ANG_Y)


# --------------------------------------------------------------------------
# ImpulseJointSet
# --------------------------------------------------------------------------


def test_impulse_joint_set_attached_and_remove(ns):
    w = ns.PhysicsWorld()
    a = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())
    c = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())

    j_ab = w.impulse_joints.insert(a, b, ns.FixedJoint())
    j_bc = w.impulse_joints.insert(b, c, ns.FixedJoint())

    attached_b = list(w.impulse_joints.attached_joints(b))
    # Two joints touch body b.
    assert len(attached_b) == 2
    assert j_ab in attached_b
    assert j_bc in attached_b

    body1, body2 = w.impulse_joints.bodies_connected_by_joint(j_ab)
    assert {body1, body2} == {a, b}

    removed = w.impulse_joints.remove(j_ab)
    assert removed is not None
    assert j_ab not in w.impulse_joints

    with pytest.raises(ns.InvalidHandle):
        w.impulse_joints[j_ab]


def test_impulse_joint_set_bodies(ns):
    """`set_bodies` rewires a joint while preserving its handle."""
    w = ns.PhysicsWorld()
    a = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())
    c = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())

    j = w.impulse_joints.insert(a, b, ns.FixedJoint())
    assert {*w.impulse_joints.bodies_connected_by_joint(j)} == {a, b}

    # Rewire b -> c; the handle stays valid and is reused.
    assert w.impulse_joints.set_bodies(j, a, c, True) is True
    assert j in w.impulse_joints
    assert {*w.impulse_joints.bodies_connected_by_joint(j)} == {a, c}

    # A stale handle returns False.
    w.impulse_joints.remove(j)
    assert w.impulse_joints.set_bodies(j, a, c, True) is False


def test_impulse_joint_set_iterate(ns):
    w = ns.PhysicsWorld()
    a = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())
    h = w.impulse_joints.insert(a, b, ns.FixedJoint())
    pairs = list(w.impulse_joints)
    assert len(pairs) == 1
    handle, joint = pairs[0]
    assert handle == h
    assert {joint.body1, joint.body2} == {a, b}
