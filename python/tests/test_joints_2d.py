"""2D joint tests.

Parametrized across `rapier.dim2` (f32) and `rapier.dim2.f64`.
"""

from __future__ import annotations

import pytest

import rapier2d as dim2
import rapier2d_f64 as dim2_f64


@pytest.fixture(params=[dim2, dim2_f64], ids=["f32", "f64"])
def ns(request):
    return request.param


def test_joint_axis_2d_has_three_variants(ns):
    axes = [ns.JointAxis.LIN_X, ns.JointAxis.LIN_Y, ns.JointAxis.ANG_X]
    # 2D has no LIN_Z/ANG_Y/ANG_Z.
    assert not hasattr(ns.JointAxis, "LIN_Z")
    assert not hasattr(ns.JointAxis, "ANG_Y")
    assert not hasattr(ns.JointAxis, "ANG_Z")
    for i, a in enumerate(axes):
        for j, b in enumerate(axes):
            if i == j:
                assert a == b
            else:
                assert a != b


def test_joint_axes_mask_2d_has_only_lin_ang_x(ns):
    # 2D mask: only LIN_X, LIN_Y, ANG_X.
    assert not hasattr(ns.JointAxesMask, "LIN_Z")
    assert not hasattr(ns.JointAxesMask, "ANG_Y")
    assert not hasattr(ns.JointAxesMask, "ANG_Z")
    mask = ns.JointAxesMask.LIN_X | ns.JointAxesMask.ANG_X
    assert mask.contains(ns.JointAxesMask.LIN_X)
    assert mask.contains(ns.JointAxesMask.ANG_X)


def test_no_spherical_joint_in_2d(ns):
    assert not hasattr(ns, "SphericalJoint")


def test_pinslot_joint_constructable(ns):
    j = (
        ns.PinSlotJoint.builder(axis=(1, 0))
        .local_anchor1((0, 0))
        .local_anchor2((0, 0))
        .build()
    )
    assert j is not None
    # Axis should have been recorded.
    a1 = j.local_axis1
    assert abs(a1.x - 1.0) < 1e-6


def test_fixed_joint_2d(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81))
    a = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(0, 5)).build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(1, 5)).build())
    j = ns.FixedJoint.builder().local_anchor1((1, 0)).build()
    h = w.impulse_joints.insert(a, b, j)
    assert h in w.impulse_joints
    for _ in range(60):
        w.step()
    ba = w.rigid_bodies[a]
    bb = w.rigid_bodies[b]
    # Relative offset constant under fixed joint.
    dx = bb.translation.x - ba.translation.x
    dy = bb.translation.y - ba.translation.y
    assert abs(dx) < 1.5
    assert abs(dy) < 1.5


def test_revolute_joint_2d_no_axis_arg(ns):
    # In 2D, RevoluteJoint takes no axis argument.
    j = ns.RevoluteJoint.builder().limits(-1.0, 1.0).build()
    lim = j.limits()
    assert lim is not None
    assert abs(lim.min + 1.0) < 1e-6


def test_prismatic_joint_2d(ns):
    j = ns.PrismaticJoint.builder(axis=(1, 0)).limits(-1.0, 1.0).build()
    assert j.limits() is not None


def test_rope_joint_2d(ns):
    j = ns.RopeJoint.builder(max_distance=3.0).build()
    assert abs(j.max_distance - 3.0) < 1e-6


def test_spring_joint_2d(ns):
    j = ns.SpringJoint.builder(rest_length=1.0, stiffness=10.0, damping=0.5).build()
    assert abs(j.rest_length - 1.0) < 1e-6


def test_generic_joint_2d_with_locked_axes(ns):
    mask = ns.JointAxesMask.LIN_X | ns.JointAxesMask.ANG_X
    j = ns.GenericJoint.builder(locked_axes=mask).build()
    assert j.locked_axes.contains(ns.JointAxesMask.LIN_X)
    assert j.locked_axes.contains(ns.JointAxesMask.ANG_X)


def test_impulse_joint_set_attached_and_remove_2d(ns):
    w = ns.PhysicsWorld()
    a = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())

    j = w.impulse_joints.insert(a, b, ns.FixedJoint())
    attached = list(w.impulse_joints.attached_joints(a))
    assert j in attached

    removed = w.impulse_joints.remove(j)
    assert removed is not None
    with pytest.raises(ns.InvalidHandle):
        w.impulse_joints[j]
