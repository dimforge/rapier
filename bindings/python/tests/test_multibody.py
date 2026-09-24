"""Multibody joint tests (3D)."""

from __future__ import annotations

import math

import pytest

import rapier3d as dim3


@pytest.fixture(params=[dim3], ids=["f32"])
def ns(request):
    return request.param


def test_multibody_three_body_chain(ns):
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    a = w.rigid_bodies.insert(ns.RigidBody.fixed(translation=(0, 0, 0)).build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(1, 0, 0)).build())
    c = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(2, 0, 0)).build())

    # Three-body chain via Revolute joints around Y.
    j_ab = w.multibody_joints.insert(
        a, b,
        ns.RevoluteJoint.builder(axis=(0, 1, 0))
            .local_anchor1((0.5, 0, 0))
            .local_anchor2((-0.5, 0, 0))
            .build(),
    )
    j_bc = w.multibody_joints.insert(
        b, c,
        ns.RevoluteJoint.builder(axis=(0, 1, 0))
            .local_anchor1((0.5, 0, 0))
            .local_anchor2((-0.5, 0, 0))
            .build(),
    )
    assert j_ab is not None
    assert j_bc is not None

    mb = w.multibody_joints.multibody(j_bc)
    assert mb is not None
    assert mb.num_links == 3
    # Iteration over links yields three.
    links = list(mb)
    assert len(links) == 3


def test_multibody_link_coords_and_joint_rot(ns):
    """A link exposes its joint's generalized coordinates and rotation."""
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    a = w.rigid_bodies.insert(ns.RigidBody.fixed(translation=(0, 0, 0)).build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic(translation=(1, 0, 0)).build())
    h = w.multibody_joints.insert(
        a, b,
        ns.RevoluteJoint.builder(axis=(0, 1, 0))
            .local_anchor1((0.5, 0, 0))
            .local_anchor2((-0.5, 0, 0))
            .build(),
    )
    assert h is not None
    mb = w.multibody_joints.multibody(h)
    assert mb is not None
    link = mb.get_link(1)
    assert link is not None
    # `coords` is a flat list (6 entries in 3D); a freshly built revolute
    # joint sits at zero angle.
    coords = link.coords
    assert isinstance(coords, list)
    assert len(coords) == 6
    # The joint rotation is a valid unit quaternion at (near) zero angle.
    q = link.joint_rot
    w, x, y, z = q.quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    assert abs(norm - 1.0) < 1e-5
    assert abs(q.angle) < 1e-5


def test_multibody_link_id(ns):
    w = ns.PhysicsWorld()
    a = w.rigid_bodies.insert(ns.RigidBody.fixed().build())
    b = w.rigid_bodies.insert(ns.RigidBody.dynamic().build())
    h = w.multibody_joints.insert(
        a, b,
        ns.RevoluteJoint.builder(axis=(0, 1, 0)).build(),
    )
    assert h is not None
    link_id = w.multibody_joints.rigid_body_link(b)
    assert link_id is not None
    # The link id for body b is 1 (root is 0).
    assert link_id.id == 1


def test_multibody_step_preserves_connectivity(ns):
    w = ns.PhysicsWorld(gravity=(0, -9.81, 0))
    handles = [
        w.rigid_bodies.insert(ns.RigidBody.fixed().build())
    ]
    for i in range(3):
        h = w.rigid_bodies.insert(
            ns.RigidBody.dynamic(translation=(float(i + 1), 0, 0)).build()
        )
        # Provide mass via a collider.
        w.colliders.insert_with_parent(
            ns.Collider.cuboid(0.4, 0.4, 0.4).density(1.0).build(),
            h, w.rigid_bodies,
        )
        handles.append(h)

    # Chain them.
    for parent, child in zip(handles[:-1], handles[1:]):
        w.multibody_joints.insert(
            parent, child,
            ns.RevoluteJoint.builder(axis=(0, 0, 1))
                .local_anchor1((0.5, 0, 0))
                .local_anchor2((-0.5, 0, 0))
                .build(),
        )

    for _ in range(30):
        w.step()

    # Still alive in the multibody set.
    mb = w.multibody_joints.multibody(
        ns.MultibodyJointHandle.from_raw_parts(*handles[-1].into_raw_parts() if False else (handles[-1].index, handles[-1].generation))
    )
    # The above gymnastics: MultibodyJointHandle == RigidBodyHandle of the
    # last link by construction in the rapier engine. Skip directly.
    h_last = ns.MultibodyJointHandle.from_raw_parts(
        handles[-1].index, handles[-1].generation
    )
    mb = w.multibody_joints.multibody(h_last)
    assert mb is not None
    assert mb.num_links == 4


def _chain(ns, n=3):
    """A fixed root and ``n`` spherical links stacked along +Y."""
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    root = w.add_body(ns.RigidBody.fixed())
    parent = root
    joints = []
    for i in range(1, n + 1):
        link = w.add_body(
            ns.RigidBody.dynamic(translation=(0.0, i - 0.5, 0.0)),
            colliders=[ns.Collider.ball(0.25)],
        )
        joint = (
            ns.SphericalJoint.builder()
            .local_anchor1((0.0, 0.5 * (i != 1), 0.0))
            .local_anchor2((0.0, -0.5, 0.0))
        )
        joints.append(w.multibody_joints.insert(parent, link, joint))
        parent = link
    w.step()
    return w, joints


def test_multibody_joint_set_len_matches_iteration(ns):
    w, joints = _chain(ns, 3)
    # A second, independent multibody.
    a = w.add_body(ns.RigidBody.fixed(translation=(5, 0, 0)))
    b = w.add_body(ns.RigidBody.dynamic(translation=(6, 0, 0)))
    joints.append(w.multibody_joints.insert(a, b, ns.RevoluteJoint.builder(axis=(0, 1, 0))))
    assert len(w.multibody_joints) == len(list(w.multibody_joints)) == 4
    w.multibody_joints.remove(joints[-1])
    assert len(w.multibody_joints) == len(list(w.multibody_joints)) == 3


def test_multibody_joint_live_view(ns):
    w, joints = _chain(ns, 3)
    joint = w.multibody_joints[joints[1]]
    assert isinstance(joint, ns.MultibodyJoint)
    assert joint.link_id == 2
    assert joint.kinematic is False
    assert len(joint.coords) == 6
    assert joint.multibody.ndofs == 9
    assert "MultibodyJoint(" in repr(joint)

    # Mutations of `data` persist in the set.
    joint.data.set_motor_velocity(ns.JointAxis.ANG_X, 1.0, 0.5)
    motor = w.multibody_joints[joints[1]].data.motor(ns.JointAxis.ANG_X)
    assert motor is not None and motor.target_vel == 1.0 and motor.damping == 0.5
    mb, link_id = w.multibody_joints.get(joints[1])
    assert mb.get_link(link_id).rigid_body is not None

    # Assigning a whole description with the same locked axes works.
    new_data = ns.SphericalJoint.builder().local_anchor2((0.0, -0.5, 0.0)).build().data
    new_data.contacts_enabled = False
    joint.data = new_data
    assert w.multibody_joints[joints[1]].data.contacts_enabled is False


def test_multibody_joint_locked_axes_cannot_change(ns):
    w, joints = _chain(ns, 2)
    joint = w.multibody_joints[joints[0]]
    locked = joint.data.locked_axes
    with pytest.raises(ValueError):
        joint.data.locked_axes = ns.JointAxesMask.LOCKED_FIXED_AXES
    with pytest.raises(ValueError):
        joint.data.lock_axes(ns.JointAxesMask.ANG_X)
    with pytest.raises(ValueError):
        joint.data = ns.RevoluteJoint.builder(axis=(1, 0, 0)).build().data
    # Setting the same locked axes is allowed.
    joint.data.locked_axes = locked
    joint.data.lock_axes(ns.JointAxesMask.LIN_X)
    assert w.multibody_joints[joints[0]].data.locked_axes == locked
    assert w.multibody_joints.multibody(joints[0]).ndofs == 6


def test_multibody_joint_invalid_handle(ns):
    w, joints = _chain(ns, 2)
    joint = w.multibody_joints[joints[1]]
    w.multibody_joints.remove(joints[1])
    with pytest.raises(ns.InvalidHandle):
        w.multibody_joints[joints[1]]
    with pytest.raises(ns.InvalidHandle):
        joint.coords
    with pytest.raises(ns.InvalidHandle):
        w.multibody_joints[ns.MultibodyJointHandle.invalid()]


def test_multibody_joint_modification_wakes_up_bodies(ns):
    w = ns.PhysicsWorld(gravity=(0, 0, 0))
    root = w.add_body(ns.RigidBody.fixed())
    link = w.add_body(
        ns.RigidBody.dynamic(translation=(0, -1, 0)), colliders=[ns.Collider.ball(0.25)]
    )
    h = w.multibody_joints.insert(
        root, link, ns.RevoluteJoint.builder(axis=(1, 0, 0)).local_anchor2((0, 1, 0))
    )
    w.step()
    w.rigid_bodies[link].sleep()
    w.step()
    assert w.rigid_bodies[link].is_sleeping
    w.multibody_joints[h].data.set_motor_velocity(ns.JointAxis.ANG_X, 2.0, 10.0)
    for _ in range(5):
        w.step()
    assert not w.rigid_bodies[link].is_sleeping
    assert abs(w.multibody_joints.multibody(h).generalized_velocity()[0]) > 0.1


def test_multibody_inverse_kinematics_and_apply_displacements(ns):
    w, joints = _chain(ns, 3)
    end_effector = joints[-1]
    options = ns.InverseKinematicsOption(
        constrained_axes=ns.JointAxesMask.LIN_AXES, max_iters=50
    )
    target = ns.Isometry3(translation=(0.5, 1.5, 0.0))
    mb = w.multibody_joints.multibody(end_effector)
    displacements = w.multibody_joints.inverse_kinematics_for_link(
        w.rigid_bodies, end_effector, target, options
    )
    assert len(displacements) == mb.ndofs
    mb.apply_displacements(displacements)
    mb.forward_kinematics(w.rigid_bodies)
    mb.update_rigid_bodies(w.rigid_bodies, False)
    # The IK target is the pose of the end link (its body frame).
    link_body = mb.get_link(w.multibody_joints[end_effector].link_id).rigid_body
    t = w.rigid_bodies[link_body].translation
    assert math.dist((t.x, t.y, t.z), (0.5, 1.5, 0.0)) < 0.05

    with pytest.raises(ValueError):
        mb.apply_displacements([0.0] * (mb.ndofs + 1))


def test_multibody_inverse_kinematics_joint_can_move(ns):
    w, joints = _chain(ns, 3)
    end_effector = joints[-1]
    target = ns.Isometry3(translation=(0.5, 1.5, 0.0))
    options = ns.InverseKinematicsOption(constrained_axes=ns.JointAxesMask.LIN_AXES)
    seen = []

    def joint_can_move(link):
        seen.append(link.link_id)
        return link.link_id != 1

    disp = w.multibody_joints.inverse_kinematics_for_link(
        w.rigid_bodies, end_effector, target, options, joint_can_move=joint_can_move
    )
    assert sorted(seen) == [0, 1, 2, 3]
    # The root is fixed (no DoF): link 1 owns DoFs 0..3.
    assert disp[0:3] == [0.0, 0.0, 0.0]
    assert any(abs(d) > 1e-4 for d in disp[3:])

    def failing(link):
        raise RuntimeError("boom")

    with pytest.raises(RuntimeError, match="boom"):
        w.multibody_joints.inverse_kinematics_for_link(
            w.rigid_bodies, end_effector, target, options, joint_can_move=failing
        )
