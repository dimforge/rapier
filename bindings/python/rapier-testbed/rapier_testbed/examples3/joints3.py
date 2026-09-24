"""Port of examples3d/joints3.rs.

Registers two examples: ``Impulse Joints`` and ``Multibody Joints``,
sharing the same scene builder.
"""
from __future__ import annotations

import math

import rapier3d as rp
from .._registry import register

X = (1.0, 0.0, 0.0)
Y = (0.0, 1.0, 0.0)
Z = (0.0, 0.0, 1.0)


def _normalize(v):
    n = math.sqrt(sum(x * x for x in v))
    return (v[0] / n, v[1] / n, v[2] / n)


def _add(*vs):
    return (sum(v[0] for v in vs), sum(v[1] for v in vs), sum(v[2] for v in vs))


def _scale(v, s):
    return (v[0] * s, v[1] * s, v[2] * s)


def _neg(v):
    return (-v[0], -v[1], -v[2])


def _insert_joint(impulse, multibody, parent, child, joint, use_articulations):
    if use_articulations:
        multibody.insert(parent, child, joint, wake_up=True)
    else:
        impulse.insert(parent, child, joint, wake_up=True)


def _create_coupled_joints(bodies, colliders, impulse, multibody, origin, use_articulations):
    ground = bodies.insert(rp.RigidBody.fixed().translation(origin))
    body1 = bodies.insert(
        rp.RigidBody.dynamic().translation(origin).linvel((5.0, 5.0, 5.0))
    )
    colliders.insert_with_parent(rp.Collider.cuboid(1.0, 1.0, 1.0), body1, bodies)

    joint1 = (
        rp.GenericJoint.builder(locked_axes=rp.JointAxesMask.empty())
        .limits(rp.JointAxis.LIN_X, -3.0, 3.0)
        .limits(rp.JointAxis.LIN_Y, 0.0, 3.0)
        .coupled_axes(rp.JointAxesMask.LIN_Y | rp.JointAxesMask.LIN_Z)
        .build()
    )
    _insert_joint(impulse, multibody, ground, body1, joint1, use_articulations)


def _create_prismatic_joints(bodies, colliders, impulse, multibody, origin, num, use_articulations):
    rad = 0.4
    shift = 2.0
    curr_parent = bodies.insert(rp.RigidBody.fixed().translation(origin))
    colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), curr_parent, bodies)

    for i in range(num):
        z = origin[2] + (i + 1) * shift
        body = rp.RigidBody.dynamic().translation((origin[0], origin[1], z))
        curr_child = bodies.insert(body)
        colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), curr_child, bodies)

        axis = _normalize((1.0, 1.0, 0.0)) if i % 2 == 0 else _normalize((-1.0, 1.0, 0.0))
        prism = (
            rp.PrismaticJoint.builder(axis=axis)
            .local_anchor1((0.0, 0.0, 0.0))
            .local_anchor2((0.0, 0.0, -shift))
            .limits(-2.0, 2.0)
            .build()
        )
        _insert_joint(impulse, multibody, curr_parent, curr_child, prism, use_articulations)
        curr_parent = curr_child


def _create_actuated_prismatic_joints(
    bodies, colliders, impulse, multibody, origin, num, use_articulations
):
    rad = 0.4
    shift = 2.0
    curr_parent = bodies.insert(rp.RigidBody.fixed().translation(origin))
    colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), curr_parent, bodies)

    for i in range(num):
        z = origin[2] + (i + 1) * shift
        body = rp.RigidBody.dynamic().translation((origin[0], origin[1], z))
        curr_child = bodies.insert(body)
        colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), curr_child, bodies)

        axis = _normalize((1.0, 1.0, 0.0)) if i % 2 == 0 else _normalize((-1.0, 1.0, 0.0))
        prism = (
            rp.PrismaticJoint.builder(axis=axis)
            .local_anchor1((0.0, 0.0, shift))
            .local_anchor2((0.0, 0.0, 0.0))
            .build()
        )
        if i == 0:
            prism.set_motor_velocity(2.0, 1.0e5)
            prism.set_limits(-2.0, 5.0)
            prism.set_motor_max_force(100.0)
        elif i == 1:
            prism.set_limits(-1.0e30, 5.0)
            prism.set_motor_velocity(6.0, 1.0e3)
            prism.set_motor_max_force(100.0)
        else:
            prism.set_motor_position(2.0, 1.0e3, 1.0e2)
            prism.set_motor_max_force(60.0)

        _insert_joint(impulse, multibody, curr_parent, curr_child, prism, use_articulations)
        curr_parent = curr_child


def _create_revolute_joints(
    bodies, colliders, impulse, multibody, origin, num, use_articulations
):
    rad = 0.4
    shift = 2.0
    ground = rp.RigidBody.fixed().translation((origin[0], origin[1], 0.0))
    curr_parent = bodies.insert(ground)
    colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), curr_parent, bodies)

    for i in range(num):
        z = origin[2] + i * shift * 2.0 + shift
        positions = [
            (origin[0], origin[1], z),
            (origin[0] + shift, origin[1], z),
            (origin[0] + shift, origin[1], z + shift),
            (origin[0], origin[1], z + shift),
        ]
        handles = []
        for k in range(4):
            body = rp.RigidBody.dynamic().translation(positions[k])
            h = bodies.insert(body)
            handles.append(h)
            colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), h, bodies)

        revs = [
            rp.RevoluteJoint.builder(axis=Z).local_anchor2((0.0, 0.0, -shift)).build(),
            rp.RevoluteJoint.builder(axis=X).local_anchor2((-shift, 0.0, 0.0)).build(),
            rp.RevoluteJoint.builder(axis=Z).local_anchor2((0.0, 0.0, -shift)).build(),
            rp.RevoluteJoint.builder(axis=X).local_anchor2((shift, 0.0, 0.0)).build(),
        ]
        if use_articulations:
            multibody.insert(curr_parent, handles[0], revs[0], wake_up=True)
            multibody.insert(handles[0], handles[1], revs[1], wake_up=True)
            multibody.insert(handles[1], handles[2], revs[2], wake_up=True)
            multibody.insert(handles[2], handles[3], revs[3], wake_up=True)
        else:
            impulse.insert(curr_parent, handles[0], revs[0], wake_up=True)
            impulse.insert(handles[0], handles[1], revs[1], wake_up=True)
            impulse.insert(handles[1], handles[2], revs[2], wake_up=True)
            impulse.insert(handles[2], handles[3], revs[3], wake_up=True)

        curr_parent = handles[3]


def _create_revolute_joints_with_limits(
    bodies, colliders, impulse, multibody, origin, use_articulations
):
    ground = bodies.insert(rp.RigidBody.fixed().translation(origin))
    platform1 = bodies.insert(rp.RigidBody.dynamic().translation(origin))
    colliders.insert_with_parent(rp.Collider.cuboid(4.0, 0.2, 2.0), platform1, bodies)

    shift = (0.0, 0.0, 6.0)
    platform2 = bodies.insert(rp.RigidBody.dynamic().translation(_add(origin, shift)))
    colliders.insert_with_parent(rp.Collider.cuboid(4.0, 0.2, 2.0), platform2, bodies)

    joint1 = rp.RevoluteJoint.builder(axis=Z).limits(-0.2, 0.2).build()
    _insert_joint(impulse, multibody, ground, platform1, joint1, use_articulations)

    joint2 = (
        rp.RevoluteJoint.builder(axis=Z)
        .local_anchor2(_neg(shift))
        .limits(-0.2, 0.2)
        .build()
    )
    _insert_joint(impulse, multibody, platform1, platform2, joint2, use_articulations)

    c1 = bodies.insert(
        rp.RigidBody.dynamic().translation(_add(origin, (-2.0, 4.0, 0.0)))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(0.6, 0.6, 0.6).friction(1.0), c1, bodies
    )
    c2 = bodies.insert(
        rp.RigidBody.dynamic().translation(_add(origin, shift, (2.0, 16.0, 0.0)))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(0.6, 0.6, 0.6).friction(1.0), c2, bodies
    )


def _create_fixed_joints(
    bodies, colliders, impulse, multibody, origin, num, use_articulations
):
    rad = 0.4
    shift = 1.0
    body_handles = []
    for i in range(num):
        for k in range(num):
            fk = float(k)
            fi = float(i)
            if i == 0 and ((k % 4 == 0 and k != num - 2) or k == num - 1):
                bb = rp.RigidBody.fixed()
            else:
                bb = rp.RigidBody.dynamic()
            bb = bb.translation((origin[0] + fk * shift, origin[1], origin[2] + fi * shift))
            child = bodies.insert(bb)
            colliders.insert_with_parent(rp.Collider.ball(rad), child, bodies)

            if i > 0:
                parent = body_handles[len(body_handles) - num]
                j = (
                    rp.FixedJoint.builder()
                    .local_anchor2((0.0, 0.0, -shift))
                    .build()
                )
                _insert_joint(impulse, multibody, parent, child, j, use_articulations)
            if k > 0:
                parent = body_handles[-1]
                j = (
                    rp.FixedJoint.builder()
                    .local_anchor2((-shift, 0.0, 0.0))
                    .build()
                )
                impulse.insert(parent, child, j, wake_up=True)
            body_handles.append(child)


def _create_spherical_joints(
    bodies, colliders, impulse, multibody, num, use_articulations
):
    rad = 0.4
    shift = 1.0
    body_handles = []
    for k in range(num):
        for i in range(num):
            fk = float(k)
            fi = float(i)
            if i == 0 and (k % 4 == 0 or k == num - 1):
                bb = rp.RigidBody.fixed()
            else:
                bb = rp.RigidBody.dynamic()
            bb = bb.translation((fk * shift, 0.0, fi * shift * 2.0))
            child = bodies.insert(bb)
            colliders.insert_with_parent(rp.Collider.capsule_z(rad * 1.25, rad), child, bodies)

            if i > 0:
                parent = body_handles[-1]
                j = (
                    rp.SphericalJoint.builder()
                    .local_anchor2((0.0, 0.0, -shift * 2.0))
                    .build()
                )
                _insert_joint(impulse, multibody, parent, child, j, use_articulations)
            if k > 0:
                parent = body_handles[len(body_handles) - num]
                j = (
                    rp.SphericalJoint.builder()
                    .local_anchor2((-shift, 0.0, 0.0))
                    .build()
                )
                impulse.insert(parent, child, j, wake_up=True)
            body_handles.append(child)


def _create_spherical_joints_with_limits(
    bodies, colliders, impulse, multibody, origin, use_articulations
):
    shift = (0.0, 0.0, 3.0)
    ground = bodies.insert(rp.RigidBody.fixed().translation(origin))

    ball1 = bodies.insert(
        rp.RigidBody.dynamic().translation(_add(origin, shift)).linvel((20.0, 20.0, 0.0))
    )
    colliders.insert_with_parent(rp.Collider.cuboid(1.0, 1.0, 1.0), ball1, bodies)

    ball2 = bodies.insert(
        rp.RigidBody.dynamic().translation(_add(origin, _scale(shift, 2.0)))
    )
    colliders.insert_with_parent(rp.Collider.cuboid(1.0, 1.0, 1.0), ball2, bodies)

    joint1 = (
        rp.SphericalJoint.builder()
        .local_anchor2(_neg(shift))
        .limits(rp.JointAxis.LIN_X, -0.2, 0.2)
        .limits(rp.JointAxis.LIN_Y, -0.2, 0.2)
        .build()
    )
    joint2 = (
        rp.SphericalJoint.builder()
        .local_anchor2(_neg(shift))
        .limits(rp.JointAxis.LIN_X, -0.3, 0.3)
        .limits(rp.JointAxis.LIN_Y, -0.3, 0.3)
        .build()
    )
    _insert_joint(impulse, multibody, ground, ball1, joint1, use_articulations)
    _insert_joint(impulse, multibody, ball1, ball2, joint2, use_articulations)


def _create_actuated_revolute_joints(
    bodies, colliders, impulse, multibody, origin, num, use_articulations
):
    rad = 0.4
    shift = 2.0
    parent_handle = None
    for i in range(num):
        fi = float(i)
        if i == 0:
            bb = rp.RigidBody.fixed()
        else:
            bb = rp.RigidBody.dynamic()
        shifty = -2.0 if i >= 1 else 0.0
        bb = bb.translation((origin[0], origin[1] + shifty, origin[2] + fi * shift))
        child = bodies.insert(bb)
        colliders.insert_with_parent(
            rp.Collider.cuboid(rad * 2.0, rad * 6.0 / (fi + 1.0), rad), child, bodies
        )

        if i > 0:
            b = rp.RevoluteJoint.builder(axis=Z).local_anchor2((0.0, 0.0, -shift))
            b = b.motor_model(rp.MotorModel.ACCELERATION_BASED)
            if i % 3 == 1:
                b = b.motor_velocity(-20.0, 100.0)
            elif i == num - 1:
                b = b.motor_position(math.pi / 2.0, 200.0, 100.0)
            if i == 1:
                b = b.local_anchor2((0.0, 2.0, -shift)).motor_velocity(-2.0, 1000.0)
            joint = b.build()
            _insert_joint(impulse, multibody, parent_handle, child, joint, use_articulations)
        parent_handle = child


def _create_actuated_spherical_joints(
    bodies, colliders, impulse, multibody, origin, num, use_articulations
):
    rad = 0.4
    shift = 2.0
    parent_handle = None
    for i in range(num):
        fi = float(i)
        if i == 0:
            bb = rp.RigidBody.fixed()
        else:
            bb = rp.RigidBody.dynamic()
        bb = bb.translation((origin[0], origin[1], origin[2] + fi * shift))
        child = bodies.insert(bb)
        colliders.insert_with_parent(
            rp.Collider.capsule_y(rad * 2.0 / (fi + 1.0), rad), child, bodies
        )

        if i > 0:
            b = rp.SphericalJoint.builder().local_anchor1((0.0, 0.0, shift))
            if i == 1:
                b = (
                    b.motor_velocity(rp.JointAxis.ANG_X, 0.0, 0.1)
                    .motor_velocity(rp.JointAxis.ANG_Y, 0.5, 0.1)
                    .motor_velocity(rp.JointAxis.ANG_Z, -2.0, 0.1)
                )
            elif i == num - 1:
                stiffness = 0.2
                damping = 1.0
                b = (
                    b.motor_position(rp.JointAxis.ANG_X, 0.0, stiffness, damping)
                    .motor_position(rp.JointAxis.ANG_Y, 1.0, stiffness, damping)
                    .motor_position(rp.JointAxis.ANG_Z, math.pi / 2.0, stiffness, damping)
                )
            joint = b.build()
            _insert_joint(impulse, multibody, parent_handle, child, joint, use_articulations)
        parent_handle = child


def _do_init(testbed, use_articulations):
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    _create_prismatic_joints(bodies, colliders, impulse_joints, multibody_joints, (20.0, 5.0, 0.0), 4, use_articulations)
    _create_actuated_prismatic_joints(bodies, colliders, impulse_joints, multibody_joints, (25.0, 5.0, 0.0), 4, use_articulations)
    _create_revolute_joints(bodies, colliders, impulse_joints, multibody_joints, (20.0, 0.0, 0.0), 3, use_articulations)
    _create_revolute_joints_with_limits(bodies, colliders, impulse_joints, multibody_joints, (34.0, 0.0, 0.0), use_articulations)
    _create_fixed_joints(bodies, colliders, impulse_joints, multibody_joints, (0.0, 10.0, 0.0), 10, use_articulations)
    _create_actuated_revolute_joints(bodies, colliders, impulse_joints, multibody_joints, (20.0, 10.0, 0.0), 6, use_articulations)
    _create_actuated_spherical_joints(bodies, colliders, impulse_joints, multibody_joints, (13.0, 10.0, 0.0), 3, use_articulations)
    _create_spherical_joints(bodies, colliders, impulse_joints, multibody_joints, 15, use_articulations)
    _create_spherical_joints_with_limits(bodies, colliders, impulse_joints, multibody_joints, (-5.0, 0.0, 0.0), use_articulations)
    _create_coupled_joints(bodies, colliders, impulse_joints, multibody_joints, (0.0, 20.0, 0.0), use_articulations)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((15.0, 5.0, 42.0), (13.0, 1.0, 1.0))


def init_world_with_joints(testbed) -> None:
    _do_init(testbed, False)


def init_world_with_articulations(testbed) -> None:
    _do_init(testbed, True)


register("Joints", "Impulse Joints", init_world_with_joints, dim=3)
register("Joints", "Multibody Joints", init_world_with_articulations, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial="Joints / Impulse Joints")
