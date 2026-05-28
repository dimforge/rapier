"""Port of examples2d/s2d_ball_and_chain.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Inspired by Solver 2D"
NAME = "Ball and chain"


def _local_anchor(body, pivot):
    """Pure Python equivalent of ``body.position.inverse_transform_point(pivot)``."""
    pose = body.position
    return pose.inverse().transform_point(pivot)


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground = bodies.insert(rp.RigidBody.fixed())

    count = 40
    hx = 0.5
    density = 20.0
    friction = 0.6

    prev = ground
    for i in range(count):
        rb = (
            rp.RigidBody.dynamic()
            .linear_damping(0.1)
            .angular_damping(0.1)
            .translation(((1.0 + 2.0 * i) * hx, count * hx))
        )
        handle = bodies.insert(rb)
        colliders.insert_with_parent(
            rp.Collider.capsule_x(hx, 0.125).friction(friction).density(density),
            handle,
            bodies,
        )

        pivot = ((2.0 * i) * hx, count * hx)
        a1 = _local_anchor(bodies.get(prev), pivot)
        a2 = _local_anchor(bodies.get(handle), pivot)
        joint = (
            rp.RevoluteJoint.builder()
            .local_anchor1((float(a1.x), float(a1.y)))
            .local_anchor2((float(a2.x), float(a2.y)))
            .contacts_enabled(False)
        )
        impulse_joints.insert(prev, handle, joint, wake_up=True)
        prev = handle

    radius = 8.0
    handle = bodies.insert(
        rp.RigidBody.dynamic()
        .linear_damping(0.1)
        .angular_damping(0.1)
        .translation(((1.0 + 2.0 * count) * hx + radius - hx, count * hx))
    )
    colliders.insert_with_parent(
        rp.Collider.ball(radius).friction(friction).density(density), handle, bodies
    )

    pivot = ((2.0 * count) * hx, count * hx)
    a1 = _local_anchor(bodies.get(prev), pivot)
    a2 = _local_anchor(bodies.get(handle), pivot)
    joint = (
        rp.RevoluteJoint.builder()
        .local_anchor1((float(a1.x), float(a1.y)))
        .local_anchor2((float(a2.x), float(a2.y)))
        .contacts_enabled(False)
    )
    impulse_joints.insert(prev, handle, joint, wake_up=True)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
