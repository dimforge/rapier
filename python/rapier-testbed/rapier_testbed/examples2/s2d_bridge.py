"""Port of examples2d/s2d_bridge.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Inspired by Solver 2D"
NAME = "Bridge"


def _local_anchor(body, pivot):
    return body.position.inverse().transform_point(pivot)


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground = bodies.insert(rp.RigidBody.fixed())

    density = 20.0
    x_base = -80.0
    count = 160
    prev = ground

    for i in range(count):
        rb = (
            rp.RigidBody.dynamic()
            .linear_damping(0.1)
            .angular_damping(0.1)
            .translation((x_base + 0.5 + 1.0 * i, 20.0))
        )
        handle = bodies.insert(rb)
        colliders.insert_with_parent(
            rp.Collider.cuboid(0.5, 0.125).density(density), handle, bodies
        )

        pivot = (x_base + 1.0 * i, 20.0)
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

    pivot = (x_base + 1.0 * count, 20.0)
    a1 = _local_anchor(bodies.get(prev), pivot)
    a2 = _local_anchor(bodies.get(ground), pivot)
    joint = (
        rp.RevoluteJoint.builder()
        .local_anchor1((float(a1.x), float(a1.y)))
        .local_anchor2((float(a2.x), float(a2.y)))
        .contacts_enabled(False)
    )
    impulse_joints.insert(prev, ground, joint, wake_up=True)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
