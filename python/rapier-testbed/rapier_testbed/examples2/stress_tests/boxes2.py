"""Port of examples2d/stress_tests/boxes2.rs."""
from __future__ import annotations

import math

import rapier2d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "Boxes"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 25.0

    handle = bodies.insert(rp.RigidBody.fixed())
    colliders.insert_with_parent(rp.Collider.cuboid(ground_size, 1.2), handle, bodies)

    handle = bodies.insert(
        rp.RigidBody.fixed()
        .rotation(math.pi / 2.0)
        .translation((ground_size, ground_size * 2.0))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size * 2.0, 1.2), handle, bodies
    )

    handle = bodies.insert(
        rp.RigidBody.fixed()
        .rotation(math.pi / 2.0)
        .translation((-ground_size, ground_size * 2.0))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size * 2.0, 1.2), handle, bodies
    )

    num = 26
    rad = 0.5
    shift = rad * 2.0
    centerx = shift * num / 2.0
    centery = shift / 2.0

    for i in range(num):
        for j in range(num * 5):
            x = i * shift - centerx
            y = j * shift + centery + 2.0
            handle = bodies.insert(rp.RigidBody.dynamic().translation((x, y)))
            colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 50.0), zoom=10.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
