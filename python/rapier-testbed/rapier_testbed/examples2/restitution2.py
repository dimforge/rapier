"""Port of examples2d/restitution2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Dynamics"
NAME = "Restitution"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 20.0
    ground_height = 1.0

    handle = bodies.insert(rp.RigidBody.fixed().translation((0.0, -ground_height)))
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height).restitution(1.0),
        handle,
        bodies,
    )

    num = 10
    rad = 0.5

    for j in range(2):
        for i in range(num + 1):
            x = i - num / 2.0
            handle = bodies.insert(
                rp.RigidBody.dynamic().translation((x * 2.0, 10.0 * (j + 1)))
            )
            colliders.insert_with_parent(
                rp.Collider.ball(rad).restitution(i / num), handle, bodies
            )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 1.0), zoom=25.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
