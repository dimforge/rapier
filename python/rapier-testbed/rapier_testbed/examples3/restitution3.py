"""Port of examples3d/restitution3.rs."""
from __future__ import annotations

import rapier3d as rp
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
    ground = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_h = bodies.insert(ground)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, 2.0).restitution(1.0),
        ground_h,
        bodies,
    )

    num = 10
    rad = 0.5
    for j in range(2):
        for i in range(num + 1):
            x = i - num / 2.0
            body = rp.RigidBody.dynamic().translation((x * 2.0, 10.0 * (j + 1.0), 0.0))
            h = bodies.insert(body)
            colliders.insert_with_parent(
                rp.Collider.ball(rad).restitution(i / float(num)), h, bodies
            )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((0.0, 3.0, 30.0), (0.0, 3.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
