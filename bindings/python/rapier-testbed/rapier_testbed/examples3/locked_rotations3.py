"""Port of examples3d/locked_rotations3.rs."""
from __future__ import annotations

import rapier3d as rp
from .._registry import register

CATEGORY = "Dynamics"
NAME = "Locked rotations"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 5.0
    ground_height = 0.1
    ground = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_h = bodies.insert(ground)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size), ground_h, bodies
    )

    # Rectangle that only rotates along the x axis.
    body = (
        rp.RigidBody.dynamic()
        .translation((0.0, 3.0, 0.0))
        .locked_axes(rp.LockedAxes.TRANSLATION_LOCKED)
        .enabled_rotations((True, False, False))
    )
    h = bodies.insert(body)
    colliders.insert_with_parent(rp.Collider.cuboid(0.2, 0.6, 2.0), h, bodies)

    # Tilted capsule that cannot rotate.
    body = (
        rp.RigidBody.dynamic()
        .translation((0.0, 5.0, 0.0))
        .rotation((1.0, 0.0, 0.0))
        .locked_axes(rp.LockedAxes.ROTATION_LOCKED)
    )
    h = bodies.insert(body)
    colliders.insert_with_parent(rp.Collider.capsule_y(0.6, 0.4), h, bodies)
    colliders.insert_with_parent(rp.Collider.capsule_x(0.6, 0.4), h, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 3.0, 0.0), (0.0, 3.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
