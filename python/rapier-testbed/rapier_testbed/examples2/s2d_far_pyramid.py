"""Port of examples2d/s2d_far_pyramid.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Inspired by Solver 2D"
NAME = "Far pyramid"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    origin = (100_000.0, -80_000.0)
    friction = 0.6

    ground_handle = bodies.insert(
        rp.RigidBody.fixed().translation((origin[0] + 0.0, origin[1] + -1.0))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(100.0, 1.0).friction(friction), ground_handle, bodies
    )

    base_count = 10
    h = 0.5
    shift = 1.25 * h

    for i in range(base_count):
        y = (2.0 * i + 1.0) * shift + 0.5
        for j in range(i, base_count):
            x = (i + 1.0) * shift + 2.0 * (j - i) * shift - h * base_count
            handle = bodies.insert(
                rp.RigidBody.dynamic().translation((origin[0] + x, origin[1] + y))
            )
            colliders.insert_with_parent(
                rp.Collider.cuboid(h, h).friction(friction), handle, bodies
            )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(origin[0], origin[1] + 2.5), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
