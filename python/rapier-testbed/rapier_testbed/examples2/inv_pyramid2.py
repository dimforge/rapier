"""Port of examples2d/inv_pyramid2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Inv pyramid"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 10.0
    ground_thickness = 1.0

    ground_handle = bodies.insert(rp.RigidBody.fixed())
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_thickness), ground_handle, bodies
    )

    num = 6
    rad = 0.5
    y = rad

    for _ in range(num):
        handle = bodies.insert(
            rp.RigidBody.dynamic().translation((0.0, y + ground_thickness))
        )
        colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), handle, bodies)
        y += rad + rad * 2.0
        rad *= 2.0

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
