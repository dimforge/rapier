"""Port of examples2d/pyramid2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Pyramid"


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

    num = 10
    rad = 0.5
    shift = rad * 2.0
    centerx = shift * num / 2.0
    centery = shift / 2.0 + ground_thickness + rad * 1.5

    for i in range(num):
        for j in range(i, num):
            x = (i * shift / 2.0) + (j - i) * shift - centerx
            y = i * shift + centery
            handle = bodies.insert(rp.RigidBody.dynamic().translation((x, y)))
            colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
