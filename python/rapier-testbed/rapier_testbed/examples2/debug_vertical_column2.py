"""Port of examples2d/debug_vertical_column2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Debug"
NAME = "Vertical column"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    num = 80
    rad = 0.5

    ground_size = 1.0
    ground_thickness = 1.0

    ground_handle = bodies.insert(rp.RigidBody.fixed())
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_thickness).friction(0.3),
        ground_handle,
        bodies,
    )

    for i in range(num):
        y = i * rad * 2.0 + ground_thickness + rad
        handle = bodies.insert(rp.RigidBody.dynamic().translation((0.0, y)))
        colliders.insert_with_parent(
            rp.Collider.cuboid(rad, rad).friction(0.3), handle, bodies
        )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=5.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
