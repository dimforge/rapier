"""Port of examples2d/stress_tests/vertical_stacks2.rs."""
from __future__ import annotations

import rapier2d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "Verticals stacks"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    num = 80
    rad = 0.5
    ground_size = num * rad * 10.0
    ground_thickness = 1.0

    ground_handle = bodies.insert(rp.RigidBody.fixed())
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_thickness), ground_handle, bodies
    )

    shiftx_centerx = [
        (rad * 2.0 + 0.0002, -num * rad * 2.0 * 1.5),
        (rad * 2.0 + rad, num * rad * 2.0 * 1.5),
    ]

    for shiftx, centerx in shiftx_centerx:
        shifty = rad * 2.0
        centery = shifty / 2.0 + ground_thickness

        for i in range(num):
            for j in range(1 + i * 2):
                x = (j - i) * shiftx + centerx
                y = (num - i - 1) * shifty + centery
                handle = bodies.insert(rp.RigidBody.dynamic().translation((x, y)))
                colliders.insert_with_parent(
                    rp.Collider.cuboid(rad, rad), handle, bodies
                )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=5.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
