"""Port of examples2d/s2d_confined.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Inspired by Solver 2D"
NAME = "Confined"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    radius = 0.5
    grid_count = 25
    friction = 0.6
    max_count = grid_count * grid_count

    walls = [
        ((-10.5, 0.0), (10.5, 0.0)),
        ((-10.5, 0.0), (-10.5, 20.5)),
        ((10.5, 0.0), (10.5, 20.5)),
        ((-10.5, 20.5), (10.5, 20.5)),
    ]
    for a, b in walls:
        colliders.insert(
            rp.Collider.capsule_from_endpoints(a, b, radius).friction(friction)
        )

    count = 0
    column = 0
    while count < max_count:
        row = 0
        for _ in range(grid_count):
            x = -8.75 + column * 18.0 / grid_count
            y = 1.5 + row * 18.0 / grid_count
            body_handle = bodies.insert(
                rp.RigidBody.dynamic().translation((x, y)).gravity_scale(0.0)
            )
            colliders.insert_with_parent(
                rp.Collider.ball(radius).friction(friction), body_handle, bodies
            )
            count += 1
            row += 1
        column += 1

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
