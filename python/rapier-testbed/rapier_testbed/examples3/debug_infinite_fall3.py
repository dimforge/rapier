"""Port of ``examples3d/debug_infinite_fall3.rs``.

A ground floating above the origin with two balls dropped from below it.
Reproduces a corner-case where bodies that start under the ground end up
falling indefinitely.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Infinite fall"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 100.1
    ground_height = 2.1
    ground_body = rp.RigidBody.fixed().translation((0.0, 4.0, 0.0))
    ground_handle = bodies.insert(ground_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size),
        ground_handle,
        bodies,
    )

    rad = 1.0
    for y in (7.0 * rad, 2.0 * rad):
        body = (
            rp.RigidBody.dynamic()
            .translation((0.0, y, 0.0))
            .can_sleep(False)
        )
        handle = bodies.insert(body)
        colliders.insert_with_parent(rp.Collider.ball(rad), handle, bodies)

    testbed.look_at((100.0, -10.0, 100.0), (0.0, 0.0, 0.0))
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
