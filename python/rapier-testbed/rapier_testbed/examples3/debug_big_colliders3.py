"""Port of ``examples3d/debug_big_colliders3.rs``.

Stacked cuboids of decreasing size on top of a half-space ground. The
half-space shape is not exposed in the Python bindings yet, so we
substitute a very large but finite cuboid floor; the rest of the scene is
unchanged.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Big colliders"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    # Substitute the HalfSpace ground with a huge cuboid (binding gap).
    ground_body = rp.RigidBody.fixed().translation((0.0, -1.0, 0.0))
    ground_handle = bodies.insert(ground_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(100_000.0, 1.0, 100_000.0), ground_handle, bodies
    )

    curr_y = 0.0
    curr_width = 10_000.0
    for _ in range(12):
        curr_height = min(0.1, curr_width)
        curr_y += curr_height * 4.0
        body = rp.RigidBody.dynamic().translation((0.0, curr_y, 0.0))
        handle = bodies.insert(body)
        colliders.insert_with_parent(
            rp.Collider.cuboid(curr_width, curr_height, curr_width), handle, bodies
        )
        curr_width /= 5.0

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
