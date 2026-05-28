"""Port of examples2d/stress_tests/convex_polygons2.rs."""
from __future__ import annotations

import math
import random

import numpy as np

import rapier2d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "Convex polygons"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 30.0

    handle = bodies.insert(rp.RigidBody.fixed())
    colliders.insert_with_parent(rp.Collider.cuboid(ground_size, 1.2), handle, bodies)

    handle = bodies.insert(
        rp.RigidBody.fixed()
        .rotation(math.pi / 2.0)
        .translation((ground_size, ground_size * 2.0))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size * 2.0, 1.2), handle, bodies
    )

    handle = bodies.insert(
        rp.RigidBody.fixed()
        .rotation(math.pi / 2.0)
        .translation((-ground_size, ground_size * 2.0))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size * 2.0, 1.2), handle, bodies
    )

    num = 26
    scale = 2.0
    shift = scale
    centerx = shift * num / 2.0
    centery = shift / 2.0

    rng = random.Random(0)

    for i in range(num):
        for j in range(num * 5):
            x = i * shift - centerx
            y = j * shift * 2.0 + centery + 2.0
            handle = bodies.insert(rp.RigidBody.dynamic().translation((x, y)))
            pts = np.array(
                [
                    [rng.random() * scale, rng.random() * scale]
                    for _ in range(10)
                ],
                dtype=np.float32,
            )
            colliders.insert_with_parent(rp.Collider.convex_hull(pts), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 50.0), zoom=10.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
