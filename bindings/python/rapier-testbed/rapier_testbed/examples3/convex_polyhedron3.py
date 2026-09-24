"""Port of ``examples3d/convex_polyhedron3.rs``.

Random clouds of 10 points each are turned into convex polyhedra via
``ColliderBuilder::convex_hull`` (the Rust example uses
``round_convex_hull`` which is not exposed in the Python bindings yet,
so we fall back to the plain convex hull).
"""
from __future__ import annotations

import random

import numpy as np

import rapier3d as rp

from .._registry import register

CATEGORY = "Collisions"
NAME = "Convex polyhedron"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 40.0
    ground_height = 0.1
    ground_body = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_handle = bodies.insert(ground_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size),
        ground_handle,
        bodies,
    )

    num = 5
    scale = 2.0
    border_rad = 0.1
    shift = border_rad * 2.0 + scale
    centerx = shift * (num // 2)
    centery = shift / 2.0
    centerz = shift * (num // 2)

    rng = random.Random(0)
    for j in range(25):
        for i in range(num):
            for k in range(num):
                x = i * shift - centerx
                y = j * shift + centery + 3.0
                z = k * shift - centerz

                pts = np.array(
                    [
                        [rng.random() * scale, rng.random() * scale, rng.random() * scale]
                        for _ in range(10)
                    ],
                    dtype=np.float32,
                )

                body = rp.RigidBody.dynamic().translation((x, y, z))
                handle = bodies.insert(body)
                try:
                    coll = rp.Collider.convex_hull(pts)
                except Exception:
                    # Degenerate point cloud; skip.
                    continue
                colliders.insert_with_parent(coll, handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((30.0, 30.0, 30.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
