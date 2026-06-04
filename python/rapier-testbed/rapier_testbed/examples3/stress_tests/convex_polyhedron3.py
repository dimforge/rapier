"""Port of examples3d/stress_tests/convex_polyhedron3.rs.

The Rust example uses ``ColliderBuilder::round_convex_hull`` which is not
exposed in Python; we fall back to :meth:`Collider.convex_hull` (no
border-radius rounding) which is functionally equivalent for the stress
test.
"""
from __future__ import annotations

import numpy as np

import rapier3d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "Convex polyhedron"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 200.1
    ground_height = 0.1
    ground = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_h = bodies.insert(ground)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size), ground_h, bodies
    )

    num = 8
    scale = 2.0
    rad = 1.0
    border_rad = 0.1
    shift = border_rad * 2.0 + scale
    centerx = shift * (num // 2)
    centery = shift / 2.0
    centerz = shift * (num // 2)
    offset = -float(num) * shift * 0.5

    rng = np.random.default_rng(0)
    for j in range(47):
        for i in range(num):
            for k in range(num):
                x = i * shift - centerx + offset
                y = j * shift + centery + 3.0
                z = k * shift - centerz + offset
                pts = (rng.random((10, 3)).astype(np.float32)) * scale
                body = rp.RigidBody.dynamic().translation((x, y, z))
                h = bodies.insert(body)
                col = rp.Collider.convex_hull(pts)
                if col is None:
                    continue
                colliders.insert_with_parent(col, h, bodies)
        offset -= 0.05 * rad * (num - 1)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
