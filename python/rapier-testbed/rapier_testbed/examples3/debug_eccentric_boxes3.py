"""Port of ``examples3d/debug_eccentric_boxes3.rs``.

Two dynamic convex meshes — cuboid vertex sets shifted far from the
origin — exercise the solver's handling of off-centre mass properties.
Orphan example: not listed in ``all_examples3.rs`` so we file it under
``Misc``.
"""
from __future__ import annotations

import numpy as np

import rapier3d as rp

from .._registry import register

CATEGORY = "Misc"
NAME = "Eccentric boxes"


def _shifted_cuboid_points():
    h = 1.0
    pts = np.array(
        [
            [-h, -h, -h], [h, -h, -h], [-h, h, -h], [h, h, -h],
            [-h, -h, h], [h, -h, h], [-h, h, h], [h, h, h],
        ],
        dtype=np.float32,
    )
    pts += np.array([100.0, 100.0, 100.0], dtype=np.float32)
    return pts


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 100.1
    ground_height = 0.1
    ground_body = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_handle = bodies.insert(ground_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size),
        ground_handle,
        bodies,
    )

    pts = _shifted_cuboid_points()
    for _ in range(2):
        body = (
            rp.RigidBody.dynamic()
            .translation((-100.0, -100.0 + 10.0, -100.0))
            .can_sleep(False)
        )
        handle = bodies.insert(body)
        colliders.insert_with_parent(rp.Collider.convex_hull(pts), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
