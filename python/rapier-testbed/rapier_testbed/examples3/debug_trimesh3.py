"""Port of examples3d/debug_trimesh3.rs."""
from __future__ import annotations

import numpy as np

import rapier3d as rp
from .._registry import register

CATEGORY = "Debug"
NAME = "Trimesh"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    # 8-vertex box trimesh as the ground.
    width = 0.5
    vtx = np.array(
        [
            [-width, 0.0, -width],
            [width, 0.0, -width],
            [width, 0.0, width],
            [-width, 0.0, width],
            [-width, -width, -width],
            [width, -width, -width],
            [width, -width, width],
            [-width, -width, width],
        ],
        dtype=np.float32,
    )
    idx = np.array(
        [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 4, 7],
            [0, 7, 3],
            [1, 6, 5],
            [1, 2, 6],
            [3, 7, 2],
            [2, 7, 6],
            [0, 1, 5],
            [0, 5, 4],
        ],
        dtype=np.uint32,
    )

    # Dynamic box rigid body.
    box = rp.RigidBody.dynamic().translation((0.0, 35.0, 0.0)).can_sleep(False)
    box_handle = bodies.insert(box)
    colliders.insert_with_parent(rp.Collider.cuboid(1.0, 2.0, 1.0), box_handle, bodies)

    # Trimesh ground.
    ground = rp.RigidBody.fixed().translation((0.0, 0.0, 0.0))
    ground_handle = bodies.insert(ground)
    colliders.insert_with_parent(rp.Collider.trimesh(vtx, idx), ground_handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
