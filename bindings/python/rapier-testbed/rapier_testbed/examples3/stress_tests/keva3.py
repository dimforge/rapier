"""Port of examples3d/stress_tests/keva3.rs (variant without the per-block color recording)."""
from __future__ import annotations

import rapier3d as rp
from ..._registry import register
from ..keva3 import _build_block  # reuse the helper from the top-level port

CATEGORY = "Stress Tests"
NAME = "Keva tower"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 50.0
    ground_height = 0.1
    ground = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_h = bodies.insert(ground)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size), ground_h, bodies
    )

    half_extents = (0.02 / 2.0 * 10.0, 0.1 / 2.0 * 10.0, 0.4 / 2.0 * 10.0)
    block_height = 0.0
    numy = [0, 9, 13, 17, 21, 41]
    for i in range(5, 0, -1):
        numx = i
        ny = numy[i]
        numz = numx * 3 + 1
        block_width = numx * half_extents[2] * 2.0
        _build_block(
            bodies,
            colliders,
            half_extents,
            (-block_width / 2.0, block_height, -block_width / 2.0),
            (numx, ny, numz),
        )
        block_height += ny * half_extents[1] * 2.0 + half_extents[0] * 2.0

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
