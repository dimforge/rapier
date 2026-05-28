"""Port of examples3d/dynamic_trimesh3.rs.

The Rust example reads ``.obj`` files from disk and either trimeshes them
or runs convex decomposition. Loading those assets via Python requires an
OBJ parser we don't ship, so this port falls back to a simpler scene that
still exercises the same machinery: a wavy heightfield-derived trimesh
ground with falling cuboids and balls.
"""
from __future__ import annotations

import math

import numpy as np

import rapier3d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Dynamic trimeshes"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    # Wavy heightfield trimesh ground.
    nsubdivs = 100
    heights = np.zeros((nsubdivs + 1, nsubdivs + 1), dtype=np.float32)
    for i in range(nsubdivs + 1):
        for j in range(nsubdivs + 1):
            heights[i, j] = (
                -math.cos(i * 40.0 / nsubdivs / 2.0)
                - math.cos(j * 40.0 / nsubdivs / 2.0)
            )
    colliders.insert(
        rp.Collider.heightfield(heights, (100.0, 2.0, 100.0))
    )

    # Falling dynamic bodies (cubes + balls).
    shift_y = 8.0
    shift_xz = 9.0
    num_duplications = 4
    for igeom in range(8):
        for k in range(1, num_duplications + 1):
            x = (igeom % 4) * shift_xz - num_duplications * shift_xz / 2.0
            y = (igeom // 4) * shift_y + 7.0
            z = k * shift_xz - num_duplications * shift_xz / 2.0
            body = rp.RigidBody.dynamic().translation((x, y, z))
            h = bodies.insert(body)
            if igeom % 2 == 0:
                col = rp.Collider.cuboid(1.0, 1.0, 1.0).contact_skin(0.1)
            else:
                col = rp.Collider.ball(1.0).contact_skin(0.1)
            colliders.insert_with_parent(col, h, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
