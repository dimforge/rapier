"""Port of examples2d/heightfield2.rs."""
from __future__ import annotations

import math

import numpy as np

import rapier2d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Heightfield"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = (50.0, 1.0)
    nsubdivs = 2000

    heights = np.zeros(nsubdivs + 1, dtype=np.float32)
    for i in range(nsubdivs + 1):
        if i == 0 or i == nsubdivs:
            heights[i] = 8.0
        else:
            heights[i] = math.cos(i * ground_size[0] / nsubdivs) * 2.0

    handle = bodies.insert(rp.RigidBody.fixed())
    colliders.insert_with_parent(
        rp.Collider.heightfield(heights, ground_size), handle, bodies
    )

    num = 20
    rad = 0.5
    shift = rad * 2.0
    centerx = shift * (num // 2)
    centery = shift / 2.0

    for i in range(num):
        for j in range(num):
            x = i * shift - centerx
            y = j * shift + centery + 3.0
            handle = bodies.insert(rp.RigidBody.dynamic().translation((x, y)))
            if j % 2 == 0:
                colliders.insert_with_parent(
                    rp.Collider.cuboid(rad, rad), handle, bodies
                )
            else:
                colliders.insert_with_parent(rp.Collider.ball(rad), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 0.0), zoom=10.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
