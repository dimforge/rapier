"""Port of examples2d/polyline2.rs."""
from __future__ import annotations

import math

import numpy as np

import rapier2d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Polyline"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 50.0
    nsubdivs = 2000
    step_size = ground_size / nsubdivs

    pts = [[-ground_size / 2.0, 40.0]]
    for i in range(1, nsubdivs - 1):
        x = -ground_size / 2.0 + i * step_size
        y = math.cos(i * step_size) * 2.0
        pts.append([x, y])
    pts.append([ground_size / 2.0, 40.0])

    vertices = np.array(pts, dtype=np.float32)
    handle = bodies.insert(rp.RigidBody.fixed())
    colliders.insert_with_parent(rp.Collider.polyline(vertices), handle, bodies)

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
