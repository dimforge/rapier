"""Port of examples2d/stress_tests/balls2.rs."""
from __future__ import annotations

import rapier2d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "Balls"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    num = 50
    rad = 1.0
    shiftx = rad * 2.5
    shifty = rad * 2.0
    centerx = shiftx * num / 2.0
    centery = shifty / 2.0

    for i in range(num):
        for j in range(num * 5):
            x = i * shiftx - centerx
            y = j * shifty + centery
            rb = rp.RigidBody.fixed() if j == 0 else rp.RigidBody.dynamic()
            handle = bodies.insert(rb.translation((x, y)))
            colliders.insert_with_parent(rp.Collider.ball(rad), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=5.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
