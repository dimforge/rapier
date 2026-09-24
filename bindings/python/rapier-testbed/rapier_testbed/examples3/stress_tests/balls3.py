"""Port of examples3d/stress_tests/balls3.rs."""
from __future__ import annotations

import rapier3d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "Balls"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    num = 20
    rad = 1.0
    shift = rad * 2.0 + 1.0
    centerx = shift * num / 2.0
    centery = shift / 2.0
    centerz = shift * num / 2.0

    for i in range(num):
        for j in range(num):
            for k in range(num):
                x = i * shift - centerx
                y = j * shift + centery
                z = k * shift - centerz
                if j == 0:
                    body = rp.RigidBody.fixed().translation((x, y, z))
                else:
                    body = rp.RigidBody.dynamic().translation((x, y, z))
                h = bodies.insert(body)
                colliders.insert_with_parent(rp.Collider.ball(rad).density(0.477), h, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
