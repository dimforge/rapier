"""Port of examples3d/stress_tests/compound3.rs."""
from __future__ import annotations

import rapier3d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "Compound"


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
    rad = 0.2
    shift = rad * 4.0 + rad
    centerx = shift * (num // 2)
    centery = shift / 2.0
    centerz = shift * (num // 2)
    offset = -float(num) * (rad * 2.0 + rad) * 0.5

    for j in range(25):
        for i in range(num):
            for k in range(num):
                x = i * shift * 5.0 - centerx + offset
                y = j * (shift * 5.0) + centery + 3.0
                z = k * shift * 2.0 - centerz + offset
                body = rp.RigidBody.dynamic().translation((x, y, z))
                h = bodies.insert(body)
                c1 = rp.Collider.cuboid(rad * 10.0, rad, rad)
                c2 = rp.Collider.cuboid(rad, rad * 10.0, rad).translation(
                    (rad * 10.0, rad * 10.0, 0.0)
                )
                c3 = rp.Collider.cuboid(rad, rad * 10.0, rad).translation(
                    (-rad * 10.0, rad * 10.0, 0.0)
                )
                colliders.insert_with_parent(c1, h, bodies)
                colliders.insert_with_parent(c2, h, bodies)
                colliders.insert_with_parent(c3, h, bodies)
        offset -= 0.05 * rad * (num - 1)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
