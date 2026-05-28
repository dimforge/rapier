"""Port of examples3d/stress_tests/ccd3.rs."""
from __future__ import annotations

import rapier3d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "CCD"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 100.1
    ground_height = 0.1
    ground = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_h = bodies.insert(ground)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size), ground_h, bodies
    )

    num = 4
    numj = 20
    rad = 1.0
    shift = rad * 2.0 + rad
    centerx = shift * (num // 2)
    centery = shift / 2.0
    centerz = shift * (num // 2)
    offset = -float(num) * (rad * 2.0 + rad) * 0.5

    for j in range(numj):
        for i in range(num):
            for k in range(num):
                x = i * shift - centerx + offset
                y = j * shift + centery + 3.0
                z = k * shift - centerz + offset
                body = (
                    rp.RigidBody.dynamic()
                    .translation((x, y, z))
                    .linvel((0.0, -1000.0, 0.0))
                    .ccd_enabled(True)
                )
                h = bodies.insert(body)
                kind = j % 5
                if kind == 0:
                    col = rp.Collider.cuboid(rad, rad, rad)
                elif kind == 1:
                    col = rp.Collider.ball(rad)
                elif kind == 2:
                    col = rp.Collider.round_cylinder(rad, rad, rad / 10.0)
                elif kind == 3:
                    col = rp.Collider.cone(rad, rad)
                else:
                    col = rp.Collider.capsule_y(rad, rad)
                colliders.insert_with_parent(col, h, bodies)
        offset -= 0.05 * rad * (num - 1)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
