"""Port of examples3d/stress_tests/pyramid3.rs."""
from __future__ import annotations

import rapier3d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "Pyramid"


def _create_pyramid(bodies, colliders, offset, stack_height, hext):
    shift = (hext[0] * 2.5, hext[1] * 2.5, hext[2] * 2.5)
    for i in range(stack_height):
        for j in range(i, stack_height):
            for k in range(i, stack_height):
                fi = float(i)
                fj = float(j)
                fk = float(k)
                x = (
                    (fi * shift[0] / 2.0)
                    + (fk - fi) * shift[0]
                    + offset[0]
                    - stack_height * hext[0]
                )
                y = fi * shift[1] + offset[1]
                z = (
                    (fi * shift[2] / 2.0)
                    + (fj - fi) * shift[2]
                    + offset[2]
                    - stack_height * hext[2]
                )
                body = rp.RigidBody.dynamic().translation((x, y, z))
                h = bodies.insert(body)
                colliders.insert_with_parent(
                    rp.Collider.cuboid(hext[0], hext[1], hext[2]), h, bodies
                )


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

    cube_size = 1.0
    hext = (cube_size, cube_size, cube_size)
    bottomy = cube_size
    _create_pyramid(bodies, colliders, (0.0, bottomy, 0.0), 24, hext)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
