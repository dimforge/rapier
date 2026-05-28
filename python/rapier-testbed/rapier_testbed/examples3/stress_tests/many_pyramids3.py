"""Port of examples3d/stress_tests/many_pyramids3.rs."""
from __future__ import annotations

import rapier3d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "Many pyramids"


def _create_pyramid(bodies, colliders, offset, stack_height, rad):
    shift = rad * 2.0
    for i in range(stack_height):
        for j in range(i, stack_height):
            fj = float(j)
            fi = float(i)
            x = (fi * shift / 2.0) + (fj - fi) * shift
            y = fi * shift
            body = rp.RigidBody.dynamic().translation(
                (x + offset[0], y + offset[1], 0.0 + offset[2])
            )
            h = bodies.insert(body)
            colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), h, bodies)


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.5
    pyramid_count = 40
    spacing = 4.0

    ground_size = 50.0
    ground_height = 0.1
    ground = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_h = bodies.insert(ground)
    colliders.insert_with_parent(
        rp.Collider.cuboid(
            ground_size, ground_height, pyramid_count * spacing / 2.0 + ground_size
        ),
        ground_h,
        bodies,
    )

    for p in range(pyramid_count):
        bottomy = rad
        _create_pyramid(
            bodies,
            colliders,
            (0.0, bottomy, (p - pyramid_count / 2.0) * spacing),
            20,
            rad,
        )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
