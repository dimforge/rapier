"""Port of ``examples3d/compound3.rs``.

Falling U-shapes built two ways: half are several cuboid colliders sharing
a parent body, half are a single compound collider with three subshapes.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Collisions"
NAME = "Compound"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 50.0
    ground_height = 0.1
    ground_body = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_handle = bodies.insert(ground_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size),
        ground_handle,
        bodies,
    )

    num = 8
    numy = 15
    rad = 0.2
    shift = rad * 4.0 + rad
    centerx = shift * (num // 2)
    centery = shift / 2.0
    centerz = shift * (num // 2)

    offset = -num * (rad * 2.0 + rad) * 0.5

    for j in range(numy):
        for i in range(num):
            for k in range(num):
                x = i * shift * 5.0 - centerx + offset
                y = j * (shift * 5.0) + centery + 3.0
                z = k * shift * 2.0 - centerz + offset

                body = rp.RigidBody.dynamic().translation((x, y, z))
                handle = bodies.insert(body)

                if j < numy // 2:
                    c1 = rp.Collider.cuboid(rad * 10.0, rad, rad)
                    c2 = rp.Collider.cuboid(rad, rad * 10.0, rad).translation(
                        (rad * 10.0, rad * 10.0, 0.0)
                    )
                    c3 = rp.Collider.cuboid(rad, rad * 10.0, rad).translation(
                        (-rad * 10.0, rad * 10.0, 0.0)
                    )
                    colliders.insert_with_parent(c1, handle, bodies)
                    colliders.insert_with_parent(c2, handle, bodies)
                    colliders.insert_with_parent(c3, handle, bodies)
                else:
                    shapes = [
                        (
                            rp.Isometry3.identity(),
                            rp.SharedShape.cuboid(rad * 10.0, rad, rad),
                        ),
                        (
                            rp.Isometry3.from_translation(
                                rad * 10.0, rad * 10.0, 0.0
                            ),
                            rp.SharedShape.cuboid(rad, rad * 10.0, rad),
                        ),
                        (
                            rp.Isometry3.from_translation(
                                -rad * 10.0, rad * 10.0, 0.0
                            ),
                            rp.SharedShape.cuboid(rad, rad * 10.0, rad),
                        ),
                    ]
                    colliders.insert_with_parent(
                        rp.Collider.compound(shapes), handle, bodies
                    )

        offset -= 0.05 * rad * (num - 1)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
