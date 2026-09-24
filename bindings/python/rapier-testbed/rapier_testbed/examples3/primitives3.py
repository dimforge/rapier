"""Port of ``examples3d/primitives3.rs``.

A grid of 8 x 8 x 20 mixed primitives (ball, round_cylinder, cone,
capsule_y) falling onto a wide ground cuboid. Same shape selection rule as
the Rust example: ``j % 5`` cycles primitive kind every vertical layer.
"""

from __future__ import annotations

import rapier3d as rp

from .._registry import register


CATEGORY = "Collisions"
NAME = "Primitives"


def init_world(testbed) -> None:  # type: ignore[no-untyped-def]
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    # Ground.
    ground_size = 100.1
    ground_height = 2.1

    ground_body = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_handle = bodies.insert(ground_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size),
        ground_handle,
        bodies,
    )

    # Falling primitive grid.
    num = 8
    rad = 1.0
    shiftx = rad * 2.0 + rad
    shifty = rad * 2.0 + rad
    shiftz = rad * 2.0 + rad
    centerx = shiftx * (num // 2)
    centery = shifty / 2.0
    centerz = shiftz * (num // 2)

    offset = -float(num) * (rad * 2.0 + rad) * 0.5

    for j in range(20):
        for i in range(num):
            for k in range(num):
                x = i * shiftx - centerx + offset
                y = j * shifty + centery + 3.0
                z = k * shiftz - centerz + offset

                body = rp.RigidBody.dynamic().translation((x, y, z))
                handle = bodies.insert(body)

                kind = j % 5
                if kind == 1:
                    collider = rp.Collider.ball(rad)
                elif kind == 2:
                    # Rounded cylinder is much faster than cylinder even
                    # for a small rounding margin (matches the Rust note).
                    collider = rp.Collider.round_cylinder(rad, rad, rad / 10.0)
                elif kind == 3:
                    collider = rp.Collider.cone(rad, rad)
                else:
                    collider = rp.Collider.capsule_y(rad, rad)

                colliders.insert_with_parent(collider, handle, bodies)

        offset -= 0.05 * rad * (num - 1)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


# Register at import time so ``rapier_testbed.EXAMPLES`` sees us.
register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    # ``python -m rapier_testbed.examples3.primitives3``
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
