"""Port of ``examples3d/collision_groups3.rs``.

Two coloured floors and a stack of cubes split into ``GREEN`` and ``BLUE``
interaction groups. Cubes only land on the floor matching their group.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Collisions"
NAME = "Collision groups"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 5.0
    ground_height = 0.1
    floor_body = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    floor_handle = bodies.insert(floor_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size),
        floor_handle,
        bodies,
    )

    green_group = rp.InteractionGroups(
        memberships=rp.Group.GROUP_1,
        filter=rp.Group.GROUP_1,
        test_mode=rp.InteractionTestMode.AND,
    )
    blue_group = rp.InteractionGroups(
        memberships=rp.Group.GROUP_2,
        filter=rp.Group.GROUP_2,
        test_mode=rp.InteractionTestMode.AND,
    )

    green_floor = (
        rp.Collider.cuboid(1.0, 0.1, 1.0)
        .translation((0.0, 1.0, 0.0))
        .collision_groups(green_group)
    )
    colliders.insert_with_parent(green_floor, floor_handle, bodies)

    blue_floor = (
        rp.Collider.cuboid(1.0, 0.1, 1.0)
        .translation((0.0, 2.0, 0.0))
        .collision_groups(blue_group)
    )
    colliders.insert_with_parent(blue_floor, floor_handle, bodies)

    num = 8
    rad = 0.1
    shift = rad * 2.0
    centerx = shift * (num // 2)
    centery = 2.5
    centerz = shift * (num // 2)

    for j in range(4):
        for i in range(num):
            for k in range(num):
                x = i * shift - centerx
                y = j * shift + centery
                z = k * shift - centerz

                group = green_group if k % 2 == 0 else blue_group
                body = rp.RigidBody.dynamic().translation((x, y, z))
                handle = bodies.insert(body)
                colliders.insert_with_parent(
                    rp.Collider.cuboid(rad, rad, rad).collision_groups(group),
                    handle,
                    bodies,
                )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
