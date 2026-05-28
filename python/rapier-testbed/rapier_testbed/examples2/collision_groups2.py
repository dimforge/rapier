"""Port of examples2d/collision_groups2.rs."""
from __future__ import annotations

import rapier2d as rp
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

    floor_handle = bodies.insert(rp.RigidBody.fixed().translation((0.0, -ground_height)))
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height), floor_handle, bodies
    )

    green_group = rp.InteractionGroups(
        memberships=rp.Group.GROUP_1, filter=rp.Group.GROUP_1
    )
    blue_group = rp.InteractionGroups(
        memberships=rp.Group.GROUP_2, filter=rp.Group.GROUP_2
    )

    green_floor = (
        rp.Collider.cuboid(1.0, 0.1)
        .translation((0.0, 1.0))
        .collision_groups(green_group)
    )
    colliders.insert_with_parent(green_floor, floor_handle, bodies)

    blue_floor = (
        rp.Collider.cuboid(1.0, 0.1)
        .translation((0.0, 2.0))
        .collision_groups(blue_group)
    )
    colliders.insert_with_parent(blue_floor, floor_handle, bodies)

    num = 8
    rad = 0.1
    shift = rad * 2.0
    centerx = shift * (num // 2)
    centery = 2.5

    for j in range(4):
        for i in range(num):
            x = i * shift - centerx
            y = j * shift + centery
            group = green_group if i % 2 == 0 else blue_group
            handle = bodies.insert(rp.RigidBody.dynamic().translation((x, y)))
            colliders.insert_with_parent(
                rp.Collider.cuboid(rad, rad).collision_groups(group), handle, bodies
            )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 1.0), zoom=100.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
