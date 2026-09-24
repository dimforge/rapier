"""Port of examples3d/domino3.rs."""
from __future__ import annotations

import math

import rapier3d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Domino"


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

    num = 4000
    width = 1.0
    thickness = 0.1

    curr_angle = 0.0
    curr_rad = 10.0
    skip = 0
    two_pi = 2.0 * math.pi
    for i in range(num):
        perimeter = 2.0 * math.pi * curr_rad
        spacing = thickness * 4.0
        prev_angle = curr_angle
        curr_angle += 2.0 * math.pi * spacing / perimeter
        x = math.sin(curr_angle)
        z = math.cos(curr_angle)

        nudged = (curr_angle % two_pi) < (prev_angle % two_pi)
        tilt = 0.2 if (nudged or i == num - 1) else 0.0

        if skip == 0:
            rot = rp.Rotation3.from_axis_angle((0.0, 1.0, 0.0), curr_angle)
            tilt_axis = rot.transform_vector((0.0, 0.0, 1.0))
            tilt_rot = rp.Rotation3.from_axis_angle(tilt_axis, tilt)
            final_rot = tilt_rot * rot
            pose = rp.Isometry3(
                translation=(x * curr_rad, width * 2.0 + ground_height, z * curr_rad),
                rotation=final_rot,
            )
            body = rp.RigidBody.dynamic().position(pose)
            handle = bodies.insert(body)
            colliders.insert_with_parent(
                rp.Collider.cuboid(thickness, width * 2.0, width), handle, bodies
            )
        else:
            skip -= 1

        if nudged:
            skip = 5

        curr_rad += 1.5 / perimeter

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
