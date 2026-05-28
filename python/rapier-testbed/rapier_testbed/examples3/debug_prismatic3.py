"""Port of ``examples3d/debug_prismatic3.rs``.

A "car" made of a chassis with four wheels held by prismatic joints
configured as soft suspensions. A small piece of gravel sits under one
wheel to compress the suspension.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Prismatic"


def _prismatic_repro(bodies, colliders, impulse_joints, box_center):
    box_body = rp.RigidBody.dynamic().translation(box_center)
    box_handle = bodies.insert(box_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(1.0, 0.25, 1.0), box_handle, bodies
    )

    wheel_y = -1.0
    wheel_positions = [
        (1.0, wheel_y, -1.0),
        (-1.0, wheel_y, -1.0),
        (1.0, wheel_y, 1.0),
        (-1.0, wheel_y, 1.0),
    ]

    for pos in wheel_positions:
        wheel_pos_in_world = (
            box_center[0] + pos[0],
            box_center[1] + pos[1],
            box_center[2] + pos[2],
        )
        wheel_body = rp.RigidBody.dynamic().translation(wheel_pos_in_world)
        wheel_handle = bodies.insert(wheel_body)
        colliders.insert_with_parent(rp.Collider.ball(0.5), wheel_handle, bodies)

        stiffness, damping = 0.05, 0.2
        prismatic = (
            rp.PrismaticJoint.builder(axis=(0.0, 1.0, 0.0))
            .local_anchor1(pos)
            .motor_position(0.0, stiffness, damping)
            .build()
        )
        impulse_joints.insert(box_handle, wheel_handle, prismatic, wake_up=True)

    # Gravel under one wheel.
    gravel_body = rp.RigidBody.dynamic().translation(
        (box_center[0] + 1.0, box_center[1] - 2.4, -1.0)
    )
    gravel_handle = bodies.insert(gravel_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(0.5, 0.1, 0.5), gravel_handle, bodies
    )


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

    _prismatic_repro(bodies, colliders, impulse_joints, (0.0, 5.0, 0.0))

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
