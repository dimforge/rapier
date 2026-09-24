"""Port of examples3d/joint_motor_position3.rs."""
from __future__ import annotations

import math

import rapier3d as rp
from .._registry import register

CATEGORY = "Joints"
NAME = "Joint Motor Position"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground = bodies.insert(rp.RigidBody.fixed())

    # Row 1: rectangles with target angle motor.
    for num in range(9):
        x_pos = -6.0 + 1.5 * num
        body = (
            rp.RigidBody.dynamic()
            .translation((x_pos, 2.0, 0.0))
            .can_sleep(False)
        )
        h = bodies.insert(body)
        colliders.insert_with_parent(rp.Collider.cuboid(0.1, 0.5, 0.1), h, bodies)

        target_angle = -math.pi + math.pi / 4.0 * num
        joint = (
            rp.RevoluteJoint.builder(axis=(0.0, 0.0, 1.0))
            .local_anchor1((x_pos, 1.5, 0.0))
            .local_anchor2((0.0, -0.5, 0.0))
            .motor_position(target_angle, 1000.0, 150.0)
            .build()
        )
        impulse_joints.insert(ground, h, joint, wake_up=True)

    # Row 2: rectangles with a motor with limits.
    for num in range(8):
        x_pos = -6.0 + 1.5 * num
        body = (
            rp.RigidBody.dynamic()
            .translation((x_pos, 4.5, 0.0))
            .rotation((0.0, 0.0, math.pi))
            .can_sleep(False)
        )
        h = bodies.insert(body)
        colliders.insert_with_parent(rp.Collider.cuboid(0.1, 0.5, 0.1), h, bodies)

        max_angle_limit = -math.pi + math.pi / 4.0 * num
        joint = (
            rp.RevoluteJoint.builder(axis=(0.0, 0.0, 1.0))
            .local_anchor1((x_pos, 5.0, 0.0))
            .local_anchor2((0.0, -0.5, 0.0))
            .motor_velocity(1.5, 30.0)
            .motor_max_force(100.0)
            .limits(-math.pi, max_angle_limit)
            .build()
        )
        impulse_joints.insert(ground, h, joint, wake_up=True)

    testbed.set_world_with_params(
        bodies, colliders, impulse_joints, multibody_joints, (0.0, 0.0, 0.0)
    )
    testbed.look_at((15.0, 5.0, 42.0), (13.0, 1.0, 1.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
