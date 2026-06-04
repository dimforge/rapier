"""Port of examples2d/joint_motor_position2.rs."""
from __future__ import annotations

import math

import rapier2d as rp
from .._registry import register

CATEGORY = "Joints"
NAME = "Joint motor position"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_handle = bodies.insert(rp.RigidBody.fixed())

    # Rectangles on motor (position-target).
    for num in range(9):
        x_pos = -6.0 + 1.5 * num
        handle = bodies.insert(
            rp.RigidBody.dynamic().translation((x_pos, 2.0)).can_sleep(False)
        )
        colliders.insert_with_parent(rp.Collider.cuboid(0.1, 0.5), handle, bodies)

        joint = (
            rp.RevoluteJoint.builder()
            .local_anchor1((x_pos, 1.5))
            .local_anchor2((0.0, -0.5))
            .motor_position(-math.pi + math.pi / 4.0 * num, 1000.0, 150.0)
        )
        impulse_joints.insert(ground_handle, handle, joint, wake_up=True)

    # Rectangles on motor (velocity + limits).
    for num in range(8):
        x_pos = -6.0 + 1.5 * num
        handle = bodies.insert(
            rp.RigidBody.dynamic()
            .translation((x_pos, 4.5))
            .rotation(math.pi)
            .can_sleep(False)
        )
        colliders.insert_with_parent(rp.Collider.cuboid(0.1, 0.5), handle, bodies)

        joint = (
            rp.RevoluteJoint.builder()
            .local_anchor1((x_pos, 5.0))
            .local_anchor2((0.0, -0.5))
            .motor_velocity(1.5, 30.0)
            .motor_max_force(100.0)
            .limits(-math.pi, -math.pi + math.pi / 4.0 * num)
        )
        impulse_joints.insert(ground_handle, handle, joint, wake_up=True)

    testbed.set_world_with_params(
        bodies, colliders, impulse_joints, multibody_joints, (0.0, 0.0)
    )
    testbed.set_camera_2d(center=(0.0, 0.0), zoom=40.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
