"""Port of examples3d/vehicle_joints3.rs.

Builds the joint-based car rig (body + four axles + four wheels + four
suspension joints + four wheel revolute joints). The mini-testbed has no
keyboard input, so the engine/steering inputs stay zero.
"""
from __future__ import annotations

import math

import numpy as np

import rapier3d as rp
from .._registry import register

CATEGORY = "Controls"
NAME = "Vehicle joints"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    # Wavy heightfield ground.
    ground_extent = (60.0, 0.4, 60.0)
    nsubdivs = 100
    heights = np.zeros((nsubdivs + 1, nsubdivs + 1), dtype=np.float32)
    for i in range(nsubdivs + 1):
        for j in range(nsubdivs + 1):
            heights[i, j] = (
                -math.cos(i * ground_extent[0] / nsubdivs / 2.0)
                - math.cos(j * ground_extent[2] / nsubdivs / 2.0)
            )
    col = (
        rp.Collider.heightfield(heights, ground_extent)
        .translation((-7.0, 0.0, 0.0))
        .friction(1.0)
    )
    colliders.insert(col)

    car_group = rp.Group.GROUP_1
    car_groups = rp.InteractionGroups(
        memberships=car_group,
        filter=~car_group,
        test_mode=rp.InteractionTestMode.AND,
    )

    wheel_params = [
        (0.6874, 0.2783, -0.7802),
        (-0.6874, 0.2783, -0.7802),
        (0.64, 0.2783, 1.0254),
        (-0.64, 0.2783, 1.0254),
    ]

    suspension_height = 0.12
    max_steering_angle = math.radians(35.0)
    wheel_radius = 0.28
    car_position = (0.0, wheel_radius + suspension_height, 0.0)
    body_position_in_car_space = (0.0, 0.4739, 0.0)
    body_position = (
        car_position[0] + body_position_in_car_space[0],
        car_position[1] + body_position_in_car_space[1],
        car_position[2] + body_position_in_car_space[2],
    )

    body_co = (
        rp.Collider.cuboid(0.65, 0.3, 0.9)
        .density(100.0)
        .collision_groups(car_groups)
    )
    body_handle = bodies.insert(
        rp.RigidBody.dynamic().translation(body_position).build()
    )
    colliders.insert_with_parent(body_co, body_handle, bodies)

    for wheel_id, wheel_pos in enumerate(wheel_params):
        is_front = wheel_id >= 2
        wheel_center = (
            car_position[0] + wheel_pos[0],
            car_position[1] + wheel_pos[1],
            car_position[2] + wheel_pos[2],
        )

        axle_mass = rp.MassProperties.from_ball(100.0, wheel_radius)
        axle = (
            rp.RigidBody.dynamic()
            .translation(wheel_center)
            .additional_mass_properties(axle_mass)
        )
        axle_h = bodies.insert(axle)

        fake_co = (
            rp.Collider.cylinder(wheel_radius / 2.0, wheel_radius)
            .rotation((0.0, 0.0, math.pi / 2.0))
            .sensor(True)
            .density(0.0)
            .collision_groups(rp.InteractionGroups.none())
        )

        wheel_co = (
            rp.Collider.ball(wheel_radius)
            .density(100.0)
            .collision_groups(car_groups)
            .friction(1.0)
        )
        wheel_h = bodies.insert(rp.RigidBody.dynamic().translation(wheel_center))
        colliders.insert_with_parent(wheel_co, wheel_h, bodies)
        colliders.insert_with_parent(fake_co, wheel_h, bodies)

        suspension_attachment = (
            wheel_pos[0] - body_position_in_car_space[0],
            wheel_pos[1] - body_position_in_car_space[1],
            wheel_pos[2] - body_position_in_car_space[2],
        )

        locked_axes = (
            rp.JointAxesMask.LIN_X
            | rp.JointAxesMask.LIN_Z
            | rp.JointAxesMask.ANG_X
            | rp.JointAxesMask.ANG_Z
        )
        if not is_front:
            locked_axes = locked_axes | rp.JointAxesMask.ANG_Y

        suspension = (
            rp.GenericJoint.builder(locked_axes=locked_axes)
            .limits(rp.JointAxis.LIN_Y, 0.0, suspension_height)
            .motor_position(rp.JointAxis.LIN_Y, 0.0, 1.0e4, 1.0e3)
            .local_anchor1(suspension_attachment)
        )
        if is_front:
            suspension = suspension.limits(
                rp.JointAxis.ANG_Y, -max_steering_angle, max_steering_angle
            )

        impulse_joints.insert(body_handle, axle_h, suspension.build(), wake_up=True)
        wheel_joint = rp.RevoluteJoint.builder(axis=(1.0, 0.0, 0.0)).build()
        impulse_joints.insert(axle_h, wheel_h, wheel_joint, wake_up=True)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
