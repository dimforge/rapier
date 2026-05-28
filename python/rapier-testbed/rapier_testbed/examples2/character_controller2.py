"""Port of examples2d/character_controller2.rs.

The Rust example wires a keyboard-driven character via
``utils::character`` and ``KinematicCharacterController``. The Python
mini-testbed has no keyboard / egui layer, so the port keeps the static
scenery (floor + stairs + slopes + walls + tilting platform + heightfield
+ revolute-limited dynamic body) without the interactive controls.
"""
from __future__ import annotations

import math

import numpy as np

import rapier2d as rp
from .._registry import register

CATEGORY = "Controls"
NAME = "Character controller"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 5.0
    ground_height = 0.1

    floor_handle = bodies.insert(
        rp.RigidBody.fixed().translation((0.0, -ground_height))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height), floor_handle, bodies
    )

    character_handle = bodies.insert(
        rp.RigidBody.kinematic_position_based()
        .translation((-3.0, 5.0))
        .gravity_scale(10.0)
        .soft_ccd_prediction(10.0)
    )
    colliders.insert_with_parent(
        rp.Collider.capsule_y(0.3, 0.15), character_handle, bodies
    )

    num = 8
    rad = 0.1
    shift = rad * 2.0
    centerx = shift * (num // 2)
    centery = rad
    for j in range(4):
        for i in range(num):
            x = i * shift - centerx
            y = j * shift + centery
            h = bodies.insert(rp.RigidBody.dynamic().translation((x, y)))
            colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), h, bodies)

    # Stairs.
    stair_width = 1.0
    stair_height = 0.1
    for i in range(10):
        x = i * stair_width / 2.0
        y = i * stair_height * 1.5 + 3.0
        colliders.insert(
            rp.Collider.cuboid(stair_width / 2.0, stair_height / 2.0).translation((x, y))
        )

    # Slope we can climb.
    slope_angle = 0.2
    slope_size = 2.0
    colliders.insert(
        rp.Collider.cuboid(slope_size, ground_height)
        .translation((ground_size + slope_size, -ground_height + 0.4))
        .rotation(slope_angle)
    )

    # Impossible slope.
    impossible_slope_angle = 0.9
    impossible_slope_size = 2.0
    colliders.insert(
        rp.Collider.cuboid(slope_size, ground_height)
        .translation(
            (
                ground_size + slope_size * 2.0 + impossible_slope_size - 0.9,
                -ground_height + 2.3,
            )
        )
        .rotation(impossible_slope_angle)
    )

    # Wall.
    wall_angle = math.pi / 2.0
    wall_size = 2.0
    wall_pos = (
        ground_size + slope_size * 2.0 + impossible_slope_size + 0.35,
        -ground_height + 2.5 * 2.3,
    )
    colliders.insert(
        rp.Collider.cuboid(wall_size, ground_height)
        .translation(wall_pos)
        .rotation(wall_angle)
    )
    colliders.insert(rp.Collider.cuboid(wall_size, ground_height).translation(wall_pos))

    # Moving platform.
    platform_handle = bodies.insert(
        rp.RigidBody.kinematic_velocity_based().translation((-8.0, 0.0))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(2.0, ground_height), platform_handle, bodies
    )

    # Heightfield ground.
    hf_size = (10.0, 1.0)
    nsubdivs = 20
    heights = np.array(
        [math.cos(i * hf_size[0] / nsubdivs / 2.0) * 1.5 for i in range(nsubdivs + 1)],
        dtype=np.float32,
    )
    colliders.insert(
        rp.Collider.heightfield(heights, hf_size).translation((-8.0, 5.0))
    )

    # Tilting limited-revolute body.
    ground = bodies.insert(rp.RigidBody.fixed().translation((0.0, 5.0)))
    body = bodies.insert(rp.RigidBody.dynamic().translation((0.0, 5.0)))
    colliders.insert_with_parent(rp.Collider.cuboid(1.0, 0.1), body, bodies)
    joint = rp.RevoluteJoint.builder().limits(-0.3, 0.3)
    impulse_joints.insert(ground, body, joint, wake_up=True)

    state = {"t": 0.0}

    def cb(tb) -> None:
        state["t"] += tb._integration_parameters.dt
        linvel = (
            math.sin(state["t"] * 2.0) * 2.0,
            math.sin(state["t"] * 5.0) * 1.5,
        )
        platform = tb.bodies.get(platform_handle)
        if platform is not None:
            platform.linvel = linvel
            tb.bodies.replace(platform_handle, platform)

    testbed.add_callback(cb)
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 1.0), zoom=100.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
