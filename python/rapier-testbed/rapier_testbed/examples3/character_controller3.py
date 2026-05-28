"""Port of ``examples3d/character_controller3.rs``.

A kinematic character on top of a heightfield, stairs and a moving
platform. The Python testbed has no keyboard/egui plumbing, so the
character itself stands still — but the whole scene is built and the
moving platform's velocity callback ticks each frame.
"""
from __future__ import annotations

import math

import numpy as np

import rapier3d as rp

from .._registry import register
from .utils.character import CharacterControlMode

CATEGORY = "Controls"
NAME = "Character controller"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    scale = 1.0
    ground_size = 5.0
    ground_height = 0.1

    floor_body = rp.RigidBody.fixed().translation(
        (0.0, -ground_height * scale, 0.0)
    )
    floor_handle = bodies.insert(floor_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size * scale, ground_height * scale, ground_size * scale),
        floor_handle,
        bodies,
    )

    wall_body = rp.RigidBody.fixed().translation(
        (0.0, -ground_height * scale, -ground_size * scale)
    )
    wall_handle = bodies.insert(wall_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size * scale, ground_size * scale, ground_height * scale),
        wall_handle,
        bodies,
    )

    # Character.
    char_body = (
        rp.RigidBody.kinematic_position_based()
        .translation((0.0, 0.5 * scale, 0.0))
        .gravity_scale(10.0)
        .soft_ccd_prediction(10.0)
    )
    character_handle = bodies.insert(char_body)
    colliders.insert_with_parent(
        rp.Collider.capsule_y(0.3 * scale, 0.15 * scale), character_handle, bodies
    )

    # Cubes.
    num = 8
    rad = 0.1
    shift = rad * 2.0
    centerx = shift * (num // 2)
    centery = rad
    for j in range(4):
        for k in range(4):
            for i in range(num):
                x = i * shift - centerx
                y = j * shift + centery
                z = k * shift + centerx
                body = rp.RigidBody.dynamic().translation(
                    (x * scale, y * scale, z * scale)
                )
                handle = bodies.insert(body)
                colliders.insert_with_parent(
                    rp.Collider.cuboid(rad * scale, rad * scale, rad * scale),
                    handle,
                    bodies,
                )

    # Stairs.
    stair_width = 1.0
    stair_height = 0.1
    for i in range(10):
        x = i * stair_width / 2.0
        y = i * stair_height * 1.5 + 3.0
        col = (
            rp.Collider.cuboid(
                stair_width / 2.0 * scale,
                stair_height / 2.0 * scale,
                stair_width * scale,
            )
            .translation((x * scale, y * scale, 0.0))
        )
        colliders.insert(col)

    # Climbable slope.
    slope_angle = 0.2
    slope_size = 2.0
    col = (
        rp.Collider.cuboid(slope_size, ground_height, slope_size)
        .translation((0.1 + slope_size, -ground_height + 0.4, 0.0))
        .rotation((0.0, 0.0, slope_angle))
    )
    colliders.insert(col)

    # Steep slope.
    impossible_slope_angle = 0.6
    impossible_slope_size = 2.0
    col = (
        rp.Collider.cuboid(slope_size * scale, ground_height * scale, ground_size * scale)
        .translation(
            (
                (0.1 + slope_size * 2.0 + impossible_slope_size - 0.9) * scale,
                (-ground_height + 1.7) * scale,
                0.0,
            )
        )
        .rotation((0.0, 0.0, impossible_slope_angle))
    )
    colliders.insert(col)

    # Moving platform.
    platform_body = rp.RigidBody.kinematic_velocity_based().translation(
        (-8.0 * scale, 0.0, 0.0)
    )
    platform_handle = bodies.insert(platform_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(2.0 * scale, ground_height * scale, 2.0 * scale),
        platform_handle,
        bodies,
    )

    # Heightfield.
    nsubdivs = 20
    heights = np.zeros((nsubdivs + 1, nsubdivs + 1), dtype=np.float32)
    for i in range(nsubdivs + 1):
        for j in range(nsubdivs + 1):
            heights[i, j] = (
                math.cos(i * 10.0 / nsubdivs / 2.0)
                + math.cos(j * 10.0 / nsubdivs / 2.0)
            )
    hf = rp.Collider.heightfield(heights, (10.0 * scale, 1.0 * scale, 10.0 * scale)).translation(
        (-8.0 * scale, 5.0 * scale, 0.0)
    )
    colliders.insert(hf)

    # Tilting body with a revolute joint.
    ground_anchor = rp.RigidBody.fixed().translation((0.0, 5.0 * scale, 0.0))
    ground_anchor_handle = bodies.insert(ground_anchor)
    tilt_body = rp.RigidBody.dynamic().translation((0.0, 5.0 * scale, 0.0))
    tilt_handle = bodies.insert(tilt_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(1.0 * scale, 0.1 * scale, 2.0 * scale), tilt_handle, bodies
    )
    rev = rp.RevoluteJoint.builder(axis=(0.0, 0.0, 1.0)).limits(-0.3, 0.3).build()
    impulse_joints.insert(ground_anchor_handle, tilt_handle, rev, wake_up=True)

    # Move the platform every frame (mirrors the Rust callback).
    def _platform_callback(tb) -> None:
        t = tb._step_count * tb._integration_parameters.dt
        linvel = (
            math.sin(t * 2.0) * 2.0 * scale,
            math.sin(t * 5.0) * 1.5 * scale,
            0.0,
        )
        plat = tb.bodies.get(platform_handle)
        if plat is not None:
            plat.linvel = rp.Vec3(*linvel)

    testbed.add_callback(_platform_callback)

    # Character control mode object kept around for parity even though the
    # mini-testbed has no keyboard input wired up yet.
    _control_mode = CharacterControlMode.kinematic(0.1)
    _controller = rp.KinematicCharacterController(
        max_slope_climb_angle=impossible_slope_angle - 0.02,
        min_slope_slide_angle=impossible_slope_angle - 0.02,
        slide=True,
    )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
