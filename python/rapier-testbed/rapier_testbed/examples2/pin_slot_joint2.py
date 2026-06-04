"""Port of examples2d/pin_slot_joint2.rs.

The Rust version drives the character with the keyboard via
``utils::character``. The Python port keeps the static scene (pin-slot
joint + tethered cube) but omits the keyboard-driven controller.
"""
from __future__ import annotations

import math

import rapier2d as rp
from .._registry import register

CATEGORY = "Joints"
NAME = "Pin Slot Joint"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 3.0
    ground_height = 0.1

    floor_handle = bodies.insert(
        rp.RigidBody.fixed().translation((0.0, -ground_height))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height), floor_handle, bodies
    )

    character_handle = bodies.insert(
        rp.RigidBody.kinematic_position_based().translation((0.0, 0.3))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(0.15, 0.3), character_handle, bodies
    )

    rad = 0.4
    cube_handle = bodies.insert(rp.RigidBody.dynamic().translation((1.0, 1.0)))
    colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), cube_handle, bodies)

    ball_handle = bodies.insert(rp.RigidBody.dynamic().translation((1.0, 1.0)))
    colliders.insert_with_parent(rp.Collider.ball(0.1), ball_handle, bodies)

    fixed_joint = (
        rp.FixedJoint.builder()
        .local_anchor1((0.0, 0.0))
        .local_anchor2((0.0, -0.4))
    )
    impulse_joints.insert(cube_handle, ball_handle, fixed_joint, wake_up=True)

    axis_len = math.sqrt(2.0)
    axis = (1.0 / axis_len, 1.0 / axis_len)
    pin_slot_joint = (
        rp.PinSlotJoint.builder(axis)
        .local_anchor1((2.0, 2.0))
        .local_anchor2((0.0, 0.4))
        .limits(-1.0, float("inf"))
    )
    impulse_joints.insert(
        character_handle, cube_handle, pin_slot_joint, wake_up=True
    )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 1.0), zoom=100.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
