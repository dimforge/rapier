"""Port of examples2d/rope_joints2.rs.

The original example wires a manually-driven character via
``utils::character``. The Python port keeps the static scene (tether on a
kinematic character body) but omits the keyboard-driven controller; this
keeps the example useful for debug-rendering and headless smoke testing.
"""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Joints"
NAME = "Rope Joints"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 0.75
    ground_height = 0.1

    handle = bodies.insert(rp.RigidBody.fixed().translation((0.0, -ground_height)))
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height), handle, bodies
    )

    handle = bodies.insert(
        rp.RigidBody.fixed().translation((-ground_size - ground_height, ground_size))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_height, ground_size), handle, bodies
    )

    handle = bodies.insert(
        rp.RigidBody.fixed().translation((ground_size + ground_height, ground_size))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_height, ground_size), handle, bodies
    )

    character_handle = bodies.insert(
        rp.RigidBody.kinematic_position_based().translation((0.0, 0.3))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(0.15, 0.3), character_handle, bodies
    )

    rad = 0.04
    child_handle = bodies.insert(rp.RigidBody.dynamic().translation((1.0, 1.0)))
    colliders.insert_with_parent(rp.Collider.ball(rad), child_handle, bodies)

    joint = rp.RopeJoint.builder(2.0).local_anchor2((0.0, 0.0))
    impulse_joints.insert(character_handle, child_handle, joint, wake_up=True)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 1.0), zoom=100.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
