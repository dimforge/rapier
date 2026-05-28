"""Port of ``examples3d/debug_multibody_ang_motor_pos3.rs``.

A single dynamic cube hanging off a fixed cube via a spherical multibody
joint with target angular position around the X axis.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Multibody ang. motor pos."


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_body = rp.RigidBody.fixed().translation((0.0, 0.0, 0.0))
    ground_handle = bodies.insert(ground_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(1.0, 1.0, 1.0), ground_handle, bodies
    )

    part_body = rp.RigidBody.dynamic().position(
        rp.Isometry3.from_translation(0.0, 1.0, 0.0)
    )
    part_handle = bodies.insert(part_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(1.0, 1.0, 1.0).density(1.0), part_handle, bodies
    )

    joint = (
        rp.SphericalJoint.builder()
        .local_anchor1((0.0, 4.0, 0.0))
        .local_anchor2((0.0, 0.0, 0.0))
        .motor_position(rp.JointAxis.ANG_X, 1.0, 1000.0, 200.0)
        .motor_position(rp.JointAxis.ANG_Y, 0.0, 1000.0, 200.0)
        .motor_position(rp.JointAxis.ANG_Z, 0.0, 1000.0, 200.0)
        .build()
    )
    multibody_joints.insert(ground_handle, part_handle, joint, wake_up=True)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((20.0, 0.0, 0.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
