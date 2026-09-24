"""Port of examples3d/rope_joints3.rs.

The Rust example also drives a kinematic-character controller via the
mouse. The mini-testbed lacks that input plumbing, so we keep the
character body in place and just exercise the tethered-ball + rope-joint
geometry.
"""
from __future__ import annotations

import rapier3d as rp
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

    # Floor + 4 wall slabs.
    f1 = bodies.insert(rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0)))
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size), f1, bodies
    )

    f2 = bodies.insert(
        rp.RigidBody.fixed().translation((-ground_size - ground_height, ground_height, 0.0))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_height, ground_height, ground_size), f2, bodies
    )

    f3 = bodies.insert(
        rp.RigidBody.fixed().translation((ground_size + ground_height, ground_height, 0.0))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_height, ground_height, ground_size), f3, bodies
    )

    f4 = bodies.insert(
        rp.RigidBody.fixed().translation((0.0, ground_height, -ground_size - ground_height))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_height), f4, bodies
    )

    f5 = bodies.insert(
        rp.RigidBody.fixed().translation((0.0, ground_height, ground_size + ground_height))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_height), f5, bodies
    )

    # Character (kinematic).
    char = rp.RigidBody.kinematic_position_based().translation((0.0, 0.3, 0.0))
    char_h = bodies.insert(char)
    colliders.insert_with_parent(rp.Collider.cuboid(0.15, 0.3, 0.15), char_h, bodies)

    # Tethered ball.
    rad = 0.04
    ball = rp.RigidBody.dynamic().translation((1.0, 1.0, 0.0))
    ball_h = bodies.insert(ball)
    colliders.insert_with_parent(rp.Collider.ball(rad), ball_h, bodies)

    joint = rp.RopeJoint.builder(2.0).build()
    impulse_joints.insert(char_h, ball_h, joint, wake_up=True)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
