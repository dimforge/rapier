"""Port of examples3d/spring_joints3.rs."""
from __future__ import annotations

import math

import rapier3d as rp
from .._registry import register

CATEGORY = "Joints"
NAME = "Spring Joints"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_h = bodies.insert(rp.RigidBody.fixed())

    num = 30
    radius = 0.5
    # Ball mass with default density (1.0) is (4/3 pi r^3).
    mass = rp.Ball(radius).__class__  # not strictly used, but mirror the original
    # Mirror the Rust formula: mass = Ball::new(r).mass_properties(1.0).mass()
    mass = rp.SharedShape.ball(radius).mass_properties(1.0).mass
    stiffness = 1.0e3
    critical_damping = 2.0 * math.sqrt(stiffness * mass)

    for i in range(num + 1):
        x_pos = -6.0 + 1.5 * i
        ball_pos = (x_pos, 4.5, 0.0)
        body = rp.RigidBody.dynamic().translation(ball_pos).can_sleep(False)
        h = bodies.insert(body)
        colliders.insert_with_parent(rp.Collider.ball(radius), h, bodies)

        damping_ratio = i / (num / 2.0)
        damping = damping_ratio * critical_damping
        joint = (
            rp.SpringJoint.builder(0.0, stiffness, damping)
            .local_anchor1((ball_pos[0], ball_pos[1] - 3.0, ball_pos[2]))
            .build()
        )
        impulse_joints.insert(ground_h, h, joint, wake_up=True)

        box = rp.RigidBody.dynamic().translation((ball_pos[0], ball_pos[1] + 5.0, ball_pos[2]))
        bh = bodies.insert(box)
        colliders.insert_with_parent(
            rp.Collider.cuboid(radius, radius, radius).density(100.0), bh, bodies
        )

    testbed.set_world_with_params(
        bodies, colliders, impulse_joints, multibody_joints, (0.0, -9.81, 0.0)
    )
    testbed.look_at((15.0, 5.0, 42.0), (13.0, 1.0, 1.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
