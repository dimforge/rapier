"""Port of examples3d/newton_cradle3.rs."""
from __future__ import annotations

import rapier3d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Newton cradle"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    radius = 0.5
    length = 10.0 * radius
    n = 5

    for i in range(n):
        ball_pos = (i * 2.2 * radius, 0.0, 0.0)
        attach = (0.0, length, 0.0)
        if i >= n - 1:
            vel = (7.0, 0.0, 0.0)
        else:
            vel = (0.0, 0.0, 0.0)

        ground = bodies.insert(
            rp.RigidBody.fixed().translation(
                (ball_pos[0] + attach[0], ball_pos[1] + attach[1], ball_pos[2] + attach[2])
            )
        )
        body = rp.RigidBody.dynamic().translation(ball_pos).linvel(vel)
        h = bodies.insert(body)
        colliders.insert_with_parent(rp.Collider.ball(radius).restitution(1.0), h, bodies)

        joint = (
            rp.SphericalJoint.builder()
            .local_anchor2(attach)
            .build()
        )
        impulse_joints.insert(ground, h, joint, wake_up=True)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
