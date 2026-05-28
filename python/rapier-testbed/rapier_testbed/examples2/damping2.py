"""Port of examples2d/damping2.rs."""
from __future__ import annotations

import math

import rapier2d as rp
from .._registry import register

CATEGORY = "Dynamics"
NAME = "Damping"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    num = 10
    rad = 0.2
    subdiv = 1.0 / num

    for i in range(num):
        angle = i * subdiv * math.pi * 2.0
        x = math.sin(angle)
        y = math.cos(angle)

        rb = (
            rp.RigidBody.dynamic()
            .translation((x, y))
            .linvel((x * 10.0, y * 10.0))
            .angvel(100.0)
            .linear_damping((i + 1) * subdiv * 10.0)
            .angular_damping((num - i) * subdiv * 10.0)
        )
        rb_handle = bodies.insert(rb)
        colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), rb_handle, bodies)

    testbed.set_world_with_params(
        bodies, colliders, impulse_joints, multibody_joints, (0.0, 0.0)
    )
    testbed.set_camera_2d(center=(3.0, 2.0), zoom=50.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
