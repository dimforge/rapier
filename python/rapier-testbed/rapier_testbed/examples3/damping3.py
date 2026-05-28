"""Port of ``examples3d/damping3.rs``.

A ring of cubes with progressively-different linear/angular damping
spinning in zero-gravity space. The first body has minimum linear damping
and maximum angular damping; the relation flips going around the ring.
"""
from __future__ import annotations

import math

import rapier3d as rp

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
        ang = i * subdiv * math.pi * 2.0
        sx, cy = math.sin(ang), math.cos(ang)
        # Rust's sin_cos returns (sin, cos); we keep the same variable names.
        x, y = sx, cy
        body = (
            rp.RigidBody.dynamic()
            .translation((x, y, 0.0))
            .linvel((x * 10.0, y * 10.0, 0.0))
            .angvel((0.0, 0.0, 100.0))
            .linear_damping((i + 1) * subdiv * 10.0)
            .angular_damping((num - i) * subdiv * 10.0)
        )
        handle = bodies.insert(body)
        colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), handle, bodies)

    testbed.set_world_with_params(
        bodies, colliders, impulse_joints, multibody_joints, (0.0, 0.0, 0.0)
    )
    testbed.look_at((2.0, 2.5, 20.0), (2.0, 2.5, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
