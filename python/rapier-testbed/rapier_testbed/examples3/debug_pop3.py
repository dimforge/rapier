"""Port of ``examples3d/debug_pop3.rs``.

Tiny scene: one dynamic cube starts inside a fixed cuboid, and pops out
when the simulation starts.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Pop"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 10.0
    ground_height = 10.0

    body = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    handle = bodies.insert(body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size), handle, bodies
    )

    body = rp.RigidBody.dynamic().can_sleep(False)
    handle = bodies.insert(body)
    colliders.insert_with_parent(rp.Collider.cuboid(1.0, 1.0, 1.0), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
