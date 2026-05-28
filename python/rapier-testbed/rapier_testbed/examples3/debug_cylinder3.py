"""Port of ``examples3d/debug_cylinder3.rs``.

A single cylinder dropping onto a very thin cuboid floor — repros an
older EPA bug where the cylinder fell through the floor.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Cylinder"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 100.1
    ground_height = 0.1
    ground_body = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_handle = bodies.insert(ground_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size),
        ground_handle,
        bodies,
    )

    rad = 1.0
    centery = (rad * 2.0 + rad) / 2.0
    offset = -(1.0) * (rad * 2.0 + rad) * 0.5
    x = 0.0 + offset
    y = centery + 3.0
    z = 0.0 + offset

    body = rp.RigidBody.dynamic().translation((x, y, z))
    handle = bodies.insert(body)
    colliders.insert_with_parent(rp.Collider.cylinder(rad, rad), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
