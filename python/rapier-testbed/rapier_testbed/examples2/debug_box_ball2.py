"""Port of examples2d/debug_box_ball2.rs."""
from __future__ import annotations

import math

import rapier2d as rp
from .._registry import register

CATEGORY = "Debug"
NAME = "Box ball"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 1.0
    handle = bodies.insert(
        rp.RigidBody.fixed().translation((0.0, -rad)).rotation(math.pi / 4.0)
    )
    colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), handle, bodies)

    handle = bodies.insert(
        rp.RigidBody.dynamic().translation((0.0, 3.0 * rad)).can_sleep(False)
    )
    colliders.insert_with_parent(rp.Collider.ball(rad), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 0.0), zoom=50.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
