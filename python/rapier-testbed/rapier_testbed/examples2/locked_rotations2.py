"""Port of examples2d/locked_rotations2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Dynamics"
NAME = "Locked rotations"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 5.0
    ground_height = 0.1

    handle = bodies.insert(rp.RigidBody.fixed().translation((0.0, -ground_height)))
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height), handle, bodies
    )

    # A rectangle that only rotates (translations locked).
    handle = bodies.insert(
        rp.RigidBody.dynamic()
        .translation((0.0, 3.0))
        .locked_axes(rp.LockedAxes.TRANSLATION_LOCKED)
    )
    colliders.insert_with_parent(rp.Collider.cuboid(2.0, 0.6), handle, bodies)

    # A tilted capsule that cannot rotate.
    handle = bodies.insert(
        rp.RigidBody.dynamic()
        .translation((0.0, 5.0))
        .rotation(1.0)
        .locked_axes(rp.LockedAxes.ROTATION_LOCKED)
    )
    colliders.insert_with_parent(rp.Collider.capsule_y(0.6, 0.4), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 0.0), zoom=40.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
