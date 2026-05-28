"""Port of examples2d/s2d_high_mass_ratio_2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Inspired by Solver 2D"
NAME = "High mass ratio 2"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    extent = 1.0
    friction = 0.6
    ground_width = 66.0 * extent

    ground_handle = bodies.insert(rp.RigidBody.fixed())
    colliders.insert_with_parent(
        rp.Collider.segment(
            (-0.5 * 2.0 * ground_width, 0.0), (0.5 * 2.0 * ground_width, 0.0)
        ).friction(friction),
        ground_handle,
        bodies,
    )

    handle = bodies.insert(
        rp.RigidBody.dynamic().translation((-9.0 * extent, 0.5 * extent))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(0.5 * extent, 0.5 * extent).friction(friction),
        handle,
        bodies,
    )

    handle = bodies.insert(
        rp.RigidBody.dynamic().translation((9.0 * extent, 0.5 * extent))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(0.5 * extent, 0.5 * extent).friction(friction),
        handle,
        bodies,
    )

    handle = bodies.insert(
        rp.RigidBody.dynamic().translation((0.0, (10.0 + 16.0) * extent))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(10.0 * extent, 10.0 * extent).friction(friction),
        handle,
        bodies,
    )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
