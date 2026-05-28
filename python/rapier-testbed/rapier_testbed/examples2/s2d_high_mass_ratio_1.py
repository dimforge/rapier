"""Port of examples2d/s2d_high_mass_ratio_1.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Inspired by Solver 2D"
NAME = "High mass ratio 1"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    extent = 1.0
    friction = 0.5
    ground_width = 66.0 * extent

    ground_handle = bodies.insert(rp.RigidBody.fixed())
    colliders.insert_with_parent(
        rp.Collider.segment(
            (-0.5 * 2.0 * ground_width, 0.0), (0.5 * 2.0 * ground_width, 0.0)
        ).friction(friction),
        ground_handle,
        bodies,
    )

    for j in range(3):
        count = 10
        offset = -20.0 * extent + 2.0 * (count + 1) * extent * j
        y = extent

        while count > 0:
            for i in range(count):
                coeff = i - 0.5 * count
                yy = y + 2.0 if count == 1 else y
                position = (2.0 * coeff * extent + offset, yy)
                parent = bodies.insert(rp.RigidBody.dynamic().translation(position))
                density = (j + 1.0) * 100.0 if count == 1 else 1.0
                colliders.insert_with_parent(
                    rp.Collider.cuboid(extent, extent)
                    .density(density)
                    .friction(friction),
                    parent,
                    bodies,
                )

            count -= 1
            y += 2.0 * extent

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
