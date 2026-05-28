"""Port of examples2d/voxels2.rs.

binding gap: ``SharedShape::voxelized_mesh`` and
``ColliderBuilder::voxels_from_points`` are not yet exposed in the Python
bindings (no ``Collider.voxels*`` / ``SharedShape.voxelized_mesh``). The
port falls back to the dynamic-falling-objects scene without the voxel
ground / voxelized obstacle.
"""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Voxels"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    nx = 50
    for i in range(nx):
        for j in range(10):
            rb = rp.RigidBody.dynamic().translation(
                (i * 2.0 - nx / 2.0, 20.0 + j * 2.0)
            )
            rb_handle = bodies.insert(rb)
            ball_radius = 0.5
            kind = j % 3
            if kind == 0:
                co = rp.Collider.ball(ball_radius)
            elif kind == 1:
                co = rp.Collider.cuboid(ball_radius, ball_radius)
            else:
                co = rp.Collider.capsule_y(ball_radius, ball_radius)
            colliders.insert_with_parent(co, rb_handle, bodies)

    # Stand-in for the voxelized wavy floor — a wide cuboid ground.
    ground_handle = bodies.insert(rp.RigidBody.fixed().translation((0.0, -2.0)))
    colliders.insert_with_parent(
        rp.Collider.cuboid(200.0, 1.0), ground_handle, bodies
    )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 20.0), zoom=17.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
