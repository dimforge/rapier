"""Port of examples2d/trimesh2.rs.

The Rust example tessellates the Rapier SVG logo and instantiates the
resulting trimeshes. The Python port uses the ``utils.svg`` placeholder
which returns a few regular n-gon meshes — visually different from the
logo, but exercises the trimesh code path.
"""
from __future__ import annotations

import math

import rapier2d as rp
from .._registry import register
from .utils import svg as svg_utils

CATEGORY = "Collisions"
NAME = "Trimesh"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 25.0

    handle = bodies.insert(rp.RigidBody.fixed())
    colliders.insert_with_parent(rp.Collider.cuboid(ground_size, 1.2), handle, bodies)

    handle = bodies.insert(
        rp.RigidBody.fixed()
        .rotation(math.pi / 2.0)
        .translation((ground_size, ground_size))
    )
    colliders.insert_with_parent(rp.Collider.cuboid(ground_size, 1.2), handle, bodies)

    handle = bodies.insert(
        rp.RigidBody.fixed()
        .rotation(math.pi / 2.0)
        .translation((-ground_size, ground_size))
    )
    colliders.insert_with_parent(rp.Collider.cuboid(ground_size, 1.2), handle, bodies)

    rapier_logo_buffers = svg_utils.rapier_logo()

    for ith, (vtx, idx) in enumerate(rapier_logo_buffers):
        for k in range(5):
            collider = rp.Collider.trimesh(vtx, idx).contact_skin(0.2)
            rb = rp.RigidBody.dynamic().translation(
                (ith * 8.0 - 20.0, 20.0 + k * 11.0)
            )
            handle = bodies.insert(rb)
            colliders.insert_with_parent(collider, handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 20.0), zoom=17.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
