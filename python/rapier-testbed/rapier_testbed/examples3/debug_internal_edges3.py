"""Port of ``examples3d/debug_internal_edges3.rs``.

Three dynamic bodies (ball, cube, cylinder) thrown at a flat
heightfield. The Rust version sets ``HeightFieldFlags::all()`` to mark
all internal edges for the contact pruning pass; the Python bindings
don't expose those flags yet, so we use the default heightfield.
"""
from __future__ import annotations

import math

import numpy as np

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Internal edges"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    heights = np.zeros((100, 100), dtype=np.float32)
    hf = rp.Collider.heightfield(heights, (60.0, 1.0, 60.0))
    colliders.insert(hf)

    # Ball thrown down at angle.
    body = (
        rp.RigidBody.dynamic()
        .translation((4.0, 0.5, 0.0))
        .linvel((0.0, -40.0, 20.0))
        .can_sleep(False)
    )
    handle = bodies.insert(body)
    colliders.insert_with_parent(rp.Collider.ball(0.5), handle, bodies)

    # Cube falling more gently.
    body = (
        rp.RigidBody.dynamic()
        .translation((-3.0, 5.0, 0.0))
        .linvel((0.0, -4.0, 20.0))
        .can_sleep(False)
    )
    handle = bodies.insert(body)
    colliders.insert_with_parent(rp.Collider.cuboid(0.5, 0.5, 0.5), handle, bodies)

    # Cylinder, lying on its side.
    body = (
        rp.RigidBody.dynamic()
        .translation((8.0, 0.2, 0.0))
        .linvel((0.0, -4.0, 20.0))
        .can_sleep(False)
    )
    handle = bodies.insert(body)
    colliders.insert_with_parent(
        rp.Collider.cylinder(0.5, 0.2).rotation((0.0, 0.0, math.pi / 2.0)),
        handle,
        bodies,
    )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
