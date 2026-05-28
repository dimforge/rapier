"""Port of examples3d/debug_thin_cube_on_mesh3.rs."""
from __future__ import annotations

import numpy as np

import rapier3d as rp
from .._registry import register

CATEGORY = "Debug"
NAME = "Thin cube"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    # Flat 2x2 heightfield (all zeros).
    heights = np.zeros((2, 2), dtype=np.float32)
    colliders.insert(rp.Collider.heightfield(heights, (50.0, 1.0, 50.0)))

    # Falling tilted thin cube with high downward velocity + soft CCD.
    body = (
        rp.RigidBody.dynamic()
        .translation((0.0, 5.0, 0.0))
        .rotation((0.5, 0.0, 0.5))
        .linvel((0.0, -100.0, 0.0))
        .soft_ccd_prediction(10.0)
    )
    handle = bodies.insert(body)
    colliders.insert_with_parent(rp.Collider.cuboid(5.0, 0.015, 5.0), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
