"""Port of examples2d/debug_total_overlap2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Debug"
NAME = "Total overlap"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.5
    for _ in range(100):
        handle = bodies.insert(rp.RigidBody.dynamic())
        colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 0.0), zoom=50.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
