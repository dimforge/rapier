"""Port of examples3d/debug_two_cubes3.rs."""
from __future__ import annotations

import rapier3d as rp
from .._registry import register

CATEGORY = "Debug"
NAME = "Two cubes"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    dyn = rp.RigidBody.dynamic().translation((0.0, 2.0, 0.0))
    dyn_h = bodies.insert(dyn)
    colliders.insert_with_parent(rp.Collider.cuboid(0.5, 0.5, 0.5), dyn_h, bodies)

    fixed = rp.RigidBody.fixed()
    fixed_h = bodies.insert(fixed)
    colliders.insert_with_parent(rp.Collider.cuboid(0.5, 0.5, 0.5), fixed_h, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
