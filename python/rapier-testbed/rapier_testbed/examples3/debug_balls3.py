"""Port of ``examples3d/debug_balls3.rs``.

A box of balls: the outer shell is fixed, the inner balls dynamic. The
dynamic ones start at double the vertical spacing so they collapse into
the fixed lattice on the first step.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Balls"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    num_j = 10
    num_ik = 10
    rad = 0.5
    shift = rad * 2.0
    centerx = shift * num_ik / 2.0
    centery = shift / 2.0
    centerz = shift * num_ik / 2.0

    for i in range(num_ik):
        for j in range(num_j):
            for k in range(num_ik):
                x = i * shift - centerx
                z = k * shift - centerz

                is_fixed = (
                    j == 0 or i == 0 or k == 0 or i == num_ik - 1 or k == num_ik - 1
                )
                body_type = (
                    rp.RigidBodyType.FIXED if is_fixed else rp.RigidBodyType.DYNAMIC
                )
                y = (
                    j * shift + centery
                    if is_fixed
                    else j * shift * 2.0 + centery
                )

                body = (
                    rp.RigidBody.new_body(body_type)
                    .translation((x, y, z))
                    .can_sleep(False)
                )
                handle = bodies.insert(body)
                colliders.insert_with_parent(
                    rp.Collider.ball(rad).friction(0.0), handle, bodies
                )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
