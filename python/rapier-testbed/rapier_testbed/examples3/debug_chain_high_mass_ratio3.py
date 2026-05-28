"""Port of ``examples3d/debug_chain_high_mass_ratio3.rs``.

A spherical-joint chain ending with a ball 10x larger (1000x heavier
since mass scales with volume cubed). Each link runs 16 extra solver
iterations to avoid blowing up.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "High mass ratio: chain"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    use_articulations = False

    num = 17
    rad = 0.2
    body_handles = []

    for i in range(num):
        body_type = (
            rp.RigidBodyType.FIXED if i == 0 else rp.RigidBodyType.DYNAMIC
        )
        ball_rad = rad * 10.0 if i == num - 1 else rad
        shift1 = rad * 1.1
        shift2 = ball_rad + rad * 0.1
        if i == 0:
            z = 0.0
        else:
            z = (i - 1.0) * 2.0 * shift1 + shift1 + shift2

        body = (
            rp.RigidBody.new_body(body_type)
            .translation((0.0, 0.0, z))
            .additional_solver_iterations(16)
        )
        child_handle = bodies.insert(body)
        colliders.insert_with_parent(rp.Collider.ball(ball_rad), child_handle, bodies)

        if i > 0:
            parent_handle = body_handles[-1]
            if i == 1:
                joint = (
                    rp.SphericalJoint.builder()
                    .local_anchor2((0.0, 0.0, -shift1 * 2.0))
                    .build()
                )
            else:
                joint = (
                    rp.SphericalJoint.builder()
                    .local_anchor1((0.0, 0.0, shift1))
                    .local_anchor2((0.0, 0.0, -shift2))
                    .build()
                )

            if use_articulations:
                multibody_joints.insert(parent_handle, child_handle, joint, wake_up=True)
            else:
                impulse_joints.insert(parent_handle, child_handle, joint, wake_up=True)

        body_handles.append(child_handle)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
