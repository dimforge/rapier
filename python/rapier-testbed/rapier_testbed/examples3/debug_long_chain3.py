"""Port of ``examples3d/debug_long_chain3.rs``.

100-ball chain joined by spherical impulse joints. The first ball is
fixed, the rest dynamic.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Long chain"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()
    use_articulations = False

    num = 100
    rad = 0.2
    shift = rad * 2.2
    body_handles = []

    for i in range(num):
        body_type = (
            rp.RigidBodyType.FIXED if i == 0 else rp.RigidBodyType.DYNAMIC
        )
        body = rp.RigidBody.new_body(body_type).translation((0.0, 0.0, i * shift))
        child_handle = bodies.insert(body)
        colliders.insert_with_parent(rp.Collider.ball(rad), child_handle, bodies)

        if i > 0:
            parent_handle = body_handles[-1]
            if i == 1:
                joint = (
                    rp.SphericalJoint.builder()
                    .local_anchor2((0.0, 0.0, -shift))
                    .build()
                )
            else:
                joint = (
                    rp.SphericalJoint.builder()
                    .local_anchor1((0.0, 0.0, shift / 2.0))
                    .local_anchor2((0.0, 0.0, -shift / 2.0))
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
