"""Port of ``examples3d/debug_articulations3.rs``.

A ``num x num`` grid of capsules joined vertically by spherical multibody
joints and horizontally by spherical impulse joints, sitting on a slightly
tilted dynamic platform stacked on a fixed one.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Multibody joints"


def _create_ball_articulations(
    bodies, colliders, impulse_joints, multibody_joints, num
):
    rad = 0.4
    shift = 1.0
    body_handles = []

    for k in range(num):
        for i in range(num):
            body_type = (
                rp.RigidBodyType.FIXED if i == 0 else rp.RigidBodyType.DYNAMIC
            )
            body = rp.RigidBody.new_body(body_type).translation(
                (k * shift, 0.0, i * shift * 2.0)
            )
            child_handle = bodies.insert(body)
            colliders.insert_with_parent(
                rp.Collider.capsule_z(rad * 1.25, rad), child_handle, bodies
            )

            # Vertical multibody joint.
            if i > 0:
                parent_handle = body_handles[-1]
                joint = (
                    rp.SphericalJoint.builder()
                    .local_anchor2((0.0, 0.0, -shift * 2.0))
                    .build()
                )
                multibody_joints.insert(parent_handle, child_handle, joint, wake_up=True)

            # Horizontal impulse joint.
            if k > 0 and i > 0:
                parent_index = len(body_handles) - num
                parent_handle = body_handles[parent_index]
                joint = (
                    rp.SphericalJoint.builder()
                    .local_anchor2((-shift, 0.0, 0.0))
                    .build()
                )
                impulse_joints.insert(parent_handle, child_handle, joint, wake_up=True)

            body_handles.append(child_handle)


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    # Fixed lower platform.
    colliders.insert(
        rp.Collider.cuboid(30.0, 0.01, 30.0)
        .translation((0.0, -3.02, 0.0))
        .rotation((0.1, 0.0, 0.1))
    )

    # Dynamic upper platform.
    plat_body = rp.RigidBody.dynamic()
    plat_handle = bodies.insert(plat_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(30.0, 0.01, 30.0)
        .translation((0.0, -3.0, 0.0))
        .rotation((0.1, 0.0, 0.1)),
        plat_handle,
        bodies,
    )

    _create_ball_articulations(bodies, colliders, impulse_joints, multibody_joints, 15)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((15.0, 5.0, 42.0), (13.0, 1.0, 1.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
