"""Port of examples2d/s2d_joint_grid.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Inspired by Solver 2D"
NAME = "Joint grid"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.4
    numi = 100
    numk = 100
    shift = 1.0

    handles = [None] * (numi * numk)
    index = 0

    for k in range(numk):
        for i in range(numi):
            is_fixed = numk // 2 - 3 <= k <= numk // 2 + 3 and i == 0
            rb_builder = rp.RigidBody.fixed() if is_fixed else rp.RigidBody.dynamic()
            handle = bodies.insert(rb_builder.translation((k * shift, -i * shift)))
            colliders.insert_with_parent(rp.Collider.ball(rad), handle, bodies)

            if i > 0:
                joint = (
                    rp.RevoluteJoint.builder()
                    .local_anchor1((0.0, -0.5 * shift))
                    .local_anchor2((0.0, 0.5 * shift))
                    .contacts_enabled(False)
                )
                impulse_joints.insert(
                    handles[index - 1], handle, joint, wake_up=True
                )

            if k > 0:
                joint = (
                    rp.RevoluteJoint.builder()
                    .local_anchor1((0.5 * shift, 0.0))
                    .local_anchor2((-0.5 * shift, 0.0))
                    .contacts_enabled(False)
                )
                impulse_joints.insert(
                    handles[index - numi], handle, joint, wake_up=True
                )

            handles[index] = handle
            index += 1

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
