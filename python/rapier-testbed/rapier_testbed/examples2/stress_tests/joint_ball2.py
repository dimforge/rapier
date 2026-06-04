"""Port of examples2d/stress_tests/joint_ball2.rs."""
from __future__ import annotations

import rapier2d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "(Stress test) joint ball"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.4
    numi = 100
    numk = 100
    shift = 1.0

    body_handles = []

    for k in range(numk):
        for i in range(numi):
            is_fixed = numk // 2 - 3 <= k <= numk // 2 + 3 and i == 0
            rb_builder = rp.RigidBody.fixed() if is_fixed else rp.RigidBody.dynamic()
            child_handle = bodies.insert(rb_builder.translation((k * shift, -i * shift)))
            colliders.insert_with_parent(rp.Collider.ball(rad), child_handle, bodies)

            if i > 0:
                parent = body_handles[-1]
                joint = rp.RevoluteJoint.builder().local_anchor2((0.0, shift))
                impulse_joints.insert(parent, child_handle, joint, wake_up=True)

            if k > 0:
                parent_index = len(body_handles) - numi
                parent = body_handles[parent_index]
                joint = rp.RevoluteJoint.builder().local_anchor2((-shift, 0.0))
                impulse_joints.insert(parent, child_handle, joint, wake_up=True)

            body_handles.append(child_handle)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(numk * rad, numi * -rad), zoom=5.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
