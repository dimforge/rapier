"""Port of examples2d/joints2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Joints"
NAME = "Joints"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.4
    numi = 10
    numk = 10
    shift = 1.0

    body_handles = []

    for k in range(numk):
        for i in range(numi):
            if i == 0 and k == 0:
                rb_builder = rp.RigidBody.fixed()
            else:
                rb_builder = rp.RigidBody.dynamic()
            rb = rb_builder.translation((k * shift, -i * shift))
            child_handle = bodies.insert(rb)
            colliders.insert_with_parent(rp.Collider.ball(rad), child_handle, bodies)

            # Vertical joint.
            if i > 0:
                parent_handle = body_handles[-1]
                joint = rp.RevoluteJoint.builder().local_anchor2((0.0, shift))
                impulse_joints.insert(parent_handle, child_handle, joint, wake_up=True)

            # Horizontal joint.
            if k > 0:
                parent_index = len(body_handles) - numi
                parent_handle = body_handles[parent_index]
                joint = rp.RevoluteJoint.builder().local_anchor2((-shift, 0.0))
                impulse_joints.insert(parent_handle, child_handle, joint, wake_up=True)

            body_handles.append(child_handle)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(numk * rad, numi * -rad), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
