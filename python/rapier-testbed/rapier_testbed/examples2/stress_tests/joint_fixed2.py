"""Port of examples2d/stress_tests/joint_fixed2.rs."""
from __future__ import annotations

import rapier2d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "(Stress test) joint fixed"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.4
    num = 30
    shift = 1.0

    body_handles = []

    for xx in range(4):
        x = xx * shift * (num + 2)

        for yy in range(4):
            y = yy * shift * (num + 4)

            for k in range(num):
                for i in range(num):
                    rb_builder = (
                        rp.RigidBody.fixed() if k == 0 else rp.RigidBody.dynamic()
                    )
                    child_handle = bodies.insert(
                        rb_builder.translation((x + k * shift, y - i * shift))
                    )
                    colliders.insert_with_parent(
                        rp.Collider.ball(rad), child_handle, bodies
                    )

                    if i > 0:
                        parent = body_handles[-1]
                        joint = (
                            rp.FixedJoint.builder()
                            .local_frame2(rp.Isometry2.from_translation(0.0, shift))
                        )
                        impulse_joints.insert(
                            parent, child_handle, joint, wake_up=True
                        )

                    if k > 0:
                        parent_index = len(body_handles) - num
                        parent = body_handles[parent_index]
                        joint = (
                            rp.FixedJoint.builder()
                            .local_frame2(rp.Isometry2.from_translation(-shift, 0.0))
                        )
                        impulse_joints.insert(
                            parent, child_handle, joint, wake_up=True
                        )

                    body_handles.append(child_handle)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(50.0, 50.0), zoom=5.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
