"""Port of examples2d/inverse_kinematics2.rs.

The Rust example uses the mouse position via the testbed graphics layer to
drive a multibody chain through inverse kinematics each frame. The Python
mini-testbed has no mouse picking, so this port just builds the static
chain — the chain is still a useful debug-render demo.
"""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Joints"
NAME = "Inverse kinematics"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 1.0
    ground_height = 0.01

    floor_handle = bodies.insert(rp.RigidBody.fixed().translation((0.0, -ground_height)))
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height), floor_handle, bodies
    )

    num_segments = 10
    last_body = bodies.insert(rp.RigidBody.fixed())

    for i in range(num_segments):
        size = 1.0 / num_segments
        new_body = bodies.insert(rp.RigidBody.dynamic().can_sleep(False))
        colliders.insert_with_parent(
            rp.Collider.cuboid(size / 8.0, size / 2.0).density(0.0).sensor(True),
            new_body,
            bodies,
        )

        anchor1_y = (size / 2.0) if i != 0 else 0.0
        link_ab = (
            rp.RevoluteJoint.builder()
            .local_anchor1((0.0, anchor1_y))
            .local_anchor2((0.0, -size / 2.0))
        )

        multibody_joints.insert(last_body, new_body, link_ab, wake_up=True)
        last_body = new_body

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 0.0), zoom=300.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
