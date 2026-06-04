"""Port of examples3d/inverse_kinematics3.rs.

The Rust example drives the IK solver every frame using the mouse pointer
projected onto a camera plane. The Python mini-testbed has no mouse-input
plumbing, so we just build the kinematic chain — the IK targeting logic
itself relies on ``multibody.apply_displacements`` which is not yet
exposed (see ``MultibodyJointSet.inverse_kinematics_for_link``).
"""
from __future__ import annotations

import rapier3d as rp
from .._registry import register

CATEGORY = "Joints"
NAME = "Inverse kinematics"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 0.2
    ground_height = 0.01
    floor = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    floor_h = bodies.insert(floor)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size), floor_h, bodies
    )

    num_segments = 10
    last_body = bodies.insert(rp.RigidBody.fixed())

    for i in range(num_segments):
        size = 1.0 / num_segments
        new_body = bodies.insert(rp.RigidBody.dynamic().can_sleep(False))
        col = (
            rp.Collider.cuboid(size / 8.0, size / 2.0, size / 8.0)
            .density(0.0)
            .sensor(True)
        )
        colliders.insert_with_parent(col, new_body, bodies)

        link = (
            rp.SphericalJoint.builder()
            .local_anchor1((0.0, size / 2.0 * (1.0 if i != 0 else 0.0), 0.0))
            .local_anchor2((0.0, -size / 2.0, 0.0))
            .build()
        )
        multibody_joints.insert(last_body, new_body, link, wake_up=True)
        last_body = new_body

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((0.0, 0.5, 2.5), (0.0, 0.5, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
