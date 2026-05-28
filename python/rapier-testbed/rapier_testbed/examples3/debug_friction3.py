"""Port of ``examples3d/debug_friction3.rs``.

A dynamic box launched along its local Z axis across a high-friction
floor. The box's rotation is baked into the launched velocity vector so
the impulse direction matches the visual heading.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Friction"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 100.0
    ground_height = 0.1
    ground_body = rp.RigidBody.fixed()
    ground_handle = bodies.insert(ground_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size).friction(1.5),
        ground_handle,
        bodies,
    )

    box_body = (
        rp.RigidBody.dynamic()
        .translation((0.0, 1.1, 0.0))
        .rotation((0.0, 0.3, 0.0))
    )
    box_handle = bodies.insert(box_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(2.0, 1.0, 3.0).friction(1.5), box_handle, bodies
    )

    # Apply rotation * Z * 50.
    box = bodies.get(box_handle)
    if box is not None:
        local_z = rp.Vec3(0.0, 0.0, 50.0)
        box.linvel = box.rotation.transform_vector(local_z)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
