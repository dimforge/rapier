"""Port of ``examples3d/ccd3.rs``.

Two fast balls slam through stacks of pyramids: one ball is a sensor
(triggers ``COLLISION_EVENTS``), the other a regular collider. Both
have CCD enabled so they don't tunnel through the walls.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Dynamics"
NAME = "CCD"


def _create_wall(bodies, colliders, offset, stack_height, half_extents):
    shift_y = half_extents[1] * 2.0
    shift_z = half_extents[2] * 2.0
    for i in range(stack_height):
        for j in range(i, stack_height):
            x = offset[0]
            y = i * shift_y + offset[1]
            z = (
                (i * shift_z / 2.0)
                + (j - i) * shift_z
                + offset[2]
                - stack_height * half_extents[2]
            )
            body = rp.RigidBody.dynamic().translation((x, y, z))
            handle = bodies.insert(body)
            colliders.insert_with_parent(
                rp.Collider.cuboid(half_extents[0], half_extents[1], half_extents[2]),
                handle,
                bodies,
            )


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 50.0
    ground_height = 0.1
    ground_body = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_handle = bodies.insert(ground_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size),
        ground_handle,
        bodies,
    )

    num_z = 8
    num_x = 5
    shift_y = ground_height + 0.5
    shift_z = (num_z + 2.0) * 2.0

    for i in range(num_x):
        x = i * 6.0
        _create_wall(bodies, colliders, (x, shift_y, 0.0), num_z, (0.5, 0.5, 1.0))
        _create_wall(bodies, colliders, (x, shift_y, shift_z), num_z, (0.5, 0.5, 1.0))

    # Sensor ball with CCD.
    sensor_collider = (
        rp.Collider.ball(1.0)
        .density(10.0)
        .sensor(True)
        .active_events(rp.ActiveEvents.COLLISION_EVENTS)
    )
    sensor_body = (
        rp.RigidBody.dynamic()
        .linvel((1000.0, 0.0, 0.0))
        .translation((-20.0, shift_y + 2.0, 0.0))
        .ccd_enabled(True)
    )
    sensor_handle = bodies.insert(sensor_body)
    colliders.insert_with_parent(sensor_collider, sensor_handle, bodies)

    # Second ball with CCD.
    ball_collider = rp.Collider.ball(1.0).density(10.0)
    ball_body = (
        rp.RigidBody.dynamic()
        .linvel((1000.0, 0.0, 0.0))
        .translation((-20.0, shift_y + 2.0, shift_z))
        .ccd_enabled(True)
    )
    handle = bodies.insert(ball_body)
    colliders.insert_with_parent(ball_collider, handle, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
