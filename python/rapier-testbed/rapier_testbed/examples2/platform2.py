"""Port of examples2d/platform2.rs."""
from __future__ import annotations

import math

import rapier2d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Platform"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 10.0
    ground_height = 0.1

    handle = bodies.insert(rp.RigidBody.fixed().translation((0.0, -ground_height)))
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height), handle, bodies
    )

    num = 6
    rad = 0.2
    shift = rad * 2.0
    centerx = shift * num / 2.0
    centery = shift / 2.0 + 3.04

    for i in range(num):
        for j in range(num):
            x = i * shift - centerx
            y = j * shift + centery
            handle = bodies.insert(rp.RigidBody.dynamic().translation((x, y)))
            colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), handle, bodies)

    velocity_based_platform_handle = bodies.insert(
        rp.RigidBody.kinematic_velocity_based().translation((-10.0 * rad, 1.5 + 0.8))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(rad * 10.0, rad), velocity_based_platform_handle, bodies
    )

    position_based_platform_handle = bodies.insert(
        rp.RigidBody.kinematic_position_based().translation(
            (-10.0 * rad, 2.0 + 1.5 + 0.8)
        )
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(rad * 10.0, rad), position_based_platform_handle, bodies
    )

    state = {"t": 0.0}

    def callback(tb) -> None:
        dt = tb._integration_parameters.dt
        state["t"] += dt
        velocity = (math.sin(state["t"]) * 5.0, math.sin(state["t"] * 5.0))

        platform = tb.bodies.get(velocity_based_platform_handle)
        if platform is not None:
            platform.linvel = velocity

        platform = tb.bodies.get(position_based_platform_handle)
        if platform is not None:
            tr = platform.translation
            new_pos = rp.Isometry2(
                (float(tr.x) + velocity[0] * dt, float(tr.y) + velocity[1] * dt),
                0.0,
            )
            platform.position = new_pos

    testbed.add_callback(callback)
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 1.0), zoom=40.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
