"""Port of examples3d/platform3.rs."""
from __future__ import annotations

import math

import rapier3d as rp
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
    ground = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_h = bodies.insert(ground)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size), ground_h, bodies
    )

    num = 6
    rad = 0.2
    shift = rad * 2.0
    centerx = shift * num / 2.0
    centerz = shift * num / 2.0

    for i in range(num):
        for j in range(num):
            for k in range(num):
                centery = 5.0 if j >= num // 2 else 3.0
                x = i * shift - centerx
                y = j * shift + centery
                z = k * shift - centerz
                body = rp.RigidBody.dynamic().translation((x, y, z))
                h = bodies.insert(body)
                colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), h, bodies)

    # Kinematic platforms.
    vb_platform_h = bodies.insert(
        rp.RigidBody.kinematic_velocity_based().translation((0.0, 1.5 + 0.8, 0.0))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(rad * 10.0, rad, rad * 10.0), vb_platform_h, bodies
    )

    pb_platform_h = bodies.insert(
        rp.RigidBody.kinematic_position_based().translation((0.0, 3.0 + 1.5 + 0.8, 0.0))
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(rad * 10.0, rad, rad * 10.0), pb_platform_h, bodies
    )

    def _cb(tb) -> None:
        t = tb._step_count * tb._integration_parameters.dt
        velocity = (0.0, math.cos(t * 2.0), math.sin(t) * 2.0)

        platform = tb.bodies.get(vb_platform_h)
        if platform is not None:
            platform.linvel = rp.Vec3(*velocity)
            platform.angvel = rp.Vec3(0.0, 1.0, 0.0)

        platform = tb.bodies.get(pb_platform_h)
        if platform is not None:
            dt = tb._integration_parameters.dt
            tra = platform.translation
            new_tra = (
                tra.x - velocity[0] * dt,
                tra.y - velocity[1] * dt,
                tra.z - velocity[2] * dt,
            )
            platform.translation = rp.Vec3(*new_tra)
            delta = rp.Rotation3.from_axis_angle((0.0, 1.0, 0.0), -0.5 * dt)
            platform.rotation = delta * platform.rotation

    testbed.add_callback(_cb)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 5.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
