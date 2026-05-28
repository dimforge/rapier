"""Port of examples2d/drum2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Drum"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    num = 30
    rad = 0.2

    shift = rad * 2.0
    centerx = shift * num / 2.0
    centery = shift * num / 2.0

    for i in range(num):
        for j in range(num):
            x = i * shift - centerx
            y = j * shift - centery
            handle = bodies.insert(rp.RigidBody.dynamic().translation((x, y)))
            colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), handle, bodies)

    # Velocity-based kinematic drum.
    platform_handle = bodies.insert(rp.RigidBody.kinematic_velocity_based())

    sides = [
        (10.0, 0.25, (0.0, 10.0)),
        (10.0, 0.25, (0.0, -10.0)),
        (0.25, 10.0, (10.0, 0.0)),
        (0.25, 10.0, (-10.0, 0.0)),
    ]
    balls = [
        (1.25, (6.0, 6.0)),
        (1.25, (-6.0, 6.0)),
        (1.25, (6.0, -6.0)),
        (1.25, (-6.0, -6.0)),
    ]

    for hx, hy, pos in sides:
        colliders.insert_with_parent(
            rp.Collider.cuboid(hx, hy).translation(pos), platform_handle, bodies
        )
    for r, pos in balls:
        colliders.insert_with_parent(
            rp.Collider.ball(r).translation(pos), platform_handle, bodies
        )

    def callback(tb) -> None:
        platform = tb.bodies.get(platform_handle)
        if platform is not None:
            platform.angvel = -0.15
            tb.bodies.replace(platform_handle, platform)

    testbed.add_callback(callback)
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 1.0), zoom=40.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
