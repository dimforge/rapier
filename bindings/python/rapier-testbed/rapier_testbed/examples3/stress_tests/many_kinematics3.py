"""Port of examples3d/stress_tests/many_kinematics3.rs."""
from __future__ import annotations

import random

import rapier3d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "Many kinematics"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    num = 30
    rad = 1.0
    shift = rad * 6.0 + 1.0
    centerx = shift * num / 2.0
    centery = shift * num / 2.0
    centerz = shift * num / 2.0

    rng = random.Random(0)
    half = shift * num / 2.0
    for i in range(num):
        for j in range(num):
            for k in range(num):
                x = i * shift - centerx
                y = j * shift - centery
                z = k * shift - centerz
                velocity = (
                    (rng.random() - 0.5) * 30.0,
                    (rng.random() - 0.5) * 30.0,
                    (rng.random() - 0.5) * 30.0,
                )
                body = (
                    rp.RigidBody.kinematic_velocity_based()
                    .translation((x, y, z))
                    .linvel(velocity)
                )
                h = bodies.insert(body)
                colliders.insert_with_parent(rp.Collider.ball(rad), h, bodies)

    def _cb(tb) -> None:
        for _, rb in tb.bodies:
            lv = rb.linvel
            new_x = lv.x
            new_y = lv.y
            new_z = lv.z
            t = rb.translation
            if (new_x > 0.0 and t.x > half) or (new_x < 0.0 and t.x < -half):
                new_x = -new_x
            if (new_y > 0.0 and t.y > half) or (new_y < 0.0 and t.y < -half):
                new_y = -new_y
            if (new_z > 0.0 and t.z > half) or (new_z < 0.0 and t.z < -half):
                new_z = -new_z
            rb.linvel = rp.Vec3(new_x, new_y, new_z)

    testbed.add_callback(_cb)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 100.0, 100.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
