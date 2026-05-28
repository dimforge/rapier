"""Port of examples2d/stress_tests/joint_prismatic2.rs."""
from __future__ import annotations

import math

import rapier2d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "(Stress test) joint prismatic"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.4
    num = 10
    shift = 1.0

    inv_sqrt2 = 1.0 / math.sqrt(2.0)

    for l in range(25):
        y_l = l * shift * (num + 2) * 2.0

        for j in range(50):
            x = j * shift * 4.0

            ground = bodies.insert(rp.RigidBody.fixed().translation((x, y_l)))
            colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), ground, bodies)
            curr_parent = ground

            for i in range(num):
                yy = y_l - (i + 1) * shift
                density = 1.0
                child = bodies.insert(rp.RigidBody.dynamic().translation((x, yy)))
                colliders.insert_with_parent(
                    rp.Collider.cuboid(rad, rad).density(density), child, bodies
                )

                axis = (
                    (inv_sqrt2, inv_sqrt2) if i % 2 == 0 else (-inv_sqrt2, inv_sqrt2)
                )
                prism = (
                    rp.PrismaticJoint.builder(axis)
                    .local_anchor2((0.0, shift))
                    .limits(-1.5, 1.5)
                )
                impulse_joints.insert(curr_parent, child, prism, wake_up=True)
                curr_parent = child

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(80.0, 80.0), zoom=15.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
