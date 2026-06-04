"""Port of examples3d/stress_tests/joint_prismatic3.rs."""
from __future__ import annotations

import math

import rapier3d as rp
from ..._registry import register

CATEGORY = "Stress Tests"
NAME = "ImpulseJoint prismatic"


def _normalize(v):
    n = math.sqrt(sum(x * x for x in v))
    return (v[0] / n, v[1] / n, v[2] / n)


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.4
    num = 5
    shift = 1.0

    for m in range(8):
        z0 = m * shift * (num + 2.0)
        for l in range(8):
            y = l * shift * num * 2.0
            for j in range(50):
                x = j * shift * 4.0
                curr_parent = bodies.insert(rp.RigidBody.fixed().translation((x, y, z0)))
                colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), curr_parent, bodies)
                for i in range(num):
                    z = z0 + (i + 1) * shift
                    body = rp.RigidBody.dynamic().translation((x, y, z))
                    child = bodies.insert(body)
                    colliders.insert_with_parent(
                        rp.Collider.cuboid(rad, rad, rad).density(1.0), child, bodies
                    )
                    axis = (
                        _normalize((1.0, 1.0, 0.0)) if i % 2 == 0 else _normalize((-1.0, 1.0, 0.0))
                    )
                    prism = (
                        rp.PrismaticJoint.builder(axis=axis)
                        .local_anchor2((0.0, 0.0, -shift))
                        .limits(-2.0, 0.0)
                        .build()
                    )
                    impulse_joints.insert(curr_parent, child, prism, wake_up=True)
                    curr_parent = child

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((262.0, 63.0, 124.0), (101.0, 4.0, -3.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from ..._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
