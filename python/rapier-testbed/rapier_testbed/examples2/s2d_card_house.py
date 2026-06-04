"""Port of examples2d/s2d_card_house.rs."""
from __future__ import annotations

import math

import rapier2d as rp
from .._registry import register

CATEGORY = "Inspired by Solver 2D"
NAME = "Card house"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    friction = 0.7

    ground_handle = bodies.insert(rp.RigidBody.fixed().translation((0.0, -2.0)))
    colliders.insert_with_parent(
        rp.Collider.cuboid(40.0, 2.0).friction(friction), ground_handle, bodies
    )

    scale = 10.0
    card_height = 0.2 * scale
    card_thickness = 0.001 * scale
    angle0 = 25.0 * math.pi / 180.0
    angle1 = -25.0 * math.pi / 180.0
    angle2 = 0.5 * math.pi

    def card_box():
        return rp.Collider.cuboid(card_thickness, card_height).friction(friction)

    nb = 5
    z0 = 0.0
    y = card_height - 0.02 * scale

    while nb != 0:
        z = z0
        for i in range(nb):
            if i != nb - 1:
                h = bodies.insert(
                    rp.RigidBody.dynamic()
                    .translation((z + 0.25 * scale, y + card_height - 0.015 * scale))
                    .rotation(angle2)
                )
                colliders.insert_with_parent(card_box(), h, bodies)

            h = bodies.insert(
                rp.RigidBody.dynamic().translation((z, y)).rotation(angle1)
            )
            colliders.insert_with_parent(card_box(), h, bodies)

            z += 0.175 * scale

            h = bodies.insert(
                rp.RigidBody.dynamic().translation((z, y)).rotation(angle0)
            )
            colliders.insert_with_parent(card_box(), h, bodies)

            z += 0.175 * scale

        y += card_height * 2.0 - 0.03 * scale
        z0 += 0.175 * scale
        nb -= 1

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
