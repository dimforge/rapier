"""Port of examples2d/s2d_arch.rs."""
from __future__ import annotations

import numpy as np

import rapier2d as rp
from .._registry import register

CATEGORY = "Inspired by Solver 2D"
NAME = "Arch"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ps1 = [
        (16.0, 0.0),
        (14.93803712795643, 5.133601056842984),
        (13.79871746027416, 10.24928069555078),
        (12.56252963284711, 15.34107019122473),
        (11.20040987372525, 20.39856541571217),
        (9.66521217819836, 25.40369899225096),
        (7.87179930638133, 30.3179337000085),
        (5.635199558196225, 35.03820717801641),
        (2.405937953536585, 39.09554102558315),
    ]
    ps2 = [
        (24.0, 0.0),
        (22.33619528222415, 6.02299846205841),
        (20.54936888969905, 12.00964361211476),
        (18.60854610798073, 17.9470321677465),
        (16.46769273811807, 23.81367936585418),
        (14.05325025774858, 29.57079353071012),
        (11.23551045834022, 35.13775818285372),
        (7.752568160730571, 40.30450679009583),
        (3.016931552701656, 44.28891593799322),
    ]
    scale = 0.25
    friction = 0.6

    ps1 = [(x * scale, y * scale) for x, y in ps1]
    ps2 = [(x * scale, y * scale) for x, y in ps2]

    colliders.insert(
        rp.Collider.segment((-100.0, 0.0), (100.0, 0.0)).friction(0.6)
    )

    def insert_chunk(pts):
        pts_arr = np.array(pts, dtype=np.float32)
        handle = bodies.insert(rp.RigidBody.dynamic())
        colliders.insert_with_parent(
            rp.Collider.convex_hull(pts_arr).friction(friction), handle, bodies
        )

    for i in range(8):
        insert_chunk([ps1[i], ps2[i], ps2[i + 1], ps1[i + 1]])

    for i in range(8):
        insert_chunk(
            [
                (-ps2[i][0], ps2[i][1]),
                (-ps1[i][0], ps1[i][1]),
                (-ps1[i + 1][0], ps1[i + 1][1]),
                (-ps2[i + 1][0], ps2[i + 1][1]),
            ]
        )

    insert_chunk(
        [
            ps1[8],
            ps2[8],
            (-ps1[8][0], ps1[8][1]),
            (-ps2[8][0], ps2[8][1]),
        ]
    )

    for i in range(4):
        handle = bodies.insert(
            rp.RigidBody.dynamic().translation((0.0, 0.5 + ps2[8][1] + 1.0 * i))
        )
        colliders.insert_with_parent(
            rp.Collider.cuboid(2.0, 0.5).friction(friction), handle, bodies
        )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
