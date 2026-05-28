"""Port of ``examples3d/debug_cube_high_mass_ratio3.rs``.

Stack of thin sticks supporting a single heavy cube, with 36 extra solver
iterations on the cube to keep the stack from squishing flat.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "High mass ratio: cube"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    num_levels = 4
    stick_len = 2.0
    stick_rad = 0.2

    # Floor.
    floor_body = rp.RigidBody.fixed().translation(
        (0.0, -stick_len - stick_rad, 0.0)
    )
    floor_handle = bodies.insert(floor_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(stick_len, stick_len, stick_len), floor_handle, bodies
    )

    for i in range(num_levels):
        # Two horizontal capsules (front/back) and two perpendicular (left/right).
        configs = [
            ((0.0, i * stick_rad * 4.0, -(stick_len / 2.0 - stick_rad)),
             (stick_len / 2.0, stick_rad, stick_rad)),
            ((0.0, i * stick_rad * 4.0, stick_len / 2.0 - stick_rad),
             (stick_len / 2.0, stick_rad, stick_rad)),
            ((-(stick_len / 2.0 - stick_rad), (i + 0.5) * stick_rad * 4.0, 0.0),
             (stick_rad, stick_rad, stick_len / 2.0)),
            ((stick_len / 2.0 - stick_rad, (i + 0.5) * stick_rad * 4.0, 0.0),
             (stick_rad, stick_rad, stick_len / 2.0)),
        ]
        for pos, half_ext in configs:
            body = rp.RigidBody.dynamic().translation(pos)
            handle = bodies.insert(body)
            colliders.insert_with_parent(
                rp.Collider.cuboid(*half_ext), handle, bodies
            )

    # Big cube on top.
    cube_len = stick_len * 2.0
    big_body = (
        rp.RigidBody.dynamic()
        .translation(
            (
                0.0,
                cube_len / 2.0 + (num_levels - 0.25) * stick_rad * 4.0,
                0.0,
            )
        )
        .additional_solver_iterations(36)
    )
    big_handle = bodies.insert(big_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(cube_len / 2.0, cube_len / 2.0, cube_len / 2.0),
        big_handle,
        bodies,
    )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
