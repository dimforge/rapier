"""Port of ``examples3d/debug_disabled3.rs``.

A platform that flips its ``is_enabled`` every 250 steps, with extra
cubes spawned every 25 steps so the dynamic body count keeps growing.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Disabled"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.5

    ground_size = 10.1
    ground_height = 2.1
    ground_body = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_handle = bodies.insert(ground_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size),
        ground_handle,
        bodies,
    )

    # Platform to toggle on/off.
    plat_body = rp.RigidBody.dynamic().translation((0.0, 5.0, 0.0))
    plat_handle = bodies.insert(plat_body)
    handle_to_disable = colliders.insert_with_parent(
        rp.Collider.cuboid(5.0, 1.0, 5.0), plat_handle, bodies
    )

    def _cb(tb) -> None:
        step = tb._step_count
        if step % 250 == 0:
            co = tb.colliders.get(handle_to_disable)
            if co is not None:
                co.is_enabled = not co.is_enabled
        if step % 25 == 0:
            body = rp.RigidBody.dynamic().translation((0.0, 20.0, 0.0))
            handle = tb.bodies.insert(body)
            tb.colliders.insert_with_parent(
                rp.Collider.cuboid(rad, rad, rad), handle, tb.bodies
            )

    testbed.add_callback(_cb)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((30.0, 4.0, 30.0), (0.0, 1.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
