"""Port of ``examples3d/debug_dynamic_collider_add3.rs``.

A rolling ball whose collider keeps growing each frame while extra ground
colliders are also added. Snaps the ball's velocity at frame 51, restores
it at frame 100, and removes the extra colliders.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Dyn. collider add"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 20.0
    ground_height = 0.1
    ground_body = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_handle = bodies.insert(ground_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, 0.4).friction(0.15),
        ground_handle,
        bodies,
    )

    ball_rad = 0.1
    ball_body = (
        rp.RigidBody.dynamic()
        .translation((0.0, 0.2, 0.0))
        .linvel((10.0, 0.0, 0.0))
    )
    ball_handle = bodies.insert(ball_body)
    colliders.insert_with_parent(
        rp.Collider.ball(ball_rad).density(100.0), ball_handle, bodies
    )

    state = {
        "linvel": rp.Vec3(0.0, 0.0, 0.0),
        "angvel": rp.Vec3(0.0, 0.0, 0.0),
        "pos": rp.Isometry3.identity(),
        "step": 0,
        "extras": [],
    }
    snapped_frame = 51

    def _cb(tb) -> None:
        state["step"] += 1
        step = state["step"]

        # Add a bigger ball collider.
        new_handle = tb.colliders.insert_with_parent(
            rp.Collider.ball(ball_rad + 0.01 * step).density(100.0),
            ball_handle,
            tb.bodies,
        )
        state["extras"].append(new_handle)

        ball = tb.bodies.get(ball_handle)
        if ball is None:
            return
        if step == snapped_frame:
            state["linvel"] = ball.linvel
            state["angvel"] = ball.angvel
            state["pos"] = ball.position
        if step == 100:
            ball.linvel = state["linvel"]
            ball.angvel = state["angvel"]
            ball.position = state["pos"]
            state["step"] = snapped_frame
            for h in state["extras"]:
                tb.colliders.remove(h, tb._islands, tb.bodies, True)
            state["extras"].clear()

        # Add a fresh ground collider too.
        new_ground = tb.colliders.insert_with_parent(
            rp.Collider.cuboid(ground_size, ground_height + step * 0.01, 0.4).friction(0.15),
            ground_handle,
            tb.bodies,
        )
        state["extras"].append(new_ground)

    testbed.add_callback(_cb)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
