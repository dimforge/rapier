"""Port of ``examples3d/debug_rollback3.rs``.

Rolling ball whose state is snapshotted at frame 51 and restored at
frame 100, mimicking a rollback netcode flow.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Rollback"


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
    }
    snapped_frame = 51

    def _cb(tb) -> None:
        state["step"] += 1
        step = state["step"]
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

    testbed.add_callback(_cb)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
