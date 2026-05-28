"""Port of ``examples3d/debug_add_remove_collider3.rs``.

Each step removes the ground collider and re-inserts an identical one on
the same parent body. Verifies that the rolling ball keeps in contact
across collider churn.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Add/rm collider"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 3.0
    ground_height = 0.1
    ground_body = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_handle = bodies.insert(ground_body)
    ground_collider_handle = colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, 0.4), ground_handle, bodies
    )

    ball_rad = 0.1
    ball_body = rp.RigidBody.dynamic().translation((0.0, 0.2, 0.0))
    ball_handle = bodies.insert(ball_body)
    colliders.insert_with_parent(
        rp.Collider.ball(ball_rad).density(100.0), ball_handle, bodies
    )

    # Mutable cell via a list so the callback can rebind.
    state = {"handle": ground_collider_handle}

    def _cycle(tb) -> None:
        coll = tb.colliders.remove(
            state["handle"], tb._islands, tb.bodies, True
        )
        if coll is None:
            return
        state["handle"] = tb.colliders.insert_with_parent(coll, ground_handle, tb.bodies)

    testbed.add_callback(_cycle)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
