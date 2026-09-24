"""Port of examples3d/one_way_platforms3.rs.

The Rust example installs a ``PhysicsHooks`` impl that uses
``ContactModificationContext::update_as_oneway_platform`` plus per-contact
tangent velocity overrides. The Python testbed doesn't yet wire up
``set_hooks`` to a Python callable in a way that lets us drive
``modify_solver_contacts`` per pair, so we fall back to a passive scene:
the two platforms still exist, gravity flips depending on body
y-coordinate (mirrors the Rust per-frame callback), and bodies are
spawned every 200 steps.
"""
from __future__ import annotations

import rapier3d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "One-way platforms"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground = rp.RigidBody.fixed()
    ground_h = bodies.insert(ground)

    col1 = (
        rp.Collider.cuboid(9.0, 0.5, 25.0)
        .translation((0.0, 2.0, 30.0))
        .active_hooks(rp.ActiveHooks.MODIFY_SOLVER_CONTACTS)
    )
    colliders.insert_with_parent(col1, ground_h, bodies)

    col2 = (
        rp.Collider.cuboid(9.0, 0.5, 25.0)
        .translation((0.0, -2.0, -30.0))
        .active_hooks(rp.ActiveHooks.MODIFY_SOLVER_CONTACTS)
    )
    colliders.insert_with_parent(col2, ground_h, bodies)

    def _cb(tb) -> None:
        if tb._step_count % 200 == 0 and len(tb.bodies) <= 7:
            body = rp.RigidBody.dynamic().translation((0.0, 6.0, 20.0))
            h = tb.bodies.insert(body)
            tb.colliders.insert_with_parent(
                rp.Collider.cuboid(1.0, 2.0, 1.5), h, tb.bodies
            )
        for handle in tb._islands.active_dynamic_set():
            body = tb.bodies.get(handle)
            if body is None:
                continue
            y = body.translation.y
            if y > 1.0:
                body.gravity_scale = 1.0
            elif y < -1.0:
                body.gravity_scale = -1.0

    testbed.add_callback(_cb)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((100.0, 0.0, 0.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
