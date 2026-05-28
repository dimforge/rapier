"""Port of examples2d/one_way_platforms2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "One-way platforms"


class OneWayPlatformHook:
    """PhysicsHook that makes two platforms only allow contacts from one side."""

    def __init__(self, platform1, platform2) -> None:
        self.platform1 = platform1
        self.platform2 = platform2

    def modify_solver_contacts(self, ctx) -> None:
        allowed_local_n1 = (0.0, 0.0)

        if ctx.collider1 == self.platform1:
            allowed_local_n1 = (0.0, 1.0)
        elif ctx.collider2 == self.platform1:
            allowed_local_n1 = (0.0, -1.0)

        if ctx.collider1 == self.platform2:
            allowed_local_n1 = (0.0, -1.0)
        elif ctx.collider2 == self.platform2:
            allowed_local_n1 = (0.0, 1.0)

        ctx.update_as_oneway_platform(allowed_local_n1, 0.1)

        tangent_velocity = -12.0 if (
            ctx.collider1 == self.platform1 or ctx.collider2 == self.platform2
        ) else 12.0
        for i in range(ctx.num_solver_contacts()):
            sc = ctx.solver_contacts[i]
            try:
                sc.tangent_velocity = (tangent_velocity, sc.tangent_velocity.y)
            except Exception:
                # Some bindings expose tangent_velocity as a scalar; skip if
                # the assignment shape doesn't match.
                pass


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    handle = bodies.insert(rp.RigidBody.fixed())
    platform1 = colliders.insert_with_parent(
        rp.Collider.cuboid(25.0, 0.5)
        .translation((30.0, 2.0))
        .active_hooks(rp.ActiveHooks.MODIFY_SOLVER_CONTACTS),
        handle,
        bodies,
    )
    platform2 = colliders.insert_with_parent(
        rp.Collider.cuboid(25.0, 0.5)
        .translation((-30.0, -2.0))
        .active_hooks(rp.ActiveHooks.MODIFY_SOLVER_CONTACTS),
        handle,
        bodies,
    )

    hooks = OneWayPlatformHook(platform1, platform2)

    state = {"step": 0}

    def spawn_cube_callback(tb) -> None:
        state["step"] += 1
        if state["step"] % 200 == 0 and len(tb.bodies) <= 7:
            h = tb.bodies.insert(
                rp.RigidBody.dynamic().translation((20.0, 10.0))
            )
            tb.colliders.insert_with_parent(
                rp.Collider.cuboid(1.5, 2.0), h, tb.bodies
            )

        for h in tb._islands.active_dynamic_set():
            rb = tb.bodies.get(h)
            if rb is None:
                continue
            if rb.translation.y > 1.0:
                rb.gravity_scale = 1.0
                tb.bodies.replace(h, rb)
            elif rb.translation.y < -1.0:
                rb.gravity_scale = -1.0
                tb.bodies.replace(h, rb)

    testbed.add_callback(spawn_cube_callback)
    testbed.set_world_with_params(
        bodies, colliders, impulse_joints, multibody_joints, (0.0, -9.81), hooks
    )
    testbed.set_camera_2d(center=(0.0, 0.0), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
