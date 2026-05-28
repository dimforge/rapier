"""Port of ``examples3d/debug_sleeping_kinematic3.rs``.

Two velocity-based kinematic platforms. From step ``start_tick=500`` they
start moving (one fast, one very slow); at ``stop_tick=1000`` both stop
and should eventually fall asleep.
"""
from __future__ import annotations

import math

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Sleeping kinematics"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    plat_body = rp.RigidBody.kinematic_velocity_based().translation(
        (0.0, 1.5 + 0.8, 0.0)
    )
    plat_handle = bodies.insert(plat_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(5.0, 0.5, 5.0), plat_handle, bodies
    )

    slow_body = rp.RigidBody.kinematic_velocity_based().translation((0.0, 0.0, 0.0))
    slow_handle = bodies.insert(slow_body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(5.0, 0.5, 5.0), slow_handle, bodies
    )

    start_tick = 500
    stop_tick = 1000

    def _cb(tb) -> None:
        step = tb._step_count
        if step == stop_tick:
            for h in (plat_handle, slow_handle):
                b = tb.bodies.get(h)
                if b is not None:
                    b.linvel = rp.Vec3(0.0, 0.0, 0.0)
        if step < start_tick or step >= stop_tick:
            return
        if step == start_tick:
            slow_velocity = rp.Vec3(0.0, 0.01, 0.0)
            slow = tb.bodies.get(slow_handle)
            if slow is not None:
                slow.linvel = slow_velocity
        t = step * tb._integration_parameters.dt
        velocity = rp.Vec3(0.0, math.cos(t * 2.0), math.sin(t) * 2.0)
        plat = tb.bodies.get(plat_handle)
        if plat is not None:
            plat.linvel = velocity

    testbed.add_callback(_cb)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 5.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
