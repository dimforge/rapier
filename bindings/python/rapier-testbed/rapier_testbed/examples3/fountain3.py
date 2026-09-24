"""Port of examples3d/fountain3.rs.

Continuously spawns dynamic primitives that fall on three stacked ground
slabs. When the dynamic body count exceeds a cap, the farthest ones are
removed each frame.
"""
from __future__ import annotations

import rapier3d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Fountain"

MAX_NUMBER_OF_BODIES = 400


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.5
    ground_size = 40.0
    ground_height = 2.1

    for k in range(3):
        body = rp.RigidBody.fixed().translation((0.0, -ground_height - float(k), 0.0))
        h = bodies.insert(body)
        colliders.insert_with_parent(
            rp.Collider.cuboid(ground_size, ground_height, ground_size), h, bodies
        )

    def _cb(tb) -> None:
        rb = rp.RigidBody.dynamic().translation((0.0, 10.0, 0.0))
        h = tb.bodies.insert(rb)
        step = tb._step_count
        kind = step % 3
        if kind == 0:
            col = rp.Collider.round_cylinder(rad, rad, rad / 10.0)
        elif kind == 1:
            col = rp.Collider.cone(rad, rad)
        else:
            col = rp.Collider.cuboid(rad, rad, rad)
        tb.colliders.insert_with_parent(col, h, tb.bodies)

        if len(tb.bodies) > MAX_NUMBER_OF_BODIES:
            dyn = [
                (hh, body)
                for hh, body in tb.bodies
                if body.is_dynamic
            ]
            dyn.sort(
                key=lambda pair: abs(pair[1].translation.x) + abs(pair[1].translation.z),
                reverse=True,
            )
            num_remove = max(0, len(dyn) - MAX_NUMBER_OF_BODIES)
            for handle, _ in dyn[:num_remove]:
                tb.bodies.remove(
                    handle,
                    tb._islands,
                    tb.colliders,
                    tb.impulse_joints,
                    tb.multibody_joints,
                    True,
                )

    testbed.add_callback(_cb)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((30.0, 4.0, 30.0), (0.0, 1.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
