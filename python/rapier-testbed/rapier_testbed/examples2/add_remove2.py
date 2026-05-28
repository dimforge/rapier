"""Port of examples2d/add_remove2.rs."""
from __future__ import annotations

import random
import math

import rapier2d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Add remove"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    rad = 0.5

    positions = [(5.0, -1.0), (-5.0, -1.0)]
    platform_handles = []
    for pos in positions:
        handle = bodies.insert(rp.RigidBody.kinematic_position_based().translation(pos))
        colliders.insert_with_parent(
            rp.Collider.cuboid(rad * 10.0, rad), handle, bodies
        )
        platform_handles.append(handle)

    rng = random.Random(0)
    state = {"step": 0}

    def callback(tb) -> None:
        state["step"] += 1
        # Rotate the platforms. Rapier-py exposes no
        # `set_next_kinematic_rotation` — emulate by overwriting position.
        angle = -state["step"] * 0.016
        for rb_handle in platform_handles:
            rb = tb.bodies.get(rb_handle)
            if rb is None:
                continue
            tr = rb.translation
            new_pose = rp.Isometry2((float(tr.x), float(tr.y)), angle)
            rb.position = new_pose
            tb.bodies.replace(rb_handle, rb)

        if state["step"] % 10 == 0:
            x = rng.random() * 10.0 - 5.0
            y = rng.random() * 10.0 + 10.0
            h = tb.bodies.insert(rp.RigidBody.dynamic().translation((x, y)))
            tb.colliders.insert_with_parent(
                rp.Collider.cuboid(rad, rad), h, tb.bodies
            )

        # Remove bodies that fall below y = -10.
        to_remove = []
        for h in list(tb.bodies.handles()):
            rb = tb.bodies.get(h)
            if rb is None:
                continue
            if rb.translation.y < -10.0:
                to_remove.append(h)
        for h in to_remove:
            tb.bodies.remove(
                h,
                tb._islands,
                tb.colliders,
                tb.impulse_joints,
                tb.multibody_joints,
                True,
            )

    testbed.add_callback(callback)
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 0.0), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
