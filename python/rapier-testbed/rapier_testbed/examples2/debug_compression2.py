"""Port of examples2d/debug_compression2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Debug"
NAME = "Compression"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    width = 75.0
    thickness = 2.0
    ys = [-30.0 - thickness, 30.0 + thickness]

    for y in ys:
        h = bodies.insert(rp.RigidBody.fixed().translation((0.0, y)))
        colliders.insert_with_parent(rp.Collider.cuboid(width, thickness), h, bodies)

    half_height = (ys[1] - ys[0]) / 2.0 - thickness
    xs = [-width + thickness, width - thickness]
    handles = []
    for x in xs:
        h = bodies.insert(rp.RigidBody.dynamic().translation((x, 0.0)))
        colliders.insert_with_parent(
            rp.Collider.cuboid(thickness, half_height), h, bodies
        )
        handles.append(h)

    num = 8
    rad = half_height / num
    for i in range(num):
        for j in range(num):
            x = i * rad * 2.0 - num * rad
            y = j * rad * 2.0 - num * rad + rad
            h = bodies.insert(rp.RigidBody.dynamic().translation((x, y)))
            colliders.insert_with_parent(rp.Collider.ball(rad), h, bodies)

    state = {"force_x": 0.0}

    def callback(tb) -> None:
        left = tb.bodies.get(handles[0])
        right = tb.bodies.get(handles[1])
        if left is not None:
            left.reset_forces(True)
            left.add_force((state["force_x"], 0.0), True)
            tb.bodies.replace(handles[0], left)
        if right is not None:
            right.reset_forces(True)
            right.add_force((-state["force_x"], 0.0), True)
            tb.bodies.replace(handles[1], right)
        state["force_x"] += 10000.0

    testbed.add_callback(callback)
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 0.0), zoom=50.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
