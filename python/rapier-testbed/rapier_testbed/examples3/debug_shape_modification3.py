"""Port of ``examples3d/debug_shape_modification3.rs``.

A rolling ball whose collider shape grows each step, plus a separate
"shapeshifting" collider that cycles through ball / cuboid / cone /
cylinder every 50 steps. Combined with the primitives3-style stack of
falling cubes.
"""
from __future__ import annotations

import rapier3d as rp

from .._registry import register

CATEGORY = "Debug"
NAME = "Shape modification"


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
        rp.Collider.cuboid(ground_size, ground_height, ground_size).friction(0.15),
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
    ball_coll_handle = colliders.insert_with_parent(
        rp.Collider.ball(ball_rad).density(100.0), ball_handle, bodies
    )

    # Static collider away from the action.
    shape_size = 3.0
    colliders.insert(
        rp.Collider.ball(shape_size).translation((-15.0, shape_size, 18.0))
    )

    shapes = [
        rp.SharedShape.ball(shape_size),
        rp.SharedShape.cuboid(shape_size, shape_size, shape_size),
        rp.SharedShape.cone(shape_size, shape_size),
        rp.SharedShape.cylinder(shape_size, shape_size),
    ]
    shapeshifting_coll_handle = colliders.insert(
        rp.Collider.new(shapes[0]).translation((-15.0, shape_size, 9.0))
    )

    state = {
        "linvel": rp.Vec3(0.0, 0.0, 0.0),
        "angvel": rp.Vec3(0.0, 0.0, 0.0),
        "pos": rp.Isometry3.identity(),
        "step": 0,
        "shape_idx": 0,
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

        shape_coll = tb.colliders.get(shapeshifting_coll_handle)
        if shape_coll is not None and step % 50 == 0:
            state["shape_idx"] = (state["shape_idx"] + 1) % 4
            shape_coll.shape = shapes[state["shape_idx"]]

        if step == 100:
            ball.linvel = state["linvel"]
            ball.angvel = state["angvel"]
            ball.position = state["pos"]
            state["step"] = snapped_frame

        ball_coll = tb.colliders.get(ball_coll_handle)
        if ball_coll is not None:
            ball_coll.shape = rp.SharedShape.ball(ball_rad * step * 2.0)

    testbed.add_callback(_cb)

    # Falling primitive grid (same as primitives3.rs but offset).
    num = 8
    rad = 1.0
    shiftx = rad * 2.0 + rad
    shifty = rad * 2.0 + rad
    shiftz = rad * 2.0 + rad
    centerx = shiftx * (num // 2)
    centery = shifty / 2.0
    centerz = shiftz * (num // 2)
    offset = -float(num) * (rad * 2.0 + rad) * 0.5
    for j in range(20):
        for i in range(num):
            for k in range(num):
                x = i * shiftx - centerx + offset + 5.0
                y = j * shifty + centery + 3.0
                z = k * shiftz - centerz + offset
                body = rp.RigidBody.dynamic().translation((x, y, z))
                handle = bodies.insert(body)
                kind = j % 5
                if kind == 0:
                    coll = rp.Collider.cuboid(rad, rad, rad)
                elif kind == 1:
                    coll = rp.Collider.ball(rad)
                elif kind == 2:
                    coll = rp.Collider.round_cylinder(rad, rad, rad / 10.0)
                elif kind == 3:
                    coll = rp.Collider.cone(rad, rad)
                else:
                    coll = rp.Collider.capsule_y(rad, rad)
                colliders.insert_with_parent(coll, handle, bodies)
        offset -= 0.05 * rad * (num - 1)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((40.0, 40.0, 40.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
