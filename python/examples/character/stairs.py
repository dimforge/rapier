"""KinematicCharacterController climbing a 0.1m step using autostep.

We drive a sphere-shaped character forward on flat ground, then over a
0.1m-tall step. With autostep enabled, the controller lifts the character
on top of the step and it keeps moving forward.

Run::

    python python/examples/character/stairs.py
"""

from __future__ import annotations

import rapier3d as rp


def main() -> None:
    world = rp.PhysicsWorld(gravity=(0, -9.81, 0), auto_update_query=True)

    # Big flat ground.
    world.add_body(
        rp.RigidBody.fixed(translation=(0, -1.0, 0)),
        colliders=[rp.Collider.cuboid(50, 1, 50)],
    )
    # 0.1m-high step starting at x=0.5, extending forward.
    world.add_body(
        rp.RigidBody.fixed(translation=(2.5, 0.05, 0)),
        colliders=[rp.Collider.cuboid(2.0, 0.05, 50)],
    )
    world.update_query_pipeline()

    autostep = rp.CharacterAutostep(
        max_height=rp.CharacterLength.absolute(0.4),
        min_width=rp.CharacterLength.absolute(0.05),
        include_dynamic_bodies=True,
    )
    ctrl = rp.KinematicCharacterController(
        up=(0, 1, 0),
        offset=rp.CharacterLength.absolute(0.01),
        slide=True,
        autostep=autostep,
        snap_to_ground=rp.CharacterLength.absolute(0.5),
    )

    shape = rp.SharedShape.ball(0.3)
    pose = rp.Isometry3.from_translation(-1.0, 0.31, 0.0)
    last = pose.translation

    # Drive forward at 3 m/s for ~5s of sim time.
    for _ in range(300):
        mv = ctrl.move_shape(
            1.0 / 60.0,
            world.rigid_bodies,
            world.colliders,
            world.query_pipeline,
            shape,
            pose,
            (0.05, -0.05, 0.0),
            rp.QueryFilter(),
        )
        last = mv.translation
        pose = rp.Isometry3.from_translation(
            pose.translation.x + mv.translation.x,
            pose.translation.y + mv.translation.y,
            0.0,
        )

    print(f"climbed: x={pose.translation.x:.2f} y={pose.translation.y:.2f}")
    _ = last  # unused, just kept for readability


if __name__ == "__main__":
    main()
