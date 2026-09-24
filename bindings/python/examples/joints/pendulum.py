"""Double pendulum built from two revolute joints.

Two arms hang from a fixed anchor and swing under gravity. After 500 steps
we report the tip's position so the script is deterministic enough to
regression-test.

Run::

    python python/examples/joints/pendulum.py
"""

from __future__ import annotations

import rapier3d as rp


def main() -> None:
    world = rp.PhysicsWorld(gravity=(0, -9.81, 0))

    # Fixed anchor at the origin.
    anchor = world.add_body(rp.RigidBody.fixed(translation=(0, 5, 0)))

    # First link: a 1m long arm whose top connects to the anchor.
    link1 = world.add_body(
        rp.RigidBody.dynamic(translation=(0, 4, 0)),
        colliders=[rp.Collider.cuboid(0.05, 0.5, 0.05).density(1.0)],
    )
    j1 = (
        rp.RevoluteJoint.builder(axis=(0, 0, 1))
        .local_anchor1((0, 0, 0))
        .local_anchor2((0, 0.5, 0))
        .build()
    )
    world.impulse_joints.insert(anchor, link1, j1)

    # Second link: another 1m arm pinned to the bottom of the first.
    link2 = world.add_body(
        rp.RigidBody.dynamic(translation=(0, 3, 0)),
        colliders=[rp.Collider.cuboid(0.05, 0.5, 0.05).density(1.0)],
    )
    j2 = (
        rp.RevoluteJoint.builder(axis=(0, 0, 1))
        .local_anchor1((0, -0.5, 0))
        .local_anchor2((0, 0.5, 0))
        .build()
    )
    world.impulse_joints.insert(link1, link2, j2)

    for _ in range(500):
        world.step()

    tip = world.rigid_bodies[link2].translation
    print(f"tip: x={tip.x:+.2f} y={tip.y:+.2f}")


if __name__ == "__main__":
    main()
