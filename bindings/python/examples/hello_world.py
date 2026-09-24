"""Hello-world: drop a ball onto a static plane and report its final position.

Run::

    python python/examples/hello_world.py
"""

from __future__ import annotations

import rapier3d as rp


def main() -> None:
    world = rp.PhysicsWorld(gravity=(0, -9.81, 0))

    # Static ground plane.
    world.colliders.insert(rp.Collider.cuboid(50, 0.1, 50).build())

    # Dynamic ball at y = 5.
    ball = world.add_body(
        rp.RigidBody.dynamic(translation=(0, 5, 0)),
        colliders=[rp.Collider.ball(0.5)],
    )

    for _ in range(240):
        world.step()

    pos = world.rigid_bodies[ball].translation
    print(f"final: y={pos.y:.2f} (rest height ~0.6)")


if __name__ == "__main__":
    main()
