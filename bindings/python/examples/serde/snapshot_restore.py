"""Snapshot a world mid-sim, modify the live world, then restore.

Demonstrates ``PhysicsWorld.snapshot() / .restore()``.

Run::

    python python/examples/serde/snapshot_restore.py
"""

from __future__ import annotations

import rapier3d as rp


def main() -> None:
    world = rp.PhysicsWorld(gravity=(0, -9.81, 0))
    world.colliders.insert(rp.Collider.cuboid(50, 0.1, 50).build())
    ball = world.add_body(
        rp.RigidBody.dynamic(translation=(0, 5, 0)),
        colliders=[rp.Collider.ball(0.5)],
    )

    # Step 60 frames, snapshot, then step 60 more.
    for _ in range(60):
        world.step()
    blob = world.snapshot()

    for _ in range(60):
        world.step()
    later = world.rigid_bodies[ball].translation.y

    # Restore from the mid-snapshot and confirm we land back at the same
    # snapshot state.
    restored = rp.PhysicsWorld.restore(blob)
    snap_y = restored.rigid_bodies[ball].translation.y

    print(f"snapshot: snap.y={snap_y:.2f} later.y={later:.2f} bytes={len(blob)}")


if __name__ == "__main__":
    main()
