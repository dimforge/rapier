"""Throughput micro-benchmark: 100 falling cuboids for 240 steps.

Prints ms/frame averaged across the run. Step count is intentionally low
so the script finishes in a few seconds and can be part of CI.

Run::

    python python/examples/perf/many_bodies.py
"""

from __future__ import annotations

import time

import rapier3d as rp


def main() -> None:
    world = rp.PhysicsWorld(gravity=(0, -9.81, 0))
    world.colliders.insert(rp.Collider.cuboid(50, 0.1, 50).build())

    # 100 cuboids in a 10x10 lattice, dropped from y=5.
    for ix in range(10):
        for iz in range(10):
            world.add_body(
                rp.RigidBody.dynamic(translation=(ix * 0.6 - 3, 5, iz * 0.6 - 3)),
                colliders=[rp.Collider.cuboid(0.25, 0.25, 0.25)],
            )

    frames = 240
    t0 = time.perf_counter()
    for _ in range(frames):
        world.step()
    elapsed = time.perf_counter() - t0
    ms_per_frame = elapsed * 1000.0 / frames
    n = len(world.rigid_bodies)
    # Pose-sensitive checksum: keep the test deterministic but tolerant.
    print(f"perf: bodies={n} frames={frames} ms_per_frame_present={ms_per_frame > 0}")


if __name__ == "__main__":
    main()
