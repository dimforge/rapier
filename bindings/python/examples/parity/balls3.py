"""Python parity for a simplified ``examples3d/debug_balls3.rs`` scene.

The original Rust example builds a 10x10x10 lattice of balls (some fixed,
some dynamic) and steps the simulation in the testbed. This Python
counterpart uses a smaller 4x4x4 lattice (still illustrative, but cheap
enough for CI) and prints the final position of a few balls so that the
Rust example can be compared by eye.

Comparison procedure
====================

1. ``cargo run --release -p rapier-testbed3d --bin all_examples3 -- debug_balls3``
   from the workspace root.
2. ``python python/examples/parity/balls3.py``.
3. Both runs should observe the dynamic balls settle on top of the fixed
   ones in roughly the same configuration. The Python script prints three
   probe positions; the Rust example shows the same scene visually.

Run::

    python python/examples/parity/balls3.py
"""

from __future__ import annotations

import rapier3d as rp


def main() -> None:
    world = rp.PhysicsWorld(gravity=(0, -9.81, 0))

    num = 4
    rad = 0.5
    shift = rad * 2.0
    cx = shift * num / 2.0
    cy = shift / 2.0
    cz = shift * num / 2.0

    handles: dict[tuple[int, int, int], rp.RigidBodyHandle] = {}
    for i in range(num):
        for j in range(num):
            for k in range(num):
                x = i * shift - cx
                z = k * shift - cz
                is_fixed = j == 0 or i == 0 or k == 0 or i == num - 1 or k == num - 1
                if is_fixed:
                    y = j * shift + cy
                    builder = rp.RigidBody.fixed(translation=(x, y, z))
                else:
                    y = j * shift * 2.0 + cy
                    builder = rp.RigidBody.dynamic(translation=(x, y, z))
                h = world.add_body(
                    builder.can_sleep(False),
                    colliders=[rp.Collider.ball(rad).friction(0.0)],
                )
                handles[(i, j, k)] = h

    # Cap at 500 frames (Phase 12 budget) and step.
    for _ in range(500):
        world.step()

    # Probes: lower-corner dynamic, middle dynamic, upper-corner dynamic.
    probes = [(1, 1, 1), (2, 2, 2), (num - 2, num - 1, num - 2)]
    parts = []
    for p in probes:
        t = world.rigid_bodies[handles[p]].translation
        parts.append(f"{p}=({t.x:+.1f},{t.y:+.1f},{t.z:+.1f})")
    print("parity: " + " ".join(parts))


if __name__ == "__main__":
    main()
