"""Animate a 2D world using the DebugLineCollector + matplotlib.

If matplotlib is not installed we still run the simulation; the script only
exits with an error if neither code path works. A summary of the number of
debug segments rendered is always printed.

Run::

    python python/examples/render/matplotlib_animation.py
"""

from __future__ import annotations

import sys

import rapier2d as rp


def main() -> None:
    world = rp.PhysicsWorld(gravity=(0, -9.81))

    # Floor + walls.
    world.colliders.insert(rp.Collider.cuboid(10, 0.1).translation((0, -2)).build())
    world.colliders.insert(rp.Collider.cuboid(0.1, 5).translation((-5, 0)).build())
    world.colliders.insert(rp.Collider.cuboid(0.1, 5).translation((5, 0)).build())

    # A few falling balls.
    for x in (-2.0, 0.0, 2.0):
        world.add_body(
            rp.RigidBody.dynamic(translation=(x, 3.0)),
            colliders=[rp.Collider.ball(0.3)],
        )

    pipe = rp.DebugRenderPipeline()
    total = 0
    for _ in range(120):
        world.step()
        lines, _colors, _objects = pipe.render_to_arrays(
            world.rigid_bodies,
            world.colliders,
            world.impulse_joints,
            world.multibody_joints,
            world.narrow_phase,
        )
        total += int(lines.shape[0])

    # Best-effort plot of the final frame.
    try:
        import matplotlib  # noqa: F401

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection

        lines, _, _ = pipe.render_to_arrays(
            world.rigid_bodies,
            world.colliders,
            world.impulse_joints,
            world.multibody_joints,
            world.narrow_phase,
        )
        fig, ax = plt.subplots()
        ax.add_collection(LineCollection(lines))
        ax.set_aspect("equal")
        ax.autoscale_view()
        out = "matplotlib_animation.png"
        fig.savefig(out)
        print(f"saved: {out}", file=sys.stderr)
    except ImportError:
        pass

    print(f"matplotlib: segments={total} frames=120")


if __name__ == "__main__":
    main()
