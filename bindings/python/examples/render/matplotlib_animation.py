"""Animate a 3D world using the DebugLineCollector + matplotlib.

The debug line segments are projected onto the XY plane for a simple 2D plot.
If matplotlib is not installed we still run the simulation; the script only
exits with an error if neither code path works. A summary of the number of
debug segments rendered is always printed.

Run::

    python python/examples/render/matplotlib_animation.py
"""

from __future__ import annotations

import sys

import rapier3d as rp


def main() -> None:
    world = rp.PhysicsWorld(gravity=(0, -9.81, 0))

    # Floor + walls.
    world.colliders.insert(rp.Collider.cuboid(10, 0.1, 10).translation((0, -2, 0)).build())
    world.colliders.insert(rp.Collider.cuboid(0.1, 5, 10).translation((-5, 0, 0)).build())
    world.colliders.insert(rp.Collider.cuboid(0.1, 5, 10).translation((5, 0, 0)).build())

    # A few falling balls.
    for x in (-2.0, 0.0, 2.0):
        world.add_body(
            rp.RigidBody.dynamic(translation=(x, 3.0, 0.0)),
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

    # Best-effort plot of the final frame, projected onto the XY plane.
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
        # lines has shape (N, 2, 3); keep the X and Y columns for a 2D plot.
        segments = lines[:, :, :2]
        fig, ax = plt.subplots()
        ax.add_collection(LineCollection(segments))
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
