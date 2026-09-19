"""A cloth pinned by two corners drops over a ball; a jelly cube lands beside it.

Shows the soft-body API: generators, pinned particles, materials, particle
positions read back as NumPy arrays, and a tear event.
"""

from __future__ import annotations

import rapier3d as rp

world = rp.PhysicsWorld(gravity=(0, -9.81, 0))
world.colliders.insert(rp.Collider.cuboid(20, 0.1, 20).build())
world.colliders.insert(rp.Collider.ball(0.8).translation((0, 0.8, 0)).build())

# A cloth pinned by two corners, with a stiff material that tears past 40% strain.
n = 16
cloth = world.add_soft_body(
    rp.SoftBody.cloth((-1.5, 3.0, -1.5), (0.2, 0, 0), (0, 0, 0.2), n, n)
    .pinned_particles([0, n * (n - 1)])
    .softness((30.0, 1.0))
    .particle_mass(0.05)
)

# A jelly cube with corotational elastic cells.
jelly = world.add_soft_body(
    rp.SoftBody.cuboid(
        (3.0, 1.0, 0.0),
        (0.5, 0.5, 0.5),
        4,
        4,
        4,
        cell_model=rp.SoftBodyCellModel.COROTATIONAL,
        material=rp.SoftBodyMaterial(young_modulus=2.0e3, poisson_ratio=0.35),
        particle_mass=0.2,
    )
)

collector = rp.ChannelEventCollector()
world.event_handler = collector
for _ in range(120):
    world.step()

# Tear one edge by hand: the event reports it at the end of the step.
world.soft_bodies[cloth].tear_edge(20)
world.step()
tears = collector.drain_soft_body_tear_events()

positions = world.soft_bodies[cloth].particle_positions
lowest = float(positions[:, 1].min())
jelly_y = world.soft_bodies[jelly].center_of_mass.y
print(
    f"soft: cloth particles={positions.shape[0]} lowest_y={lowest:+.2f} "
    f"jelly_y={jelly_y:+.2f} tears={len(tears)}"
)
