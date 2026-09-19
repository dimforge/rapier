"""Port of examples3d/soft_tearing3.rs: a pinned sheet ripped by a dropped ball, a curtain
shot through by a box, and a jelly bar pulled apart by its kinematic ends. Tear events move
the driven particles to their new bodies and indices."""
from __future__ import annotations

import rapier3d as rp
from .._registry import register

CATEGORY = "Soft bodies"
NAME = "Tearing"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()
    soft_bodies = rp.SoftBodySet()

    ground = bodies.insert(rp.RigidBody.fixed(translation=(0.0, -0.1, 0.0)))
    colliders.insert_with_parent(rp.Collider.cuboid(30.0, 0.1, 30.0), ground, bodies)

    # Stiff cloth springs (100 Hz) so only impacts pass the tear strain.
    def cloth_material(tear_strain):
        m = rp.SoftBodyMaterial.uniform((100.0, 1.0))
        m.bend_softness = (3.0, 1.0)
        m.tear_strain = tear_strain
        return m

    # A sheet pinned along its border: the heavy ball dropped on it rips through.
    n = 40
    border = [k for k in range(n * n) if k // n in (0, n - 1) or k % n in (0, n - 1)]
    sheet = rp.SoftBody.cloth(
        (-6.0, 3.0, -2.0), (0.1, 0.0, 0.0), (0.0, 0.0, 0.1), n, n,
        pinned_particles=border, material=cloth_material(0.4), particle_mass=0.02,
        particle_radius=0.05, surface_collider=rp.Collider.ball(0.05).friction(0.8),
    )
    soft_bodies.insert(sheet, bodies, colliders)
    ball = bodies.insert(rp.RigidBody.dynamic(translation=(-4.0, 6.0, 0.0)))
    colliders.insert_with_parent(rp.Collider.ball(0.6).density(30.0), ball, bodies)

    # A curtain pinned along its top edge, with a heavy box shot through it.
    curtain = rp.SoftBody.cloth(
        (0.0, 4.5, 4.0), (0.1, 0.0, 0.0), (0.0, -0.1, 0.0), 50, 40,
        pinned_particles=[i * 40 for i in range(50)], material=cloth_material(0.2),
        particle_mass=0.02,
    )
    soft_bodies.insert(curtain, bodies, colliders)
    bullet = bodies.insert(rp.RigidBody.dynamic(translation=(2.5, 2.5, 12.0), linvel=(0.0, 0.0, -25.0)))
    colliders.insert_with_parent(
        rp.Collider.cuboid(0.3, 0.3, 0.3).rotation((0.5, 0.5, 0.5)).density(20.0), bullet, bodies
    )

    # A jelly bar pinned at both ends, torn by pulling its right end away.
    bar_material = rp.SoftBodyMaterial(young_modulus=5.0e4, poisson_ratio=0.3, elastic_damping_ratio=1.0,
                                       tear_strain=0.4)
    bar = rp.SoftBody.cuboid(
        (3.0, 1.0, -4.0), (2.0, 0.4, 0.4), 21, 5, 5,
        cell_model=rp.SoftBodyCellModel.COROTATIONAL, material=bar_material, particle_mass=0.05,
        particle_radius=0.1, surface_collider=rp.Collider.ball(0.1).friction(0.8),
    )
    bar_handle = soft_bodies.insert(bar, bodies, colliders)
    sb = soft_bodies[bar_handle]
    ends = []
    for i in range(sb.num_particles):
        p = sb.particle_position(i)
        if p.x < 1.01:
            ends.append({"body": bar_handle, "particle": i, "rest": (p.x, p.y, p.z), "right": False})
        elif p.x > 4.99:
            ends.append({"body": bar_handle, "particle": i, "rest": (p.x, p.y, p.z), "right": True})
    for end in ends:
        sb.set_particle_pinned(end["particle"], True)

    # The tears of every step, to follow the driven particles through them.
    collector = rp.ChannelEventCollector()
    testbed.set_event_handler(collector)
    state = {"t": 0.0}

    def drive(tb) -> None:
        state["t"] += tb._integration_parameters.dt
        # The right end starts moving after a second, at half a meter per second, and stops
        # once the bar has doubled its length.
        shift = min(max(state["t"] - 1.0, 0.0) * 0.5, 4.0)
        for end in ends:
            if end["right"] and end["body"] in tb.soft_bodies:
                rest = end["rest"]
                tb.soft_bodies[end["body"]].set_particle_kinematic_target(
                    end["particle"], (rest[0] + shift, rest[1], rest[2])
                )
        for event in collector.drain_soft_body_tear_events():
            for end in ends:
                if event.soft_body == end["body"]:
                    destination = event.particle_destination(end["particle"])
                    if destination is not None:
                        end["body"], end["particle"] = destination

    testbed.add_callback(drive)
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints, soft_bodies)
    testbed.look_at((9.0, 8.0, 16.0), (0.0, 1.5, 1.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
