"""Port of examples3d/soft_bodies3.rs: a pinned cloth catching a box, a pressurized
balloon, jelly cubes (corotational, Neo-Hookean, volume constraints) and a rope holding a
rigid weight."""
from __future__ import annotations

import rapier3d as rp
from .._registry import register

CATEGORY = "Soft bodies"
NAME = "Showcase"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()
    soft_bodies = rp.SoftBodySet()

    # Ground.
    ground = bodies.insert(rp.RigidBody.fixed(translation=(0.0, -0.1, 0.0)))
    colliders.insert_with_parent(rp.Collider.cuboid(12.0, 0.1, 12.0), ground, bodies)

    # A cloth pinned by its four corners, with a box dropped on it.
    n = 24
    cloth = rp.SoftBody.cloth(
        (-3.5, 2.5, -1.2), (0.1, 0.0, 0.0), (0.0, 0.0, 0.1), n, n,
        pinned_particles=[0, n - 1, n * (n - 1), n * n - 1],
        softness=(30.0, 1.0),
        particle_mass=0.05,
    )
    soft_bodies.insert(cloth, bodies, colliders)
    box = bodies.insert(rp.RigidBody.dynamic(translation=(-2.35, 4.0, 0.0)))
    colliders.insert_with_parent(rp.Collider.cuboid(0.3, 0.3, 0.3).density(0.5), box, bodies)

    # A balloon: a hollow sphere inflated by volume preservation.
    balloon = rp.SoftBody.sphere((0.5, 3.0, 0.0), 0.8, 2, softness=(15.0, 1.0), volume_factor=1.2,
                                 particle_mass=0.05)
    soft_bodies.insert(balloon, bodies, colliders)

    # Jelly cubes: corotational, Neo-Hookean, and per-cell volume constraints.
    jelly_material = rp.SoftBodyMaterial(young_modulus=2.0e3, poisson_ratio=0.35, elastic_damping_ratio=0.5)
    for z, model in [(1.5, rp.SoftBodyCellModel.COROTATIONAL), (4.5, rp.SoftBodyCellModel.NEO_HOOKEAN)]:
        jelly = rp.SoftBody.cuboid((3.0, 1.0, z), (0.6, 0.6, 0.6), 5, 5, 5, cell_model=model,
                                   material=jelly_material, particle_mass=0.2)
        soft_bodies.insert(jelly, bodies, colliders)
    volume_jelly = rp.SoftBody.cuboid((3.0, 1.0, -1.5), (0.6, 0.6, 0.6), 5, 5, 5,
                                      cell_model=rp.SoftBodyCellModel.VOLUME, softness=(20.0, 1.0),
                                      particle_mass=0.2)
    soft_bodies.insert(volume_jelly, bodies, colliders)

    # A rope hanging from a fixed anchor, holding a rigid ball attached by its last particle.
    rope = rp.SoftBody.rope((-0.5, 5.0, 3.0), (2.5, 5.0, 3.0), 30, pinned_particles=[0],
                            softness=(40.0, 1.0), particle_mass=0.05)
    rope_handle = soft_bodies.insert(rope, bodies, colliders)
    last = soft_bodies[rope_handle].particle_position(29)
    weight = bodies.insert(rp.RigidBody.dynamic(translation=(last.x, last.y - 0.3, last.z)))
    colliders.insert_with_parent(rp.Collider.ball(0.25).density(2.0), weight, bodies)
    soft_bodies[rope_handle].attach_particle(29, weight, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints, soft_bodies)
    testbed.look_at((9.0, 6.0, 12.0), (0.0, 1.5, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
