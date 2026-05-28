"""Port of examples3d/gyroscopic3.rs (Dzhanibekov effect)."""
from __future__ import annotations

import rapier3d as rp
from .._registry import register

CATEGORY = "Dynamics"
NAME = "Gyroscopic"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    shapes = [
        (rp.Isometry3.identity(), rp.SharedShape.cuboid(2.0, 0.2, 0.2)),
        (
            rp.Isometry3.from_translation(0.0, 0.8, 0.0),
            rp.SharedShape.cuboid(0.2, 0.4, 0.2),
        ),
    ]

    body = (
        rp.RigidBody.dynamic()
        .gravity_scale(0.0)
        .angvel((0.0, 20.0, 0.1))
        .gyroscopic_forces(True)
    )
    body_h = bodies.insert(body)
    colliders.insert_with_parent(rp.Collider.compound(shapes), body_h, bodies)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((8.0, 0.0, 8.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
