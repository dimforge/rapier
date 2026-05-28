"""Port of examples2d/ccd2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Dynamics"
NAME = "CCD"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    # Ground (CCD-enabled).
    ground_size = 25.0
    ground_thickness = 0.1

    ground_handle = bodies.insert(rp.RigidBody.fixed().ccd_enabled(True))
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_thickness), ground_handle, bodies
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_thickness, ground_size).translation((-3.0, 0.0)),
        ground_handle,
        bodies,
    )
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_thickness, ground_size).translation((6.0, 0.0)),
        ground_handle,
        bodies,
    )

    # Sensor cuboid.
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_thickness, ground_size)
        .translation((2.5, 0.0))
        .sensor(True)
        .active_events(rp.ActiveEvents.COLLISION_EVENTS),
        ground_handle,
        bodies,
    )

    # Compound projectile (Rust uses a compound of three sub-cuboids).
    radx = 0.4
    rady = 0.05

    horizontal = rp.SharedShape.cuboid(radx, rady)
    vertical = rp.SharedShape.cuboid(rady, radx)

    delta1 = rp.Isometry2.from_translation(0.0, radx - rady)
    delta2 = rp.Isometry2.from_translation(-radx + rady, 0.0)
    delta3 = rp.Isometry2.from_translation(radx - rady, 0.0)

    compound_shape = rp.SharedShape.compound(
        [(delta1, horizontal), (delta2, vertical), (delta3, vertical)]
    )

    num = 6
    shift = (radx + 0.01) * 2.0
    centerx = shift * num / 2.0 - 0.5
    centery = shift / 2.0 + 4.0

    for i in range(num):
        for j in range(num):
            x = i * shift - centerx
            y = j * shift + centery
            handle = bodies.insert(
                rp.RigidBody.dynamic()
                .translation((x, y))
                .linvel((100.0, -10.0))
                .ccd_enabled(True)
            )
            colliders.insert_with_parent(
                rp.Collider.new(compound_shape), handle, bodies
            )

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 2.5), zoom=20.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
