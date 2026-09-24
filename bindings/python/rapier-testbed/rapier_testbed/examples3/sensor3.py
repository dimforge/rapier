"""Port of examples3d/sensor3.rs."""
from __future__ import annotations

import rapier3d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Sensor"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 10.1
    ground_height = 0.1
    ground = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    ground_h = bodies.insert(ground)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size), ground_h, bodies
    )

    num = 10
    rad = 0.2
    shift = rad * 2.0
    centerx = shift * num / 2.0
    centerz = shift * num / 2.0
    for i in range(num):
        for k in range(num):
            x = i * shift - centerx
            y = 3.0
            z = k * shift - centerz
            body = rp.RigidBody.dynamic().translation((x, y, z))
            h = bodies.insert(body)
            colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), h, bodies)

    # Sensor body.
    sensor_body = rp.RigidBody.dynamic().translation((0.0, 5.0, 0.0))
    sensor_h = bodies.insert(sensor_body)
    colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), sensor_h, bodies)

    sensor_col = (
        rp.Collider.ball(rad * 5.0)
        .density(0.0)
        .sensor(True)
        .active_events(rp.ActiveEvents.COLLISION_EVENTS)
    )
    colliders.insert_with_parent(sensor_col, sensor_h, bodies)

    # Set up an event collector so the sensor proximities don't go to /dev/null.
    events = rp.ChannelEventCollector()
    testbed.set_event_handler(events)

    def _cb(tb) -> None:
        # Drain events each frame; the Rust example also recolors bodies but
        # we don't have that hook in the mini-testbed yet.
        events.drain_collision_events()
        events.drain_contact_force_events()

    testbed.add_callback(_cb)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((6.0, 4.0, 6.0), (0.0, 1.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
