"""Port of examples2d/sensor2.rs."""
from __future__ import annotations

import rapier2d as rp
from .._registry import register

CATEGORY = "Collisions"
NAME = "Sensor"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 200.1
    ground_height = 0.1

    ground_handle = bodies.insert(rp.RigidBody.fixed().translation((0.0, -ground_height)))
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height), ground_handle, bodies
    )

    num = 10
    rad = 0.2
    shift = rad * 2.0
    centerx = shift * num / 2.0

    for i in range(num):
        x = i * shift - centerx
        y = 3.0
        handle = bodies.insert(rp.RigidBody.dynamic().translation((x, y)))
        colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), handle, bodies)

    sensor_handle = bodies.insert(rp.RigidBody.dynamic().translation((0.0, 10.0)))
    colliders.insert_with_parent(rp.Collider.cuboid(rad, rad), sensor_handle, bodies)
    colliders.insert_with_parent(
        rp.Collider.ball(rad * 5.0)
        .density(0.0)
        .sensor(True)
        .active_events(rp.ActiveEvents.COLLISION_EVENTS),
        sensor_handle,
        bodies,
    )

    events = rp.ChannelEventCollector()

    def callback(tb) -> None:
        # Drain so the queue does not grow without bound.
        events.drain_collision_events()

    testbed.set_event_handler(events)
    testbed.add_callback(callback)
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.set_camera_2d(center=(0.0, 1.0), zoom=100.0)


register(CATEGORY, NAME, init_world, dim=2)

if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
