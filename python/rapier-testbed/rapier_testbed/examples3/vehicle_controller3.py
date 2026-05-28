"""Port of examples3d/vehicle_controller3.rs.

Uses :class:`DynamicRayCastVehicleController`. Without keyboard input
plumbing the vehicle just sits there, but the controller is created and
its per-frame ``update_vehicle`` is wired up so the suspensions tick.
"""
from __future__ import annotations

import math

import numpy as np

import rapier3d as rp
from .._registry import register

CATEGORY = "Controls"
NAME = "Vehicle controller"


def init_world(testbed) -> None:
    bodies = rp.RigidBodySet()
    colliders = rp.ColliderSet()
    impulse_joints = rp.ImpulseJointSet()
    multibody_joints = rp.MultibodyJointSet()

    ground_size = 5.0
    ground_height = 0.1
    floor = rp.RigidBody.fixed().translation((0.0, -ground_height, 0.0))
    floor_h = bodies.insert(floor)
    colliders.insert_with_parent(
        rp.Collider.cuboid(ground_size, ground_height, ground_size), floor_h, bodies
    )

    hw = 0.3
    hh = 0.15
    body = rp.RigidBody.dynamic().translation((0.0, 1.0, 0.0))
    vehicle_h = bodies.insert(body)
    colliders.insert_with_parent(
        rp.Collider.cuboid(hw * 2.0, hh, hw).density(100.0), vehicle_h, bodies
    )

    tuning = rp.WheelTuning(suspension_stiffness=100.0, suspension_damping=10.0)
    vehicle = rp.DynamicRayCastVehicleController(vehicle_h)
    wheel_positions = [
        (hw * 1.5, -hh, hw),
        (hw * 1.5, -hh, -hw),
        (-hw * 1.5, -hh, hw),
        (-hw * 1.5, -hh, -hw),
    ]
    for pos in wheel_positions:
        vehicle.add_wheel(pos, (0.0, -1.0, 0.0), (0.0, 0.0, 1.0), hh, hh / 4.0, tuning)

    # Falling cubes.
    num = 8
    rad = 0.1
    shift = rad * 2.0
    centerx = shift * (num // 2)
    centery = rad
    for k in range(4):
        for i in range(num):
            x = i * shift - centerx
            y = centery
            z = k * shift + centerx
            body = rp.RigidBody.dynamic().translation((x, y, z))
            h = bodies.insert(body)
            colliders.insert_with_parent(rp.Collider.cuboid(rad, rad, rad), h, bodies)

    # Climbable slope.
    slope_angle = 0.2
    slope_size = 2.0
    col = (
        rp.Collider.cuboid(slope_size, ground_height, ground_size)
        .translation((ground_size + slope_size, -ground_height + 0.4, 0.0))
        .rotation((0.0, 0.0, slope_angle))
    )
    colliders.insert(col)

    impossible_slope_angle = 0.9
    impossible_slope_size = 2.0
    col = (
        rp.Collider.cuboid(slope_size, ground_height, ground_size)
        .translation(
            (
                ground_size + slope_size * 2.0 + impossible_slope_size - 0.9,
                -ground_height + 2.3,
                0.0,
            )
        )
        .rotation((0.0, 0.0, impossible_slope_angle))
    )
    colliders.insert(col)

    # Wavy heightfield.
    ground_extent = (10.0, 0.4, 10.0)
    nsubdivs = 20
    heights = np.zeros((nsubdivs + 1, nsubdivs + 1), dtype=np.float32)
    for i in range(nsubdivs + 1):
        for j in range(nsubdivs + 1):
            heights[i, j] = (
                -math.cos(i * ground_extent[0] / nsubdivs / 2.0)
                - math.cos(j * ground_extent[2] / nsubdivs / 2.0)
            )
    col = rp.Collider.heightfield(heights, ground_extent).translation((-7.0, 0.0, 0.0))
    colliders.insert(col)

    # Per-step callback: advance the vehicle controller. It needs a
    # QueryPipeline, so we construct one from the broad-phase each step.
    def _cb(tb) -> None:
        try:
            queries = tb._broad_phase.as_query_pipeline(
                tb._narrow_phase.query_dispatcher(),
                tb.bodies,
                tb.colliders,
            )
        except AttributeError:
            return
        vehicle.update_vehicle(
            tb._integration_parameters.dt, tb.bodies, tb.colliders, queries
        )

    testbed.add_callback(_cb)

    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints)
    testbed.look_at((10.0, 10.0, 10.0), (0.0, 0.0, 0.0))


register(CATEGORY, NAME, init_world, dim=3)


if __name__ == "__main__":
    from .._picker import run

    run(initial=f"{CATEGORY} / {NAME}")
