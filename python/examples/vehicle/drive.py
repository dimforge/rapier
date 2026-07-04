"""DynamicRayCastVehicleController: chassis with four wheels drives forward.

Builds a small box-chassis with four wheels (parameters mirror
`examples3d/vehicle_controller3.rs`), lets it settle, then applies engine
force on the rear wheels and reports the chassis velocity.

Run::

    python python/examples/vehicle/drive.py
"""

from __future__ import annotations

import rapier3d as rp


def main() -> None:
    world = rp.PhysicsWorld(gravity=(0, -9.81, 0), auto_update_query=True)

    # Ground.
    world.add_body(
        rp.RigidBody.fixed(translation=(0, -0.1, 0)),
        colliders=[rp.Collider.cuboid(50, 0.1, 50)],
    )

    # Chassis.
    hw, hh = 0.3, 0.15
    chassis = world.add_body(
        rp.RigidBody.dynamic(translation=(0, 1.0, 0)),
        colliders=[rp.Collider.cuboid(hw * 2.0, hh, hw, density=100.0)],
    )

    veh = rp.DynamicRayCastVehicleController(chassis)
    tuning = rp.WheelTuning(suspension_stiffness=100.0, suspension_damping=10.0)
    for x, z in [(hw * 1.5, hw), (hw * 1.5, -hw), (-hw * 1.5, hw), (-hw * 1.5, -hw)]:
        veh.add_wheel(
            chassis_connection_cs=(x, -hh, z),
            direction_cs=(0, -1, 0),
            axle_cs=(0, 0, 1),
            suspension_rest_length=hh,
            radius=hh / 4.0,
            tuning=tuning,
        )

    # Settle.
    for _ in range(120):
        world.step()
        world.update_query_pipeline()
        veh.update_vehicle(1.0 / 60.0, world.rigid_bodies, world.colliders, world.query_pipeline)

    # Accelerate via the rear wheels.
    veh.apply_engine_force(2, 100.0)
    veh.apply_engine_force(3, 100.0)
    for _ in range(240):
        world.step()
        world.update_query_pipeline()
        veh.update_vehicle(1.0 / 60.0, world.rigid_bodies, world.colliders, world.query_pipeline)

    speed = veh.current_speed_km_hour()
    vx = world.rigid_bodies[chassis].linvel.x
    print(f"vehicle: speed={speed:+.1f} km/h vx={vx:+.2f}")


if __name__ == "__main__":
    main()
