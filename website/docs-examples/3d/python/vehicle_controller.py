import rapier3d as rp

world = rp.PhysicsWorld(gravity=(0.0, -9.81, 0.0))
world.colliders.insert(rp.Collider.cuboid(100.0, 0.1, 100.0).build())

# DOCUSAURUS: Vehicle start
# The chassis is an ordinary dynamic rigid-body.
hw = 0.3
hh = 0.15
chassis_handle = world.add_body(
    rp.RigidBody.dynamic(translation=(0.0, 1.0, 0.0)),
    colliders=[rp.Collider.cuboid(hw * 2.0, hh, hw).density(100.0)],
)

# The tuning shared by the wheels: the suspension and the grip. The
# parameters that aren't given keep their default values.
tuning = rp.WheelTuning(suspension_stiffness=100.0, suspension_damping=10.0)

vehicle = rp.DynamicRayCastVehicleController(chassis_handle)
wheel_positions = [
    (hw * 1.5, -hh, hw),
    (hw * 1.5, -hh, -hw),
    (-hw * 1.5, -hh, hw),
    (-hw * 1.5, -hh, -hw),
]

for position in wheel_positions:
    # The position of the wheel, the direction its suspension pushes along, its axle, the
    # rest length of its suspension, and its radius; all in the local frame of the chassis.
    vehicle.add_wheel(position, (0.0, -1.0, 0.0), (0.0, 0.0, 1.0), hh, hh / 4.0, tuning)
# DOCUSAURUS: Vehicle stop
assert len(vehicle.wheels()) == 4

# DOCUSAURUS: VehicleUpdate start
for _ in range(200):
    # The vehicle is driven by setting the engine force, the brake, and the steering angle of
    # its wheels (from their indices). Here the two front wheels are the driving and steering ones.
    vehicle.apply_engine_force(0, 30.0)
    vehicle.set_steering(0, 0.2)
    vehicle.apply_engine_force(1, 30.0)
    vehicle.set_steering(1, 0.2)

    # The wheels are ray-casted against the scene: the chassis itself is always excluded from
    # these ray-casts, and every other dynamic body is generally excluded too.
    vehicle.update_vehicle(
        world.integration_parameters.dt,
        world.rigid_bodies,
        world.colliders,
        world.query_pipeline,
        rp.QueryFilter.exclude_dynamic(),
    )

    world.step()

print("Vehicle speed:", vehicle.current_vehicle_speed)
# DOCUSAURUS: VehicleUpdate stop
assert vehicle.current_vehicle_speed > 0.0

# DOCUSAURUS: VehicleWheels start
# The wheels are copies of the state of the wheels after the last update.
for wheel in vehicle.wheels():
    contact = wheel.raycast_info
    print(
        "Wheel center:", wheel.center,  # World-space center of the wheel.
        "axle:", wheel.axle,  # World-space direction of its axle.
        "rotation:", wheel.rotation,  # Rotation angle around its axle.
        "suspension length:", contact.suspension_length,
        "touches the ground:", contact.is_in_contact,
        "ground collider:", contact.ground_object,  # None if it doesn’t touch the ground.
    )
# DOCUSAURUS: VehicleWheels stop

for wheel in vehicle.wheels():
    contact = wheel.raycast_info
    assert (wheel.center - world.rigid_bodies[chassis_handle].translation).norm() < 1.0
    assert contact.is_in_contact
    assert contact.ground_object is not None
    assert contact.suspension_length > 0.0
    assert wheel.rotation != 0.0
