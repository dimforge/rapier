use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;

fn main() {
    let mut world = PhysicsWorld::new();
    world.insert_collider(ColliderBuilder::cuboid(100.0, 0.1, 100.0), None);

    // DOCUSAURUS: Vehicle start
    // The chassis is an ordinary dynamic rigid-body.
    let hw = 0.3;
    let hh = 0.15;
    let (chassis_handle, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.0, 0.0)),
        ColliderBuilder::cuboid(hw * 2.0, hh, hw).density(100.0),
    );

    // The tuning shared by the wheels: the suspension and the grip.
    let tuning = WheelTuning {
        suspension_stiffness: 100.0,
        suspension_damping: 10.0,
        ..WheelTuning::default()
    };

    let mut vehicle = DynamicRayCastVehicleController::new(chassis_handle);
    let wheel_positions = [
        Vector::new(hw * 1.5, -hh, hw),
        Vector::new(hw * 1.5, -hh, -hw),
        Vector::new(-hw * 1.5, -hh, hw),
        Vector::new(-hw * 1.5, -hh, -hw),
    ];

    for position in wheel_positions {
        // The position of the wheel, the direction its suspension pushes along, its axle, the
        // rest length of its suspension, and its radius; all in the local frame of the chassis.
        vehicle.add_wheel(position, -Vector::Y, Vector::Z, hh, hh / 4.0, &tuning);
    }
    // DOCUSAURUS: Vehicle stop

    // DOCUSAURUS: VehicleUpdate start
    for _ in 0..200 {
        // The vehicle is driven by setting the engine force, the brake, and the steering angle of
        // its wheels. Here the two front wheels are the driving and steering ones.
        let wheels = vehicle.wheels_mut();
        wheels[0].engine_force = 30.0;
        wheels[0].steering = 0.2;
        wheels[1].engine_force = 30.0;
        wheels[1].steering = 0.2;

        // The wheels are ray-casted against the scene: the chassis itself, as well as every other
        // dynamic body, is generally excluded from these ray-casts.
        let queries = world.broad_phase.as_query_pipeline_mut(
            world.narrow_phase.query_dispatcher(),
            &mut world.bodies,
            &mut world.colliders,
            QueryFilter::exclude_dynamic().exclude_rigid_body(chassis_handle),
        );
        vehicle.update_vehicle(world.integration_parameters.dt, queries);

        world.step();
    }

    println!("Vehicle speed: {}", vehicle.current_vehicle_speed);
    // DOCUSAURUS: VehicleUpdate stop
}
