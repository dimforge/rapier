use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_physics)
        .add_systems(FixedUpdate, drive_vehicle)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera so we can see the debug-render.
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-3.0, 3.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn setup_physics(mut commands: Commands) {
    /* Create the ground. */
    commands
        .spawn(Collider::cuboid(100.0, 0.1, 100.0))
        .insert(Transform::from_xyz(0.0, -0.1, 0.0));

    // DOCUSAURUS: Vehicle start
    // The tuning shared by the wheels: the suspension and the grip.
    let tuning = WheelTuning {
        suspension_stiffness: 100.0,
        suspension_damping: 10.0,
        ..WheelTuning::default()
    };

    let hw = 0.3;
    let hh = 0.15;
    let wheel_positions = [
        Vec3::new(hw * 1.5, -hh, hw),
        Vec3::new(hw * 1.5, -hh, -hw),
        Vec3::new(-hw * 1.5, -hh, hw),
        Vec3::new(-hw * 1.5, -hh, -hw),
    ];
    let wheels = wheel_positions
        .into_iter()
        .map(|position| {
            // The position of the wheel, the direction its suspension pushes along, its axle, the
            // rest length of its suspension, and its radius; all in the local frame of the chassis.
            VehicleWheel::new(position, -Vec3::Y, Vec3::Z, hh, hh / 4.0, tuning)
        })
        .collect();

    // The chassis is an ordinary dynamic rigid-body, with the vehicle controller attached to it.
    commands.spawn((
        Transform::from_xyz(0.0, 1.0, 0.0),
        RigidBody::Dynamic,
        Collider::cuboid(hw * 2.0, hh, hw),
        ColliderMassProperties::Density(100.0),
        RayCastVehicleController::new(wheels),
    ));
    // DOCUSAURUS: Vehicle stop
}

// DOCUSAURUS: VehicleUpdate start
fn drive_vehicle(mut vehicles: Query<&mut RayCastVehicleController>) {
    for mut vehicle in vehicles.iter_mut() {
        // The vehicle is driven by setting the engine force, the brake, and the steering angle of
        // its wheels. Here the two front wheels are the driving and steering ones.
        for wheel in &mut vehicle.wheels[0..2] {
            wheel.engine_force = 30.0;
            wheel.steering = 0.2;
        }

        // The colliders of the chassis are always ignored by the ray-casts of the wheels. Other
        // colliders, here the ones attached to dynamic rigid-bodies, can be ignored too.
        vehicle.filter_flags =
            QueryFilterFlags::EXCLUDE_SENSORS | QueryFilterFlags::EXCLUDE_DYNAMIC;

        // The results of the last update are written back into the component.
        println!("Vehicle speed: {}", vehicle.current_vehicle_speed);
        for wheel in &vehicle.wheels {
            println!(
                "Wheel in contact: {}, suspension length: {}",
                wheel.state.is_in_contact, wheel.state.suspension_length
            );
        }
    }
}
// DOCUSAURUS: VehicleUpdate stop
