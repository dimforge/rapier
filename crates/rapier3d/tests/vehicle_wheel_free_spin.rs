//! Wheels spin from the engine force while airborne or slipping (burnout), and stop or
//! lock from the brake, without any extra tuning parameter.

use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;

const HW: Real = 0.3;
const HH: Real = 0.15;

fn build(
    world: &mut PhysicsWorld,
    with_ground: bool,
    tuning: &WheelTuning,
) -> (RigidBodyHandle, DynamicRayCastVehicleController) {
    if with_ground {
        let ground =
            ColliderBuilder::cuboid(100.0, 0.1, 100.0).translation(Vector::new(0.0, -0.1, 0.0));
        world.insert_collider(ground, None);
    }
    let chassis = RigidBodyBuilder::dynamic()
        .translation(Vector::new(0.0, 0.3, 0.0))
        .gravity_scale(if with_ground { 1.0 } else { 0.0 });
    let collider = ColliderBuilder::cuboid(HW * 2.0, HH, HW).density(100.0);
    let (handle, _) = world.insert(chassis, collider);

    let mut vehicle = DynamicRayCastVehicleController::new(handle);
    for pos in [
        Vector::new(HW * 1.5, -HH, HW),
        Vector::new(HW * 1.5, -HH, -HW),
        Vector::new(-HW * 1.5, -HH, HW),
        Vector::new(-HW * 1.5, -HH, -HW),
    ] {
        vehicle.add_wheel(pos, -Vector::Y, -Vector::Z, HH, HH / 4.0, tuning);
    }
    (handle, vehicle)
}

fn step(
    world: &mut PhysicsWorld,
    vehicle: &mut DynamicRayCastVehicleController,
    handle: RigidBodyHandle,
) {
    let q = world.broad_phase.as_query_pipeline_mut(
        world.narrow_phase.query_dispatcher(),
        &mut world.bodies,
        &mut world.colliders,
        QueryFilter::exclude_dynamic().exclude_rigid_body(handle),
    );
    vehicle.update_vehicle(world.integration_parameters.dt, q);
    world.step();
}

/// Tire surface speed over the last step, in m/s.
fn spin_speeds(vehicle: &DynamicRayCastVehicleController, before: &[Real], dt: Real) -> Vec<Real> {
    vehicle
        .wheels()
        .iter()
        .zip(before)
        .map(|(w, b)| (w.rotation - b) * w.radius / dt)
        .collect()
}

#[test]
fn airborne_wheels_spin_up_under_engine_force() {
    let mut world = PhysicsWorld::new();
    let tuning = WheelTuning::default();
    let (handle, mut vehicle) = build(&mut world, false, &tuning);
    let dt = world.integration_parameters.dt;

    // No engine: the wheels stay still.
    for _ in 0..10 {
        step(&mut world, &mut vehicle, handle);
    }
    assert!(vehicle.wheels().iter().all(|w| w.rotation == 0.0));

    for wheel in vehicle.wheels_mut() {
        wheel.engine_force = 200.0;
    }
    let mut speeds = vec![];
    for _ in 0..60 {
        let before: Vec<Real> = vehicle.wheels().iter().map(|w| w.rotation).collect();
        step(&mut world, &mut vehicle, handle);
        speeds.push(spin_speeds(&vehicle, &before, dt)[0]);
    }
    assert!(
        speeds[0] > 0.0 && speeds.windows(2).all(|w| w[1] > w[0]),
        "the wheel should keep spinning up under throttle: {speeds:?}"
    );

    // Reversing the engine spins the wheel the other way.
    for wheel in vehicle.wheels_mut() {
        wheel.engine_force = -200.0;
    }
    let mut last = 0.0;
    for _ in 0..600 {
        let before: Vec<Real> = vehicle.wheels().iter().map(|w| w.rotation).collect();
        step(&mut world, &mut vehicle, handle);
        last = spin_speeds(&vehicle, &before, dt)[0];
    }
    assert!(
        last < 0.0,
        "reverse engine force should spin the wheel backward ({last})"
    );

    // Engine off: the spin decays.
    for wheel in vehicle.wheels_mut() {
        wheel.engine_force = 0.0;
    }
    for _ in 0..600 {
        step(&mut world, &mut vehicle, handle);
    }
    let before: Vec<Real> = vehicle.wheels().iter().map(|w| w.rotation).collect();
    step(&mut world, &mut vehicle, handle);
    let coasting = spin_speeds(&vehicle, &before, dt)[0];
    assert!(
        coasting.abs() < last.abs() * 0.1,
        "the free spin should decay ({coasting})"
    );
}

#[test]
fn wheels_spin_faster_than_the_ground_on_burnout() {
    let mut world = PhysicsWorld::new();
    let tuning = WheelTuning {
        suspension_stiffness: 100.0,
        suspension_damping: 10.0,
        friction_slip: 0.5, // very little grip
        ..WheelTuning::default()
    };
    let (handle, mut vehicle) = build(&mut world, true, &tuning);
    let dt = world.integration_parameters.dt;

    for _ in 0..50 {
        step(&mut world, &mut vehicle, handle);
    }

    for wheel in vehicle.wheels_mut() {
        wheel.engine_force = 5000.0;
    }
    let mut speeds = vec![];
    for _ in 0..30 {
        let before: Vec<Real> = vehicle.wheels().iter().map(|w| w.rotation).collect();
        step(&mut world, &mut vehicle, handle);
        speeds = spin_speeds(&vehicle, &before, dt);
    }
    let chassis_speed = world.bodies[handle].linvel().length();
    for (i, s) in speeds.iter().enumerate() {
        assert!(
            s.abs() > chassis_speed * 1.5,
            "wheel {i} should spin faster than the ground ({s} vs chassis {chassis_speed})"
        );
    }
}

#[test]
fn airborne_brake_stops_the_wheel_spin() {
    let mut world = PhysicsWorld::new();
    let tuning = WheelTuning::default();
    let (handle, mut vehicle) = build(&mut world, false, &tuning);
    let dt = world.integration_parameters.dt;

    for wheel in vehicle.wheels_mut() {
        wheel.engine_force = 200.0;
    }
    for _ in 0..60 {
        step(&mut world, &mut vehicle, handle);
    }

    for wheel in vehicle.wheels_mut() {
        wheel.engine_force = 0.0;
        wheel.brake = 50.0;
    }
    for _ in 0..10 {
        step(&mut world, &mut vehicle, handle);
    }
    let before: Vec<Real> = vehicle.wheels().iter().map(|w| w.rotation).collect();
    step(&mut world, &mut vehicle, handle);
    let speeds = spin_speeds(&vehicle, &before, dt);
    assert!(
        speeds.iter().all(|s| *s == 0.0),
        "braked airborne wheels should stop: {speeds:?}"
    );
}

/// Drives straight on the ground until fast, then brakes with `brake` for 10 steps.
/// Returns the tire surface speeds and the chassis speed of the last step.
fn drive_then_brake(brake: Real) -> (Vec<Real>, Real) {
    let mut world = PhysicsWorld::new();
    let tuning = WheelTuning {
        suspension_stiffness: 100.0,
        suspension_damping: 10.0,
        friction_slip: 1.0,
        ..WheelTuning::default()
    };
    let (handle, mut vehicle) = build(&mut world, true, &tuning);
    let dt = world.integration_parameters.dt;

    for _ in 0..50 {
        step(&mut world, &mut vehicle, handle);
    }
    for wheel in vehicle.wheels_mut() {
        wheel.engine_force = 100.0;
    }
    for _ in 0..60 {
        step(&mut world, &mut vehicle, handle);
    }
    let start_speed = world.bodies[handle].linvel().length();
    assert!(
        start_speed > 3.0,
        "the vehicle should be moving ({start_speed})"
    );

    for wheel in vehicle.wheels_mut() {
        wheel.engine_force = 0.0;
        wheel.brake = brake;
    }
    let mut speeds = vec![];
    for _ in 0..10 {
        let before: Vec<Real> = vehicle.wheels().iter().map(|w| w.rotation).collect();
        step(&mut world, &mut vehicle, handle);
        speeds = spin_speeds(&vehicle, &before, dt);
    }
    (speeds, world.bodies[handle].linvel().length())
}

#[test]
fn hard_braking_locks_the_wheels_while_sliding() {
    let (speeds, chassis_speed) = drive_then_brake(1000.0);
    assert!(
        chassis_speed > 1.0,
        "the vehicle should still be sliding ({chassis_speed})"
    );
    assert!(
        speeds.iter().all(|s| *s == 0.0),
        "locked wheels should not rotate: {speeds:?}"
    );
}

#[test]
fn gentle_braking_keeps_the_wheels_rolling() {
    let (speeds, chassis_speed) = drive_then_brake(0.2);
    assert!(
        chassis_speed > 1.0,
        "the vehicle should be moving ({chassis_speed})"
    );
    for (i, s) in speeds.iter().enumerate() {
        assert!(
            (s.abs() - chassis_speed).abs() < chassis_speed * 0.1,
            "wheel {i} should roll with the ground ({s} vs chassis {chassis_speed})"
        );
    }
}
