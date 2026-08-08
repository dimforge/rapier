//! Regression test for issue #925: `Wheel::rotation` was derived from the chassis'
//! `index_forward_axis` instead of the wheel's own rolling direction, so a vehicle whose
//! wheels roll along a different axis (or, through the JS bindings, one that simply drives
//! without steering) never saw its wheels rotate.

use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;

/// A vehicle with wheel axles along +X drives along -Z (its per-wheel forward direction),
/// which is orthogonal to the default `index_forward_axis` (X). The wheels must still
/// accumulate rotation, consistently across all four wheels.
#[test]
fn wheels_rotate_when_rolling_off_the_chassis_forward_axis() {
    let mut world = PhysicsWorld::new();

    let ground =
        ColliderBuilder::cuboid(100.0, 0.1, 100.0).translation(Vector::new(0.0, -0.1, 0.0));
    world.insert_collider(ground, None);

    let hw = 0.3;
    let hh = 0.15;
    let chassis = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.3, 0.0));
    let collider = ColliderBuilder::cuboid(hw, hh, hw * 2.0).density(100.0);
    let (vehicle_handle, _) = world.insert(chassis, collider);

    let tuning = WheelTuning {
        suspension_stiffness: 100.0,
        suspension_damping: 10.0,
        ..WheelTuning::default()
    };
    let mut vehicle = DynamicRayCastVehicleController::new(vehicle_handle);
    // Axles along +X: the wheels roll along -Z, not along the default forward axis (X).
    for pos in [
        Vector::new(hw, -hh, hw * 1.5),
        Vector::new(-hw, -hh, hw * 1.5),
        Vector::new(hw, -hh, -hw * 1.5),
        Vector::new(-hw, -hh, -hw * 1.5),
    ] {
        vehicle.add_wheel(pos, -Vector::Y, Vector::X, hh, hh / 4.0, &tuning);
    }

    // Settle, then drive straight with no steering.
    for _ in 0..50 {
        let q = world.broad_phase.as_query_pipeline_mut(
            world.narrow_phase.query_dispatcher(),
            &mut world.bodies,
            &mut world.colliders,
            QueryFilter::exclude_dynamic().exclude_rigid_body(vehicle_handle),
        );
        vehicle.update_vehicle(world.integration_parameters.dt, q);
        world.step();
    }

    let rotation_before: Vec<Real> = vehicle.wheels().iter().map(|w| w.rotation).collect();
    let z_before = world.bodies[vehicle_handle].translation().z;

    for _ in 0..120 {
        for wheel in vehicle.wheels_mut() {
            wheel.engine_force = 20.0;
        }
        let q = world.broad_phase.as_query_pipeline_mut(
            world.narrow_phase.query_dispatcher(),
            &mut world.bodies,
            &mut world.colliders,
            QueryFilter::exclude_dynamic().exclude_rigid_body(vehicle_handle),
        );
        vehicle.update_vehicle(world.integration_parameters.dt, q);
        world.step();
    }

    let z_after = world.bodies[vehicle_handle].translation().z;
    assert!(
        (z_after - z_before).abs() > 0.5,
        "the vehicle should have driven along Z, moved {}",
        z_after - z_before
    );

    let deltas: Vec<Real> = vehicle
        .wheels()
        .iter()
        .zip(&rotation_before)
        .map(|(w, before)| w.rotation - before)
        .collect();
    for (i, delta) in deltas.iter().enumerate() {
        assert!(
            delta.abs() > 1.0,
            "wheel {i} did not rotate while driving straight (delta {delta})"
        );
        assert!(
            delta.signum() == deltas[0].signum(),
            "wheels rotated in inconsistent directions: {deltas:?}"
        );
    }
}
