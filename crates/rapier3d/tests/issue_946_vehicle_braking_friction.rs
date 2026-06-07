//! Regression test for issue #946: when a wheel's side impulse is exactly zero (e.g. no
//! steering and no lateral slip), the skid clamp was skipped entirely and the brake
//! impulse was applied unclamped, giving the vehicle infinite effective braking friction.

use rapier3d::control::{DynamicRayCastVehicleController, WheelTuning};
use rapier3d::prelude::*;

/// Braking in a straight line must be limited by tire friction (`friction_slip`): a huge
/// brake input on a fast vehicle must slow it down over several steps, not stop it dead
/// in a single step, and must never push it backwards.
#[test]
fn straight_line_braking_is_friction_limited() {
    let mut world = PhysicsWorld::new();

    let ground =
        ColliderBuilder::cuboid(100.0, 0.1, 100.0).translation(Vector::new(0.0, -0.1, 0.0));
    world.insert_collider(ground, None);

    let hw = 0.3;
    let hh = 0.15;
    let chassis = RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.3, 0.0));
    let collider = ColliderBuilder::cuboid(hw * 2.0, hh, hw).density(100.0);
    let (vehicle_handle, _) = world.insert(chassis, collider);

    let tuning = WheelTuning {
        suspension_stiffness: 100.0,
        suspension_damping: 10.0,
        // A small friction budget so that friction-limited braking takes many steps.
        friction_slip: 2.0,
        // No lateral tire force: the side impulse stays exactly 0.0, which is the
        // configuration that used to skip the skid clamp entirely.
        side_friction_stiffness: 0.0,
        ..WheelTuning::default()
    };
    let mut vehicle = DynamicRayCastVehicleController::new(vehicle_handle);
    for pos in [
        Vector::new(hw * 1.5, -hh, hw),
        Vector::new(hw * 1.5, -hh, -hw),
        Vector::new(-hw * 1.5, -hh, hw),
        Vector::new(-hw * 1.5, -hh, -hw),
    ] {
        vehicle.add_wheel(pos, -Vector::Y, Vector::Z, hh, hh / 4.0, &tuning);
    }

    // Let the suspension settle before launching the vehicle.
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

    let speed_before_braking = 10.0;
    {
        let body = &mut world.bodies[vehicle_handle];
        let vel = Vector::new(speed_before_braking, body.linvel().y, 0.0);
        body.set_linvel(vel, true);
    }

    // An absurdly large brake input: the resulting impulse must be clamped by the
    // friction budget (suspension force * dt * friction_slip), not applied verbatim.
    let mut min_speed: Real = Real::MAX;
    let mut max_step_decel: Real = 0.0;
    let mut speed_after_5_steps: Real = 0.0;
    for i in 0..400 {
        for wheel in vehicle.wheels_mut() {
            wheel.brake = 1.0e9;
        }

        let speed_before = world.bodies[vehicle_handle].linvel().x;
        let q = world.broad_phase.as_query_pipeline_mut(
            world.narrow_phase.query_dispatcher(),
            &mut world.bodies,
            &mut world.colliders,
            QueryFilter::exclude_dynamic().exclude_rigid_body(vehicle_handle),
        );
        vehicle.update_vehicle(world.integration_parameters.dt, q);
        world.step();
        let speed_after = world.bodies[vehicle_handle].linvel().x;

        min_speed = min_speed.min(speed_after);
        max_step_decel = max_step_decel.max(speed_before - speed_after);
        if i == 4 {
            speed_after_5_steps = speed_after;
        }
    }

    // With the friction clamp, each step can only remove
    // ~2 * suspension_force * dt * friction_slip / mass of velocity (a fraction of the
    // initial speed). Without it, the first braking step zeroed the velocity outright.
    assert!(
        speed_after_5_steps > 0.5 * speed_before_braking,
        "friction-limited braking must take many steps; after 5 steps speed was {speed_after_5_steps}"
    );
    assert!(
        max_step_decel < 0.2 * speed_before_braking,
        "per-step deceleration must be bounded by the friction budget, got {max_step_decel}"
    );
    assert!(
        min_speed > -0.5,
        "an over-braked vehicle must not be pushed backwards, got min speed {min_speed}"
    );
    let final_speed = world.bodies[vehicle_handle].linvel().x;
    assert!(
        final_speed.abs() < 0.5,
        "vehicle should still have braked to a stop, got {final_speed}"
    );
}
