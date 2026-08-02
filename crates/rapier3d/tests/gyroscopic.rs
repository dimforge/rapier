//! Verifies the staged solver's per-substep gyroscopic pass.
//!
//! A torque-free rigid body spun about an axis that is *not* one of its
//! principal axes has an angular momentum `I·w` that is not parallel to `w`, so
//! the gyroscopic term `w × I·w` continuously reorients the angular-velocity
//! vector (precession/nutation). With the term integrated in the substep loop
//! the world-space angular velocity must therefore change over time; with
//! gyroscopic forces disabled there is no torque at all, so it must stay fixed.
//!
//! Crucially, a torque-free body conserves its *world-space angular momentum*
//! `L = R·I_local·Rᵀ·w`, both in magnitude and direction. That invariant only
//! holds if the solver rotates the angular velocity into the true principal
//! frame — i.e. accounts for `MassProperties::principal_inertia_local_frame` —
//! before applying the diagonal inertia. It is grossly violated (tested at ~50%
//! magnitude drift) if the solver naively assumes the principal axes coincide
//! with the body's local frame.

use rapier3d::prelude::*;

/// A torque-free world with one box (half-extents 1×2×3 → three distinct
/// principal inertias) spun about a non-principal axis.
fn spinning_box(gyroscopic: bool) -> (PhysicsWorld, RigidBodyHandle) {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO; // torque-free: isolate the gyroscopic term.

    let handle = world.insert_body(
        RigidBodyBuilder::dynamic()
            .angvel(Vector::new(6.0, 6.0, 0.0)) // spans two different-inertia axes
            .gyroscopic_forces_enabled(gyroscopic)
            .can_sleep(false),
    );
    world.insert_collider(
        ColliderBuilder::cuboid(1.0, 2.0, 3.0).density(1.0),
        Some(handle),
    );
    (world, handle)
}

/// Cosine of the angle between the current and initial angular-velocity vectors.
fn cos_with_initial(w: Vector, w0: Vector) -> f32 {
    w.dot(w0) / (w.length() * w0.length())
}

/// World-space angular momentum `L = R · I_local · Rᵀ · w`.
fn angular_momentum(world: &PhysicsWorld, handle: RigidBodyHandle) -> Vector {
    let body = &world.bodies[handle];
    let rot = *body.rotation();
    let inertia_local = body
        .mass_properties()
        .local_mprops
        .reconstruct_inertia_matrix();
    let w = body.angvel();
    rot * (inertia_local * (rot.inverse() * w))
}

#[test]
fn angular_velocity_precesses_with_gyroscopic() {
    let (mut world, handle) = spinning_box(true);
    let w0 = world.bodies[handle].angvel();

    let mut min_cos = f32::INFINITY;
    for _ in 0..300 {
        world.step();
        min_cos = min_cos.min(cos_with_initial(world.bodies[handle].angvel(), w0));
    }

    // The gyroscopic torque must swing the angular-velocity direction well away
    // from its start (a ~26°+ deflection, cos < 0.9).
    assert!(
        min_cos < 0.9,
        "expected the angular-velocity direction to precess (min cos = {min_cos})"
    );
}

#[test]
fn angular_velocity_fixed_without_gyroscopic() {
    let (mut world, handle) = spinning_box(false);
    let w0 = world.bodies[handle].angvel();

    let mut min_cos = f32::INFINITY;
    let mut max_speed_err = 0.0f32;
    for _ in 0..300 {
        world.step();
        let w = world.bodies[handle].angvel();
        min_cos = min_cos.min(cos_with_initial(w, w0));
        max_speed_err = max_speed_err.max((w.length() - w0.length()).abs());
    }

    // No torque: the world-space angular velocity is rigorously constant.
    assert!(
        min_cos > 0.9999,
        "expected a fixed angular-velocity direction without gyroscopic forces (min cos = {min_cos})"
    );
    assert!(
        max_speed_err < 1.0e-3,
        "expected a fixed angular-velocity magnitude without gyroscopic forces (err = {max_speed_err})"
    );
}

#[test]
fn angular_momentum_conserved_with_tilted_principal_frame() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;

    // Attach the box with a tilted local rotation so its principal axes do NOT
    // line up with the body's local frame (a non-identity
    // `principal_inertia_local_frame`) — exactly the configuration the "trees"
    // benchmark bodies have.
    let handle = world.insert_body(
        RigidBodyBuilder::dynamic()
            .angvel(Vector::new(3.0, 7.0, 2.0))
            .gyroscopic_forces_enabled(true)
            .can_sleep(false),
    );
    let tilt = Rotation::from_axis_angle(Vector::new(1.0, 1.0, 1.0).normalize(), 0.7);
    world.insert_collider(
        ColliderBuilder::cuboid(1.0, 2.0, 3.0)
            .density(1.0)
            .rotation(tilt.to_scaled_axis()),
        Some(handle),
    );

    // Sanity: the principal frame really is tilted (otherwise the test is moot).
    let frame = world.bodies[handle]
        .mass_properties()
        .local_mprops
        .principal_inertia_local_frame;
    let frame_angle = frame.to_axis_angle().1;
    assert!(
        frame_angle > 0.1,
        "test setup expects a tilted principal frame (angle = {frame_angle})"
    );

    let l0 = angular_momentum(&world, handle);
    let n0 = l0.length();

    let mut max_mag_err = 0.0f32;
    let mut min_cos = f32::INFINITY;
    for _ in 0..600 {
        world.step();
        let l = angular_momentum(&world, handle);
        max_mag_err = max_mag_err.max((l.length() - n0).abs() / n0);
        min_cos = min_cos.min(l.dot(l0) / (l.length() * n0));
    }

    // A torque-free body conserves world angular momentum. Only correct handling
    // of the tilted principal frame achieves this; the naive body-frame
    // assumption drifts by tens of percent.
    assert!(
        max_mag_err < 0.02,
        "world angular-momentum magnitude drifted (max relative error = {max_mag_err})"
    );
    assert!(
        min_cos > 0.999,
        "world angular-momentum direction drifted (min cos = {min_cos})"
    );
}
