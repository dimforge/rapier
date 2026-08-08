//! Characterization tests for issue #949 (AccelerationBased coupled linear-axes motor
//! misbehaving in 2D).
//!
//! The simplest headless setups do NOT reproduce the reported misbehavior: a symmetric
//! soft pin (both linear axes coupled, position motor targeting zero separation)
//! converges with both motor models, and the acceleration-based model is properly
//! mass-invariant. These tests pin that down so the working baseline can't regress.
//!
//! What the issue's videos show (a chain of offset-anchor soft pins sagging while
//! dragged) still misbehaves subtly: with an offset anchor the acceleration-based
//! steady-state droop deviates from the force-based one noticeably more than the
//! coupled row's single-scalar effective-mass treatment would suggest. Reproducing the
//! full pathology headless needs the reporter's scene; the issue stays open.

use rapier2d::prelude::*;

fn separation_trajectory(model: MotorModel, mass: Real, k: Real, c: Real) -> Vec<Real> {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;

    let a = world
        .bodies
        .insert(RigidBodyBuilder::dynamic().additional_mass(mass));
    let b = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(1.0, 0.0))
            .additional_mass(mass),
    );

    let mut joint = GenericJoint::new(JointAxesMask::FREE_FIXED_AXES);
    joint.coupled_axes = JointAxesMask::LIN_AXES;
    joint
        .set_motor(JointAxis::LinX, 0.0, 0.0, k, c)
        .set_motor_model(JointAxis::LinX, model)
        .set_motor_max_force(JointAxis::LinX, Real::MAX)
        .set_motor(JointAxis::LinY, 0.0, 0.0, k, c)
        .set_motor_model(JointAxis::LinY, model)
        .set_motor_max_force(JointAxis::LinY, Real::MAX);
    world.impulse_joints.insert(a, b, joint, true);

    let mut out = Vec::new();
    for _ in 0..600 {
        world.step();
        out.push((world.bodies[b].translation() - world.bodies[a].translation()).length());
    }
    out
}

/// A force-based coupled soft pin converges to zero separation.
#[test]
fn coupled_linear_force_based_motor_converges() {
    let traj = separation_trajectory(MotorModel::ForceBased, 1.0, 50.0, 15.0);
    let last = *traj.last().unwrap();
    let late_max = traj[300..].iter().cloned().fold(0.0, Real::max);
    assert!(
        last < 1.0e-2 && late_max < 5.0e-2,
        "force-based soft pin should have converged; final {last}, late max {late_max}"
    );
}

/// An acceleration-based coupled soft pin converges too (the issue's basic setup works).
#[test]
fn coupled_linear_acceleration_based_motor_converges() {
    let traj = separation_trajectory(MotorModel::AccelerationBased, 1.0, 100.0, 30.0);
    let last = *traj.last().unwrap();
    let late_max = traj[300..].iter().cloned().fold(0.0, Real::max);
    assert!(
        last < 1.0e-2 && late_max < 5.0e-2,
        "acceleration-based soft pin should have converged; final {last}, late max {late_max}"
    );
}

/// The whole point of `AccelerationBased` is mass independence: the same coefficients
/// must produce the same trajectory whatever the body masses.
#[test]
fn coupled_linear_acceleration_based_motor_is_mass_invariant() {
    let light = separation_trajectory(MotorModel::AccelerationBased, 1.0, 100.0, 30.0);
    let heavy = separation_trajectory(MotorModel::AccelerationBased, 10.0, 100.0, 30.0);
    for (i, (l, h)) in light.iter().zip(&heavy).enumerate() {
        assert!(
            (l - h).abs() < 1.0e-3,
            "acceleration-based response must not depend on mass; step {i}: {l} vs {h}"
        );
    }
}
