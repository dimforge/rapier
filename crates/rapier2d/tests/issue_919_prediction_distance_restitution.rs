//! Regression test for https://github.com/dimforge/rapier/issues/919
//!
//! Raising `normalized_prediction_distance` used to kill restitution: the speculative
//! contact cleared the contact's "newness" before the impact step could bounce. Restitution
//! is now an end-of-step pass seeded at prepare time, whenever the contact was created.

use rapier2d::prelude::*;

/// Drops a ball with restitution `e` from 2m and returns the fraction of the
/// drop height recovered at the first apex after impact.
fn rebound(e: f32, prediction_distance: Option<f32>) -> f32 {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();

    let ground = bodies.insert(RigidBodyBuilder::fixed());
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(30.0, 0.1).restitution(e),
        ground,
        &mut bodies,
    );
    let ball = bodies.insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.3)));
    colliders.insert_with_parent(
        ColliderBuilder::ball(0.2).density(1.0).restitution(e),
        ball,
        &mut bodies,
    );

    let mut pipeline = PhysicsPipeline::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd = CCDSolver::new();
    let mut params = IntegrationParameters::default();
    if let Some(pred) = prediction_distance {
        params.normalized_prediction_distance = pred;
    }
    let gravity = Vector::new(0.0, -9.81);

    let mut touched = false;
    let mut apex = 0.0f32;
    for _ in 0..400 {
        pipeline.step(
            gravity,
            &params,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &(),
            &(),
        );
        let y = bodies[ball].translation().y;
        if !touched && y < 0.35 {
            touched = true;
        }
        if touched && y > apex {
            apex = y;
        }
    }
    (apex - 0.3) / 2.0 // fraction of the 2 m drop recovered
}

/// A large prediction distance must not change how much a ball bounces.
#[test]
fn large_prediction_distance_preserves_restitution() {
    for e in [0.5f32, 0.8] {
        let baseline = rebound(e, None);
        let with_large_prediction = rebound(e, Some(0.1));
        let expected = e * e;
        assert!(
            (baseline - expected).abs() < 0.05,
            "restitution {e} (default prediction): rebound {baseline:.3}, expected ~{expected:.3}"
        );
        assert!(
            (with_large_prediction - baseline).abs() < 0.05,
            "restitution {e}: rebound with prediction 0.1 is {with_large_prediction:.3}, \
             but {baseline:.3} with the default prediction distance"
        );
    }
}
