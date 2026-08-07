//! Regression test for https://github.com/dimforge/rapier/issues/974
//!
//! A ball dropped from 2m rebounded ~1% of the drop height whatever its restitution
//! (expected: the textbook `e²` fraction), because the speculative contact cleared the
//! contact's "newness" one step early and the impact step then skipped the restitution seed.

use rapier3d::prelude::*;

/// Drops a ball with restitution `e` from 2m and returns the fraction of the
/// drop height recovered at the first apex after impact.
fn rebound(e: f32) -> f32 {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();

    let ground = bodies.insert(RigidBodyBuilder::fixed());
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(30.0, 0.1, 30.0)
            .restitution(e)
            .restitution_combine_rule(CoefficientCombineRule::Average),
        ground,
        &mut bodies,
    );
    let ball = bodies.insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.3, 0.0)));
    colliders.insert_with_parent(
        ColliderBuilder::ball(0.2)
            .density(1.0)
            .restitution(e)
            .restitution_combine_rule(CoefficientCombineRule::Average),
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
    let params = IntegrationParameters::default();
    let gravity = Vector::new(0.0, -9.81, 0.0);

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

#[test]
fn restitution_rebound_matches_e_squared() {
    for e in [0.3f32, 0.5, 0.8, 0.95] {
        let measured = rebound(e);
        let expected = e * e;
        assert!(
            (measured - expected).abs() < 0.05,
            "restitution {e}: rebound fraction {measured:.3}, expected ~{expected:.3}"
        );
    }
}

#[test]
fn zero_restitution_does_not_bounce() {
    let measured = rebound(0.0);
    assert!(
        measured < 0.02,
        "restitution 0.0: rebound fraction {measured:.3}, expected ~0"
    );
}
