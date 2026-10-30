//! Regression test for https://github.com/dimforge/rapier/issues/1050
//!
//! `Collider::set_active_collision_types` did not set any change flag, so the
//! broad phase never re-tested the collider: widening the types after creation
//! (e.g. enabling kinematic-fixed contacts at runtime) never created the pair.

use rapier3d::prelude::*;

fn contact_found_after(widen_at: Option<usize>) -> bool {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let ground = bodies.insert(RigidBodyBuilder::fixed());
    let ground_collider =
        colliders.insert_with_parent(ColliderBuilder::cuboid(2.0, 0.5, 2.0), ground, &mut bodies);
    let ball = bodies.insert(RigidBodyBuilder::kinematic_position_based());
    let types = if widen_at.is_some() {
        ActiveCollisionTypes::default()
    } else {
        ActiveCollisionTypes::all()
    };
    let ball_collider = colliders.insert_with_parent(
        ColliderBuilder::ball(0.5).active_collision_types(types),
        ball,
        &mut bodies,
    );

    let mut pipeline = PhysicsPipeline::new();
    let params = IntegrationParameters::default();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut soft_bodies = SoftBodySet::new();
    let mut ccd = CCDSolver::new();

    for step in 0..10 {
        if Some(step) == widen_at {
            colliders[ball_collider].set_active_collision_types(ActiveCollisionTypes::all());
        }
        pipeline.step(
            Vector::new(0.0, -9.81, 0.0),
            &params,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut soft_bodies,
            &mut ccd,
            &(),
            &(),
        );
    }

    narrow_phase
        .contact_pair(ground_collider, ball_collider)
        .is_some_and(|pair| pair.has_any_active_contact())
}

#[test]
fn widening_active_collision_types_creates_pair() {
    // Sanity check: kinematic-fixed contacts work when enabled from the start.
    assert!(contact_found_after(None));
    // Widening the collision types after the pipeline ran must also create
    // the pair. Without a change flag on `set_active_collision_types` the
    // broad phase never re-tests the collider and this stays false.
    assert!(contact_found_after(Some(2)));
}
