//! Regression test for https://github.com/dimforge/rapier/issues/662
//!
//! `CollisionPipeline::step` never cleared the bodies' `RigidBodyChanges`, so after the
//! first step user changes stopped propagating to colliders and the broad phase.

use rapier2d::prelude::*;

#[test]
fn body_changes_keep_propagating_across_steps() {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut pipeline = CollisionPipeline::new();
    let prediction = IntegrationParameters::default().prediction_distance();

    let body = bodies.insert(RigidBodyBuilder::kinematic_position_based());
    let co = colliders.insert_with_parent(ColliderBuilder::ball(0.5), body, &mut bodies);

    let mut step = |bodies: &mut RigidBodySet, colliders: &mut ColliderSet| {
        pipeline.step(
            prediction,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            bodies,
            colliders,
            &(),
            &(),
        );
    };

    step(&mut bodies, &mut colliders);

    // Move the body once: the collider must follow.
    bodies
        .get_mut(body)
        .unwrap()
        .set_translation(Vector::new(1.0, 0.0), false);
    step(&mut bodies, &mut colliders);
    assert_eq!(colliders[co].translation().x, 1.0);

    // Move it a second time: before the fix the body's MODIFIED flag was never
    // cleared, so this change was silently ignored and the collider stayed at x=1.
    bodies
        .get_mut(body)
        .unwrap()
        .set_translation(Vector::new(2.0, 0.0), false);
    step(&mut bodies, &mut colliders);
    assert_eq!(colliders[co].translation().x, 2.0);
}
