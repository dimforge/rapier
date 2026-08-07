//! Regression test for https://github.com/dimforge/rapier/issues/734
//!
//! The narrow phase's "same parent body" filter used to run only when a pair was added, so
//! reparenting a collider left stale state in both directions. It now runs on every pair
//! update; this pins both transitions.

use rapier3d::prelude::*;

fn has_active_contact(world: &PhysicsWorld, c1: ColliderHandle, c2: ColliderHandle) -> bool {
    world
        .contact_pair(c1, c2)
        .is_some_and(|pair| pair.has_any_active_contact())
}

#[test]
fn reparenting_updates_narrow_phase_state() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, 0.0, 0.0);

    // Two overlapping colliders on two separate bodies at the origin. The
    // dynamic body's collider opts out of solver forces (solver_groups none) so
    // the penetration is never resolved and the overlap persists across all
    // transitions below.
    let body1 = world.insert_body(RigidBodyBuilder::dynamic());
    let c1 = world.insert_collider(
        ColliderBuilder::ball(1.0).solver_groups(InteractionGroups::none()),
        Some(body1),
    );
    let body2 = world.insert_body(RigidBodyBuilder::fixed());
    let c2 = world.insert_collider(
        ColliderBuilder::ball(1.0).translation(Vector::new(0.5, 0.0, 0.0)),
        Some(body2),
    );

    for _ in 0..3 {
        world.step();
    }
    assert!(
        has_active_contact(&world, c1, c2),
        "overlapping colliders on different bodies must collide"
    );

    // Move c2 onto the same body as c1: their contact must disappear even
    // though both colliders still overlap and never moved.
    world
        .colliders
        .set_parent(c2, Some(body1), &mut world.bodies);
    world.step();
    assert!(
        !has_active_contact(&world, c1, c2),
        "same-parent colliders must not collide after reparenting"
    );
    for _ in 0..3 {
        world.step();
    }
    assert!(!has_active_contact(&world, c1, c2));

    // Split them apart again: the contact must come back.
    world
        .colliders
        .set_parent(c2, Some(body2), &mut world.bodies);
    for _ in 0..2 {
        world.step();
    }
    assert!(
        has_active_contact(&world, c1, c2),
        "contact not re-discovered after reparenting back onto separate bodies"
    );
}
