//! The broad phase no longer creates pairs that the narrow phase's
//! `ActiveCollisionTypes` filter would drop every frame (e.g. fixed-vs-fixed):
//! big static environments would otherwise flood the contact graph with
//! millions of dead edges. These tests pin the filtering itself and the
//! re-discovery paths for colliders whose filter inputs change afterwards.

#![cfg(feature = "dim3")]

use rapier3d::prelude::*;

fn overlapping_world() -> (PhysicsWorld, ColliderHandle, ColliderHandle) {
    // Two overlapping parentless (thus fixed) colliders.
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    let c1 = world.insert_collider(ColliderBuilder::cuboid(1.0, 1.0, 1.0), None);
    let c2 = world.insert_collider(
        ColliderBuilder::cuboid(1.0, 1.0, 1.0).translation(Vector::new(0.5, 0.5, 0.0)),
        None,
    );
    (world, c1, c2)
}

#[test]
fn no_fixed_fixed_pairs() {
    let (mut world, _, _) = overlapping_world();
    for _ in 0..3 {
        world.step();
    }
    assert_eq!(world.narrow_phase.contact_pairs().count(), 0);
}

#[test]
fn fixed_fixed_pair_kept_when_opted_in() {
    // A collider opting into FIXED_FIXED collision types keeps its pairs.
    let mut world = PhysicsWorld::new();
    let _ = world.insert_collider(
        ColliderBuilder::cuboid(1.0, 1.0, 1.0).active_collision_types(ActiveCollisionTypes::all()),
        None,
    );
    let _ = world.insert_collider(
        ColliderBuilder::cuboid(1.0, 1.0, 1.0).translation(Vector::new(0.5, 0.5, 0.0)),
        None,
    );
    for _ in 0..3 {
        world.step();
    }
    assert_eq!(world.narrow_phase.contact_pairs().count(), 1);
}

/// A body whose type changes from fixed to dynamic must re-discover the pairs
/// that were suppressed while it was fixed.
#[test]
fn body_type_change_rediscovers_pairs() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);

    // Static ground, and a FIXED body resting slightly above it (AABBs overlap).
    let _ground = world.insert_collider(ColliderBuilder::cuboid(5.0, 0.5, 5.0), None);
    let (body, _) = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, 1.01, 0.0)),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );

    for _ in 0..3 {
        world.step();
    }
    // Fixed vs fixed: no pair.
    assert_eq!(world.narrow_phase.contact_pairs().count(), 0);

    world
        .bodies
        .get_mut(body)
        .unwrap()
        .set_body_type(RigidBodyType::Dynamic, true);

    for _ in 0..10 {
        world.step();
    }
    assert_eq!(
        world.narrow_phase.contact_pairs().count(),
        1,
        "pair not re-discovered after fixed -> dynamic switch"
    );

    // And the contact must actually resolve: the box stays on the ground.
    for _ in 0..100 {
        world.step();
    }
    let y = world.bodies.get(body).unwrap().translation().y;
    assert!(
        (y - 1.0).abs() < 0.05,
        "box did not rest on the ground: y = {y}"
    );
}

/// A parentless collider attached to a dynamic body afterwards must re-discover
/// its suppressed pairs.
#[test]
fn set_parent_rediscovers_pairs() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);

    let _ground = world.insert_collider(ColliderBuilder::cuboid(5.0, 0.5, 5.0), None);
    let orphan = world.insert_collider(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5).translation(Vector::new(0.0, 1.01, 0.0)),
        None,
    );

    for _ in 0..3 {
        world.step();
    }
    assert_eq!(world.narrow_phase.contact_pairs().count(), 0);

    let body = world
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.01, 0.0)));
    let bodies = &mut world.bodies;
    world.colliders.set_parent(orphan, Some(body), bodies);

    for _ in 0..10 {
        world.step();
    }
    assert_eq!(
        world.narrow_phase.contact_pairs().count(),
        1,
        "pair not re-discovered after attaching the collider to a dynamic body"
    );
}
