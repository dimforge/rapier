//! Regression test for https://github.com/dimforge/rapier/issues/866
//!
//! `ImpulseJointSet::joints_between` only returned the first parallel edge between
//! two bodies, so with several joints attached to the same body pair the narrow
//! phase's `contacts_enabled` filtering only saw one of them (the last inserted
//! joint's flag won).

use rapier2d::prelude::*;

/// Builds two overlapping balls with two joints between them, one of which has
/// `contacts_enabled(false)`, and returns the world plus the collider handles.
/// `disabled_first` controls the joint insertion order.
fn build_world(disabled_first: bool) -> (PhysicsWorld, ColliderHandle, ColliderHandle) {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;

    let a = world.bodies.insert(RigidBodyBuilder::dynamic());
    let b = world.bodies.insert(RigidBodyBuilder::dynamic());
    let ca = world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(1.0), a, &mut world.bodies);
    let cb = world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(1.0), b, &mut world.bodies);

    let disabled = RevoluteJointBuilder::new().contacts_enabled(false);
    let enabled = RevoluteJointBuilder::new().contacts_enabled(true);

    if disabled_first {
        world.impulse_joints.insert(a, b, disabled, true);
        world.impulse_joints.insert(a, b, enabled, true);
    } else {
        world.impulse_joints.insert(a, b, enabled, true);
        world.impulse_joints.insert(a, b, disabled, true);
    }

    // `joints_between` must yield every parallel joint, in both query orders.
    assert_eq!(world.impulse_joints.joints_between(a, b).count(), 2);
    assert_eq!(world.impulse_joints.joints_between(b, a).count(), 2);

    (world, ca, cb)
}

fn has_active_contact(world: &PhysicsWorld, ca: ColliderHandle, cb: ColliderHandle) -> bool {
    world
        .narrow_phase
        .contact_pair(ca, cb)
        .is_some_and(|pair| pair.has_any_active_contact())
}

#[test]
fn parallel_joints_disable_contacts_regardless_of_insertion_order() {
    for disabled_first in [true, false] {
        let (mut world, ca, cb) = build_world(disabled_first);
        world.step();
        assert!(
            !has_active_contact(&world, ca, cb),
            "contacts must be disabled when ANY joint between the bodies disables them \
             (disabled_first: {disabled_first})"
        );
    }
}

/// Sanity check: with contacts enabled on every joint, the overlapping balls do collide,
/// proving the assertion above isn't vacuous.
#[test]
fn parallel_joints_with_contacts_enabled_still_collide() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;

    let a = world.bodies.insert(RigidBodyBuilder::dynamic());
    let b = world.bodies.insert(RigidBodyBuilder::dynamic());
    let ca = world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(1.0), a, &mut world.bodies);
    let cb = world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(1.0), b, &mut world.bodies);

    world.impulse_joints.insert(
        a,
        b,
        RevoluteJointBuilder::new().contacts_enabled(true),
        true,
    );
    world.impulse_joints.insert(
        a,
        b,
        RevoluteJointBuilder::new().contacts_enabled(true),
        true,
    );

    world.step();
    assert!(has_active_contact(&world, ca, cb));
}
