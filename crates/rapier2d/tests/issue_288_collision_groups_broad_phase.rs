//! Regression test for https://github.com/dimforge/rapier/issues/288
//!
//! The broad phase must filter candidate pairs by collision groups: co-located colliders
//! with `InteractionGroups::none()` used to materialize all n^2/2 pairs, making the step
//! quadratic. Runtime group changes must still re-discover the suppressed pairs.

use rapier2d::prelude::*;

/// Many overlapping group-none colliders must produce zero narrow-phase pairs.
#[test]
fn group_none_colliders_produce_no_pairs() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81);

    for _ in 0..300 {
        world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0)),
            ColliderBuilder::ball(0.5)
                .restitution(0.7)
                .collision_groups(InteractionGroups::none())
                .solver_groups(InteractionGroups::none()),
        );
    }

    for _ in 0..10 {
        world.step();
    }

    assert_eq!(
        world.narrow_phase.contact_pairs().count(),
        0,
        "group-filtered pairs must not reach the narrow phase"
    );
}

fn overlapping_grouped_world(
    groups1: InteractionGroups,
    groups2: InteractionGroups,
) -> (PhysicsWorld, ColliderHandle, ColliderHandle) {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, 0.0);

    let (_, c1) = world.insert(
        RigidBodyBuilder::dynamic(),
        ColliderBuilder::ball(1.0).collision_groups(groups1),
    );
    let c2 = world.insert_collider(
        ColliderBuilder::cuboid(2.0, 2.0)
            .translation(Vector::new(0.5, 0.5))
            .collision_groups(groups2),
        None,
    );
    (world, c1, c2)
}

fn has_active_contact(world: &PhysicsWorld, c1: ColliderHandle, c2: ColliderHandle) -> bool {
    world
        .contact_pair(c1, c2)
        .is_some_and(|pair| pair.has_any_active_contact())
}

/// Two overlapping colliders with disjoint groups must gain a contact when
/// their groups are made compatible at runtime, without being moved.
#[test]
fn group_change_rediscovers_pair_without_motion() {
    let (mut world, c1, c2) = overlapping_grouped_world(
        InteractionGroups::new(Group::GROUP_1, Group::GROUP_1, InteractionTestMode::And),
        InteractionGroups::new(Group::GROUP_2, Group::GROUP_2, InteractionTestMode::And),
    );

    for _ in 0..3 {
        world.step();
    }
    assert!(!has_active_contact(&world, c1, c2));
    assert_eq!(world.narrow_phase.contact_pairs().count(), 0);

    world
        .colliders
        .get_mut(c2)
        .unwrap()
        .set_collision_groups(InteractionGroups::new(
            Group::GROUP_1,
            Group::GROUP_1,
            InteractionTestMode::And,
        ));

    world.step();
    assert!(
        has_active_contact(&world, c1, c2),
        "contact not re-discovered after making collision groups compatible"
    );
}

/// Same as above, but the collider also moves on the frame its groups change.
#[test]
fn group_change_rediscovers_pair_with_motion() {
    let (mut world, c1, c2) = overlapping_grouped_world(
        InteractionGroups::new(Group::GROUP_1, Group::GROUP_1, InteractionTestMode::And),
        InteractionGroups::new(Group::GROUP_2, Group::GROUP_2, InteractionTestMode::And),
    );

    for _ in 0..3 {
        world.step();
    }
    assert!(!has_active_contact(&world, c1, c2));

    let co2 = world.colliders.get_mut(c2).unwrap();
    co2.set_collision_groups(InteractionGroups::new(
        Group::GROUP_1,
        Group::GROUP_1,
        InteractionTestMode::And,
    ));
    co2.set_translation(Vector::new(0.4, 0.4));

    world.step();
    assert!(
        has_active_contact(&world, c1, c2),
        "contact not re-discovered after group change + motion"
    );
}

/// The reverse transition: a touching pair whose groups become disjoint must
/// lose its contact on the next step.
#[test]
fn group_change_removes_existing_contact() {
    let (mut world, c1, c2) = overlapping_grouped_world(
        InteractionGroups::new(Group::GROUP_1, Group::GROUP_1, InteractionTestMode::And),
        InteractionGroups::new(Group::GROUP_1, Group::GROUP_1, InteractionTestMode::And),
    );

    for _ in 0..3 {
        world.step();
    }
    assert!(has_active_contact(&world, c1, c2));

    world
        .colliders
        .get_mut(c2)
        .unwrap()
        .set_collision_groups(InteractionGroups::new(
            Group::GROUP_2,
            Group::GROUP_2,
            InteractionTestMode::And,
        ));

    world.step();
    assert!(
        !has_active_contact(&world, c1, c2),
        "contact must disappear after collision groups become disjoint"
    );
}
