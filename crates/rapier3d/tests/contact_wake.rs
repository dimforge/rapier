//! A falling box that lands on a sleeping island wakes it in the middle of the step, after the
//! narrow-phase skipped the island's own contacts. Those contacts must still be solved that
//! step: they used to be left out, so the whole island fell freely for one step.

use rapier3d::prelude::*;

#[test]
fn island_woken_by_a_contact_keeps_its_contacts() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0),
    );
    // A row of touching boxes resting on the ground.
    let row: Vec<_> = (0..5)
        .map(|i| {
            world
                .insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(
                        i as Real * 1.001,
                        0.5,
                        0.0,
                    )),
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5),
                )
                .0
        })
        .collect();
    for _ in 0..300 {
        world.step();
    }
    assert!(row.iter().all(|h| world.bodies[*h].is_sleeping()));

    // Drop a box on the first one.
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 3.0, 0.0)),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    let far = row[4];
    while world.bodies[far].is_sleeping() {
        world.step();
    }
    // One step of free fall would be -9.81 / 60 = -0.16 m/s.
    let vy = world.bodies[far].linvel().y;
    assert!(vy > -0.01, "the far box fell at {vy} m/s");
}

#[test]
fn island_woken_outside_a_step_reports_its_contact_forces() {
    use std::sync::mpsc::channel;

    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0),
    );
    // Two stacked boxes that report contact forces, asleep.
    let reporting = || {
        ColliderBuilder::cuboid(0.5, 0.5, 0.5)
            .active_events(ActiveEvents::CONTACT_FORCE_EVENTS)
            .contact_force_event_threshold(0.0)
    };
    let (bottom, bottom_collider) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 0.0)),
        reporting(),
    );
    let (_, top_collider) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.5, 0.0)),
        reporting(),
    );
    let (ball, _) = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(20.0, 0.3, 0.0)),
        ColliderBuilder::ball(0.3),
    );
    for _ in 0..300 {
        world.step();
    }
    assert!(world.bodies[bottom].is_sleeping());

    // A ball moved onto the stack: the contact that wakes it is found by `detect_collisions`,
    // not by a step.
    let (collisions, _collisions) = channel();
    let (forces, received_forces) = channel();
    let (tears, _tears) = channel();
    let events = ChannelEventCollector::new(collisions, forces, tears);
    world.bodies[ball].set_translation(Vector::new(0.0, 2.3005, 0.0), true);
    world.detect_collisions(&(), &events);
    assert!(!world.bodies[bottom].is_sleeping());

    for _ in 0..3 {
        world.step_with_events(&(), &events);
        let between_the_boxes = received_forces
            .try_iter()
            .filter(|force| {
                [force.collider1, force.collider2] == [bottom_collider, top_collider]
                    || [force.collider1, force.collider2] == [top_collider, bottom_collider]
            })
            .count();
        assert_eq!(
            between_the_boxes, 1,
            "the boxes' contact force was not reported"
        );
    }
}
