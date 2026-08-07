//! Regression test for https://github.com/dimforge/rapier/issues/718
//!
//! Contact reporting used to panic in the old SAP broad phase's `sort2` when a proxy was
//! paired with itself. That broad phase is gone; this pins that a manifold whose features
//! flip rapidly still reports events without panicking.

use rapier2d::prelude::*;
use std::sync::mpsc::channel;

#[test]
fn sliding_cuboid_contact_reporting_does_not_panic() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81);

    // A row of adjacent fixed cuboids: the slider's manifold feature ids flip
    // every time it crosses from one tile to the next.
    for i in 0..40 {
        world.insert_collider(
            ColliderBuilder::cuboid(0.5, 0.5)
                .translation(Vector::new(i as f32, 0.0))
                .active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS),
            None,
        );
    }

    let (_, _slider) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 1.01))
            .linvel(Vector::new(8.0, 0.0)),
        ColliderBuilder::cuboid(0.5, 0.5)
            .friction(0.0)
            .active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS),
    );

    let (collision_send, collision_recv) = channel();
    let (force_send, force_recv) = channel();
    let events = ChannelEventCollector::new(collision_send, force_send);

    let mut num_collision_events = 0;
    for _ in 0..300 {
        world.step_with_events(&(), &events);
        while collision_recv.try_recv().is_ok() {
            num_collision_events += 1;
        }
        while force_recv.try_recv().is_ok() {}
    }

    // The slider crossed many tiles: contact started/stopped events must have flowed.
    assert!(
        num_collision_events > 2,
        "expected contact events while sliding, got {num_collision_events}"
    );
}
