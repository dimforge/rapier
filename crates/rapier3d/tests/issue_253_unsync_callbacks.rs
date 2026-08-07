//! Regression test for #253: with the `unsync-callbacks` feature, an
//! `EventHandler` that is `!Sync` (e.g. contains `Rc<RefCell<..>>`) compiles
//! without a mutex and receives events.
#![cfg(feature = "unsync-callbacks")]

use rapier3d::prelude::*;
use std::cell::RefCell;
use std::rc::Rc;

// `Rc<RefCell<..>>` makes the handler neither `Sync` nor `Send`.
struct UnsyncEventCollector {
    events: Rc<RefCell<Vec<CollisionEvent>>>,
}

impl EventHandler for UnsyncEventCollector {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
        self.events.borrow_mut().push(event);
    }

    fn handle_contact_force_event(
        &self,
        _dt: Real,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        _contact_pair: &ContactPair,
        _total_force_magnitude: Real,
    ) {
    }
}

#[test]
fn unsync_event_handler_receives_events() {
    let events = Rc::new(RefCell::new(Vec::new()));
    let handler = UnsyncEventCollector {
        events: events.clone(),
    };

    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);

    let ground = world.insert_body(RigidBodyBuilder::fixed());
    world.insert_collider(
        ColliderBuilder::cuboid(10.0, 0.1, 10.0).active_events(ActiveEvents::COLLISION_EVENTS),
        Some(ground),
    );
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.0, 0.0)),
        ColliderBuilder::ball(0.5),
    );

    for _ in 0..120 {
        world.step_with_events(&(), &handler);
    }

    let events = events.borrow();
    assert!(
        events
            .iter()
            .any(|e| matches!(e, CollisionEvent::Started(..))),
        "the !Sync event handler received no collision-started event"
    );
}
