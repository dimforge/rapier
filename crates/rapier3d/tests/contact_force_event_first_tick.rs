//! `ContactForceEvent::started` must be `true` exactly on the steps where the pair's
//! total force crosses its threshold coming from below (or from separation), and `false`
//! while the force stays above on consecutive steps — the analogue of PhysX's
//! `eNOTIFY_THRESHOLD_FORCE_FOUND` vs `_PERSISTS`. In particular it is about the *force*
//! threshold, not contact newness: a pair can rest gently (no events at all) long after
//! its `CollisionEvent::Started`.

use std::sync::Mutex;

use rapier3d::pipeline::{ActiveEvents, EventHandler, PhysicsWorld};
use rapier3d::prelude::*;

#[derive(Default)]
struct Events {
    started_steps: Mutex<Vec<usize>>,
    force_events: Mutex<Vec<(usize, bool)>>,
    step: Mutex<usize>,
}

impl EventHandler for Events {
    fn handle_collision_event(
        &self,
        _: &RigidBodySet,
        _: &ColliderSet,
        event: CollisionEvent,
        _: Option<&ContactPair>,
    ) {
        if matches!(event, CollisionEvent::Started(..)) {
            self.started_steps
                .lock()
                .unwrap()
                .push(*self.step.lock().unwrap());
        }
    }

    fn handle_contact_force_event(
        &self,
        dt: Real,
        _: &RigidBodySet,
        _: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        let event = ContactForceEvent::from_contact_pair(dt, contact_pair, total_force_magnitude);
        self.force_events
            .lock()
            .unwrap()
            .push((*self.step.lock().unwrap(), event.started));
    }
}

#[test]
fn started_marks_threshold_crossings_not_contact_newness() {
    let events = Events::default();
    let mut world = PhysicsWorld::new();

    world.insert_collider(
        ColliderBuilder::cuboid(10.0, 0.5, 10.0).translation(Vector::new(0.0, -0.5, 0.0)),
        None,
    );

    // A 1 kg ball resting under standard gravity presses with ~9.81 N; the threshold sits
    // well above that, so resting alone emits no force event.
    let threshold = 30.0;
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.5, 0.0))
            .additional_mass(1.0)
            .can_sleep(false),
        ColliderBuilder::ball(0.5)
            .density(0.0)
            .active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS)
            .contact_force_event_threshold(threshold),
    );

    let mut press = false;
    let mut step_range = |world: &mut PhysicsWorld, range: core::ops::Range<usize>, p: bool| {
        press = p;
        for i in range {
            *events.step.lock().unwrap() = i;
            if press {
                world.bodies[ball].add_force(Vector::new(0.0, -100.0, 0.0), true);
            }
            world.step_with_events(&(), &events);
            world.bodies[ball].reset_forces(true);
        }
    };

    // Phase A (steps 0..100): settle and rest gently. Contact starts, no force events.
    step_range(&mut world, 0..100, false);
    let started = events.started_steps.lock().unwrap().clone();
    assert_eq!(started.len(), 1, "the ball should have touched down once");
    assert!(
        events.force_events.lock().unwrap().is_empty(),
        "resting below the threshold must not emit force events"
    );

    // Phase B (steps 100..160): press down; the crossing happens long after Started.
    step_range(&mut world, 100..160, true);
    {
        let evts = events.force_events.lock().unwrap();
        assert!(!evts.is_empty(), "pressing must emit force events");
        assert!(
            evts[0].1,
            "the first event of the episode must have started"
        );
        assert!(
            evts[0].0 >= 100 && evts[0].0 > started[0] + 50,
            "the crossing (step {}) must be decoupled from contact start (step {})",
            evts[0].0,
            started[0]
        );
        assert!(
            evts[1..].iter().all(|(_, first)| !first),
            "consecutive above-threshold steps must not have started"
        );
        assert!(evts.len() > 10, "the press lasts many steps");
    }

    // Phase C (steps 160..220): release. The pressed contact takes a step or two to
    // relax (its impulse is still high right after the release), then the forces sit
    // below the threshold and no further event fires.
    step_range(&mut world, 160..165, false);
    {
        let evts = events.force_events.lock().unwrap();
        assert!(
            evts.iter().skip(1).all(|(_, first)| !first),
            "relaxation events are continuations, never started"
        );
    }
    let evts_after_press = events.force_events.lock().unwrap().len();
    step_range(&mut world, 165..220, false);
    assert_eq!(
        events.force_events.lock().unwrap().len(),
        evts_after_press,
        "no force events while below the threshold"
    );

    // Phase D (steps 220..280): press again: a fresh episode, started fires again.
    step_range(&mut world, 220..280, true);
    {
        let evts = events.force_events.lock().unwrap();
        let episode2 = &evts[evts_after_press..];
        assert!(!episode2.is_empty());
        assert!(
            episode2[0].1,
            "a new crossing after dropping below the threshold must have started"
        );
        assert!(episode2[1..].iter().all(|(_, first)| !first));
    }

    // Phase E: separate entirely, then land hard: started fires again.
    let evts_before = events.force_events.lock().unwrap().len();
    world.bodies[ball].set_linvel(Vector::new(0.0, 8.0, 0.0), true);
    step_range(&mut world, 280..500, false);
    {
        let evts = events.force_events.lock().unwrap();
        let episode3 = &evts[evts_before..];
        assert!(
            !episode3.is_empty(),
            "the landing impact must exceed the threshold"
        );
        assert!(
            episode3[0].1,
            "the first event after separating must have started"
        );
    }
}
