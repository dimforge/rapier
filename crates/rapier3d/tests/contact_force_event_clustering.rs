//! `ContactForceEvent` must report the forces the solver actually applied.
//!
//! With contact clustering (on by default in 3D for pairs with several manifolds) the
//! solver writes its impulses back into `pair.solver_clusters`, so an event built from
//! `pair.manifolds` reports a zero `total_force` and a degenerate `max_force_*`.

use std::sync::Mutex;

use rapier3d::pipeline::{ActiveEvents, EventHandler, PhysicsWorld};
use rapier3d::prelude::*;

#[derive(Default)]
struct ForceEvents(Mutex<Vec<ContactForceEvent>>);

impl EventHandler for ForceEvents {
    fn handle_collision_event(
        &self,
        _: &RigidBodySet,
        _: &ColliderSet,
        _: CollisionEvent,
        _: Option<&ContactPair>,
    ) {
    }

    fn handle_contact_force_event(
        &self,
        dt: Real,
        _: &RigidBodySet,
        _: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        self.0
            .lock()
            .unwrap()
            .push(ContactForceEvent::from_contact_pair(
                dt,
                contact_pair,
                total_force_magnitude,
            ));
    }
}

#[test]
fn contact_force_event_is_populated_under_contact_clustering() {
    let events = ForceEvents::default();
    let mut world = PhysicsWorld::new();

    // A two-triangle quad: a box resting across the diagonal touches both triangles,
    // so the pair carries two manifolds and clustering applies to it.
    let vertices = vec![
        Vector::new(-2.0, 0.0, -2.0),
        Vector::new(2.0, 0.0, -2.0),
        Vector::new(2.0, 0.0, 2.0),
        Vector::new(-2.0, 0.0, 2.0),
    ];
    let indices = vec![[0, 1, 2], [0, 2, 3]];
    let (_, ground_collider) = world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::trimesh(vertices, indices).unwrap(),
    );

    let (_, box_collider) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.55, 0.0))
            .can_sleep(false),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5)
            .active_events(ActiveEvents::CONTACT_FORCE_EVENTS)
            .contact_force_event_threshold(0.0),
    );

    for _ in 0..60 {
        world.step_with_events(&(), &events);
    }

    // The scene must actually exercise clustering, otherwise it proves nothing.
    let pair = world
        .narrow_phase
        .contact_pair(ground_collider, box_collider)
        .expect("no contact pair between the box and the ground");
    assert!(
        pair.manifolds.len() > 1,
        "expected several manifolds on the trimesh pair, got {}",
        pair.manifolds.len()
    );
    assert!(
        !pair.solver_clusters.is_empty(),
        "contact clustering did not apply to the pair"
    );

    let events = events.0.lock().unwrap();
    let event = events.last().expect("no contact force event was emitted");

    // The box rests under gravity: the support force is ~m*g along the up axis.
    assert!(
        event.total_force_magnitude > 0.0,
        "degenerate total_force_magnitude: {}",
        event.total_force_magnitude
    );
    assert!(
        event.max_force_magnitude > 0.0,
        "degenerate max_force_magnitude: {} (total was {})",
        event.max_force_magnitude,
        event.total_force_magnitude
    );
    assert!(
        (event.max_force_direction.length() - 1.0).abs() < 1.0e-4,
        "max_force_direction is not a unit vector: {:?}",
        event.max_force_direction
    );
    assert!(
        event.total_force.length() > 0.0,
        "degenerate total_force: {:?}",
        event.total_force
    );

    // The strongest single contact force cannot exceed the sum over all contacts.
    assert!(
        event.max_force_magnitude <= event.total_force_magnitude * 1.001,
        "max_force_magnitude {} exceeds total_force_magnitude {}",
        event.max_force_magnitude,
        event.total_force_magnitude
    );
}
