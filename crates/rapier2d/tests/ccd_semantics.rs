//! Regression tests locking the CCD semantics:
//! - a time-of-impact clamp changes the pose only, never the velocity;
//! - bullets never sweep against other bullets;
//! - sensor crossings are never missed, both when a fast solid body tunnels through a
//!   sensor and when the fast body's own sensor collider tunnels through geometry;
//! - `max_ccd_substeps > 1` still prevents tunneling (the substep splitter path).

use rapier2d::prelude::*;
use std::sync::Mutex;

#[derive(Default)]
struct EventCollector {
    events: Mutex<Vec<CollisionEvent>>,
}

impl EventHandler for EventCollector {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
        self.events.lock().unwrap().push(event);
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

struct Harness {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    pipeline: PhysicsPipeline,
    bf: BroadPhaseBvh,
    nf: NarrowPhase,
    islands: IslandManager,
    ccd: CCDSolver,
    params: IntegrationParameters,
    gravity: Vector,
    events: EventCollector,
}

impl Harness {
    fn new() -> Self {
        Self {
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            pipeline: PhysicsPipeline::new(),
            bf: BroadPhaseBvh::new(),
            nf: NarrowPhase::new(),
            islands: IslandManager::new(),
            ccd: CCDSolver::new(),
            params: IntegrationParameters::default(),
            // No gravity: keep the fast bodies on a clean 1D path along +X.
            gravity: Vector::ZERO,
            events: EventCollector::default(),
        }
    }

    fn step(&mut self) {
        self.pipeline.step(
            self.gravity,
            &self.params,
            &mut self.islands,
            &mut self.bf,
            &mut self.nf,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            &(),
            &self.events,
        );
    }

    fn run(&mut self, steps: usize) {
        for _ in 0..steps {
            self.step();
        }
    }

    /// A thin fixed wall at x = 0.
    fn add_fixed_wall(&mut self) {
        self.colliders
            .insert(ColliderBuilder::cuboid(0.1, 10.0).build());
    }

    /// A fast ball flying toward +X from x = -3 at 400 m/s.
    fn add_fast_ball(&mut self, ccd_enabled: bool) -> RigidBodyHandle {
        let body = self.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(-3.0, 0.0))
                .linvel(Vector::new(400.0, 0.0))
                .ccd_enabled(ccd_enabled)
                .build(),
        );
        self.colliders.insert_with_parent(
            ColliderBuilder::ball(0.2).build(),
            body,
            &mut self.bodies,
        );
        body
    }
}

/// A TOI clamp must change the pose only: the velocity right after the clamping step is
/// exactly the pre-impact velocity (the continuous stage never touches velocities;
/// the contact is resolved by the regular solver on the *next* step).
#[test]
fn clamp_preserves_velocity() {
    let mut h = Harness::new();
    h.add_fixed_wall();
    let ball = h.add_fast_ball(false);

    h.step();

    let rb = &h.bodies[ball];
    // The ball was clamped in front of the wall instead of tunneling…
    assert!(
        rb.translation().x < 0.0,
        "ball should be clamped before the wall, got x = {}",
        rb.translation().x
    );
    assert!(rb.translation().x > -1.0, "ball should have moved forward");
    // …but its velocity is untouched.
    assert!(
        (rb.linvel().x - 400.0).abs() < 1.0e-3,
        "velocity must be preserved by the clamp, got {}",
        rb.linvel().x
    );
}

/// Bullets never sweep against other bullets: two bullets flying through each
/// other with no other obstacle pass through.
#[test]
fn bullet_ignores_other_bullet() {
    let mut h = Harness::new();

    let left = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-3.0, 0.0))
            .linvel(Vector::new(400.0, 0.0))
            .ccd_enabled(true)
            .build(),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::ball(0.2).build(), left, &mut h.bodies);

    let right = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(3.0, 0.0))
            .linvel(Vector::new(-400.0, 0.0))
            .ccd_enabled(true)
            .build(),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::ball(0.2).build(), right, &mut h.bodies);

    h.run(3);

    assert!(
        h.bodies[left].translation().x > 3.0,
        "left bullet should pass through the other bullet, got x = {}",
        h.bodies[left].translation().x
    );
    assert!(
        h.bodies[right].translation().x < -3.0,
        "right bullet should pass through the other bullet, got x = {}",
        h.bodies[right].translation().x
    );
}

/// A fast solid body crossing a thin fixed sensor in a single step must still produce the
/// intersection events the narrow phase would otherwise never observe.
#[test]
fn fast_body_through_sensor_emits_events() {
    let mut h = Harness::new();
    h.colliders.insert(
        ColliderBuilder::cuboid(0.1, 10.0)
            .sensor(true)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );
    let _ball = h.add_fast_ball(false);

    h.run(3);

    let events = h.events.events.lock().unwrap();
    let started = events
        .iter()
        .any(|e| matches!(e, CollisionEvent::Started(_, _, f) if f.contains(CollisionEventFlags::SENSOR)));
    let stopped = events
        .iter()
        .any(|e| matches!(e, CollisionEvent::Stopped(_, _, f) if f.contains(CollisionEventFlags::SENSOR)));
    assert!(
        started && stopped,
        "expected paired sensor events for the tunneled sensor, got {:?}",
        *events
    );
}

/// A fast body whose *own* collider is a sensor must also report crossings
/// (sensor shapes are swept for event detection even though they never clamp).
#[test]
fn fast_sensor_origin_emits_events() {
    let mut h = Harness::new();
    // Thin fixed *solid* wall the flying sensor crosses.
    h.colliders.insert(
        ColliderBuilder::cuboid(0.1, 10.0)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );

    let body = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-3.0, 0.0))
            .linvel(Vector::new(400.0, 0.0))
            .additional_mass(1.0)
            .build(),
    );
    h.colliders.insert_with_parent(
        ColliderBuilder::ball(0.2)
            .sensor(true)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
        body,
        &mut h.bodies,
    );

    h.run(3);

    assert!(
        h.bodies[body].translation().x > 3.0,
        "the sensor body must fly through the wall unclamped, got x = {}",
        h.bodies[body].translation().x
    );

    let events = h.events.events.lock().unwrap();
    let started = events
        .iter()
        .any(|e| matches!(e, CollisionEvent::Started(_, _, f) if f.contains(CollisionEventFlags::SENSOR)));
    let stopped = events
        .iter()
        .any(|e| matches!(e, CollisionEvent::Stopped(_, _, f) if f.contains(CollisionEventFlags::SENSOR)));
    assert!(
        started && stopped,
        "expected paired sensor events from the fast sensor origin, got {:?}",
        *events
    );
}

/// Multiple CCD substeps (`max_ccd_substeps > 1`, the substep splitter path) still
/// stop a fast body at a fixed wall.
#[test]
fn multi_ccd_substeps_no_tunnel() {
    let mut h = Harness::new();
    h.params.max_ccd_substeps = 4;
    h.add_fixed_wall();
    let ball = h.add_fast_ball(false);

    h.run(10);

    let x = h.bodies[ball].translation().x;
    assert!(x < 0.2, "ball must not tunnel with 4 CCD substeps, x = {x}");
}
