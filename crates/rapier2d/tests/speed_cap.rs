//! Tests for the per-step velocity speed cap and the CCD
//! initial-contact tolerance (the spinner-arm anti-jitter fix).

use rapier2d::prelude::*;

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
}

impl Harness {
    fn new(gravity: Vector) -> Self {
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
            gravity,
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
            &(),
        );
    }

    fn run(&mut self, steps: usize) {
        for _ in 0..steps {
            self.step();
        }
    }
}

/// A body given an absurd linear velocity is clamped to `max_linear_velocity()`.
#[test]
fn linear_speed_cap() {
    let mut h = Harness::new(Vector::ZERO);
    let cap = h.params.max_linear_velocity();
    assert!(cap.is_finite(), "linear cap should be finite by default");

    let body = h
        .bodies
        .insert(RigidBodyBuilder::dynamic().linvel(Vector::new(10_000.0, 0.0)));
    h.colliders
        .insert_with_parent(ColliderBuilder::ball(0.2), body, &mut h.bodies);

    h.step();

    let speed = h.bodies[body].linvel().length();
    assert!(
        (speed - cap).abs() < 1.0,
        "linear velocity should be capped to {cap} (got {speed})"
    );
}

/// Disabling the cap (`normalized_max_linear_velocity = Real::MAX`) restores
/// uncapped motion.
#[test]
fn linear_cap_disabled() {
    let mut h = Harness::new(Vector::ZERO);
    h.params.normalized_max_linear_velocity = Real::MAX;

    let body = h
        .bodies
        .insert(RigidBodyBuilder::dynamic().linvel(Vector::new(10_000.0, 0.0)));
    h.colliders
        .insert_with_parent(ColliderBuilder::ball(0.2), body, &mut h.bodies);

    h.step();

    let speed = h.bodies[body].linvel().length();
    assert!(
        speed > 9_000.0,
        "linear velocity should be uncapped when disabled (got {speed})"
    );
}

/// A body given an absurd angular velocity rotates at most ~`MAX_ROTATION`/step
/// unless `allow_fast_rotation` is set.
#[test]
fn angular_speed_cap() {
    // max angular speed ≈ (π/4) * 60 ≈ 47.1 rad/s at the default 60 Hz step.
    let max_ang = core::f64::consts::FRAC_PI_4 as Real * IntegrationParameters::default().inv_dt();

    // Capped body.
    let mut h = Harness::new(Vector::ZERO);
    let capped = h.bodies.insert(RigidBodyBuilder::dynamic().angvel(500.0));
    h.colliders
        .insert_with_parent(ColliderBuilder::ball(0.2), capped, &mut h.bodies);
    h.step();
    let w = h.bodies[capped].angvel().abs();
    assert!(
        (w - max_ang).abs() < 2.0,
        "angular velocity should be capped to ~{max_ang} (got {w})"
    );

    // Bypassed body.
    let mut h = Harness::new(Vector::ZERO);
    let fast = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .angvel(500.0)
            .allow_fast_rotation(true),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::ball(0.2), fast, &mut h.bodies);
    h.step();
    let w = h.bodies[fast].angvel().abs();
    assert!(
        w > 400.0,
        "allow_fast_rotation should bypass the angular cap (got {w})"
    );
}

/// A fast body sliding tangentially while in contact with a fixed floor keeps
/// translating: CCD must skip a pair the discrete solver already owns, instead
/// of clamping the body's motion toward a standstill (the spinner-arm jitter).
#[test]
fn ccd_skips_in_contact_pair() {
    let mut h = Harness::new(Vector::new(0.0, -9.81));

    // Frictionless fixed floor.
    let floor = h.bodies.insert(RigidBodyBuilder::fixed());
    h.colliders.insert_with_parent(
        ColliderBuilder::cuboid(50.0, 0.05).friction(0.0),
        floor,
        &mut h.bodies,
    );

    // A ball resting on the floor (slight overlap → persistent contact),
    // sliding fast along +X. Fast enough to be CCD-active, under the linear cap.
    let ball = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-5.0, 0.24))
            .linvel(Vector::new(50.0, 0.0)),
    );
    h.colliders.insert_with_parent(
        ColliderBuilder::ball(0.2).friction(0.0),
        ball,
        &mut h.bodies,
    );

    h.run(30);

    // ~50 m/s over ~0.5 s ⇒ it should have slid far along +X, not been frozen.
    let x = h.bodies[ball].translation().x;
    assert!(
        x > 10.0,
        "in-contact body was clamped by CCD instead of sliding freely (x = {x})"
    );
    assert!(
        h.bodies[ball].is_ccd_active(),
        "the sliding body should be CCD-active (otherwise the test proves nothing)"
    );
}
