//! Tiny scenes exercising the pipeline's unsafe hot paths (manifold store, solver
//! graph buckets, raw color-mask slices) under Miri's aliasing checks.
//!
//! Under Miri each step costs seconds, so scenes start in contact and run a
//! handful of steps; natively they run long enough to also assert behavior.
//! Run with: `cargo +nightly miri test -p rapier3d --test miri_scenes`.
//! On Apple Silicon add `--target x86_64-unknown-linux-gnu`: glam's aarch64 NEON
//! backend hits foreign intrinsics Miri does not implement, while its x86 SSE2
//! path is fully supported.

use rapier3d::prelude::*;

/// Steps per scene: enough to reach the solver's steady state paths (prepare,
/// warm-start, writeback, graph maintenance) under Miri; long enough natively
/// for the scene's behavioral assertion to be meaningful.
fn steps(native: usize) -> usize {
    if cfg!(miri) { 3 } else { native }
}

struct World {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    ccd: CCDSolver,
    pipeline: PhysicsPipeline,
    params: IntegrationParameters,
    gravity: Vector,
}

impl World {
    fn new() -> Self {
        Self {
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            ccd: CCDSolver::new(),
            pipeline: PhysicsPipeline::new(),
            params: IntegrationParameters::default(),
            gravity: Vector::Y * -9.81,
        }
    }

    fn step(&mut self) {
        self.pipeline.step(
            self.gravity,
            &self.params,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            &(),
            &(),
        );
    }

    fn run(&mut self, native_steps: usize) {
        for _ in 0..steps(native_steps) {
            self.step();
        }
        self.assert_all_finite();
    }

    fn assert_all_finite(&self) {
        for (_, rb) in self.bodies.iter() {
            let p = rb.translation();
            assert!(
                p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
                "non-finite body position: {p:?}"
            );
        }
    }

    fn floor(&mut self) -> RigidBodyHandle {
        let floor = self
            .bodies
            .insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)));
        self.colliders.insert_with_parent(
            ColliderBuilder::cuboid(10.0, 0.5, 10.0),
            floor,
            &mut self.bodies,
        );
        floor
    }
}

/// Resting contact + the contact-force-event pass (exact solver-active pair list).
#[test]
fn resting_ball_with_force_events() {
    let mut w = World::new();
    let floor = w
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)));
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(10.0, 0.5, 10.0)
            .active_events(ActiveEvents::CONTACT_FORCE_EVENTS)
            .contact_force_event_threshold(0.0),
        floor,
        &mut w.bodies,
    );

    let ball = w
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 0.0)));
    w.colliders
        .insert_with_parent(ColliderBuilder::ball(0.5), ball, &mut w.bodies);

    w.run(120);
    if !cfg!(miri) {
        let y = w.bodies[ball].translation().y;
        assert!((y - 0.5).abs() < 0.05, "ball not resting on floor: y = {y}");
    }
}

/// Multi-manifold stack: warm-starting, solver colors, manifold writeback.
#[test]
fn small_box_stack() {
    let mut w = World::new();
    w.floor();

    let mut tops = Vec::new();
    for i in 0..3 {
        let y = 0.5 + i as Real;
        let b = w
            .bodies
            .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, y, 0.0)));
        w.colliders
            .insert_with_parent(ColliderBuilder::cuboid(0.5, 0.5, 0.5), b, &mut w.bodies);
        tops.push(b);
    }

    w.run(120);
    if !cfg!(miri) {
        let y = w.bodies[tops[2]].translation().y;
        assert!((y - 2.5).abs() < 0.1, "stack collapsed: top y = {y}");
    }
}

/// Impulse-joint solver: a horizontal pendulum swinging on a revolute joint.
#[test]
fn revolute_pendulum() {
    let mut w = World::new();
    let anchor = w.bodies.insert(RigidBodyBuilder::fixed());
    let bob = w
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(1.0, 0.0, 0.0)));
    w.colliders
        .insert_with_parent(ColliderBuilder::ball(0.1), bob, &mut w.bodies);

    let joint = RevoluteJointBuilder::new(Vector::Z)
        .local_anchor1(Vector::new(0.0, 0.0, 0.0))
        .local_anchor2(Vector::new(-1.0, 0.0, 0.0));
    w.impulse_joints.insert(anchor, bob, joint, true);

    w.run(120);
    if !cfg!(miri) {
        let d = w.bodies[bob].translation().length();
        assert!((d - 1.0).abs() < 0.05, "pendulum arm stretched: |p| = {d}");
    }
}

/// Multibody-joint solver: one dynamic link articulated to a fixed base.
#[test]
fn multibody_link() {
    let mut w = World::new();
    let base = w.bodies.insert(RigidBodyBuilder::fixed());
    let link = w
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(1.0, 0.0, 0.0)));
    w.colliders
        .insert_with_parent(ColliderBuilder::ball(0.1), link, &mut w.bodies);

    let joint = RevoluteJointBuilder::new(Vector::Z)
        .local_anchor1(Vector::new(0.0, 0.0, 0.0))
        .local_anchor2(Vector::new(-1.0, 0.0, 0.0));
    w.multibody_joints.insert(base, link, joint, true);

    w.run(120);
    if !cfg!(miri) {
        let d = w.bodies[link].translation().length();
        assert!((d - 1.0).abs() < 0.05, "multibody arm stretched: |p| = {d}");
    }
}

/// Sensor overlap: the intersection graph, alongside the contact graph.
#[test]
fn sensor_overlap() {
    let mut w = World::new();
    let sensor_body = w.bodies.insert(RigidBodyBuilder::fixed());
    let sensor = w.colliders.insert_with_parent(
        ColliderBuilder::ball(0.5).sensor(true),
        sensor_body,
        &mut w.bodies,
    );

    let ball = w
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.4, 0.0)));
    let ball_co = w
        .colliders
        .insert_with_parent(ColliderBuilder::ball(0.5), ball, &mut w.bodies);

    w.step();
    assert_eq!(
        w.narrow_phase.intersection_pair(sensor, ball_co),
        Some(true),
        "overlapping sensor not detected"
    );

    w.run(120);
    if !cfg!(miri) {
        // No floor: the ball fell away from the sensor.
        assert_ne!(
            w.narrow_phase.intersection_pair(sensor, ball_co),
            Some(true)
        );
    }
}

/// CCD sweeps: a bullet ball must not tunnel through a thin floor.
#[test]
fn ccd_bullet_vs_thin_floor() {
    let mut w = World::new();
    let floor = w.bodies.insert(RigidBodyBuilder::fixed());
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(10.0, 0.05, 10.0),
        floor,
        &mut w.bodies,
    );

    let bullet = w.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 1.0, 0.0))
            .linvel(Vector::Y * -100.0)
            .ccd_enabled(true),
    );
    w.colliders
        .insert_with_parent(ColliderBuilder::ball(0.1), bullet, &mut w.bodies);

    w.run(30);
    let y = w.bodies[bullet].translation().y;
    assert!(y > 0.0, "bullet tunneled through the floor: y = {y}");
}

/// Mid-run body removal: pair removal, solver-graph and island maintenance.
#[test]
fn body_removal_midrun() {
    let mut w = World::new();
    w.floor();

    let a = w
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.5, 0.0)));
    w.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.5, 0.5, 0.5), a, &mut w.bodies);
    let b = w
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(1.0, 0.5, 0.0)));
    w.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.5, 0.5, 0.5), b, &mut w.bodies);

    w.run(2);
    w.bodies.remove(
        a,
        &mut w.islands,
        &mut w.colliders,
        &mut w.impulse_joints,
        &mut w.multibody_joints,
        true,
    );
    w.run(60);
    if !cfg!(miri) {
        let y = w.bodies[b].translation().y;
        assert!(
            (y - 0.5).abs() < 0.05,
            "surviving box sank or jumped: y = {y}"
        );
    }
}

/// Kinematic solver bodies: a dynamic box riding a velocity-based kinematic platform.
#[test]
fn kinematic_platform_carries_box() {
    let mut w = World::new();
    let platform = w
        .bodies
        .insert(RigidBodyBuilder::kinematic_velocity_based().linvel(Vector::Y * 0.5));
    w.colliders.insert_with_parent(
        ColliderBuilder::cuboid(1.0, 0.1, 1.0),
        platform,
        &mut w.bodies,
    );

    let rider = w
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.3, 0.0)));
    w.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.2, 0.2, 0.2), rider, &mut w.bodies);

    w.run(60);
    if !cfg!(miri) {
        let y = w.bodies[rider].translation().y;
        assert!(y > 0.6, "box fell off the rising platform: y = {y}");
    }
}
