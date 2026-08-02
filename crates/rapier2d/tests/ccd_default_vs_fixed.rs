//! Tests for the default CCD tier: fast dynamic bodies get continuous collision
//! detection against **fixed** colliders automatically, while
//! `ccd_enabled` upgrades a body to also sweep against kinematic/dynamic bodies.

use rapier2d::prelude::*;

/// Minimal world harness so each test reads as scenario + assertions.
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

    fn x(&self, h: RigidBodyHandle) -> Real {
        self.bodies[h].translation().x
    }
}

/// A thin fixed wall centered at the origin, spanning the Y axis.
fn insert_thin_fixed_wall(h: &mut Harness) {
    let wall = h.bodies.insert(RigidBodyBuilder::fixed());
    h.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.05, 5.0), wall, &mut h.bodies);
}

/// A small dynamic body far on the -X side, moving fast toward +X. In a single
/// `1/60`s step it moves ~3.3m, far more than the wall thickness — so without CCD
/// it tunnels straight through.
fn insert_fast_dynamic(h: &mut Harness, ccd_enabled: bool) -> RigidBodyHandle {
    let body = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-3.0, 0.0))
            .linvel(Vector::new(200.0, 0.0))
            .ccd_enabled(ccd_enabled),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.1, 0.1), body, &mut h.bodies);
    body
}

/// A default-tier fast dynamic body must NOT tunnel through a fixed wall,
/// even without `ccd_enabled`.
#[test]
fn default_ccd_vs_fixed_no_tunnel() {
    let mut h = Harness::new();
    insert_thin_fixed_wall(&mut h);
    let body = insert_fast_dynamic(&mut h, /* ccd_enabled */ false);

    h.run(120);

    // The body should have been stopped on the near (-X) side of the wall.
    assert!(
        h.x(body) < 0.0,
        "body tunneled through the fixed wall (x = {})",
        h.x(body)
    );
}

/// Two default-tier fast dynamic bodies on a head-on course DO pass through each
/// other: the default tier only sweeps fixed colliders, so there is no
/// moving-vs-moving CCD. This locks the fixed-only scope.
#[test]
fn default_tier_ignores_dynamic() {
    let mut h = Harness::new();

    let a = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-3.0, 0.0))
            .linvel(Vector::new(200.0, 0.0)),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.1, 0.1), a, &mut h.bodies);

    let b = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(3.0, 0.0))
            .linvel(Vector::new(-200.0, 0.0)),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.1, 0.1), b, &mut h.bodies);

    h.run(5);

    // They swapped sides — passed through each other untouched.
    assert!(
        h.x(a) > 0.0 && h.x(b) < 0.0,
        "default-tier dynamic bodies should tunnel through each other (a.x = {}, b.x = {})",
        h.x(a),
        h.x(b)
    );
}

/// A `ccd_enabled` ("bullet") body still collides with a dynamic body — the
/// upgrade tier is unaffected by the change.
#[test]
fn bullet_still_hits_dynamic() {
    let mut h = Harness::new();

    let bullet = insert_fast_dynamic(&mut h, /* ccd_enabled */ true);

    // A stationary dynamic target at the origin (not ccd_enabled).
    let target = h
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 0.0)));
    h.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.2, 0.2), target, &mut h.bodies);

    h.run(60);

    // The bullet hit the target and pushed it along +X (it did not tunnel).
    assert!(
        h.x(target) > 0.05,
        "bullet did not hit the dynamic target (target.x = {})",
        h.x(target)
    );
    assert!(
        h.x(bullet) < h.x(target),
        "bullet passed through its target (bullet.x = {}, target.x = {})",
        h.x(bullet),
        h.x(target)
    );
}

/// Setting `max_ccd_substeps = 0` disables CCD for the whole world: the fast
/// default-tier body tunnels through the fixed wall again.
#[test]
fn global_ccd_off_tunnels() {
    let mut h = Harness::new();
    h.params.max_ccd_substeps = 0;
    insert_thin_fixed_wall(&mut h);
    let body = insert_fast_dynamic(&mut h, /* ccd_enabled */ false);

    h.run(60);

    assert!(
        h.x(body) > 1.0,
        "body should tunnel through the wall when CCD is globally disabled (x = {})",
        h.x(body)
    );
}

/// Kinematic bodies are NOT default-tier targets: a default-tier fast body
/// tunnels through a (stationary) kinematic wall. Contrast with
/// `default_ccd_vs_fixed_no_tunnel`, where the same body is stopped by a fixed wall.
#[test]
fn kinematic_not_a_default_target() {
    let mut h = Harness::new();

    let wall = h
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based());
    h.colliders
        .insert_with_parent(ColliderBuilder::cuboid(0.05, 5.0), wall, &mut h.bodies);

    let body = insert_fast_dynamic(&mut h, /* ccd_enabled */ false);

    h.run(60);

    assert!(
        h.x(body) > 1.0,
        "default-tier body should tunnel through a kinematic wall (x = {})",
        h.x(body)
    );
}
