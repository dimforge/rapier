//! Regression test for #932: with `max_ccd_substeps = 1`, a fast ccd-enabled body used to
//! stutter, because the old CCD froze it at each time-of-impact for the rest of the step.
//!
//! The 0.35 box2d-style CCD clamps the pose at the TOI but keeps the velocity, so a ball
//! skimming a tiled floor keeps making full forward progress.

use rapier3d::prelude::*;

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
            gravity: Vector::new(0.0, -9.81, 0.0),
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
}

/// A fast ccd-enabled ball sliding along a floor made of many separate fixed
/// cuboid tiles, with `max_ccd_substeps = 1`: every step must make at least 20%
/// of the expected forward progress (the stutter froze the body at the first
/// tile-seam TOI, killing nearly the whole step's motion).
#[test]
fn low_ccd_substeps_no_stutter_over_tiles() {
    let mut h = Harness::new();
    h.params.max_ccd_substeps = 1;

    // 40 one-unit-wide fixed tiles, tops at y = 0, spanning x in [0, 40].
    for i in 0..40 {
        h.colliders.insert(
            ColliderBuilder::cuboid(0.5, 0.25, 2.0)
                .translation(Vector::new(i as Real + 0.5, -0.25, 0.0))
                .friction(0.0),
        );
    }

    // A ball resting on the first tile, kicked to 30 m/s (0.5 m per step at
    // dt = 1/60, crossing a tile seam every other step).
    let ball = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.5, 0.2, 0.0))
            .linvel(Vector::new(30.0, 0.0, 0.0))
            .ccd_enabled(true),
    );
    h.colliders.insert_with_parent(
        ColliderBuilder::ball(0.2).friction(0.0),
        ball,
        &mut h.bodies,
    );

    let expected_advance = 30.0 * h.params.dt; // 0.5 m per step.
    let mut prev_x = 0.5;
    for i in 0..90 {
        h.step();
        let x = h.bodies[ball].translation().x;
        let advance = x - prev_x;
        assert!(advance > 0.0, "ball stopped advancing at step {i}: x = {x}");
        assert!(
            advance >= expected_advance * 0.2,
            "ball stuttered at step {i}: advanced {advance} (expected ~{expected_advance})"
        );
        prev_x = x;

        if x > 38.0 {
            return; // Crossed the whole tiled floor at speed.
        }
    }
    panic!(
        "ball never crossed the tiled floor (final x = {prev_x}); it must have \
         been losing most of its per-step motion"
    );
}
