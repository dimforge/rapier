//! Regression test for #217: with CCD enabled and a large `dt`, bodies used to "hitch" —
//! their velocity dropped to ~0 mid-air, well before reaching the obstacle.
//!
//! The 0.35 sweep-TOI CCD only clamps the pose at the time of impact and never touches the
//! velocity, so a fast body may only stop advancing at the obstacle's surface.

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
            // No gravity: a clean 1D path along +X.
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
}

/// A ccd-enabled ball launched at a thin wall it hits mid-step (x = 0, v = 20, dt = 0.25,
/// so 5 m per step) must advance at full speed until the impact, then rest against the wall
/// at x ~= 11.7 — never frozen mid-air short of it.
#[test]
fn ccd_large_dt_no_mid_air_hitch() {
    let mut h = Harness::new();
    h.params.dt = 0.25;

    // Thin fixed wall: near face at x = 12.2.
    let wall = h.bodies.insert(RigidBodyBuilder::fixed());
    h.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.05, 5.0, 5.0).translation(Vector::new(12.25, 0.0, 0.0)),
        wall,
        &mut h.bodies,
    );

    let radius = 0.5;
    let contact_x = 12.2 - radius; // 11.7: ball center when touching the wall.
    let speed = 20.0;
    let ball = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .linvel(Vector::new(speed, 0.0, 0.0))
            .ccd_enabled(true),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::ball(radius), ball, &mut h.bodies);

    let step_travel = speed * h.params.dt; // 5 m per step.
    let mut prev_x = 0.0;
    for i in 0..10 {
        h.step();
        let x = h.bodies[ball].translation().x;
        let vx = h.bodies[ball].linvel().x;

        if prev_x + step_travel < contact_x - 0.5 {
            // Far from the wall: full-speed advance, no premature stop (the
            // original hitch: velocity dropping to ~0 with the body mid-air).
            assert!(
                (x - (prev_x + step_travel)).abs() < 1.0e-3,
                "ball hitched mid-air at step {i}: x = {x} (expected {})",
                prev_x + step_travel
            );
        } else {
            // Impact step and after: the ball must be adjacent to the wall
            // surface (allowing the contact prediction gap), not frozen short
            // of it, and must never pass through.
            assert!(
                x > contact_x - 0.35 && x < contact_x + 0.01,
                "ball is not resting at the wall after impact at step {i}: \
                 x = {x} (contact at {contact_x})"
            );
            // Restitution is 0: after the contact is solved the ball must not
            // retain forward speed into the wall nor bounce back fast.
            if i >= 4 {
                assert!(
                    vx.abs() < 0.1,
                    "ball still moving after settling against the wall at step {i}: vx = {vx}"
                );
            }
        }
        prev_x = x;
    }
}
