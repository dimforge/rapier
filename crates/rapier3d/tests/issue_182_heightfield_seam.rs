//! Regression test for #182: a capsule crossing the seam between two flush
//! heightfield colliders used to sink into the ground right after the crossing
//! (chunked terrains: each chunk its own heightfield, edges flush).

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

/// Two flush 16x16 flat heightfield chunks (one centered at x = 0, one at
/// x = 16, seam at x = 8) and a rotation-locked capsule "character" driven across
/// the seam at constant speed. After settling, its height must never dip more
/// than 5 cm below the resting height — before the fix it sank into the second
/// chunk right after crossing.
#[test]
fn capsule_crosses_flush_heightfield_seam_without_sinking() {
    let mut h = Harness::new();

    // Each chunk: a flat 16x16 heightfield spanning [-8, 8] in x/z.
    let heights = Array2::zeros(17, 17);
    let scale = Vector::new(16.0, 1.0, 16.0);
    h.colliders
        .insert(ColliderBuilder::heightfield(heights.clone(), scale));
    h.colliders.insert(
        ColliderBuilder::heightfield(heights, scale).translation(Vector::new(16.0, 0.0, 0.0)),
    );

    // The character: a dynamic capsule with locked rotations.
    let character = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.7, 0.0))
            .lock_rotations(),
    );
    h.colliders.insert_with_parent(
        ColliderBuilder::capsule_y(0.3, 0.3).friction(0.0),
        character,
        &mut h.bodies,
    );

    // Settle on the first chunk and record the resting height.
    for _ in 0..120 {
        h.step();
    }
    let rest_y = h.bodies[character].translation().y;
    assert!(
        (rest_y - 0.6).abs() < 0.1,
        "capsule did not settle on the heightfield (y = {rest_y})"
    );

    // Drive it from x = 0, across the seam at x = 8, to x ~ 16 (the center of
    // the second chunk): 2 m/s for 480 steps.
    for i in 0..480 {
        let vy = h.bodies[character].linvel().y;
        h.bodies[character].set_linvel(Vector::new(2.0, vy, 0.0), true);
        h.step();

        let pos = h.bodies[character].translation();
        assert!(
            pos.y > rest_y - 0.05,
            "capsule sank at the heightfield seam at step {i}: x = {}, y = {} \
             (rest y = {rest_y})",
            pos.x,
            pos.y
        );
    }

    // It crossed the seam and is still at its resting height on the second chunk.
    let end = h.bodies[character].translation();
    assert!(
        end.x > 12.0,
        "capsule never crossed the seam (x = {})",
        end.x
    );
    assert!(
        (end.y - rest_y).abs() < 0.05,
        "capsule is not at its resting height after crossing (y = {}, rest y = {rest_y})",
        end.y
    );
}
