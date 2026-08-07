//! Regression test for #629: a very large fixed cuboid ground used to destabilize the
//! simulation — objects colliding with it gained energy or fell through.
//!
//! Verified up to half-extents of 1e6 (the issue's failing size); 1e8 is an inherent f32
//! precision limit, so use the f64 build for worlds that large.

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
            gravity: Vector::new(0.0, -9.81),
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

/// Drops a small 3-box stack at the origin onto a ground cuboid of the given
/// half-width and checks that after 500 steps the stack is coherent and at rest:
/// no explosion, no tunneling, bounded velocities.
fn check_stack_on_ground(ground_half_width: Real) {
    let mut h = Harness::new();

    // Huge fixed ground with its top face at y = 0.
    let ground = h
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -10.0)));
    h.colliders.insert_with_parent(
        ColliderBuilder::cuboid(ground_half_width, 10.0),
        ground,
        &mut h.bodies,
    );

    // A small stack of unit boxes at the origin, starting slightly separated.
    let half = 0.5;
    let mut boxes = Vec::new();
    for i in 0..3 {
        let y = half + i as Real * (2.0 * half + 0.01);
        let handle = h
            .bodies
            .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, y)));
        h.colliders
            .insert_with_parent(ColliderBuilder::cuboid(half, half), handle, &mut h.bodies);
        boxes.push(handle);
    }

    for _ in 0..500 {
        h.step();

        // No explosion at any point: positions finite and near the origin,
        // velocities bounded by what the initial drop can produce.
        for (i, &handle) in boxes.iter().enumerate() {
            let pos = h.bodies[handle].translation();
            let vel = h.bodies[handle].linvel();
            assert!(
                pos.x.is_finite() && pos.y.is_finite(),
                "box {i} exploded on ground of half-width {ground_half_width}: {pos:?}"
            );
            assert!(
                pos.x.abs() < 2.0 && pos.y > 0.3 && pos.y < 4.0,
                "box {i} left the stack on ground of half-width {ground_half_width}: {pos:?}"
            );
            assert!(
                vel.length() < 10.0,
                "box {i} gained spurious energy on ground of half-width {ground_half_width}: {vel:?}"
            );
        }
    }

    // At the end, the stack is at rest, in order, one box height apart.
    for (i, &handle) in boxes.iter().enumerate() {
        let pos = h.bodies[handle].translation();
        let vel = h.bodies[handle].linvel();
        let expected_y = half + i as Real * 2.0 * half;
        assert!(
            (pos.y - expected_y).abs() < 0.1,
            "box {i} not resting at its stack height on ground of half-width \
             {ground_half_width}: y = {} (expected ~{expected_y})",
            pos.y
        );
        assert!(
            vel.length() < 0.1,
            "box {i} still moving after 500 steps on ground of half-width \
             {ground_half_width}: {vel:?}"
        );
    }
}

/// Tame baseline: a 1e4 half-width ground.
#[test]
fn stack_rests_on_1e4_ground() {
    check_stack_on_ground(1.0e4);
}

/// The issue's failing size: a 1e6 half-width ground.
#[test]
fn stack_rests_on_1e6_ground() {
    check_stack_on_ground(1.0e6);
}
