//! Regression test for #669: a cuboid sliding across the joints of a segmented polyline
//! floor used to snag or bounce on the internal vertices (box2d's "ghost collisions").
//!
//! `ColliderBuilder::oriented_polyline` clamps contact normals to the outward vertex
//! pseudo-normals, which removes the spurious impulses at segment joints.

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

/// A cuboid pushed at constant horizontal velocity across ~30 segment joints of a
/// flat oriented-polyline floor must neither snag (x stalls) nor bounce (y spikes).
#[test]
fn cuboid_slides_across_polyline_joints_without_snagging() {
    let mut h = Harness::new();

    // Flat floor chain along y = 0, one segment per unit from x = -2 to x = 38.
    // `ORIENTED` puts the solid on the right of each segment's direction, so the
    // vertices run right-to-left to make the outward side point +Y.
    let vertices: Vec<Vector> = (-2..=38)
        .rev()
        .map(|i| Vector::new(i as Real, 0.0))
        .collect();
    let floor = h.bodies.insert(RigidBodyBuilder::fixed());
    h.colliders.insert_with_parent(
        ColliderBuilder::oriented_polyline(vertices, None).friction(0.0),
        floor,
        &mut h.bodies,
    );

    // The character: a small dynamic cuboid, rotations locked, no friction.
    let character = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.21))
            .lock_rotations(),
    );
    h.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.2, 0.2)
            .friction(0.0)
            .restitution(0.0),
        character,
        &mut h.bodies,
    );

    // Let it settle onto the floor first.
    for _ in 0..60 {
        h.step();
    }
    let rest_y = h.bodies[character].translation().y;
    let start_x = h.bodies[character].translation().x;

    // Drive it right at a constant 2 m/s for 1000 steps (~33 m, ~30 joints).
    let speed = 2.0;
    let steps = 1000;
    let mut prev_x = start_x;
    for i in 0..steps {
        let vy = h.bodies[character].linvel().y;
        h.bodies[character].set_linvel(Vector::new(speed, vy), true);
        h.step();

        let pos = h.bodies[character].translation();
        assert!(
            pos.x >= prev_x - 1.0e-6,
            "x went backwards at step {i}: {} -> {}",
            prev_x,
            pos.x
        );
        assert!(
            (pos.y - rest_y).abs() < 0.05,
            "cuboid bounced at a segment joint at step {i}: y = {} (rest y = {rest_y})",
            pos.y
        );
        prev_x = pos.x;
    }

    // Overall advance must match the driven speed within 5% (no snagging).
    let expected = speed * h.params.dt * steps as Real;
    let traveled = prev_x - start_x;
    assert!(
        (traveled - expected).abs() < expected * 0.05,
        "cuboid snagged on the polyline joints: traveled {traveled}, expected {expected}"
    );
}
