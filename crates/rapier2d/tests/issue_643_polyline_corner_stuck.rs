//! Regression test for #643: a cuboid character used to snag on the corner vertices of a
//! closed square polyline (the tilemap case), where the double-sided polyline yields a
//! normal that pushes it sideways.
//!
//! `ColliderBuilder::oriented_polyline` clamps contact normals to the outward
//! pseudo-normals, so the character slides across the corner and off the edge.

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

/// Port of the issue's scenario: a 2x2 closed square polyline sitting on a flat
/// cuboid floor, and a rotation-locked cuboid character dropped on top of the
/// square then driven right at constant speed. It must cross the top-right corner
/// without getting stuck, fall off, and keep moving right on the floor.
#[test]
fn cuboid_crosses_closed_chain_corner_without_sticking() {
    let mut h = Harness::new();

    // Flat ground: a wide fixed cuboid with its top face at y = 0.
    let floor = h
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1)));
    h.colliders.insert_with_parent(
        ColliderBuilder::cuboid(10.0, 0.1).friction(0.0),
        floor,
        &mut h.bodies,
    );

    // The issue's closed 2x2 square chain, wound counter-clockwise so the
    // `ORIENTED` outward side points away from the solid tile.
    let vertices = vec![
        Vector::new(0.0, 0.0),
        Vector::new(2.0, 0.0),
        Vector::new(2.0, 2.0),
        Vector::new(0.0, 2.0),
    ];
    let indices = vec![[0, 1], [1, 2], [2, 3], [3, 0]];
    let chain = h.bodies.insert(RigidBodyBuilder::fixed());
    h.colliders.insert_with_parent(
        ColliderBuilder::oriented_polyline(vertices, Some(indices)).friction(0.0),
        chain,
        &mut h.bodies,
    );

    // The issue's character: a 1x1 rotation-locked cuboid, dropped onto the top
    // of the square.
    let character = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.5, 3.0))
            .lock_rotations(),
    );
    h.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5)
            .friction(0.0)
            .restitution(0.0),
        character,
        &mut h.bodies,
    );

    // Let the drop settle on top of the square.
    for _ in 0..120 {
        h.step();
    }
    let settled = h.bodies[character].translation();
    assert!(
        (settled.y - 2.5).abs() < 0.1,
        "character did not settle on top of the square (y = {})",
        settled.y
    );

    // Drive it right at 2 m/s: across the top edge, over the (2, 2) corner, down
    // onto the floor, and away. Before the fix it stopped dead at the corner.
    let speed = 2.0;
    let mut prev_x = settled.x;
    for i in 0..250 {
        let vy = h.bodies[character].linvel().y;
        h.bodies[character].set_linvel(Vector::new(speed, vy), true);
        h.step();

        let x = h.bodies[character].translation().x;
        assert!(
            x >= prev_x - 1.0e-6,
            "character pushed backwards at step {i}: {} -> {}",
            prev_x,
            x
        );
        prev_x = x;
    }

    // 250 steps at 2 m/s is ~8.3 m of driving; being anywhere past x = 4 proves
    // the corner never captured the character (stuck = x pinned near 1.5).
    let end = h.bodies[character].translation();
    assert!(
        end.x > 4.0,
        "character got stuck at the chain corner (x = {}, y = {})",
        end.x,
        end.y
    );
    // And it ended resting on the floor, not embedded in the chain.
    assert!(
        (end.y - 0.5).abs() < 0.1,
        "character is not resting on the floor after crossing (y = {})",
        end.y
    );
}
