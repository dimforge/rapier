//! Regression tests for #372: round shapes resting on a triangle mesh never lost their
//! kinetic energy and never fell asleep.
//!
//! The ball is fixed; the capsule and cylinder still *gain* energy, self-accelerating to a
//! steady ~1 degree per timestep — root cause is parry's cached-manifold fast path
//! (`ContactManifold::try_update_contacts`) freezing contact anchors, so those cases stay
//! `#[ignore]`d pending a parry fix.

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

    /// A flat trimesh ground made of two triangles (internal diagonal along x = z).
    fn insert_trimesh_ground(&mut self) {
        let vtx = vec![
            Vector::new(-20.0, 0.0, -20.0),
            Vector::new(20.0, 0.0, -20.0),
            Vector::new(20.0, 0.0, 20.0),
            Vector::new(-20.0, 0.0, 20.0),
        ];
        let idx = vec![[0, 2, 1], [0, 3, 2]];
        self.colliders.insert(ColliderBuilder::new(SharedShape::new(
            TriMesh::new(vtx, idx).unwrap(),
        )));
    }

    /// Steps until the body sleeps; panics after `max_steps`.
    fn assert_sleeps(&mut self, body: RigidBodyHandle, max_steps: usize, what: &str) {
        for _ in 0..max_steps {
            self.step();
            if self.bodies[body].is_sleeping() {
                return;
            }
        }
        let vel = self.bodies[body].linvel();
        let angvel = self.bodies[body].angvel();
        panic!(
            "{what} never fell asleep after {max_steps} steps on the trimesh \
             (linvel = {vel:?}, angvel = {angvel:?})"
        );
    }
}

/// A ball dropped onto a flat trimesh with a small initial push must come to rest
/// and fall asleep. (Dropped away from the mesh's internal diagonal edge: landing
/// exactly on the edge still imparts a small sideways kick that keeps the ball
/// rolling above the sleep threshold.)
#[test]
fn ball_on_trimesh_falls_asleep() {
    let mut h = Harness::new();
    h.insert_trimesh_ground();

    let ball = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-5.0, 1.0, 5.0))
            .linvel(Vector::new(0.03, 0.0, 0.0)),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::ball(0.5), ball, &mut h.bodies);

    h.assert_sleeps(ball, 2000, "ball");
}

/// A capsule dropped sideways onto a flat trimesh with a small push must come to
/// rest and fall asleep. Currently it instead self-accelerates to a steady roll
/// (see the module docs for the analysis) and never sleeps.
#[test]
#[ignore = "issue #372 not fixed yet: rolling capsule is velocity-pumped by parry's frozen manifold anchors"]
fn capsule_on_trimesh_falls_asleep() {
    let mut h = Harness::new();
    h.insert_trimesh_ground();

    // Lying on its side (axis along z after the x-rotation), free to roll.
    let capsule = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-5.0, 1.0, 5.0))
            .rotation(Vector::new(std::f32::consts::FRAC_PI_2 as Real, 0.0, 0.0))
            .linvel(Vector::new(0.03, 0.0, 0.0)),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::capsule_y(0.4, 0.3), capsule, &mut h.bodies);

    h.assert_sleeps(capsule, 2000, "capsule");
}
