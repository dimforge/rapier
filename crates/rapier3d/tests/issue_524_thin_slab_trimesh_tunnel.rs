//! Regression test for #524: thin objects dropped onto a `TriMesh` terrain used to fall
//! straight through it very easily.
//!
//! With default parameters — no explicit CCD, contact skin or soft-ccd — the automatic
//! fast-body-vs-fixed CCD tier plus the stable manifolds must keep a 0.06-thick slab,
//! dropped tilted, above the ground.

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

/// Drops the slab from 5 m with the given initial rotation (axis-angle vector)
/// and checks it settles on the trimesh instead of tunneling through it.
fn drop_thin_slab(rotation: Vector) {
    let mut h = Harness::new();

    // The issue's terrain: a flat trimesh ground made of just two triangles.
    let vtx = vec![
        Vector::new(-20.0, 0.0, -20.0),
        Vector::new(20.0, 0.0, -20.0),
        Vector::new(20.0, 0.0, 20.0),
        Vector::new(-20.0, 0.0, 20.0),
    ];
    let idx = vec![[0, 2, 1], [0, 3, 2]];
    h.colliders.insert(ColliderBuilder::new(SharedShape::new(
        TriMesh::new(vtx, idx).unwrap(),
    )));

    // The 2 x 0.06 x 2 slab (half-height 0.03), slightly rotated, 5 m up.
    let slab = h.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 5.0, 0.0))
            .rotation(rotation),
    );
    h.colliders
        .insert_with_parent(ColliderBuilder::cuboid(1.0, 0.03, 1.0), slab, &mut h.bodies);

    for i in 0..600 {
        h.step();
        let y = h.bodies[slab].translation().y;
        assert!(
            y > -0.1,
            "slab tunneled through the trimesh ground at step {i} \
             (rotation {rotation:?}, y = {y})"
        );
    }

    // It must have settled flat on the ground, not be bouncing or embedded.
    let pos = h.bodies[slab].translation();
    let vel = h.bodies[slab].linvel();
    assert!(
        pos.y > 0.0 && pos.y < 0.2,
        "slab did not settle on the trimesh (rotation {rotation:?}, y = {})",
        pos.y
    );
    assert!(
        vel.length() < 0.05,
        "slab still moving after 600 steps (rotation {rotation:?}, vel = {vel:?})"
    );
}

#[test]
fn thin_slab_settles_tilt_x() {
    drop_thin_slab(Vector::new(0.1, 0.0, 0.0));
}

#[test]
fn thin_slab_settles_tilt_z() {
    drop_thin_slab(Vector::new(0.0, 0.0, 0.12));
}

#[test]
fn thin_slab_settles_tilt_xz() {
    drop_thin_slab(Vector::new(0.15, 0.0, 0.1));
}

#[test]
fn thin_slab_settles_tilt_neg() {
    drop_thin_slab(Vector::new(-0.12, 0.0, 0.08));
}
