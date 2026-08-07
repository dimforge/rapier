//! Regression test for #810: small cubes dropped at ~20 m/s onto a thin fixed cylinder disc
//! used to pass straight through it, even with `ccd_enabled`.
//!
//! STILL BROKEN (2/20 cubes tunnel): the CCD sweep clamps the cube at the cap surface, but
//! parry's cuboid-vs-cylinder-cap manifold has a single contact point, so the solver zeroes
//! the velocity via spin (~74 rad/s in one step) and the cube corkscrews through — the fix
//! belongs in parry (cf. parry#298/parry#318).

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

#[test]
#[ignore = "issue #810 not fixed yet: single-point cylinder-cap manifold lets landing cubes spin through"]
fn cubes_do_not_fall_through_thin_cylinder_disc() {
    let mut h = Harness::new();

    // The issue's floor: a fixed cylinder disc, radius 10, half-height 0.05,
    // centered at y = -2 (top face at y = -1.95).
    let radius = 10.0;
    let disc_top = -1.95;
    let disc = h
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -2.0, 0.0)));
    h.colliders
        .insert_with_parent(ColliderBuilder::cylinder(0.05, radius), disc, &mut h.bodies);

    // ~20 small cubes (half-extents 0.05) dropped from y = 20, spread over the
    // disc with a deterministic golden-angle spiral (max spawn radius 8.55).
    let mut cubes = Vec::new();
    for k in 0..20 {
        let r = k as Real * 0.45;
        let angle = k as Real * 2.399;
        let (x, z) = (r * angle.cos(), r * angle.sin());
        let cube = h
            .bodies
            .insert(RigidBodyBuilder::dynamic().translation(Vector::new(x, 20.0, z)));
        h.colliders.insert_with_parent(
            ColliderBuilder::cuboid(0.05, 0.05, 0.05),
            cube,
            &mut h.bodies,
        );
        cubes.push(cube);
    }

    for i in 0..600 {
        h.step();

        // A cube found clearly below the disc's top must be outside the disc's
        // radius (it rolled off the side) — being inside means it tunneled
        // through the thin cylinder.
        for (k, &cube) in cubes.iter().enumerate() {
            let pos = h.bodies[cube].translation();
            if pos.y < disc_top - 0.5 {
                let horiz_dist = (pos.x * pos.x + pos.z * pos.z).sqrt();
                assert!(
                    horiz_dist > radius - 0.2,
                    "cube {k} tunneled through the thin cylinder disc at step {i}: \
                     pos = {pos:?} (horizontal distance from axis: {horiz_dist})"
                );
            }
        }
    }

    // Sanity: most cubes actually ended up resting on the disc.
    let on_disc = cubes
        .iter()
        .filter(|&&c| (h.bodies[c].translation().y - disc_top).abs() < 0.2)
        .count();
    assert!(
        on_disc >= 15,
        "only {on_disc}/20 cubes rest on the disc after 600 steps"
    );
}
