//! Regression test for https://github.com/dimforge/rapier/issues/818
//!
//! The contact graph used to report NaN (or spuriously zero) `tangent_impulse` for
//! capsule-vs-cuboid friction: constraint buffers were reused without zeroing and the
//! tangent impulse accumulator was never reset.

use rapier2d::prelude::*;
use std::f32::consts::PI;

/// A capsule with locked rotations sliding on a tilted thin cuboid must report
/// finite contact impulses at every step.
#[test]
fn capsule_on_tilted_cuboid_reports_finite_impulses() {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut islands = IslandManager::new();
    let mut ccd = CCDSolver::new();
    let params = IntegrationParameters::default();
    let gravity = Vector::new(0.0, -9.81);

    // The tilted ground.
    let ground = bodies.insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -3.0)));
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(20.0, 1.0)
            .rotation(0.25 * PI)
            .friction(0.9),
        ground,
        &mut bodies,
    );

    // A capsule that cannot rotate.
    let capsule = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 4.0))
            .lock_rotations(),
    );
    colliders.insert_with_parent(
        ColliderBuilder::capsule_y(2.0, 1.0).friction(0.9),
        capsule,
        &mut bodies,
    );

    let mut saw_tangent_impulse = false;
    for step in 0..500 {
        pipeline.step(
            gravity,
            &params,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &(),
            &(),
        );

        for pair in narrow_phase.contact_pairs() {
            for manifold in &pair.manifolds {
                for pt in &manifold.points {
                    assert!(
                        pt.data.impulse.is_finite(),
                        "step {step}: non-finite normal impulse {:?}",
                        pt.data.impulse
                    );
                    for tangent in pt.data.tangent_impulse.iter() {
                        assert!(
                            tangent.is_finite(),
                            "step {step}: non-finite tangent impulse {:?}",
                            pt.data.tangent_impulse
                        );
                        if tangent.abs() > 1.0e-4 {
                            saw_tangent_impulse = true;
                        }
                    }
                    assert!(
                        pt.data.warmstart_impulse.is_finite()
                            && pt
                                .data
                                .warmstart_tangent_impulse
                                .iter()
                                .all(|x| x.is_finite()),
                        "step {step}: non-finite warmstart impulses"
                    );
                }
            }
        }
    }

    // The scenario must actually exercise friction: the capsule rests on the
    // slope, so nonzero tangent impulses must have been reported at some point.
    assert!(
        saw_tangent_impulse,
        "expected nonzero tangent impulses to be reported"
    );
}
