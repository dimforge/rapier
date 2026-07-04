//! Reproduce the user's cassie setup to localize the NaN crash.

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot};

const CASSIE_PATH: &str = "/Users/sebcrozet/work/rapier/assets/3d/agility_cassie/scene.xml";

fn step(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    n: usize,
    gravity: Vector,
) {
    let mut ccd = CCDSolver::new();
    let mut pipeline = PhysicsPipeline::new();
    let integration_parameters = IntegrationParameters::default();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let physics_hooks = ();
    let event_handler = ();
    for i in 0..n {
        pipeline.step(
            gravity,
            &integration_parameters,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            &mut ccd,
            &physics_hooks,
            &event_handler,
        );
        // Sanity-check: no NaN should leak into a body's pose.
        for (_, b) in bodies.iter() {
            let p = b.translation();
            assert!(
                p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
                "NaN at step {i}"
            );
        }
    }
}

#[test]
#[ignore = "requires the cassie assets"]
fn cassie_multibody_loads_without_nan() {
    // Mirrors the testbed example's current settings exactly.
    let options = MjcfLoaderOptions {
        scale: 10.0,
        skip_plane_geoms: true,
        make_roots_fixed: true,
        ..Default::default()
    };
    let (mut robot, model) = MjcfRobot::from_file(CASSIE_PATH, options).unwrap();
    let gravity = Vector::new(
        model.option.gravity[0] as Real,
        model.option.gravity[1] as Real,
        model.option.gravity[2] as Real,
    );

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    robot.append_transform(&Pose::translation(0.0, 4.0, 0.0));
    robot.insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        &mut impulse_joints,
        MjcfMultibodyOptions::DISABLE_SELF_CONTACTS,
    );

    step(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        500,
        gravity,
    );
}

/// Cassie has 4 `<equality><connect>` loop closures (achilles-rod and
/// plantar-rod tie the foot/heel-spring back into the chain). This
/// configuration — loop closures active, joint motors disabled — used to
/// NaN within ~40 steps because of two bugs in the generic impulse-joint
/// solver path (com-shifted anchors composed with origin-centered link
/// poses, and a per-link two-block effective mass that dropped the
/// `J1ᵀ·W·J2` coupling for constraints between links of the same
/// multibody). With those fixed, the dangling legs just swing and the
/// closures hold; this regression keeps that configuration NaN-free.
///
/// Note: `disable_joint_motors` disables actuator/friction-loss motors only —
/// the passive `<joint stiffness>` springs are not motors and stay active
/// (integrated implicitly), so this also exercises the loop closures together
/// with cassie's stiff shin/heel springs.
#[test]
#[ignore = "requires the cassie assets"]
fn cassie_stable_without_motors_and_with_loop_closures() {
    let options = MjcfLoaderOptions {
        scale: 10.0,
        skip_plane_geoms: true,
        make_roots_fixed: true,
        disable_joint_motors: true,
        ..Default::default()
    };
    let (mut robot, model) = MjcfRobot::from_file(CASSIE_PATH, options).unwrap();
    let gravity = Vector::new(
        model.option.gravity[0] as Real,
        model.option.gravity[1] as Real,
        model.option.gravity[2] as Real,
    );

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    robot.append_transform(&Pose::translation(0.0, 4.0, 0.0));
    robot.insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        &mut impulse_joints,
        MjcfMultibodyOptions::DISABLE_SELF_CONTACTS,
    );

    // The broken solver used to NaN by step ~40; run well past that.
    step(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        500,
        gravity,
    );
}
