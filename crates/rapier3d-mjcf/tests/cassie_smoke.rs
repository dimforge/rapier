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
/// plantar-rod tie the foot/heel-spring back into the chain). Those loops
/// rely on the `<joint damping>` values declared on the leg joints to stay
/// numerically stable — without that damping, the impulse-joint solver
/// can drift under default-timestep multibody simulation and blow up
/// within a second of sim time.
///
/// This regression captures that mode: keep equality constraints active
/// but disable the motors and verify the failure mode is reproducible
/// (we don't claim it should work — but the test documents the trade-off).
/// `disable_joint_motors=true` skips the motors used for springs and
/// friction-loss but does NOT skip the per-DoF damping path (per-DoF
/// damping is dynamics-level, not a motor). Cassie still blows up under
/// this configuration however: its `<joint stiffness>` springs on the
/// shin and heel-spring joints are structural to keeping the loop
/// closures balanced, and removing the springs (motors) destabilizes the
/// loops faster than the surviving damping can compensate.
#[test]
#[ignore = "requires the cassie assets — documents an unstable configuration"]
#[should_panic(expected = "NaN")]
fn cassie_unstable_without_motors_and_with_loop_closures() {
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

    // 50 steps is enough — without damping the loops typically NaN by
    // step ~40 under default integration parameters.
    step(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        50,
        gravity,
    );
}
