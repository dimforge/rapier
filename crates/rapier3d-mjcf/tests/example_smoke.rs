//! Smoke test mirroring the example: load and step the testbed MJCF
//! through both insertion paths.

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot};

const MJCF: &str = r#"
<mujoco model="mjcf_demo">
  <compiler angle="degree"/>
  <option gravity="0 0 -9.81"/>
  <default>
    <joint damping="0.05"/>
    <geom rgba="0.7 0.3 0.3 1"/>
    <default class="link">
      <geom rgba="0.4 0.6 0.9 1"/>
    </default>
  </default>
  <worldbody>
    <geom name="floor" type="box" pos="0 0 -0.05" size="5 5 0.05" rgba="0.6 0.6 0.6 1"/>
    <body name="mount" pos="0 0 3">
      <inertial mass="1" diaginertia="0.05 0.05 0.05"/>
      <geom type="box" size="0.1 0.1 0.1" rgba="0.3 0.3 0.3 1"/>
      <body name="link1" pos="0 0 -0.3" childclass="link">
        <joint name="hinge1" type="hinge" axis="1 0 0" range="-90 90"/>
        <inertial mass="0.5" diaginertia="0.02 0.02 0.001" pos="0 0 -0.15"/>
        <geom type="capsule" size="0.04 0.15" pos="0 0 -0.15"/>
      </body>
    </body>
    <body name="cart" pos="2 0 0.05">
      <joint name="slider" type="slide" axis="1 0 0" range="-2 2"/>
      <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
      <geom type="box" size="0.1 0.06 0.04" rgba="0.2 0.7 0.3 1"/>
      <body name="pole" pos="0 0 0">
        <joint name="pivot" type="hinge" axis="0 1 0"/>
        <inertial mass="0.2" diaginertia="0.005 0.005 0.0005" pos="0 0 0.3"/>
        <geom type="capsule" size="0.02 0.3" pos="0 0 0.3" rgba="0.9 0.7 0.2 1"/>
      </body>
    </body>
    <body name="ball" pos="-2 0 4">
      <freejoint/>
      <inertial mass="1" diaginertia="0.05 0.05 0.05"/>
      <geom type="sphere" size="0.2"/>
    </body>
  </worldbody>
</mujoco>
"#;

fn step_n(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    n: usize,
) {
    let mut ccd = CCDSolver::new();
    let mut pipeline = PhysicsPipeline::new();
    let integration_parameters = IntegrationParameters::default();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let gravity = Vector::new(0.0, 0.0, -9.81);
    let physics_hooks = ();
    let event_handler = ();
    for _ in 0..n {
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
    }
}

#[test]
fn impulse_path() {
    let (robot, _) = MjcfRobot::from_str(MJCF, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    step_n(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        50,
    );
}

#[test]
fn multibody_path() {
    let (robot, _) = MjcfRobot::from_str(MJCF, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    robot.insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        &mut impulse_joints,
        MjcfMultibodyOptions::DISABLE_SELF_CONTACTS,
    );
    step_n(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        50,
    );
}

#[test]
fn double_insert_path() {
    // Mirrors the example: clone-insert as impulse joints, then transform
    // and insert as multibody joints into the same physics world.
    use rapier3d::math::Pose;
    let (mut robot, _) = MjcfRobot::from_str(MJCF, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    robot
        .clone()
        .insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    robot.append_transform(&Pose::translation(0.0, 4.0, 0.0));
    robot.insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        &mut impulse_joints,
        MjcfMultibodyOptions::DISABLE_SELF_CONTACTS,
    );
    step_n(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        50,
    );
}
