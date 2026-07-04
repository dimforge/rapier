//! Keyframe `qpos` / `qvel` application on both insertion paths.

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot};

/// A two-link arm hinged to the world: a shoulder hinge about Z then an
/// elbow hinge about Z, with a "home" keyframe that bends both joints.
const ARM: &str = r#"
<mujoco>
  <worldbody>
    <body name="upper" pos="0 0 1">
      <joint name="shoulder" type="hinge" axis="0 0 1"/>
      <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
      <geom type="capsule" fromto="0 0 0 1 0 0" size="0.05"/>
      <body name="lower" pos="1 0 0">
        <joint name="elbow" type="hinge" axis="0 0 1"/>
        <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
        <geom type="capsule" fromto="0 0 0 1 0 0" size="0.05"/>
      </body>
    </body>
  </worldbody>
  <keyframe>
    <key name="home" qpos="0.5 -0.3"/>
  </keyframe>
</mujoco>
"#;

fn world_z_angle(rb: &RigidBody) -> f32 {
    // The arm rotates in the XY plane about Z; recover the angle from the
    // body's rotation applied to the +X axis.
    let x = rb.rotation() * Vector::X;
    x.y.atan2(x.x)
}

#[test]
fn qpos_applied_via_multibody() {
    let (robot, _) = MjcfRobot::from_str(ARM, MjcfLoaderOptions::default(), ".").unwrap();
    assert_eq!(robot.qpos_dofs.len(), 2, "expected two 1-DoF hinge slots");

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let handles = robot.clone().insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        &mut impulse_joints,
        MjcfMultibodyOptions::default(),
    );

    let key = robot.keyframe_by_name("home").unwrap().clone();
    handles.apply_keyframe(&mut bodies, &mut multibody_joints, &robot, &key);

    let upper = bodies
        .get(handles.bodies[1].as_ref().unwrap().body)
        .unwrap();
    let lower = bodies
        .get(handles.bodies[2].as_ref().unwrap().body)
        .unwrap();
    // Shoulder = 0.5 (world angle); elbow = -0.3 relative → 0.2 world.
    assert!(
        (world_z_angle(upper) - 0.5).abs() < 1e-4,
        "shoulder angle = {}",
        world_z_angle(upper)
    );
    assert!(
        (world_z_angle(lower) - 0.2).abs() < 1e-4,
        "lower world angle = {}",
        world_z_angle(lower)
    );
}

#[test]
fn qpos_applied_via_impulse_joints() {
    let (robot, _) = MjcfRobot::from_str(ARM, MjcfLoaderOptions::default(), ".").unwrap();

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let handles =
        robot
            .clone()
            .insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);

    let key = robot.keyframe_by_name("home").unwrap().clone();
    handles.apply_keyframe(&mut bodies, &robot, &key);

    let upper = bodies
        .get(handles.bodies[1].as_ref().unwrap().body)
        .unwrap();
    let lower = bodies
        .get(handles.bodies[2].as_ref().unwrap().body)
        .unwrap();
    assert!(
        (world_z_angle(upper) - 0.5).abs() < 1e-4,
        "shoulder angle = {}",
        world_z_angle(upper)
    );
    assert!(
        (world_z_angle(lower) - 0.2).abs() < 1e-4,
        "lower world angle = {}",
        world_z_angle(lower)
    );
    // The lower body's anchor (the elbow) must coincide on both sides — i.e.
    // forward-kinematics kept the joint satisfied. Elbow world point is the
    // tip of the upper link.
    let elbow = upper.translation() + upper.rotation() * Vector::new(1.0, 0.0, 0.0);
    let lower_origin = lower.translation();
    assert!(
        (elbow - lower_origin).length() < 1e-4,
        "elbow joint violated: {elbow:?} vs {lower_origin:?}"
    );
}

/// A free-floating base + one hinge: the "home" keyframe lifts and rotates the
/// base (qpos[0..7]) and bends the hinge (qpos[7]).
const FLOATER: &str = r#"
<mujoco>
  <worldbody>
    <body name="base" pos="0 0 0">
      <freejoint/>
      <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
      <geom type="box" size="0.2 0.2 0.2"/>
      <body name="arm" pos="0.2 0 0">
        <joint name="j" type="hinge" axis="0 1 0"/>
        <inertial mass="0.5" diaginertia="0.01 0.01 0.01"/>
        <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.03"/>
      </body>
    </body>
  </worldbody>
  <keyframe>
    <key name="home" qpos="1 2 3 1 0 0 0  0.7"/>
  </keyframe>
</mujoco>
"#;

#[test]
fn free_base_qpos_sets_world_pose() {
    let (robot, _) = MjcfRobot::from_str(FLOATER, MjcfLoaderOptions::default(), ".").unwrap();
    assert_eq!(robot.qpos_dofs.len(), 2);

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let handles = robot.clone().insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        &mut impulse_joints,
        MjcfMultibodyOptions::default(),
    );

    let key = robot.keyframe_by_name("home").unwrap().clone();
    handles.apply_keyframe(&mut bodies, &mut multibody_joints, &robot, &key);

    let base = bodies
        .get(handles.bodies[1].as_ref().unwrap().body)
        .unwrap();
    let t = base.translation();
    assert!(
        (t - Vector::new(1.0, 2.0, 3.0)).length() < 1e-4,
        "base translation = {t:?}"
    );

    // The child arm rides along: its origin sits at base + R_base*(0.2,0,0),
    // which with an identity base orientation is (1.2, 2, 3).
    let arm = bodies
        .get(handles.bodies[2].as_ref().unwrap().body)
        .unwrap();
    let a = arm.translation();
    assert!(
        (a - Vector::new(1.2, 2.0, 3.0)).length() < 1e-4,
        "arm origin = {a:?}"
    );
}
