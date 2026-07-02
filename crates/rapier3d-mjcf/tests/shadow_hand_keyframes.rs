//! Keyframes whose `qpos` is shorter than the model's full generalized
//! coordinate vector. This is what lets the menagerie's standalone
//! `keyframes.xml` snippets (authored against the bare robot) apply to a scene
//! that *appends* free props after the robot: the keyframe drives the leading
//! DoFs and leaves the trailing ones untouched.

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot};

/// A hinged arm declared before a free-floating prop, with a keyframe that only
/// specifies the hinge's single `qpos` — the prop's 7 `qpos` are omitted.
const ARM_THEN_PROP: &str = r#"
<mujoco>
  <worldbody>
    <body name="arm" pos="0 0 1">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
      <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.05"/>
    </body>
    <body name="prop" pos="2 0 0">
      <freejoint/>
      <inertial mass="0.2" diaginertia="0.01 0.01 0.01"/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
  <keyframe>
    <key name="bent" qpos="0.6"/>
  </keyframe>
</mujoco>
"#;

fn arm_y_angle(rb: &RigidBody) -> f32 {
    // The hinge is about +Y; recover the angle from where +X lands.
    let x = rb.rotation() * Vector::X;
    (-x.z).atan2(x.x)
}

#[test]
fn short_qpos_applies_to_leading_dofs_multibody() {
    let (robot, _) = MjcfRobot::from_str(ARM_THEN_PROP, MjcfLoaderOptions::default(), ".").unwrap();
    // Two DoF slots: the hinge, then the prop's free joint (in that order).
    assert_eq!(robot.qpos_dofs.len(), 2);

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut mbj = MultibodyJointSet::new();
    let handles = robot.clone().insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut mbj,
        &mut impulse_joints,
        MjcfMultibodyOptions::default(),
    );

    let arm = robot.body_name_to_idx["arm"];
    let prop = robot.body_name_to_idx["prop"];
    let prop_before = bodies
        .get(handles.bodies[prop].as_ref().unwrap().body)
        .unwrap()
        .translation();

    let key = robot.keyframe_by_name("bent").unwrap().clone();
    handles.apply_keyframe(&mut bodies, &mut mbj, &robot, &key);

    let arm_rb = bodies
        .get(handles.bodies[arm].as_ref().unwrap().body)
        .unwrap();
    let prop_after = bodies
        .get(handles.bodies[prop].as_ref().unwrap().body)
        .unwrap()
        .translation();

    assert!(
        (arm_y_angle(arm_rb) - 0.6).abs() < 1e-4,
        "hinge angle = {}",
        arm_y_angle(arm_rb)
    );
    // The prop has no qpos in the keyframe, so it stays where it loaded.
    assert!(
        (prop_before - prop_after).length() < 1e-6,
        "prop moved: {prop_before:?} -> {prop_after:?}"
    );
}

#[test]
fn short_qpos_applies_to_leading_dofs_impulse() {
    let (robot, _) = MjcfRobot::from_str(ARM_THEN_PROP, MjcfLoaderOptions::default(), ".").unwrap();

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let handles =
        robot
            .clone()
            .insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);

    let arm = robot.body_name_to_idx["arm"];
    let prop = robot.body_name_to_idx["prop"];
    let prop_before = bodies
        .get(handles.bodies[prop].as_ref().unwrap().body)
        .unwrap()
        .translation();

    let key = robot.keyframe_by_name("bent").unwrap().clone();
    handles.apply_keyframe(&mut bodies, &robot, &key);

    let arm_rb = bodies
        .get(handles.bodies[arm].as_ref().unwrap().body)
        .unwrap();
    let prop_after = bodies
        .get(handles.bodies[prop].as_ref().unwrap().body)
        .unwrap()
        .translation();

    assert!(
        (arm_y_angle(arm_rb) - 0.6).abs() < 1e-4,
        "hinge angle = {}",
        arm_y_angle(arm_rb)
    );
    assert!(
        (prop_before - prop_after).length() < 1e-6,
        "prop moved: {prop_before:?} -> {prop_after:?}"
    );
}

/// End-to-end check against the real shadow_hand model + its standalone
/// `keyframes.xml`, mirroring what the menagerie example does. Skipped when the
/// menagerie isn't cloned next to the rapier repo.
#[test]
fn shadow_hand_sibling_keyframes_align() {
    let base = "../../../mujoco_menagerie/shadow_hand";
    let scene = format!("{base}/scene_right.xml");
    let kf = format!("{base}/keyframes.xml");
    if !std::path::Path::new(&scene).exists() {
        eprintln!("mujoco_menagerie not found; skipping shadow_hand integration test");
        return;
    }

    let (mut robot, _) = MjcfRobot::from_file(&scene, MjcfLoaderOptions::default()).unwrap();
    let (kfr, _) = MjcfRobot::from_file(&kf, MjcfLoaderOptions::default()).unwrap();
    robot.keyframes = kfr.keyframes; // the scene declares none; merge them in.
    assert_eq!(robot.keyframes.len(), 13);

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut mbj = MultibodyJointSet::new();
    let handles = robot.clone().insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut mbj,
        &mut impulse_joints,
        MjcfMultibodyOptions::default(),
    );

    let obj = robot.body_name_to_idx["object"];
    let ff = robot.body_name_to_idx["rh_ffdistal"];
    let obj_before = bodies
        .get(handles.bodies[obj].as_ref().unwrap().body)
        .unwrap()
        .translation();
    let ff_before = *bodies
        .get(handles.bodies[ff].as_ref().unwrap().body)
        .unwrap()
        .rotation();

    let key = robot.keyframe_by_name("rock").unwrap().clone();
    handles.apply_keyframe(&mut bodies, &mut mbj, &robot, &key);

    let obj_after = bodies
        .get(handles.bodies[obj].as_ref().unwrap().body)
        .unwrap()
        .translation();
    let ff_after = *bodies
        .get(handles.bodies[ff].as_ref().unwrap().body)
        .unwrap()
        .rotation();

    // The "rock" pose curls the fingers; the free object (declared after the
    // hand, so its qpos sits past the keyframe's 24 values) stays put.
    assert!(
        ff_before.angle_between(ff_after) > 0.5,
        "finger didn't curl: {}",
        ff_before.angle_between(ff_after)
    );
    assert!(
        (obj_before - obj_after).length() < 1e-6,
        "object moved: {obj_before:?} -> {obj_after:?}"
    );
    assert!(
        (obj_after - Vector::new(0.3, 0.0, 0.1)).length() < 1e-3,
        "object not at its scene pose: {obj_after:?}"
    );
}
