//! `MjcfRobot::keyframe_controls` lets an actuated model hold a keyframe pose
//! under the controls callback, instead of the zero control vector dragging
//! every position servo back to the zero configuration.

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot};

// A hinge held by a `<position>` servo, with a keyframe that bends it to 1.0.
const SERVO_ARM: &str = r#"
<mujoco>
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
      <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.03"/>
    </body>
  </worldbody>
  <actuator>
    <position name="a" joint="j" kp="20"/>
  </actuator>
  <keyframe>
    <key name="bent" qpos="1.0"/>
  </keyframe>
</mujoco>
"#;

fn hinge_angle(rb: &RigidBody) -> f32 {
    let x = rb.rotation() * Vector::X;
    (-x.z).atan2(x.x)
}

fn build() -> (
    MjcfRobot,
    RigidBodySet,
    ColliderSet,
    ImpulseJointSet,
    MultibodyJointSet,
    rapier3d_mjcf::MjcfRobotHandles<Option<MultibodyJointHandle>>,
) {
    let (robot, _) = MjcfRobot::from_str(SERVO_ARM, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut ij = ImpulseJointSet::new();
    let mut mbj = MultibodyJointSet::new();
    let handles = robot.clone().insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut mbj,
        &mut ij,
        MjcfMultibodyOptions::default(),
    );
    (robot, bodies, colliders, ij, mbj, handles)
}

fn run(
    robot: &MjcfRobot,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    ij: &mut ImpulseJointSet,
    mbj: &mut MultibodyJointSet,
    handles: &rapier3d_mjcf::MjcfRobotHandles<Option<MultibodyJointHandle>>,
    ctrl: &[Real],
) -> f32 {
    let mut pipeline = PhysicsPipeline::new();
    let ip = IntegrationParameters::default();
    let (mut isl, mut bp, mut np, mut ccd) = (
        IslandManager::new(),
        DefaultBroadPhase::new(),
        NarrowPhase::new(),
        CCDSolver::new(),
    );
    for _ in 0..200 {
        handles.apply_controls_multibody(bodies, mbj, ctrl);
        pipeline.step(
            Vector::new(0.0, 0.0, -9.81),
            &ip,
            &mut isl,
            &mut bp,
            &mut np,
            bodies,
            colliders,
            ij,
            mbj,
            &mut ccd,
            &(),
            &(),
        );
    }
    let link = robot.body_name_to_idx["link"];
    hinge_angle(
        bodies
            .get(handles.bodies[link].as_ref().unwrap().body)
            .unwrap(),
    )
}

#[test]
fn zero_ctrl_drags_keyframe_to_zero_config() {
    let (robot, mut bodies, mut colliders, mut ij, mut mbj, handles) = build();
    let key = robot.keyframe_by_name("bent").unwrap().clone();
    handles.apply_keyframe(&mut bodies, &mut mbj, &robot, &key);
    // The naive zero control vector: the servo pulls the joint back to 0.
    let angle = run(
        &robot,
        &mut bodies,
        &mut colliders,
        &mut ij,
        &mut mbj,
        &handles,
        &vec![0.0; handles.actuators.len()],
    );
    assert!(
        angle.abs() < 0.05,
        "expected joint dragged to ~0, got {angle}"
    );
}

#[test]
fn keyframe_controls_hold_the_pose() {
    let (robot, mut bodies, mut colliders, mut ij, mut mbj, handles) = build();
    let key = robot.keyframe_by_name("bent").unwrap().clone();
    handles.apply_keyframe(&mut bodies, &mut mbj, &robot, &key);
    // Driving the servo with the keyframe-derived control holds the pose.
    let ctrl = robot.keyframe_controls(&key);
    assert_eq!(ctrl.len(), 1);
    assert!(
        (ctrl[0] - 1.0).abs() < 1e-6,
        "derived control = {}",
        ctrl[0]
    );
    let angle = run(
        &robot,
        &mut bodies,
        &mut colliders,
        &mut ij,
        &mut mbj,
        &handles,
        &ctrl,
    );
    assert!(
        (angle - 1.0).abs() < 0.05,
        "expected joint held at ~1.0, got {angle}"
    );
}
