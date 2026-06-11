//! Phase-5 tests: keyframes, actuators, sensors.

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfRobot, MjcfSensorValue};

fn step(
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
fn actuator_pass_through_records_metadata() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="cart">
          <joint name="slider" type="slide" axis="1 0 0"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="box" size="0.1 0.1 0.1"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="cart_motor" joint="slider" gear="100" ctrlrange="-1 1"/>
      </actuator>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    assert_eq!(robot.actuators.len(), 1);
    assert_eq!(
        robot.actuators[0].actuator.name.as_deref(),
        Some("cart_motor")
    );
    assert!(robot.actuators[0].joint_index.is_some());
}

#[test]
fn motor_actuator_drives_slider() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="cart">
          <joint name="slider" type="slide" axis="1 0 0"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="box" size="0.1 0.1 0.1"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="cart_motor" joint="slider" gear="100"/>
      </actuator>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let handles =
        robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    // Cart starts with zero velocity. Drive constant ctrl = 1 → motor target velocity = 100 m/s.
    handles.apply_controls(&mut impulse_joints, &[1.0]);
    step(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        30,
    );
    let cart = bodies
        .get(handles.bodies[1].as_ref().unwrap().body)
        .unwrap();
    // Cart's velocity should now be substantial along +X.
    assert!(cart.linvel().x > 0.5, "cart vx = {}", cart.linvel().x);
}

// An affine `<general>` actuator is a position servo:
//   force = gainprm0·ctrl + biasprm0 + biasprm1·q + biasprm2·q̇.
// With gainprm=[k], biasprm=[0,-k,0] it holds the joint at `ctrl`. This is
// the actuator type the flybody legs use. `base` is welded to the world (no
// joint ⇒ fixed) so the `arm` hinge actually rotates in the world frame.
const AFFINE_SERVO_XML: &str = r#"
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="base">
      <geom type="box" size="0.1 0.1 0.1"/>
      <body name="arm" pos="0 0 0.15">
        <joint name="hinge" type="hinge" axis="0 0 1"/>
        <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
        <geom type="box" size="0.2 0.05 0.05"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <!-- gainprm0=10, biasprm=[0,-10,-2]: a servo with stiffness 10, velocity
         damping 2, holding the hinge at `ctrl`. -->
    <general name="servo" joint="hinge" biastype="affine" gainprm="10" biasprm="0 -10 -2"/>
  </actuator>
</mujoco>
"#;

/// The rapier handle of the `arm` body in the model above.
fn arm_handle<H: Copy>(robot: &MjcfRobot, handles: &rapier3d_mjcf::MjcfRobotHandles<H>) -> RigidBodyHandle {
    for (i, bh) in handles.bodies.iter().enumerate() {
        if robot.bodies[i].name.as_deref() == Some("arm") {
            return bh.as_ref().unwrap().body;
        }
    }
    panic!("arm body not found");
}

#[test]
fn affine_general_actuator_parses_as_servo() {
    let (robot, _) = MjcfRobot::from_str(AFFINE_SERVO_XML, MjcfLoaderOptions::default(), ".").unwrap();
    let a = &robot.actuators[0].actuator;
    assert_eq!(a.bias_type.as_deref(), Some("affine"));
    assert_eq!(a.gainprm.first().copied(), Some(10.0));
    assert_eq!(a.biasprm.get(1).copied(), Some(-10.0));
    assert_eq!(a.biasprm.get(2).copied(), Some(-2.0));
}

/// Drive the AFFINE_SERVO_XML model to a target angle and return the arm's
/// final rotation about Z. `multibody` selects the insertion + control path.
/// Re-applies the control every frame against a persistent solver state (so
/// it also exercises the per-frame wake path), and uses one `IslandManager`
/// across the whole run.
fn drive_servo_to(target: Real, multibody: bool) -> Real {
    use rapier3d_mjcf::MjcfMultibodyOptions;
    let (robot, _) = MjcfRobot::from_str(AFFINE_SERVO_XML, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    enum Ctl {
        Impulse(rapier3d_mjcf::MjcfRobotHandles<ImpulseJointHandle>),
        Multibody(rapier3d_mjcf::MjcfRobotHandles<Option<MultibodyJointHandle>>),
    }
    let (ctl, arm) = if multibody {
        let handles = robot.clone().insert_using_multibody_joints(
            &mut bodies,
            &mut colliders,
            &mut multibody_joints,
            &mut impulse_joints,
            MjcfMultibodyOptions::empty(),
        );
        let arm = arm_handle(&robot, &handles);
        (Ctl::Multibody(handles), arm)
    } else {
        let handles =
            robot.clone().insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
        let arm = arm_handle(&robot, &handles);
        (Ctl::Impulse(handles), arm)
    };

    let mut ccd = CCDSolver::new();
    let mut pipeline = PhysicsPipeline::new();
    let ip = IntegrationParameters::default();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    for _ in 0..40 {
        match &ctl {
            Ctl::Impulse(h) => h.apply_controls(&mut impulse_joints, &[target]),
            Ctl::Multibody(h) => {
                h.apply_controls_multibody(&mut bodies, &mut multibody_joints, &[target])
            }
        }
        pipeline.step(
            Vector::new(0.0, 0.0, 0.0),
            &ip,
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
    }
    bodies[arm].rotation().to_scaled_axis().z
}

#[test]
fn affine_general_actuator_drives_hinge_impulse_path() {
    let angle = drive_servo_to(0.5, false);
    assert!((angle - 0.5).abs() < 0.1, "servo didn't reach target: angle = {angle}");
}

#[test]
fn affine_general_actuator_drives_hinge_multibody_path() {
    let angle = drive_servo_to(0.5, true);
    assert!((angle - 0.5).abs() < 0.1, "multibody servo didn't reach target: angle = {angle}");
}

#[test]
fn keyframe_mocap_application() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="m" mocap="true" pos="0 0 0">
          <geom type="sphere" size="0.05"/>
        </body>
      </worldbody>
      <keyframe>
        <key name="home" mpos="1 2 3" mquat="1 0 0 0"/>
      </keyframe>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let handles =
        robot
            .clone()
            .insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    let key = robot.keyframes[0].clone();
    handles.apply_mocap_keyframe(&mut bodies, &robot, &key);
    let m = bodies
        .get(handles.bodies[1].as_ref().unwrap().body)
        .unwrap();
    let pos = m.translation();
    assert!((pos.x - 1.0).abs() < 1e-6);
    assert!((pos.y - 2.0).abs() < 1e-6);
    assert!((pos.z - 3.0).abs() < 1e-6);
}

#[test]
fn sensor_framepos_returns_world_position() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="b" pos="1 2 3">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.1"/>
        </body>
      </worldbody>
      <sensor>
        <framepos name="bp" objtype="body" objname="b"/>
      </sensor>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    assert_eq!(robot.sensors.len(), 1);
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let handles =
        robot
            .clone()
            .insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    let v = robot
        .read_sensor(0, &bodies, &handles, Vector::new(0.0, 0.0, -9.81))
        .unwrap();
    match v {
        MjcfSensorValue::Vector3(p) => {
            assert!((p.x - 1.0).abs() < 1e-3);
            assert!((p.y - 2.0).abs() < 1e-3);
            assert!((p.z - 3.0).abs() < 1e-3);
        }
        other => panic!("expected Vector3, got {other:?}"),
    }
}
