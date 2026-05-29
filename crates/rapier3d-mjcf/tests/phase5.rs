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
