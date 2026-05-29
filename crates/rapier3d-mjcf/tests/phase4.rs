//! Phase-4 tests: joint dynamics (damping, springs, armature, gravcomp).

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfRobot};

fn step_n(
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
fn damping_is_recorded_on_joint() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body><joint name="j" type="hinge" axis="0 0 1" damping="0.5"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (_, model) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    assert_eq!(model.bodies[1].body.joints[0].damping, 0.5);
}

#[test]
fn springref_is_baked_in_radians() {
    // With angle="degree" (default), springref="45" → π/4.
    let xml = r#"
    <mujoco>
      <worldbody>
        <body><joint name="j" type="hinge" axis="0 0 1" springref="45" stiffness="10"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (_, model) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let sr = model.bodies[1].body.joints[0].springref;
    assert!((sr - std::f64::consts::FRAC_PI_4).abs() < 1e-12);
}

#[test]
fn armature_changes_local_mprops_inertia() {
    // Direct check that armature is reflected in the body's
    // additional_local_mprops (before insertion). The local inertia about
    // the joint axis should be larger with armature than without.
    let xml_arm = r#"<mujoco><worldbody>
      <body><joint type="hinge" axis="1 0 0" armature="0.5"/>
        <inertial mass="1" diaginertia="0.001 0.001 0.001" pos="0 1 0"/>
      </body></worldbody></mujoco>"#;
    let xml_no = r#"<mujoco><worldbody>
      <body><joint type="hinge" axis="1 0 0"/>
        <inertial mass="1" diaginertia="0.001 0.001 0.001" pos="0 1 0"/>
      </body></worldbody></mujoco>"#;
    let (rob_arm, _) = MjcfRobot::from_str(xml_arm, MjcfLoaderOptions::default(), ".").unwrap();
    let (rob_no, _) = MjcfRobot::from_str(xml_no, MjcfLoaderOptions::default(), ".").unwrap();
    // Insert into a rapier set so update_mass_properties runs.
    fn principal_xx(robot: MjcfRobot) -> Real {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let mut impulse_joints = ImpulseJointSet::new();
        let handles =
            robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
        // Trigger a step so update_mass_properties fires.
        let mut multibody_joints = MultibodyJointSet::new();
        let gravity = Vector::new(0.0, 0.0, 0.0);
        step_n(
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            1,
            gravity,
        );
        let h = handles.bodies[1].as_ref().unwrap().body;
        let p = bodies
            .get(h)
            .unwrap()
            .mass_properties()
            .local_mprops
            .principal_inertia();
        // Sum to be insensitive to eigenvalue ordering.
        p.x + p.y + p.z
    }
    let arm = principal_xx(rob_arm);
    let no = principal_xx(rob_no);
    assert!(arm > no + 0.4, "arm trace = {arm}, no trace = {no}");
}

#[test]
fn gravcomp_zero_means_no_gravity() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="floater" pos="0 0 5" gravcomp="1">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    assert!((robot.bodies[1].gravcomp - 1.0).abs() < 1e-12);
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let handles =
        robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    let h = handles.bodies[1].as_ref().unwrap().body;
    // gravity_scale should be 0 (1.0 - 1.0).
    assert!((bodies.get(h).unwrap().gravity_scale() - 0.0).abs() < 1e-12);
    // Step and verify the body doesn't fall.
    let z0 = bodies.get(h).unwrap().translation().z;
    step_n(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        100,
        Vector::new(0.0, 0.0, -9.81),
    );
    let z1 = bodies.get(h).unwrap().translation().z;
    assert!((z1 - z0).abs() < 0.01, "z drift = {}", z1 - z0);
}

#[test]
fn multibody_path_routes_damping_through_per_dof_damping() {
    // When inserting through the multibody-joint path, MJCF `<joint damping>`
    // is written into the multibody's per-DoF damping vector rather than
    // the joint motor's damping component (more stable; isotropic for ball
    // joints). Verify that the motor's damping ends up zero after insertion
    // and the multibody's damping vector contains the expected value.
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="a"><joint name="j" type="hinge" axis="0 0 1" damping="0.7"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="box" size="0.1 0.1 0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let handles = robot.insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        &mut impulse_joints,
        rapier3d_mjcf::MjcfMultibodyOptions::empty(),
    );
    let mb_handle = handles.joints[0].joint.expect("multibody joint inserted");
    let (mb, link_id) = multibody_joints.get_mut(mb_handle).unwrap();
    // The hinge's free DoF lives at the end of the multibody's damping
    // vector (after the root's own DoFs).
    let last = mb.damping().len() - 1;
    assert!(
        (mb.damping()[last] - 0.7).abs() < 1e-6,
        "damping[{last}] = {}",
        mb.damping()[last]
    );
    // The hinge's motor damping component should be zero (we moved it).
    let link = mb.link(link_id).unwrap();
    let motor = &link.joint().data.motors[JointAxis::AngX as usize];
    assert!(
        motor.damping.abs() < 1e-6,
        "motor damping = {}",
        motor.damping
    );
}

#[test]
fn disable_joint_motors_still_applies_per_dof_damping() {
    // `disable_joint_motors=true` skips the motor setup (springs and
    // friction-loss go away), but per-DoF damping is *not* a motor — it's
    // a dynamics-level friction term — and should still be applied.
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="a"><joint name="j" type="hinge" axis="0 0 1" damping="0.7"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="box" size="0.1 0.1 0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let opts = MjcfLoaderOptions {
        disable_joint_motors: true,
        ..Default::default()
    };
    let (robot, _) = MjcfRobot::from_str(xml, opts, ".").unwrap();
    // No motor on the joint (the option skipped that setup).
    assert!(robot.joints[0].joint.motor_axes.is_empty());
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let handles = robot.insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        &mut impulse_joints,
        rapier3d_mjcf::MjcfMultibodyOptions::empty(),
    );
    let mb_handle = handles.joints[0].joint.expect("multibody joint inserted");
    let (mb, _) = multibody_joints.get_mut(mb_handle).unwrap();
    // The hinge's free DoF lives at the end of the damping vector; the
    // per-DoF damping should equal the MJCF damping value.
    let last = mb.damping().len() - 1;
    assert!(
        (mb.damping()[last] - 0.7).abs() < 1e-6,
        "damping[{last}] = {}",
        mb.damping()[last]
    );
}

#[test]
fn disable_joint_motors_clears_motor_axes() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="a"><joint name="j" type="hinge" axis="0 0 1" damping="1.0" stiffness="500"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="box" size="0.1 0.1 0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;

    // Default: motor is set up on the joint.
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    assert!(!robot.joints[0].joint.motor_axes.is_empty());

    // disable_joint_motors=true: no motor is set up.
    let opts = MjcfLoaderOptions {
        disable_joint_motors: true,
        ..Default::default()
    };
    let (robot, _) = MjcfRobot::from_str(xml, opts, ".").unwrap();
    assert!(robot.joints[0].joint.motor_axes.is_empty());
}

#[test]
fn settotalmass_rescales_uniformly() {
    let xml = r#"
    <mujoco>
      <compiler settotalmass="10"/>
      <worldbody>
        <body><freejoint/><inertial mass="1" diaginertia="0.1 0.1 0.1"/></body>
        <body><freejoint/><inertial mass="2" diaginertia="0.2 0.2 0.2"/></body>
        <body><freejoint/><inertial mass="2" diaginertia="0.2 0.2 0.2"/></body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let handles =
        robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    step_n(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        1,
        Vector::new(0.0, 0.0, 0.0),
    );
    // Total of 1 + 2 + 2 = 5 → factor of 2x → masses become 2, 4, 4.
    let mass = |i: usize| -> Real {
        bodies
            .get(handles.bodies[i].as_ref().unwrap().body)
            .unwrap()
            .mass()
    };
    let total = mass(1) + mass(2) + mass(3);
    assert!((total - 10.0).abs() < 1e-3, "total = {total}");
}
