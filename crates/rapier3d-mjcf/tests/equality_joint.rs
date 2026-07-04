//! `<equality><joint>` joint-coupling constraint (the robotiq gripper's two
//! driver joints moving together). The loader parses the polynomial coupling,
//! records its linear form, and installs it as a multibody DoF coupling.

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot};

// `base` is welded to the world (fixed); `arm1`/`arm2` are sibling hinges about
// Z, coupled `q2 = 2·q1`. A position actuator drives `q1`.
const XML: &str = r#"
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="base">
      <geom type="box" size="0.1 0.1 0.1"/>
      <body name="arm1" pos="0 0.2 0">
        <joint name="j1" type="hinge" axis="0 0 1"/>
        <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
      </body>
      <body name="arm2" pos="0 -0.2 0">
        <joint name="j2" type="hinge" axis="0 0 1"/>
        <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
      </body>
    </body>
  </worldbody>
  <equality>
    <joint joint1="j1" joint2="j2" polycoef="0 2 0 0 0"/>
  </equality>
  <actuator>
    <position name="drive1" joint="j1" kp="30"/>
  </actuator>
</mujoco>
"#;

#[test]
fn equality_joint_parses_to_linear_coupling() {
    let (robot, _) = MjcfRobot::from_str(XML, MjcfLoaderOptions::default(), ".").unwrap();
    assert_eq!(robot.joint_couplings.len(), 1);
    let c = &robot.joint_couplings[0];
    assert_eq!(robot.joints[c.joint1].name.as_deref(), Some("j1"));
    assert_eq!(robot.joints[c.joint2].name.as_deref(), Some("j2"));
    assert_eq!(c.coeff, 2.0); // polycoef[1]
    assert_eq!(c.offset, 0.0); // polycoef[0]
    assert!(c.active);
}

#[test]
fn equality_joint_couples_multibody_joints() {
    let (robot, _) = MjcfRobot::from_str(XML, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let handles = robot.clone().insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        &mut impulse_joints,
        MjcfMultibodyOptions::empty(),
    );

    // Handles of the two driver joints, in `robot.joints` order.
    let h1 = handles.joints[robot.joint_couplings[0].joint1]
        .joint
        .unwrap();
    let h2 = handles.joints[robot.joint_couplings[0].joint2]
        .joint
        .unwrap();

    // Read a hinge's generalized coordinate (AngX = spatial axis 3).
    let read_q = |mbj: &MultibodyJointSet, h| {
        let (mb, link_id) = mbj.get(h).unwrap();
        mb.links().nth(link_id).unwrap().joint().coords()[3]
    };

    let ip = IntegrationParameters::default();
    let mut pipeline = PhysicsPipeline::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut ccd = CCDSolver::new();
    // Drive j1 toward 0.3; the coupling must keep j2 = 2·j1.
    for _ in 0..400 {
        handles.apply_controls_multibody(&mut bodies, &mut multibody_joints, &[0.3]);
        pipeline.step(
            Vector::ZERO,
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

    let q1 = read_q(&multibody_joints, h1);
    let q2 = read_q(&multibody_joints, h2);
    assert!((q1 - 0.3).abs() < 0.05, "driven joint q1 = {q1}, expected ~0.3");
    assert!(
        (q2 - 2.0 * q1).abs() < 0.02,
        "coupling q2 = 2·q1 not enforced: q1 = {q1}, q2 = {q2}"
    );
}
