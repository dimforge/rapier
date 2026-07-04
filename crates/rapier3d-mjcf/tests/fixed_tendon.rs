//! Fixed-tendon support: parsing, actuator binding to the tendon's primary
//! joint, and the co-actuation coupling that drags the other joints along.

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot};

// Two serial hinges joined by a fixed tendon `j1 + j2`, driven by one position
// actuator on the tendon — the shadow-hand finger pattern in miniature.
const TENDON_ARM: &str = r#"
<mujoco>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03"/>
      <body name="b2" pos="0.3 0 0">
        <joint name="j2" type="hinge" axis="0 1 0"/>
        <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
        <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t">
      <joint joint="j1" coef="1"/>
      <joint joint="j2" coef="1"/>
    </fixed>
  </tendon>
  <actuator>
    <position name="a" tendon="t" kp="50"/>
  </actuator>
</mujoco>
"#;

#[test]
fn fixed_tendon_parses_and_binds() {
    let (robot, model) =
        MjcfRobot::from_str(TENDON_ARM, MjcfLoaderOptions::default(), ".").unwrap();

    // Parser kept the fixed tendon with its two terms.
    assert_eq!(model.tendons.len(), 1);
    assert_eq!(model.tendons[0].name.as_deref(), Some("t"));
    assert_eq!(model.tendons[0].joints.len(), 2);

    // The tendon actuator binds to the tendon's primary (first) joint, `j1`.
    let j1 = robot.joint_name_to_idx["j1"];
    let j2 = robot.joint_name_to_idx["j2"];
    assert_eq!(robot.actuators.len(), 1);
    assert_eq!(robot.actuators[0].joint_index, Some(j1));

    // A co-actuation coupling was added between the tendon's joints (coeff 1).
    let coupling = robot
        .joint_couplings
        .iter()
        .find(|c| (c.joint1 == j1 && c.joint2 == j2) || (c.joint1 == j2 && c.joint2 == j1))
        .expect("expected a co-actuation coupling between j1 and j2");
    assert!((coupling.coeff - 1.0).abs() < 1e-6);
}

fn hinge_angle(rb: &RigidBody) -> f32 {
    let x = rb.rotation() * Vector::X;
    (-x.z).atan2(x.x)
}

#[test]
fn fixed_tendon_actuator_drives_both_joints() {
    let (robot, _) = MjcfRobot::from_str(TENDON_ARM, MjcfLoaderOptions::default(), ".").unwrap();

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

    let b1 = robot.body_name_to_idx["b1"];
    let b2 = robot.body_name_to_idx["b2"];

    // Drive the single tendon actuator toward 0.5.
    let ctrl = [0.5_f32];
    let mut pipeline = PhysicsPipeline::new();
    let ip = IntegrationParameters::default();
    let (mut isl, mut bp, mut np, mut ccd) = (
        IslandManager::new(),
        DefaultBroadPhase::new(),
        NarrowPhase::new(),
        CCDSolver::new(),
    );
    for _ in 0..400 {
        handles.apply_controls_multibody(&mut bodies, &mut mbj, &ctrl);
        pipeline.step(
            Vector::new(0.0, 0.0, -9.81),
            &ip,
            &mut isl,
            &mut bp,
            &mut np,
            &mut bodies,
            &mut colliders,
            &mut ij,
            &mut mbj,
            &mut ccd,
            &(),
            &(),
        );
    }

    // j1 is actuated (proximal world angle ≈ 0.5); j2 follows via the coupling,
    // so the distal body's *world* angle is ≈ j1 + j2 ≈ 1.0.
    let a1 = hinge_angle(
        bodies
            .get(handles.bodies[b1].as_ref().unwrap().body)
            .unwrap(),
    );
    let a2_world = hinge_angle(
        bodies
            .get(handles.bodies[b2].as_ref().unwrap().body)
            .unwrap(),
    );
    let j2 = a2_world - a1; // relative (coupled) angle
    assert!(
        (a1 - 0.5).abs() < 0.05,
        "proximal didn't reach target: {a1}"
    );
    assert!(
        (j2 - 0.5).abs() < 0.05,
        "distal didn't follow via coupling: j2 = {j2}"
    );
}

/// The real shadow_hand: its four `*J0` finger tendons should now bind their
/// actuators and couple each finger's middle+distal segments. Skipped without
/// the menagerie checkout.
#[test]
fn shadow_hand_fixed_tendons_bind_and_couple() {
    let scene = "../../../mujoco_menagerie/shadow_hand/scene_right.xml";
    if !std::path::Path::new(scene).exists() {
        eprintln!("mujoco_menagerie not found; skipping");
        return;
    }
    let (robot, model) = MjcfRobot::from_file(scene, MjcfLoaderOptions::default()).unwrap();
    // Four fixed tendons (FF/MF/RF/LF J0), all two-joint, equal-coef.
    assert_eq!(model.tendons.len(), 4);

    // The four `rh_A_*J0` tendon actuators now bind (previously unbound).
    let bound = robot
        .actuators
        .iter()
        .filter(|a| a.joint_index.is_some())
        .count();
    assert_eq!(
        bound, 20,
        "all 20 actuators (incl. the 4 tendon ones) should bind"
    );

    // Each tendon added a coupling between its middle and distal joints —
    // FFJ1 (distal) follows FFJ2 (middle).
    let ffj2 = robot.joint_name_to_idx["rh_FFJ2"];
    let ffj1 = robot.joint_name_to_idx["rh_FFJ1"];
    assert!(
        robot.joint_couplings.iter().any(|c| {
            (c.joint1 == ffj2 && c.joint2 == ffj1) || (c.joint1 == ffj1 && c.joint2 == ffj2)
        }),
        "expected a coupling between rh_FFJ1 and rh_FFJ2"
    );
}
