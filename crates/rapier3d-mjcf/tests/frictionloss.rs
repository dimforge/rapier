//! `<joint frictionloss>` must reach the multibody's per-DoF friction vector,
//! where the solver turns it into a box-bounded constraint row, instead of
//! being squeezed into the joint's single motor slot.

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot};

/// A hinge with `frictionloss`, optionally also carrying a passive spring.
fn model(spring: bool) -> String {
    let stiffness = if spring { r#" stiffness="5" "# } else { " " };
    format!(
        r#"
<mujoco>
  <worldbody>
    <body name="link" pos="1 0 0">
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
      <joint name="hinge" type="hinge" axis="0 0 1" pos="-1 0 0"
             frictionloss="3"{stiffness}damping="0"/>
      <geom type="box" size="1 0.05 0.05"/>
    </body>
  </worldbody>
</mujoco>
"#
    )
}

/// Loads `xml` on the multibody path and returns the multibody plus its link.
fn load(xml: &str) -> (MultibodyJointSet, MultibodyJointHandle) {
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let handles = robot.insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        &mut impulse_joints,
        MjcfMultibodyOptions::empty(),
    );
    let handle = handles.joints[0].joint.expect("hinge was not inserted");
    (multibody_joints, handle)
}

#[test]
fn frictionloss_reaches_the_multibody_friction_vector() {
    let (mut set, handle) = load(&model(false));
    let (multibody, link_id) = set.get_mut(handle).unwrap();

    assert!(
        multibody.frictions().iter().any(|v| *v == 3.0),
        "frictionloss should land in the multibody's per-DoF vector, got {:?}",
        multibody.frictions()
    );

    // The impulse-path motor approximation must be cleared, or the joint would
    // carry both it and the constraint rows.
    let link = multibody.links().nth(link_id).unwrap();
    assert!(
        link.joint().data.motor_axes.is_empty(),
        "the frictionloss motor approximation should be cleared on the multibody path"
    );
}

#[test]
fn frictionloss_does_not_delete_a_spring_on_the_impulse_path() {
    // The impulse-joint path has no per-DoF friction vector, so friction stays
    // a motor approximation there and has to share the joint's single motor
    // slot with the `<joint stiffness>` spring. It cannot: `motor_velocity`
    // zeroes the motor's stiffness and damping. The loader now keeps the
    // spring and skips the friction approximation (with a warning) rather than
    // silently deleting the spring.
    let xml = model(true);
    let (robot, _) = MjcfRobot::from_str(&xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let handles =
        robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);

    let joint = &impulse_joints.get(handles.joints[0].joint).unwrap().data;
    let sprung = (0..SPATIAL_DIM).any(|i| joint.motors[i].stiffness > 0.0);
    assert!(
        sprung,
        "the <joint stiffness> spring must survive: {:?}",
        (0..SPATIAL_DIM)
            .map(|i| joint.motors[i].stiffness)
            .collect::<Vec<_>>()
    );
}

#[test]
fn frictionloss_still_approximated_on_the_impulse_path_without_a_spring() {
    // With the slot free, the impulse path keeps the zero-velocity motor capped
    // at `frictionloss` — the best a single motor slot can do.
    let xml = model(false);
    let (robot, _) = MjcfRobot::from_str(&xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let handles =
        robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);

    let joint = &impulse_joints.get(handles.joints[0].joint).unwrap().data;
    let capped = (0..SPATIAL_DIM).any(|i| joint.motors[i].max_force == 3.0);
    assert!(capped, "frictionloss should still cap a motor here");
}

/// A `<joint damping>` with no spring and no actuator must not be left frozen.
///
/// `move_motor_damping_to_multibody` moves the damping into the multibody's
/// per-DoF vector and zeroes the motor's damping. When damping was the motor's
/// only contribution, what remains is a zero-target row with no gains and no
/// force limit: a rigid velocity lock. The loader used to overwrite that motor
/// with the `frictionloss` approximation (capping it at a harmless value), so
/// the lock only became visible once friction stopped claiming the slot: the
/// tendon-driven finger joints of the MJCF shadow hand stopped moving.
#[test]
fn damping_without_a_spring_leaves_no_locking_motor() {
    let xml = r#"
<mujoco>
  <worldbody>
    <body name="link" pos="1 0 0">
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
      <joint name="hinge" type="hinge" axis="0 1 0" pos="-1 0 0"
             damping="0.05" frictionloss="0.01"/>
      <geom type="box" size="1 0.05 0.05"/>
    </body>
  </worldbody>
</mujoco>
"#;
    let (mut set, handle) = load(xml);
    let (multibody, link_id) = set.get_mut(handle).unwrap();

    assert_eq!(
        multibody.damping()[multibody.link(link_id).unwrap().assembly_id()],
        0.05,
        "damping should reach the multibody's per-DoF vector"
    );
    let link = multibody.links().nth(link_id).unwrap();
    assert!(
        link.joint().data.motor_axes.is_empty(),
        "a damping-only motor must be dropped, not left as an unbounded \
         zero-velocity lock: {:?}",
        link.joint().data.motor_axes
    );
}

/// The behavioural half of the check above: the link must swing under gravity.
#[test]
fn damped_joint_still_swings_under_gravity() {
    let xml = r#"
<mujoco>
  <worldbody>
    <body name="link" pos="1 0 0">
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
      <joint name="hinge" type="hinge" axis="0 1 0" pos="-1 0 0"
             damping="0.05" frictionloss="0.01"/>
      <geom type="box" size="1 0.05 0.05"/>
    </body>
  </worldbody>
</mujoco>
"#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let handles = robot.insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        &mut impulse_joints,
        MjcfMultibodyOptions::empty(),
    );
    let handle = handles.joints[0].joint.expect("hinge was not inserted");

    let mut pipeline = PhysicsPipeline::new();
    let params = IntegrationParameters::default();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut ccd = CCDSolver::new();

    for _ in 0..60 {
        pipeline.step(
            Vector::new(0.0, 0.0, -9.81),
            &params,
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

    let (multibody, link_id) = multibody_joints.get(handle).unwrap();
    let angle = multibody.link(link_id).unwrap().joint().coords()[3];
    assert!(
        angle.abs() > 0.1,
        "the damped hinge should swing under gravity, got {angle}"
    );
}
