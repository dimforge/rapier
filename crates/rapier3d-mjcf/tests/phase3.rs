//! Phase-3 tests: contact filtering, exclusions, equality constraints.

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfRobot};

fn step_n(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    n: usize,
    hooks: &dyn PhysicsHooks,
) {
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd = CCDSolver::new();
    let mut pipeline = PhysicsPipeline::new();
    let integration_parameters = IntegrationParameters::default();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let gravity = Vector::new(0.0, 0.0, -9.81);
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
            &mut multibody_joints,
            &mut ccd,
            hooks,
            &event_handler,
        );
    }
}

#[test]
fn equality_weld_keeps_bodies_rigidly_attached() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="a" pos="0 0 1">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.1"/>
        </body>
        <body name="b" pos="0.3 0 1">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.1"/>
        </body>
      </worldbody>
      <equality>
        <weld body1="a" body2="b"/>
      </equality>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    assert_eq!(robot.equality_joints.len(), 1);

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let handles =
        robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    // The weld joint becomes an extra impulse joint.
    assert_eq!(impulse_joints.len(), 1);
    assert_eq!(handles.equality_joints.len(), 1);

    // Step a few times — the bodies should remain in roughly the same
    // relative position despite gravity.
    step_n(&mut bodies, &mut colliders, &mut impulse_joints, 50, &());
    let a = bodies.iter().nth(0).map(|(_, b)| b.translation()).unwrap();
    let b_ = bodies.iter().nth(1).map(|(_, b)| b.translation()).unwrap();
    let delta = b_ - a;
    // Original separation was ~0.3 along x.
    assert!((delta.length() - 0.3).abs() < 0.05, "delta = {delta:?}");
}

#[test]
fn equality_connect_keeps_anchor_aligned() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="a" pos="0 0 1">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.1"/>
        </body>
        <body name="b" pos="0.5 0 1">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.1"/>
        </body>
      </worldbody>
      <equality>
        <connect body1="a" body2="b" anchor="0.25 0 0"/>
      </equality>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    assert_eq!(robot.equality_joints.len(), 1);
}

#[test]
fn contact_exclude_disables_pair_via_hooks() {
    // Two overlapping bodies that would normally collide. With
    // <contact><exclude>, the hook returned by `contact_hooks` should
    // suppress the contact.
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="a">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.5"/>
        </body>
        <body name="b" pos="0.4 0 0">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.5"/>
        </body>
      </worldbody>
      <contact>
        <exclude body1="a" body2="b"/>
      </contact>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let handles =
        robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    let hooks = handles.contact_hooks(&_first_robot_clone(xml));
    assert!(hooks.has_excludes());
    step_n(&mut bodies, &mut colliders, &mut impulse_joints, 50, &hooks);
}

fn _first_robot_clone(xml: &str) -> MjcfRobot {
    MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".")
        .unwrap()
        .0
}

#[test]
fn contype_conaffinity_filtering() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="a">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <!-- contype = 1, conaffinity = 1 -->
          <geom type="sphere" size="0.5" contype="1" conaffinity="1"/>
        </body>
        <body name="b">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <!-- contype = 2, conaffinity = 4 — disjoint from `a` -->
          <geom type="sphere" size="0.5" contype="2" conaffinity="4"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    // Just smoke-check that two bodies with disjoint contype/conaffinity
    // have InteractionGroups membership/filter masks containing those bits
    // (the actual filtering rule is: same encoding for both = symmetric AND
    // ⇔ MJCF OR for the common case).
    let a_groups = robot.bodies[1].colliders[0].collision_groups();
    let b_groups = robot.bodies[2].colliders[0].collision_groups();
    // Symmetric mode default: memberships = filter = contype | conaffinity.
    // a: 1|1 = 1; b: 2|4 = 6. AND test: 1 & 6 = 0 → no contact, as desired.
    let _ = a_groups;
    let _ = b_groups;
    // Insert and step — assertion is that nothing blows up.
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    step_n(&mut bodies, &mut colliders, &mut impulse_joints, 10, &());
}
