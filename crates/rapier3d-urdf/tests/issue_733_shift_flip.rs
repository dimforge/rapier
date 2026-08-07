//! Regression test for https://github.com/dimforge/rapier/issues/733
//!
//! Loading a URDF robot with a flipped axis shift (the Z-up to Y-up conversion) used to make
//! the impulse-joint version misbehave, with links flying away from their anchors. Whatever
//! global shift is applied, the robot must stay stable and its joint anchors coincide.

use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfRobot};
use std::path::Path;

const URDF: &str = r#"<?xml version="1.0"?>
<robot name="issue_733">
  <link name="base">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0.5 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision>
      <origin xyz="0.5 0 0"/>
      <geometry><box size="1.0 0.1 0.1"/></geometry>
    </collision>
  </link>
  <joint name="hinge" type="revolute">
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="10"/>
  </joint>
</robot>
"#;

/// Loads the robot with the given shift, inserts it with impulse joints, steps
/// it, and checks that the simulation stays finite, bounded, and that every
/// joint's two anchors coincide.
fn check_shift(shift: Pose) {
    let options = UrdfLoaderOptions {
        make_roots_fixed: true,
        shift,
        ..Default::default()
    };
    let (robot, _) = UrdfRobot::from_str(URDF, options, Path::new("./")).unwrap();

    let mut world = PhysicsWorld::new();
    robot.insert_using_impulse_joints(
        &mut world.bodies,
        &mut world.colliders,
        &mut world.impulse_joints,
    );

    for _ in 0..100 {
        world.step();

        for (_, rb) in world.rigid_bodies() {
            assert!(
                rb.translation().is_finite(),
                "non-finite body position with shift {shift:?}"
            );
            assert!(
                rb.translation().length() < 10.0,
                "body flew away with shift {shift:?}: {:?}",
                rb.translation()
            );
        }
    }

    let mut num_joints = 0;
    for (_, joint) in world.impulse_joints.iter() {
        let rb1 = &world.bodies[joint.body1()];
        let rb2 = &world.bodies[joint.body2()];
        let anchor1 = *rb1.position() * joint.data.local_frame1.translation;
        let anchor2 = *rb2.position() * joint.data.local_frame2.translation;
        assert!(
            (anchor1 - anchor2).length() < 1e-2,
            "joint anchors diverged with shift {shift:?}: {anchor1:?} vs {anchor2:?}"
        );
        num_joints += 1;
    }
    assert_eq!(num_joints, 1);
}

#[test]
fn urdf_shift_flip_impulse_joints_stay_stable() {
    check_shift(Pose::IDENTITY);
    // Z-up to Y-up.
    check_shift(Pose::rotation(Vector::X * std::f32::consts::FRAC_PI_2));
    // The flipped variant from the issue.
    check_shift(Pose::rotation(Vector::X * -std::f32::consts::FRAC_PI_2));
}
