//! URDF `<origin rpy="r p y">` angles are fixed-axis roll-pitch-yaw: the
//! rotation is `Rz(yaw) * Ry(pitch) * Rx(roll)`. The loader used to compose
//! them as intrinsic XYZ (`Rx * Ry * Rz`), which only agrees when at most one
//! angle is non-zero; the SO-101's `shoulder_lift` origin (`rpy="-1.5708
//! -1.5708 0"`) put every downstream link in the wrong place.

use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfRobot};
use std::path::Path;

const URDF: &str = r#"<?xml version="1.0"?>
<robot name="rpy">
  <link name="base">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="arm">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision>
      <origin xyz="0.1 0.2 0.3" rpy="0.4 0.5 0.6"/>
      <geometry><box size="0.1 0.1 0.1"/></geometry>
    </collision>
  </link>
  <link name="tip">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="lift" type="revolute">
    <origin xyz="-0.03 -0.018 -0.054" rpy="-1.5708 -1.5708 0"/>
    <parent link="base"/>
    <child link="arm"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.7" upper="1.7" effort="10" velocity="10"/>
  </joint>
  <joint name="flex" type="revolute">
    <origin xyz="-0.11 -0.028 0" rpy="0 0 1.5708"/>
    <parent link="arm"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.7" upper="1.7" effort="10" velocity="10"/>
  </joint>
</robot>
"#;

/// `Rz(yaw) * Ry(pitch) * Rx(roll)`, the URDF specification's convention.
fn urdf_rotation(rpy: [f32; 3]) -> Rotation {
    Rotation::from_rotation_z(rpy[2])
        * Rotation::from_rotation_y(rpy[1])
        * Rotation::from_rotation_x(rpy[0])
}

fn assert_pose_eq(actual: &Pose, expected: &Pose, what: &str) {
    assert!(
        (actual.translation - expected.translation).length() < 1.0e-5,
        "{what}: translation {:?} != {:?}",
        actual.translation,
        expected.translation
    );
    assert!(
        actual.rotation.angle_between(expected.rotation) < 1.0e-4,
        "{what}: rotation {:?} != {:?}",
        actual.rotation,
        expected.rotation
    );
}

#[test]
fn link_poses_follow_fixed_axis_rpy() {
    let (robot, _) = UrdfRobot::from_str(URDF, UrdfLoaderOptions::default(), Path::new("."))
        .expect("URDF parses");
    let lift = Pose::from_parts(
        Vector::new(-0.03, -0.018, -0.054),
        urdf_rotation([-1.5708, -1.5708, 0.0]),
    );
    let flex = Pose::from_parts(
        Vector::new(-0.11, -0.028, 0.0),
        urdf_rotation([0.0, 0.0, 1.5708]),
    );
    assert_pose_eq(robot.links[1].body.position(), &lift, "arm link");
    assert_pose_eq(robot.links[2].body.position(), &(lift * flex), "tip link");
}

#[test]
fn collider_origin_follows_fixed_axis_rpy() {
    let (robot, _) = UrdfRobot::from_str(URDF, UrdfLoaderOptions::default(), Path::new("."))
        .expect("URDF parses");
    let expected = Pose::from_parts(Vector::new(0.1, 0.2, 0.3), urdf_rotation([0.4, 0.5, 0.6]));
    let collider = &robot.links[1].colliders[0].collider;
    assert_pose_eq(collider.position(), &expected, "arm collider");
}

#[test]
fn multibody_forward_kinematics_matches_the_origins() {
    let (robot, _) = UrdfRobot::from_str(URDF, UrdfLoaderOptions::default(), Path::new("."))
        .expect("URDF parses");
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let handles = robot.insert_using_multibody_joints(
        &mut bodies,
        &mut colliders,
        &mut multibody_joints,
        rapier3d_urdf::UrdfMultibodyOptions::empty(),
    );
    let tip = handles.links[2].body;
    let link = *multibody_joints
        .rigid_body_link(tip)
        .expect("tip is a multibody link");
    let mb = multibody_joints.get_multibody_mut(link.multibody).unwrap();
    // Link poses are derived from the joint frames by forward kinematics.
    mb.forward_kinematics(&bodies, false);
    let lift = Pose::from_parts(
        Vector::new(-0.03, -0.018, -0.054),
        urdf_rotation([-1.5708, -1.5708, 0.0]),
    );
    let flex = Pose::from_parts(
        Vector::new(-0.11, -0.028, 0.0),
        urdf_rotation([0.0, 0.0, 1.5708]),
    );
    assert_pose_eq(
        mb.link(link.id).unwrap().local_to_world(),
        &(lift * flex),
        "tip FK",
    );
}
