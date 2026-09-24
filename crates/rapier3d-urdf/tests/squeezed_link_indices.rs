//! Checks that the links and joints of an `UrdfRobot` can be matched with the original URDF
//! links and joints after empty links were squeezed.

use rapier3d::prelude::*;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfRobot};
use std::path::Path;

// `world` is an empty root, `mount_a` and `mount_b` are empty links between `base` and `arm`,
// and `tcp` is an empty leaf. Only `base` and `arm` remain after squeezing.
const URDF: &str = r#"<?xml version="1.0"?>
<robot name="squeeze">
  <link name="world"/>
  <link name="base">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="mount_a"/>
  <link name="mount_b"/>
  <link name="arm">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision>
      <geometry><box size="1.0 0.1 0.1"/></geometry>
    </collision>
  </link>
  <link name="tcp"/>
  <joint name="world_joint" type="fixed">
    <parent link="world"/>
    <child link="base"/>
  </joint>
  <joint name="hinge" type="revolute">
    <origin xyz="0 0 1"/>
    <parent link="base"/>
    <child link="mount_a"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="10"/>
  </joint>
  <joint name="mount_a_joint" type="fixed">
    <origin xyz="0 0 0.5"/>
    <parent link="mount_a"/>
    <child link="mount_b"/>
  </joint>
  <joint name="mount_b_joint" type="fixed">
    <origin xyz="0.5 0 0"/>
    <parent link="mount_b"/>
    <child link="arm"/>
  </joint>
  <joint name="tcp_joint" type="fixed">
    <origin xyz="0.5 0 0"/>
    <parent link="arm"/>
    <child link="tcp"/>
  </joint>
</robot>
"#;

#[test]
fn squeezed_links_and_joints_map_to_urdf_indices() {
    let (robot, urdf) =
        UrdfRobot::from_str(URDF, UrdfLoaderOptions::default(), Path::new("./")).unwrap();

    let link_names: Vec<_> = robot
        .links
        .iter()
        .map(|l| urdf.links[l.urdf_link_index].name.as_str())
        .collect();
    assert_eq!(link_names, ["base", "arm"]);
    // The empty root was removed, so `base` is anchored to the world.
    assert!(robot.links[0].body.is_fixed());
    assert_eq!(robot.links[1].colliders.len(), 1);

    assert_eq!(robot.joints.len(), 1);
    let joint = &robot.joints[0];
    assert_eq!((joint.link1, joint.link2), (0, 1));
    assert_eq!(urdf.joints[joint.urdf_joint_index].name, "mount_b_joint");
    let merged: Vec<_> = joint
        .merged_urdf_joint_indices
        .iter()
        .map(|i| urdf.joints[*i].name.as_str())
        .collect();
    assert_eq!(merged, ["mount_a_joint", "hinge"]);
    assert_eq!(urdf.joints[joint.source_urdf_joint_index()].name, "hinge");
    assert_eq!(joint.joint.locked_axes, JointAxesMask::LOCKED_REVOLUTE_AXES);

    // The arm ends up at the same place as without squeezing.
    let arm_pos = robot.links[1].body.translation();
    assert!((arm_pos - Vector::new(0.5, 0.0, 1.5)).length() < 1.0e-5);
    // The hinge still pivots around the origin of `mount_a`, not around the arm's origin.
    let pivot = Vector::new(0.0, 0.0, 1.0);
    assert!((joint.joint.local_frame1.translation - pivot).length() < 1.0e-5);
    let pivot_in_arm = robot.links[1].body.position().inverse() * pivot;
    assert!((joint.joint.local_frame2.translation - pivot_in_arm).length() < 1.0e-5);
}

#[test]
fn unsqueezed_links_and_joints_map_to_urdf_indices() {
    let options = UrdfLoaderOptions {
        squeeze_empty_fixed_links: false,
        ..Default::default()
    };
    let (robot, urdf) = UrdfRobot::from_str(URDF, options, Path::new("./")).unwrap();

    assert_eq!(robot.links.len(), urdf.links.len());
    assert_eq!(robot.joints.len(), urdf.joints.len());
    for (i, link) in robot.links.iter().enumerate() {
        assert_eq!(link.urdf_link_index, i);
    }
    for (i, joint) in robot.joints.iter().enumerate() {
        assert_eq!(joint.urdf_joint_index, i);
        assert!(joint.merged_urdf_joint_indices.is_empty());
        assert_eq!(joint.source_urdf_joint_index(), i);
    }
}
