//! Checks that the children attached by fixed joints to a squeezed empty link keep moving
//! together, driven by the single moving joint of the empty link.

use rapier3d::prelude::*;
use rapier3d_urdf::urdf_rs::Robot;
use rapier3d_urdf::{UrdfLoaderOptions, UrdfMultibodyOptions, UrdfRobot};
use std::collections::HashMap;
use std::path::Path;

fn link(name: &str, radius: f32) -> String {
    format!(
        r#"<link name="{name}">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision>
      <geometry><sphere radius="{radius}"/></geometry>
    </collision>
  </link>"#
    )
}

/// `base -(hinge)-> mount -(fixed)-> a` and `mount -(fixed)-> b`, where `mount` is empty.
///
/// If `with_moving_child` is set, `mount -(wrist)-> c` is added too. It is listed first so the
/// joints don't follow the order in which the links have to be placed after squeezing.
fn urdf(with_moving_child: bool) -> String {
    let (c_link, c_joint) = if with_moving_child {
        (
            link("c", 0.05),
            r#"<joint name="wrist" type="revolute">
    <origin xyz="0 -0.3 0" rpy="0 0 0.4"/>
    <parent link="mount"/>
    <child link="c"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="10"/>
  </joint>"#,
        )
    } else {
        (String::new(), "")
    };
    format!(
        r#"<?xml version="1.0"?>
<robot name="squeeze">
  {base}
  <link name="mount"/>
  {a}
  {b}
  {c_link}
  {c_joint}
  <joint name="b_joint" type="fixed">
    <origin xyz="0 0.2 0.3" rpy="0.2 0 0.3"/>
    <parent link="mount"/>
    <child link="b"/>
  </joint>
  <joint name="hinge" type="revolute">
    <origin xyz="0 0 1" rpy="0 0 0.5"/>
    <parent link="base"/>
    <child link="mount"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="10"/>
  </joint>
  <joint name="a_joint" type="fixed">
    <origin xyz="0.5 0 0" rpy="0 0.3 0"/>
    <parent link="mount"/>
    <child link="a"/>
  </joint>
</robot>
"#,
        base = link("base", 0.05),
        a = link("a", 0.05),
        b = link("b", 0.05),
    )
}

fn load(urdf: &str, squeeze: bool) -> (UrdfRobot, Robot) {
    let options = UrdfLoaderOptions {
        make_roots_fixed: true,
        squeeze_empty_fixed_links: squeeze,
        ..Default::default()
    };
    UrdfRobot::from_str(urdf, options, Path::new("./")).unwrap()
}

fn link_names(robot: &UrdfRobot, urdf: &Robot) -> Vec<String> {
    robot
        .links
        .iter()
        .map(|l| urdf.links[l.urdf_link_index].name.clone())
        .collect()
}

fn joint_names(urdf: &Robot, indices: &[usize]) -> Vec<String> {
    indices
        .iter()
        .map(|i| urdf.joints[*i].name.clone())
        .collect()
}

/// Simulates the robot under gravity and returns the final pose of each link, by name.
fn simulate(urdf: &str, squeeze: bool, multibody: bool) -> HashMap<String, Pose> {
    let (robot, raw) = load(urdf, squeeze);
    let names = link_names(&robot, &raw);
    let mut world = PhysicsWorld::new();
    let links = if multibody {
        robot
            .insert_using_multibody_joints(
                &mut world.bodies,
                &mut world.colliders,
                &mut world.multibody_joints,
                UrdfMultibodyOptions::DISABLE_SELF_CONTACTS,
            )
            .links
    } else {
        robot
            .insert_using_impulse_joints(
                &mut world.bodies,
                &mut world.colliders,
                &mut world.impulse_joints,
            )
            .links
    };

    for _ in 0..60 {
        world.step();
    }

    // Every impulse joint keeps its anchors together (up to the drift of the iterative solver).
    for (_, joint) in world.impulse_joints.iter() {
        let anchor1 = *world.bodies[joint.body1()].position() * joint.data.local_frame1;
        let anchor2 = *world.bodies[joint.body2()].position() * joint.data.local_frame2;
        assert!(
            (anchor1.translation - anchor2.translation).length() < 5.0e-2,
            "joint anchors diverged: {anchor1:?} vs {anchor2:?}"
        );
    }

    names
        .into_iter()
        .zip(links)
        .map(|(name, link)| (name, *world.bodies[link.body].position()))
        .collect()
}

fn assert_poses_eq(pose1: &Pose, pose2: &Pose, eps: f32) {
    assert!(
        (pose1.translation - pose2.translation).length() < eps,
        "{pose1:?} != {pose2:?}"
    );
    // Twice the sine of the half-angle, more accurate than `angle_between` for small angles.
    let angle = (pose1.rotation.inverse() * pose2.rotation).xyz().length() * 2.0;
    assert!(angle < eps, "{pose1:?} != {pose2:?}");
}

#[test]
fn fixed_children_of_squeezed_link_share_its_joint() {
    let (robot, raw) = load(&urdf(false), true);
    assert_eq!(link_names(&robot, &raw), ["base", "a", "b"]);

    // `b` (the first non-empty fixed child in joint order) takes over the hinge, and `a` is
    // attached to it.
    assert_eq!(robot.joints.len(), 2);
    let b_joint = &robot.joints[0];
    assert_eq!(raw.joints[b_joint.urdf_joint_index].name, "b_joint");
    assert_eq!((b_joint.link1, b_joint.link2), (0, 2));
    assert_eq!(
        joint_names(&raw, &b_joint.merged_urdf_joint_indices),
        ["hinge"]
    );
    assert_eq!(raw.joints[b_joint.source_urdf_joint_index()].name, "hinge");
    assert_eq!(
        b_joint.joint.locked_axes,
        JointAxesMask::LOCKED_REVOLUTE_AXES
    );

    let a_joint = &robot.joints[1];
    assert_eq!(raw.joints[a_joint.urdf_joint_index].name, "a_joint");
    assert_eq!((a_joint.link1, a_joint.link2), (2, 1));
    assert!(a_joint.merged_urdf_joint_indices.is_empty());
    assert_eq!(a_joint.source_urdf_joint_index(), a_joint.urdf_joint_index);
    assert_eq!(a_joint.joint.locked_axes, JointAxesMask::LOCKED_FIXED_AXES);

    // The hinge still pivots around the origin of `mount`.
    assert!(
        (b_joint.joint.local_frame1.translation - Vector::new(0.0, 0.0, 1.0)).length() < 1.0e-5
    );

    // The links are placed as without squeezing.
    let (unsqueezed, _) = load(&urdf(false), false);
    for (i, link) in robot.links.iter().enumerate() {
        let expected = unsqueezed.links[link.urdf_link_index].body.position();
        assert_poses_eq(link.body.position(), expected, 1.0e-5);
        assert_eq!(link.urdf_link_index, [0, 2, 3][i]);
    }
}

#[test]
fn moving_child_of_squeezed_link_is_attached_to_the_representative() {
    let (robot, raw) = load(&urdf(true), true);
    assert_eq!(link_names(&robot, &raw), ["base", "a", "b", "c"]);
    assert_eq!(robot.joints.len(), 3);

    let wrist = &robot.joints[0];
    assert_eq!(raw.joints[wrist.urdf_joint_index].name, "wrist");
    assert_eq!((wrist.link1, wrist.link2), (2, 3));
    assert!(wrist.merged_urdf_joint_indices.is_empty());
    assert_eq!(wrist.joint.locked_axes, JointAxesMask::LOCKED_REVOLUTE_AXES);

    // Each URDF joint appears at most once in the mapping.
    let mut all: Vec<_> = robot
        .joints
        .iter()
        .flat_map(|j| {
            std::iter::once(j.urdf_joint_index).chain(j.merged_urdf_joint_indices.clone())
        })
        .collect();
    all.sort();
    assert_eq!(all, [0, 1, 2, 3]);

    let (unsqueezed, _) = load(&urdf(true), false);
    for link in &robot.links {
        let expected = unsqueezed.links[link.urdf_link_index].body.position();
        assert_poses_eq(link.body.position(), expected, 1.0e-5);
    }
}

#[test]
fn fixed_children_of_squeezed_link_move_together() {
    for with_moving_child in [false, true] {
        let urdf = urdf(with_moving_child);
        let (robot, raw) = load(&urdf, true);
        let initial: HashMap<_, _> = link_names(&robot, &raw)
            .into_iter()
            .zip(robot.links.iter().map(|l| *l.body.position()))
            .collect();
        let initial_rel = initial["a"].inverse() * initial["b"];

        for multibody in [false, true] {
            let poses = simulate(&urdf, true, multibody);
            // Impulse joints are solved iteratively, so they drift a bit.
            let eps = if multibody { 1.0e-3 } else { 5.0e-2 };

            // The hinge actually moved `a`.
            let swing = poses["a"].rotation.angle_between(initial["a"].rotation);
            assert!(
                swing > 0.1,
                "the hinge didn't move (multibody: {multibody})"
            );
            // `a` and `b` stayed rigidly attached.
            let rel = poses["a"].inverse() * poses["b"];
            assert_poses_eq(&rel, &initial_rel, eps);
            // `a` still rotates around the hinge's pivot, i.e., the origin of `mount`.
            let pivot = Vector::new(0.0, 0.0, 1.0);
            let dist = (poses["a"].translation - pivot).length();
            assert!((dist - 0.5).abs() < eps, "wrong pivot: {dist}");

            if multibody {
                // Same motion as the unsqueezed robot, where `mount` is a massless body.
                let expected = simulate(&urdf, false, true);
                for (name, pose) in &poses {
                    assert_poses_eq(pose, &expected[name], 1.0e-3);
                }
            }
        }
    }
}

fn joint(name: &str, ty: &str, parent: &str, child: &str, origin: &str) -> String {
    format!(
        r#"<joint name="{name}" type="{ty}">
    <origin {origin}/>
    <parent link="{parent}"/>
    <child link="{child}"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.0" upper="3.0" effort="10" velocity="10"/>
  </joint>"#
    )
}

/// Checks that the squeezed robot starts at the same poses and moves like the unsqueezed one,
/// and that each URDF joint is mapped at most once.
fn check_squeeze_equivalence(urdf: &str) -> (UrdfRobot, Robot) {
    let (robot, raw) = load(urdf, true);
    let (unsqueezed, _) = load(urdf, false);
    for link in &robot.links {
        let expected = unsqueezed.links[link.urdf_link_index].body.position();
        assert_poses_eq(link.body.position(), expected, 1.0e-5);
    }

    let mut all: Vec<_> = robot
        .joints
        .iter()
        .flat_map(|j| {
            std::iter::once(j.urdf_joint_index).chain(j.merged_urdf_joint_indices.clone())
        })
        .collect();
    all.sort();
    all.dedup();
    let num_mapped: usize = robot
        .joints
        .iter()
        .map(|j| 1 + j.merged_urdf_joint_indices.len())
        .sum();
    assert_eq!(all.len(), num_mapped, "a URDF joint is mapped twice");

    let poses = simulate(urdf, true, true);
    let expected = simulate(urdf, false, true);
    for (name, pose) in &poses {
        assert_poses_eq(pose, &expected[name], 1.0e-3);
    }
    (robot, raw)
}

#[test]
fn chain_of_squeezed_links_keeps_fixed_children_together() {
    // `m1` and `m2` are empty: `base -(hinge)-> m1 -(fixed)-> m2 -(fixed)-> {a, b}` and
    // `m1 -(fixed)-> d`.
    let urdf = format!(
        r#"<?xml version="1.0"?>
<robot name="chain">
  {base}
  <link name="m1"/>
  <link name="m2"/>
  {a}
  {b}
  {d}
  {hinge}
  {m1_m2}
  {a_joint}
  {b_joint}
  {d_joint}
</robot>"#,
        base = link("base", 0.05),
        a = link("a", 0.05),
        b = link("b", 0.05),
        d = link("d", 0.05),
        hinge = joint(
            "hinge",
            "revolute",
            "base",
            "m1",
            r#"xyz="0 0 1" rpy="0.3 0 0""#
        ),
        m1_m2 = joint(
            "m1_m2",
            "fixed",
            "m1",
            "m2",
            r#"xyz="0.4 0 0" rpy="0 0 0.7""#
        ),
        a_joint = joint("a_joint", "fixed", "m2", "a", r#"xyz="0.3 0 0""#),
        b_joint = joint("b_joint", "fixed", "m2", "b", r#"xyz="0 0.3 0.1""#),
        d_joint = joint("d_joint", "fixed", "m1", "d", r#"xyz="-0.3 0 0""#),
    );
    let (robot, raw) = check_squeeze_equivalence(&urdf);
    assert_eq!(link_names(&robot, &raw), ["base", "a", "b", "d"]);

    let joints: Vec<_> = robot
        .joints
        .iter()
        .map(|j| {
            (
                raw.joints[j.urdf_joint_index].name.as_str(),
                j.link1,
                j.link2,
                joint_names(&raw, &j.merged_urdf_joint_indices),
            )
        })
        .collect();
    // `d` (non-empty) is preferred over `m2` to take over the hinge.
    assert_eq!(
        joints,
        [
            ("a_joint", 3, 1, vec!["m1_m2".to_string()]),
            ("b_joint", 3, 2, vec![]),
            ("d_joint", 0, 3, vec!["hinge".to_string()]),
        ]
    );
}

#[test]
fn moving_child_of_fixed_empty_link_is_attached_to_its_parent() {
    // `base -(fixed)-> m -(fixed)-> a` and `m -(wrist)-> c`, where `m` is empty.
    let urdf = format!(
        r#"<?xml version="1.0"?>
<robot name="fixed_parent">
  {base}
  <link name="m"/>
  {a}
  {c}
  {wrist}
  {m_joint}
  {a_joint}
</robot>"#,
        base = link("base", 0.05),
        a = link("a", 0.05),
        c = link("c", 0.05),
        wrist = joint(
            "wrist",
            "revolute",
            "m",
            "c",
            r#"xyz="0.4 0 0" rpy="0.5 0 0""#
        ),
        m_joint = joint(
            "m_joint",
            "fixed",
            "base",
            "m",
            r#"xyz="0 0 1" rpy="0 0.4 0""#
        ),
        a_joint = joint("a_joint", "fixed", "m", "a", r#"xyz="0 0.3 0""#),
    );
    let (robot, raw) = check_squeeze_equivalence(&urdf);
    assert_eq!(link_names(&robot, &raw), ["base", "a", "c"]);

    let joints: Vec<_> = robot
        .joints
        .iter()
        .map(|j| {
            (
                raw.joints[j.urdf_joint_index].name.as_str(),
                j.link1,
                j.link2,
                joint_names(&raw, &j.merged_urdf_joint_indices),
            )
        })
        .collect();
    assert_eq!(
        joints,
        [
            ("wrist", 0, 2, vec![]),
            ("a_joint", 0, 1, vec!["m_joint".to_string()]),
        ]
    );
    assert_eq!(
        robot.joints[0].joint.locked_axes,
        JointAxesMask::LOCKED_REVOLUTE_AXES
    );
}
