//! Phase-2 tests: defaults, includes, frame folding, inertiafromgeom.

use rapier3d::dynamics::{ImpulseJointSet, RigidBodySet};
use rapier3d::geometry::ColliderSet;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfRobot};
use std::fs;
use tempfile::tempdir;

#[test]
fn defaults_baked_into_joints() {
    let xml = r#"
    <mujoco>
      <default>
        <joint damping="0.5" type="hinge"/>
        <default class="leg">
          <joint damping="0.3" range="-30 30"/>
        </default>
      </default>
      <worldbody>
        <body name="b">
          <joint name="j1"/>
          <joint name="j2" class="leg"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="box" size="0.1 0.1 0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, model) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();

    // The parser baked the defaults into the AST.
    let body = &model.bodies[1].body;
    assert_eq!(body.joints[0].damping, 0.5);
    assert_eq!(body.joints[1].damping, 0.3);
    let r = body.joints[1].range.unwrap();
    assert!((r[0] + 30.0_f64.to_radians()).abs() < 1e-12);

    // The robot has 2 joints (this is a multi-joint body so 1 intermediate).
    assert_eq!(robot.joints.len(), 2);
}

#[test]
fn childclass_propagates() {
    let xml = r#"
    <mujoco>
      <default>
        <default class="leg">
          <joint damping="0.7"/>
        </default>
      </default>
      <worldbody>
        <body childclass="leg">
          <body>
            <joint name="distal"/>
            <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (_robot, model) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    // The grand-child body's joint inherits from the legacy `leg` class.
    let inner = &model.bodies[2].body;
    assert_eq!(inner.joints[0].damping, 0.7);
}

#[test]
fn include_inlines_children() {
    let dir = tempdir().unwrap();
    let inc = dir.path().join("inc.xml");
    fs::write(
        &inc,
        r#"<mujoco>
          <default>
            <joint damping="0.42"/>
          </default>
        </mujoco>"#,
    )
    .unwrap();
    let main = dir.path().join("main.xml");
    fs::write(
        &main,
        r#"<mujoco>
          <include file="inc.xml"/>
          <worldbody>
            <body><joint name="j"/><inertial mass="1" diaginertia="0.1 0.1 0.1"/></body>
          </worldbody>
        </mujoco>"#,
    )
    .unwrap();
    let (_, model) = MjcfRobot::from_file(&main, MjcfLoaderOptions::default()).unwrap();
    let body = &model.bodies[1].body;
    assert_eq!(body.joints[0].damping, 0.42);
}

#[test]
fn frame_folds_pose_into_children() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="parent" pos="0 0 0">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <frame pos="0 0 0.5">
            <body name="child" pos="0.1 0 0">
              <inertial mass="0.1" diaginertia="0.01 0.01 0.01"/>
            </body>
          </frame>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (_robot, model) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    // The child's body pose should be (0.1, 0, 0) translated by (0, 0, 0.5).
    let child = &model.bodies[2].body;
    assert!((child.pose.pos[0] - 0.1).abs() < 1e-12);
    assert!((child.pose.pos[2] - 0.5).abs() < 1e-12);
}

#[test]
fn inertia_from_geom_when_no_inertial() {
    let xml = r#"
    <mujoco>
      <compiler inertiafromgeom="auto"/>
      <worldbody>
        <body name="ball">
          <freejoint/>
          <geom type="sphere" size="0.1" density="2000"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let handles =
        robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    // Force a mass-properties update by simulating one step.
    use rapier3d::prelude::*;
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd = CCDSolver::new();
    let mut pipeline = PhysicsPipeline::new();
    let integration_parameters = IntegrationParameters::default();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let gravity = Vector::new(0.0, 0.0, -9.81);
    let physics_hooks = ();
    let event_handler = ();
    pipeline.step(
        gravity,
        &integration_parameters,
        &mut islands,
        &mut broad_phase,
        &mut narrow_phase,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut ccd,
        &physics_hooks,
        &event_handler,
    );
    let h = handles.bodies[1].as_ref().unwrap().body;
    let mass = bodies.get(h).unwrap().mass();
    // Sphere of r=0.1 with density 2000 → m = 2000 * (4/3)π r³ ≈ 8.38
    assert!(mass > 5.0 && mass < 12.0, "mass = {mass}");
}

#[test]
fn discardvisual_drops_visual_geoms() {
    let xml = r#"
    <mujoco>
      <compiler discardvisual="true"/>
      <worldbody>
        <body><freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <!-- visual-only geom (contype=conaffinity=0) -->
          <geom type="sphere" size="0.1" contype="0" conaffinity="0"/>
          <geom type="sphere" size="0.05"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let mut opts = MjcfLoaderOptions::default();
    opts.create_colliders_from_visual_shapes = true; // would normally accept it
    let (robot, _) = MjcfRobot::from_str(xml, opts, ".").unwrap();
    // discardvisual=true should still drop the visual geom even though
    // create_colliders_from_visual_shapes is on.
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let handles =
        robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    let dyn_body = handles.bodies[1].as_ref().unwrap();
    assert_eq!(dyn_body.colliders.len(), 1);
}
