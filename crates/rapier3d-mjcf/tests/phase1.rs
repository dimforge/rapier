//! Phase-1 smoke tests: load minimal MJCF documents and verify the
//! resulting rapier objects look right.

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfRobot};

#[test]
fn cartpole_loads_and_steps() {
    let xml = r#"
    <mujoco model="cartpole">
      <worldbody>
        <body name="cart" pos="0 0 0">
          <joint name="slider" type="slide" axis="1 0 0" range="-2 2"/>
          <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="box" size="0.1 0.05 0.05"/>
          <body name="pole" pos="0 0 0.3">
            <joint name="hinge" type="hinge" axis="0 1 0" range="-180 180"/>
            <inertial pos="0 0 0" mass="0.5" diaginertia="0.02 0.02 0.02"/>
            <geom type="capsule" size="0.02 0.3"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    "#;

    let (robot, _model) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();

    // World + 2 bodies.
    assert_eq!(robot.bodies.len(), 3);
    // Two declared joints (cart-slider, pole-hinge).
    assert_eq!(robot.joints.len(), 2);

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
    // World (fixed, referenced by the cart's slider) + 2 dynamic bodies.
    assert_eq!(bodies.len(), 3);
    assert_eq!(impulse_joints.len(), 2);

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

    for _ in 0..200 {
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
    }
}

#[test]
fn freejoint_is_a_dynamic_body_with_no_joint() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="ball" pos="0 0 5">
          <freejoint/>
          <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.2"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    assert_eq!(robot.bodies.len(), 2);
    // No joint emitted for a free body.
    assert_eq!(robot.joints.len(), 0);
    assert_eq!(robot.bodies[1].body.body_type(), RigidBodyType::Dynamic);
}

#[test]
fn parent_world_no_joint_is_fixed() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="ground" pos="0 0 0">
          <geom type="plane" size="0 0 1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    // World + 1 body; the ground body is welded → fixed.
    assert_eq!(robot.bodies.len(), 2);
    assert_eq!(robot.joints.len(), 0);
    assert_eq!(robot.bodies[1].body.body_type(), RigidBodyType::Fixed);
}

#[test]
fn nested_body_no_joint_is_fixed_to_parent() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="base" pos="0 0 0">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.1"/>
          <body name="welded" pos="0.5 0 0">
            <inertial mass="0.5" diaginertia="0.05 0.05 0.05"/>
            <geom type="sphere" size="0.05"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    // World + base + welded.
    assert_eq!(robot.bodies.len(), 3);
    // One fixed joint between base and welded; base has no joint emitted.
    assert_eq!(robot.joints.len(), 1);
}

#[test]
fn make_roots_fixed_pins_freejoint_bodies() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="floater" pos="0 0 5">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;

    // Default: freejoint body is Dynamic.
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    assert_eq!(robot.bodies[1].body.body_type(), RigidBodyType::Dynamic);

    // make_roots_fixed=true: pins the freejoint body.
    let opts = MjcfLoaderOptions {
        make_roots_fixed: true,
        ..Default::default()
    };
    let (robot, _) = MjcfRobot::from_str(xml, opts, ".").unwrap();
    assert_eq!(robot.bodies[1].body.body_type(), RigidBodyType::Fixed);
    // No joint should have been emitted for the freejoint regardless.
    assert_eq!(robot.joints.len(), 0);
}

#[test]
fn make_roots_fixed_does_not_break_jointed_bodies() {
    // A body with a regular joint to the world is not a "free root" — it's
    // already constrained by its joint. `make_roots_fixed=true` must NOT
    // mark such bodies Fixed, otherwise multibody insertion panics with
    // "non-root multibody link must be Dynamic".
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="cart">
          <joint name="slider" type="slide" axis="1 0 0"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let opts = MjcfLoaderOptions {
        make_roots_fixed: true,
        ..Default::default()
    };
    let (robot, _) = MjcfRobot::from_str(xml, opts, ".").unwrap();
    // Cart stays Dynamic; the slider joint constrains it.
    assert_eq!(robot.bodies[1].body.body_type(), RigidBodyType::Dynamic);
    assert_eq!(robot.joints.len(), 1);
}

#[test]
fn skip_plane_geoms_drops_planes() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <geom name="floor" type="plane" size="0 0 0.05"/>
        <body name="ball">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;

    // skip_plane_geoms=false: plane is loaded as a halfspace collider on
    // the world body.
    let opts = MjcfLoaderOptions {
        skip_plane_geoms: false,
        ..Default::default()
    };
    let (robot, _) = MjcfRobot::from_str(xml, opts, ".").unwrap();
    assert_eq!(robot.bodies[0].colliders.len(), 1);

    // skip_plane_geoms=true (default): the floor geom is dropped.
    let opts = MjcfLoaderOptions {
        skip_plane_geoms: true,
        ..Default::default()
    };
    let (robot, _) = MjcfRobot::from_str(xml, opts, ".").unwrap();
    assert_eq!(robot.bodies[0].colliders.len(), 0);
    // The ball is unaffected.
    assert_eq!(robot.bodies[1].colliders.len(), 1);
}

#[test]
fn multi_joint_body_is_chain() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="planar" pos="0 0 0">
          <joint name="x" type="slide" axis="1 0 0"/>
          <joint name="y" type="slide" axis="0 1 0"/>
          <joint name="rot" type="hinge" axis="0 0 1"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="box" size="0.1 0.1 0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    // World + 2 intermediate spacers + 1 final body = 4.
    assert_eq!(robot.bodies.len(), 4);
    assert_eq!(robot.joints.len(), 3);
    let names: Vec<_> = robot.joints.iter().map(|j| j.name.as_deref()).collect();
    assert_eq!(names, vec![Some("x"), Some("y"), Some("rot")]);
}
