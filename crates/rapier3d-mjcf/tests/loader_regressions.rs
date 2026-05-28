//! Regression tests for loader bugs surfaced during audit. Each test
//! corresponds to a specific defect — the test asserts the *correct*
//! behavior so future regressions are caught.

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfRobot};

/// A persistent physics world. Re-creating an `IslandManager` per step (as
/// the older test helpers in this crate do) wipes the `active_island_id`s
/// recorded on the bodies and is enough to silently swallow `add_force`
/// updates — so for tests that need force integration we hold the state
/// across steps.
struct World {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    pipeline: PhysicsPipeline,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    ccd: CCDSolver,
}

impl World {
    fn new() -> Self {
        Self {
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            pipeline: PhysicsPipeline::new(),
            integration_parameters: IntegrationParameters::default(),
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            ccd: CCDSolver::new(),
        }
    }

    fn step(&mut self, gravity: Vector) {
        self.pipeline.step(
            gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd,
            &(),
            &(),
        );
    }
}

/// `apply_gravity_compensation` is supposed to net out gravity for bodies
/// with `gravcomp > 0`. The loader independently sets `gravity_scale =
/// 1 - gravcomp` so the per-step helper is meant to "take over" — but
/// because it doesn't reset `gravity_scale` to 1 before adding the
/// compensation force, the two effects stack: for `gravcomp=1` the body
/// experiences a net upward acceleration equal to gravity.
#[test]
fn gravity_compensation_does_not_double_compensate() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <body name="floater" pos="0 0 5" gravcomp="1">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let gravity = Vector::new(0.0, 0.0, -9.81);
    let mut w = World::new();
    let handles = robot.clone().insert_using_impulse_joints(
        &mut w.bodies,
        &mut w.colliders,
        &mut w.impulse_joints,
    );
    let h = handles.bodies[1].as_ref().unwrap().body;
    let z0 = w.bodies.get(h).unwrap().translation().z;

    // Per the helper's docstring it's called once per step. We expect the
    // body to neither fall nor float up — total gravity is exactly
    // cancelled by the loader's `gravity_scale=0` (for gravcomp=1) plus the
    // helper's compensation force, which together should *not* exceed one
    // body weight.
    for _ in 0..120 {
        handles.apply_gravity_compensation(&mut w.bodies, &robot, gravity);
        w.step(gravity);
    }
    let z1 = w.bodies.get(h).unwrap().translation().z;
    let drift = z1 - z0;
    assert!(
        drift.abs() < 0.2,
        "expected near-zero drift with gravcomp=1 + apply_gravity_compensation; \
         got drift = {drift} (body floated {} m)",
        if drift > 0.0 { "up" } else { "down" }
    );
}

/// MJCF `<motor ctrl gear>` is a *force* (or torque) input: the actuator
/// adds `ctrl * gear` to the joint's generalized force at every step. The
/// current `apply_controls` instead calls `set_motor_velocity(ax, ctrl*gear,
/// 0.0)`. With `stiffness=damping=0`, rapier treats that as a *hard*
/// velocity constraint, so the body's velocity snaps to `ctrl*gear`
/// instantly in the first step instead of being accelerated by a finite
/// force — observable here on the first step.
#[test]
fn motor_actuator_is_force_not_velocity_constraint() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="cart">
          <joint name="slider" type="slide" axis="1 0 0"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="box" size="0.1 0.1 0.1"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="cart_motor" joint="slider" gear="10"/>
      </actuator>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut w = World::new();
    let handles =
        robot.insert_using_impulse_joints(&mut w.bodies, &mut w.colliders, &mut w.impulse_joints);
    handles.apply_controls(&mut w.impulse_joints, &[1.0]);
    w.step(Vector::new(0.0, 0.0, 0.0));
    let v = w
        .bodies
        .get(handles.bodies[1].as_ref().unwrap().body)
        .unwrap()
        .linvel()
        .x;
    // F=10, m=1, dt=1/60 → v ≈ 0.167. The hard-constraint bug gives v = 10.
    assert!(
        v < 1.0,
        "motor actuator should apply finite force, but cart vx={v} after one step \
         (implies hard velocity constraint, not force input)",
    );
    assert!(
        v > 0.0,
        "motor actuator should at least accelerate the cart; got vx={v}",
    );
}

/// Companion to `motor_actuator_is_force_not_velocity_constraint`: with the
/// same `<motor ctrl=1 gear=10>` driving a 1 kg cart for 60 steps of dt=1/60
/// (1 s of simulated time), the cart's velocity should converge to roughly
/// `F·t/m = 10 m/s`.
#[test]
fn motor_actuator_applies_finite_force_over_time() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="cart">
          <joint name="slider" type="slide" axis="1 0 0"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="box" size="0.1 0.1 0.1"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="cart_motor" joint="slider" gear="10"/>
      </actuator>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut w = World::new();
    let handles =
        robot.insert_using_impulse_joints(&mut w.bodies, &mut w.colliders, &mut w.impulse_joints);
    for _ in 0..60 {
        handles.apply_controls(&mut w.impulse_joints, &[1.0]);
        w.step(Vector::new(0.0, 0.0, 0.0));
    }
    let v = w
        .bodies
        .get(handles.bodies[1].as_ref().unwrap().body)
        .unwrap()
        .linvel()
        .x;
    assert!(
        v > 5.0 && v < 20.0,
        "motor should accelerate cart to ~10 m/s after 1 s; got {v}",
    );
}

/// `<position>` actuator on a hinge should drive the joint toward the
/// commanded target. With kv != 0 the controller is damped enough that 300
/// steps lands close to the setpoint.
#[test]
fn position_actuator_drives_hinge_toward_target() {
    let xml = r#"
    <mujoco>
      <option gravity="0 0 0"/>
      <worldbody>
        <body name="arm">
          <joint name="hinge" type="hinge" axis="0 0 1"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="box" size="0.2 0.05 0.05"/>
        </body>
      </worldbody>
      <actuator>
        <position name="arm_pos" joint="hinge" kp="20" kv="8"/>
      </actuator>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut w = World::new();
    let handles =
        robot.insert_using_impulse_joints(&mut w.bodies, &mut w.colliders, &mut w.impulse_joints);
    for _ in 0..300 {
        handles.apply_controls(&mut w.impulse_joints, &[1.0]);
        w.step(Vector::new(0.0, 0.0, 0.0));
    }
    let arm = w
        .bodies
        .get(handles.bodies[1].as_ref().unwrap().body)
        .unwrap();
    let q = arm.rotation();
    let theta = 2.0 * q.z.atan2(q.w);
    assert!(
        (theta - 1.0).abs() < 0.25,
        "position-actuator should drive hinge toward ~1 rad; got {theta}",
    );
}

/// Regression guard for multi-joint chains: each joint's `pos` attribute
/// should be preserved on the rapier joint frames, not collapsed onto
/// joint[0]'s position.
#[test]
fn multi_joint_chain_honors_each_joint_pos() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="arm" pos="0 0 0">
          <joint name="j1" type="hinge" axis="0 0 1" pos="0 0 0"/>
          <joint name="j2" type="hinge" axis="0 1 0" pos="1 0 0"/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="box" size="0.1 0.1 0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    // World + 1 intermediate + final = 3.
    assert_eq!(robot.bodies.len(), 3);
    assert_eq!(robot.joints.len(), 2);
    let p = robot.joints[1].joint.local_frame2.translation;
    assert!(
        (p.x - 1.0).abs() < 1e-6 && p.y.abs() < 1e-6 && p.z.abs() < 1e-6,
        "j2's local_frame2 translation should reflect pos=\"1 0 0\"; got {p:?}",
    );
}

/// `<contact><pair>` lookup is keyed by the *actual collider index* in each
/// body, not the MJCF geom index. When some geoms are filtered out (visual
/// when `create_colliders_from_visual_shapes=false`), the indices diverge,
/// and a naive index mapping would point at the wrong collider.
#[test]
fn contact_pair_lookup_skips_filtered_visual_geoms() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="a">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <!-- visual-only: dropped from colliders by default -->
          <geom name="a_visual" type="sphere" size="0.1" contype="0" conaffinity="0"/>
          <geom name="a_col"    type="sphere" size="0.2"/>
        </body>
        <body name="b">
          <freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom name="b_col"    type="sphere" size="0.2"/>
        </body>
      </worldbody>
      <contact>
        <pair geom1="a_col" geom2="b_col" friction="0.7"/>
      </contact>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    // a_col is the only collider on body a (visual was dropped) → index 0,
    // not 1.
    let (a_body, a_idx) = robot.geom_name_to_collider["a_col"];
    assert_eq!(a_body, 1, "body index for `a_col`");
    assert_eq!(
        a_idx, 0,
        "collider index within body for `a_col` should be 0 \
                          (the visual geom was filtered out before insertion)"
    );
}

/// Regression guard for `<contact><pair>` hook plumbing: building hooks on
/// a robot with no overrides should yield an empty hook set.
#[test]
fn contact_hooks_are_empty_when_no_pairs_or_excludes() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="a"><freejoint/>
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <geom type="sphere" size="0.1"/>
        </body>
      </worldbody>
    </mujoco>
    "#;
    let (robot, _) = MjcfRobot::from_str(xml, MjcfLoaderOptions::default(), ".").unwrap();
    let mut w = World::new();
    let handles = robot.clone().insert_using_impulse_joints(
        &mut w.bodies,
        &mut w.colliders,
        &mut w.impulse_joints,
    );
    let hooks = handles.contact_hooks(&robot);
    assert!(!hooks.has_excludes());
    assert!(!hooks.has_overrides());
}

/// `<equality><connect>` anchors are expressed in body1's local MJCF
/// coordinates. When `MjcfLoaderOptions::scale != 1.0`, the loader
/// scales body world poses (via `scale_pose`) but used to leave the
/// anchor unscaled — so the loop-closure impulse joint ended up
/// constraining a point at `anchor` units from body1's origin in
/// scaled world space, instead of at `scale * anchor` units. The
/// constraint is still self-consistent at rest (anchor2 is computed
/// from anchor1 through the world), but the anchor lands in the
/// wrong place relative to the rest of the (scaled) body geometry
/// and joint axes. For agility_cassie at `scale=10` that's enough to
/// destabilize the multibody+impulse solver within a few hundred
/// steps.
#[test]
fn equality_connect_anchor_is_scaled_with_loader_scale() {
    let xml = r#"
    <mujoco>
      <worldbody>
        <body name="a" pos="0 0 1">
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <freejoint/>
          <geom type="sphere" size="0.05"/>
        </body>
        <body name="b" pos="0.5 0 1.2">
          <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
          <freejoint/>
          <geom type="sphere" size="0.05"/>
        </body>
      </worldbody>
      <equality>
        <connect body1="a" body2="b" anchor="0.35 0 0"/>
      </equality>
    </mujoco>
    "#;
    let options = MjcfLoaderOptions {
        scale: 10.0,
        ..Default::default()
    };
    let (robot, _) = MjcfRobot::from_str(xml, options, ".").unwrap();
    let mut w = World::new();
    let handles =
        robot.insert_using_impulse_joints(&mut w.bodies, &mut w.colliders, &mut w.impulse_joints);
    assert_eq!(handles.equality_joints.len(), 1);

    let eq = &handles.equality_joints[0];
    let joint = w.impulse_joints.get(eq.joint).unwrap();
    let body1 = w.bodies.get(eq.link1).unwrap();
    let body2 = w.bodies.get(eq.link2).unwrap();

    // Body1 has identity orientation and world position (0, 0, 10) at
    // scale=10. The MJCF anchor (0.35, 0, 0) in body1 local should map
    // to (3.5, 0, 0) in body1 local after scaling, i.e. world position
    // (3.5, 0, 10).
    let anchor1_world = body1.position() * joint.data.local_frame1;
    let expected = Vector::new(3.5, 0.0, 10.0);
    let drift1 = (anchor1_world.translation - expected).length();
    assert!(
        drift1 < 1e-4,
        "expected scaled anchor in world coords ≈ {expected:?}, got {:?} (drift {drift1})",
        anchor1_world.translation,
    );

    // Sanity check: the constraint must also be self-consistent — the
    // two local frames, when transformed by their bodies' world poses,
    // must coincide. (This already held *before* the scale fix because
    // anchor2 was derived from anchor1 through the world; the test is
    // here to guard against future regressions that break that
    // invariant alongside the scaling one.)
    let anchor2_world = body2.position() * joint.data.local_frame2;
    let drift2 = (anchor1_world.translation - anchor2_world.translation).length();
    assert!(
        drift2 < 1e-4,
        "loop-closure anchors must coincide at rest; drift = {drift2}",
    );
}

/// Models that compress collision and visual representation into a
/// single `<geom type="mesh">` per body (xarm7, softfoot's
/// attachment_cube, leap_hand's fingertip, etc.) used to vanish
/// under the "Render visual meshes" toggle because everything was a
/// collider. The loader now synthesizes a visual-mesh entry for any
/// collision-active mesh geom, gated by MuJoCo's `<geom group>`
/// convention: group < 3 synthesizes, group >= 3 is treated as
/// "collision proxy, hidden by default" (matching the MuJoCo
/// viewer's default visibility per group).
///
/// These tests exercise the synthesis path, which needs an actual
/// mesh-loader backend — gated behind the `wavefront` feature so the
/// fast default-feature test run isn't affected.
#[cfg(feature = "wavefront")]
mod mesh_synthesis {
    use super::*;

    fn make_obj_fixture(dir: &std::path::Path) -> std::path::PathBuf {
        // Minimal OBJ tetrahedron: 4 vertices, 2 triangles. Enough for
        // the loader to build a TriMesh / ConvexHull from.
        let path = dir.join("tetra.obj");
        std::fs::write(
            &path,
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 2 4\n",
        )
        .unwrap();
        path
    }

    #[test]
    fn collision_active_mesh_geoms_get_synthesized_visual() {
        let dir = tempfile::tempdir().unwrap();
        let obj = make_obj_fixture(dir.path());
        let xml = format!(
            r#"
            <mujoco>
              <compiler convexhull="false"/>
              <asset>
                <mesh name="m" file="{}"/>
              </asset>
              <worldbody>
                <body name="a">
                  <freejoint/>
                  <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
                  <geom type="mesh" mesh="m"/>
                </body>
              </worldbody>
            </mujoco>
            "#,
            obj.display()
        );
        let (robot, _) = MjcfRobot::from_str(&xml, MjcfLoaderOptions::default(), ".").unwrap();
        let body = &robot.bodies[1];
        assert_eq!(
            body.colliders.len(),
            1,
            "collision-active mesh becomes a collider"
        );
        assert_eq!(
            body.visual_meshes.len(),
            1,
            "collision-active mesh also synthesized as a visual"
        );
    }

    /// Franka pattern: the body has fine-grained visual OBJs and a
    /// separate simplified collision STL marked `group="3"` (MuJoCo's
    /// convention for "collision proxy, hidden by default"). The
    /// loader must skip synthesis for that collision mesh, otherwise
    /// every link double-renders.
    #[test]
    fn synthesis_skips_group_3_collision_mesh() {
        let dir = tempfile::tempdir().unwrap();
        let obj = make_obj_fixture(dir.path());
        let xml = format!(
            r#"
            <mujoco>
              <compiler convexhull="false"/>
              <asset>
                <mesh name="v" file="{f}"/>
                <mesh name="c" file="{f}"/>
              </asset>
              <worldbody>
                <body name="a">
                  <freejoint/>
                  <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
                  <geom type="mesh" mesh="v" contype="0" conaffinity="0" group="2"/>
                  <geom type="mesh" mesh="c" group="3"/>
                </body>
              </worldbody>
            </mujoco>
            "#,
            f = obj.display()
        );
        let (robot, _) = MjcfRobot::from_str(&xml, MjcfLoaderOptions::default(), ".").unwrap();
        let body = &robot.bodies[1];
        assert_eq!(body.colliders.len(), 1, "one collision-active mesh");
        assert_eq!(
            body.visual_meshes.len(),
            1,
            "group=3 collision mesh stays hidden — only the explicit visual is rendered"
        );
    }

    /// leap_hand pattern: the body has both an explicit visual mesh
    /// (`distal`) AND a separate collision-active mesh (`tip`) in the
    /// default group 0. The tip mesh has no visual-only counterpart;
    /// without synthesis it would disappear under the "Render visual
    /// meshes" toggle. The loader must synthesize a visual for it
    /// even though the body already has an explicit visual mesh.
    #[test]
    fn synthesis_runs_alongside_explicit_visual_when_group_is_default() {
        let dir = tempfile::tempdir().unwrap();
        let obj = make_obj_fixture(dir.path());
        let xml = format!(
            r#"
            <mujoco>
              <compiler convexhull="false"/>
              <asset>
                <mesh name="distal" file="{f}"/>
                <mesh name="tip" file="{f}"/>
              </asset>
              <worldbody>
                <body name="a">
                  <freejoint/>
                  <inertial mass="1" diaginertia="0.1 0.1 0.1"/>
                  <geom type="mesh" mesh="distal" contype="0" conaffinity="0" group="1"/>
                  <geom type="mesh" mesh="tip"/>
                </body>
              </worldbody>
            </mujoco>
            "#,
            f = obj.display()
        );
        let (robot, _) = MjcfRobot::from_str(&xml, MjcfLoaderOptions::default(), ".").unwrap();
        let body = &robot.bodies[1];
        assert_eq!(body.colliders.len(), 1, "one collision-active mesh");
        assert_eq!(
            body.visual_meshes.len(),
            2,
            "explicit visual + synthesized visual for the group-0 collision mesh"
        );
    }
}
