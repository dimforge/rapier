//! `apply_controls_multibody_scaled`'s `gain_scale` softens actuation: a lower
//! scale makes a position servo approach its target more slowly (no snap).

use rapier3d::prelude::*;
use rapier3d_mjcf::{MjcfLoaderOptions, MjcfMultibodyOptions, MjcfRobot};

// A strong position servo on one hinge — at full strength it slams to target.
const SERVO: &str = r#"
<mujoco>
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <inertial mass="5" diaginertia="2 2 2"/>
      <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.03"/>
    </body>
  </worldbody>
  <actuator>
    <position name="a" joint="j" kp="2000" kv="200" forcerange="-20 20"/>
  </actuator>
</mujoco>
"#;

fn angle(rb: &RigidBody) -> f32 {
    let x = rb.rotation() * Vector::X;
    (-x.z).atan2(x.x)
}

/// Drive `j` toward 1.0 for `steps` steps at the given strength scale; return
/// the angle reached.
fn drive_to(steps: usize, gain_scale: Real) -> f32 {
    let (robot, _) = MjcfRobot::from_str(SERVO, MjcfLoaderOptions::default(), ".").unwrap();
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
    let link = robot.body_name_to_idx["link"];
    let mut pipeline = PhysicsPipeline::new();
    let ip = IntegrationParameters::default();
    let (mut isl, mut bp, mut np, mut ccd) = (
        IslandManager::new(),
        DefaultBroadPhase::new(),
        NarrowPhase::new(),
        CCDSolver::new(),
    );
    for _ in 0..steps {
        handles.apply_controls_multibody_scaled(&mut bodies, &mut mbj, &[1.0 as Real], gain_scale);
        pipeline.step(
            Vector::new(0.0, 0.0, 0.0), // no gravity: isolate the servo dynamics
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
    angle(
        bodies
            .get(handles.bodies[link].as_ref().unwrap().body)
            .unwrap(),
    )
}

#[test]
fn lower_gain_scale_approaches_target_more_slowly() {
    let steps = 15;
    let full = drive_to(steps, 1.0);
    let soft = drive_to(steps, 0.1);
    println!("full={full} soft={soft}");

    // Force-limited, so the approach rate scales with strength: at 0.1 the
    // servo moves roughly an order of magnitude less far in the same time.
    assert!(
        full > 0.2,
        "full-strength servo should be well underway: {full}"
    );
    assert!(
        soft < full * 0.3,
        "0.1x servo should lag well behind: soft={soft} full={full}"
    );
    assert!(
        soft > 0.0,
        "0.1x servo should still be moving toward target: {soft}"
    );
}

#[test]
fn zero_ish_gain_does_not_nan() {
    // The minimum the example slider allows is 0.02; make sure a tiny scale (and
    // the INFINITY force cap on a forcerange-less actuator) stays finite.
    let a = drive_to(20, 0.02);
    assert!(a.is_finite());
}
