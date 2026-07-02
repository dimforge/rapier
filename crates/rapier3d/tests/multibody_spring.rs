//! Implicit joint springs on a multibody. A spring set on a multibody joint
//! applies a generalized force `-stiffness·(q − rest)` integrated implicitly
//! (its `dt²·k` term lands on the mass-matrix diagonal). These tests pin the
//! two defining properties: it drives the joint to its rest angle, and it does
//! so stably even on a low-inertia link where an explicit motor would diverge.

use rapier3d::prelude::*;

/// Fixed base + one dynamic link on a revolute multibody joint. Sets a spring
/// (rest angle `rest`, the given `stiffness`) on the joint plus a little joint
/// damping so it settles, then returns the link's final rotation about Z after
/// `steps` steps with gravity off.
fn settle_angle(stiffness: Real, rest: Real, damping: Real, inertia: Real, steps: usize) -> Real {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    let base = bodies.insert(RigidBodyBuilder::fixed());
    // Link rotates in place about the shared origin (COM on the hinge axis), so
    // the effective joint inertia is exactly `inertia` — no lever-arm term.
    let link = bodies.insert(RigidBodyBuilder::dynamic().additional_mass_properties(
        MassProperties::new(Vector::ZERO, 1.0, Vector::new(inertia, inertia, inertia)),
    ));
    let rev = RevoluteJointBuilder::new(Vector::Z);
    let handle = multibody_joints.insert(base, link, rev, true).unwrap();

    let gravity = Vector::ZERO;
    let integration_parameters = IntegrationParameters::default();
    let mut pipeline = PhysicsPipeline::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut ccd = CCDSolver::new();

    let mut step = |bodies: &mut RigidBodySet, mbj: &mut MultibodyJointSet| {
        pipeline.step(
            gravity,
            &integration_parameters,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            bodies,
            &mut colliders,
            &mut impulse_joints,
            mbj,
            &mut ccd,
            &(),
            &(),
        );
    };

    // Warm up one step so the fixed-base root collapses to 0 DoFs (it starts as
    // a 6-DoF free root), leaving the single revolute DoF.
    step(&mut bodies, &mut multibody_joints);

    {
        let (mb, link_id) = multibody_joints.get_mut(handle).unwrap();
        let link_mut = mb.links_mut().nth(link_id).unwrap();
        // The revolute's free axis is the angular X axis (index DIM = 3).
        link_mut.joint.set_spring(3, stiffness, rest);
        // A little joint-space damping so the (otherwise conservative) spring
        // settles in a finite number of steps.
        let n = mb.damping().len();
        mb.damping_mut()[n - 1] = damping;
    }

    for _ in 0..steps {
        step(&mut bodies, &mut multibody_joints);
    }
    bodies[link].rotation().to_scaled_axis().z
}

#[test]
fn implicit_spring_drives_joint_to_rest_angle() {
    // Spring rest = +0.6 rad; the joint starts at 0. With ~critical damping
    // (d ≈ 2·√(k·I) = 2·√(5·0.05) = 1.0) it settles at the rest angle.
    let angle = settle_angle(5.0, 0.6, 1.0, 0.05, 400);
    assert!(
        (angle - 0.6).abs() < 0.05,
        "spring did not settle at its rest angle: angle = {angle}"
    );
}

#[test]
fn implicit_spring_is_stable_on_tiny_inertia() {
    // The whole point of implicit integration: a stiff spring on a near-massless
    // link (like robotiq's follower / flybody's leg segments) stays bounded
    // where an explicit position motor diverges to thousands of rad/s.
    let angle = settle_angle(50.0, 0.3, 0.01, 1.0e-6, 600);
    assert!(angle.is_finite(), "spring blew up on tiny inertia: {angle}");
    assert!(
        (angle - 0.3).abs() < 0.1,
        "spring on tiny inertia didn't reach rest: angle = {angle}"
    );
}
