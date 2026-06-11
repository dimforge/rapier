//! Repro for loop-closure instability on multibodies (mujoco_menagerie3 /
//! agility_cassie symptom): impulse joints between links of the same
//! multibody, with off-origin link centers-of-mass.
//!
//! Chain: fixed base -> link1 -> link2 (revolute multibody joints), plus a
//! loop-closure impulse joint (3 locked linear axes) between link1 and link2
//! whose anchors coincide exactly in the initial configuration.
//!
//! Two solver bugs used to make this scene explode (|v| ~ 1e5 within one
//! step) or freeze solid:
//!
//! - The generic external constraint builder applied the com-shift of
//!   `transform_to_solver_body_space` to multibody links, but a link's solver
//!   pose (`link.local_to_world`) is origin-centered, unlike a regular
//!   rigid-body's com-centered solver pose. The resulting anchors were
//!   displaced by `-R·local_com`, creating a phantom position error that the
//!   solver could never resolve -> energy pumped every step.
//!
//! - When both endpoints of a generic constraint are links of the same
//!   multibody, the per-row effective mass was computed from two independent
//!   jacobian blocks (`J1ᵀWJ1 + J2ᵀWJ2`), dropping the `J1ᵀWJ2` coupling of
//!   the shared generalized velocities. Rows whose true relative jacobian
//!   (`J2 - J1`) nearly vanishes degenerated into divisions by float noise,
//!   producing garbage impulses in the millions.

use rapier3d::prelude::*;

struct World {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    link1: RigidBodyHandle,
    link2: RigidBodyHandle,
}

/// Builds the double-pendulum-with-closure scene. The closure pins the
/// world point `closure_anchor` (expressed in link1's local frame; it must
/// lie at `closure_anchor + (1, 0, 0)` in world space which is also used to
/// derive link2's local anchor so the constraint starts exactly satisfied).
fn build(use_multibody: bool, closure_anchor1: Vector) -> World {
    let mut bodies = RigidBodySet::new();
    let colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    // Distinct off-origin COMs (like cassie's achilles rod / heel spring).
    let com1 = Vector::new(0.0, 0.5, 0.0);
    let com2 = Vector::new(0.0, -0.3, 0.0);
    let inertia = Vector::new(0.1, 0.1, 0.1);

    let base = bodies.insert(RigidBodyBuilder::fixed());
    let link1 = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(1.0, 0.0, 0.0))
            .additional_mass_properties(MassProperties::new(com1, 1.0, inertia)),
    );
    let link2 = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(2.0, 0.0, 0.0))
            .additional_mass_properties(MassProperties::new(com2, 1.0, inertia)),
    );

    let rev1 = RevoluteJointBuilder::new(Vector::Z)
        .local_anchor1(Vector::new(0.0, 0.0, 0.0))
        .local_anchor2(Vector::new(-1.0, 0.0, 0.0));
    let rev2 = RevoluteJointBuilder::new(Vector::Z)
        .local_anchor1(Vector::new(1.0, 0.0, 0.0))
        .local_anchor2(Vector::new(0.0, 0.0, 0.0));

    if use_multibody {
        multibody_joints.insert(base, link1, rev1, true).unwrap();
        multibody_joints.insert(link1, link2, rev2, true).unwrap();
    } else {
        impulse_joints.insert(base, link1, rev1, true);
        impulse_joints.insert(link1, link2, rev2, true);
    }

    // Loop closure between a point of link1 and the same world point on
    // link2 (link2's origin sits at (2, 0, 0)).
    let anchor2 = Vector::new(1.0, 0.0, 0.0) + closure_anchor1 - Vector::new(2.0, 0.0, 0.0);
    let closure = GenericJointBuilder::new(JointAxesMask::LIN_AXES)
        .local_frame1(Pose::from_translation(closure_anchor1))
        .local_frame2(Pose::from_translation(anchor2))
        .build();
    impulse_joints.insert(link1, link2, closure, true);

    World {
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        link1,
        link2,
    }
}

/// Steps 100 frames under gravity; returns the max linear velocity reached
/// and the max world-space distance between the two closure anchors.
fn simulate(w: &mut World, closure_anchor1: Vector) -> (Real, Real) {
    let anchor2 = Vector::new(1.0, 0.0, 0.0) + closure_anchor1 - Vector::new(2.0, 0.0, 0.0);
    let gravity = Vector::new(0.0, -9.81, 0.0);
    let integration_parameters = IntegrationParameters::default();
    let mut pipeline = PhysicsPipeline::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut ccd = CCDSolver::new();

    let mut max_vel: Real = 0.0;
    let mut max_anchor_err: Real = 0.0;
    for _ in 0..100 {
        pipeline.step(
            gravity,
            &integration_parameters,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut w.bodies,
            &mut w.colliders,
            &mut w.impulse_joints,
            &mut w.multibody_joints,
            &mut ccd,
            &(),
            &(),
        );
        for h in [w.link1, w.link2] {
            max_vel = max_vel.max(w.bodies[h].linvel().length());
        }
        let p1 = w.bodies[w.link1].position() * closure_anchor1;
        let p2 = w.bodies[w.link2].position() * anchor2;
        max_anchor_err = max_anchor_err.max((p2 - p1).length());
    }
    (max_vel, max_anchor_err)
}

/// Closure anchored exactly at rev2's pivot: it is geometrically redundant
/// (the pivot is already pinned), so the scene must behave like a free
/// double pendulum — swinging (not frozen by garbage impulses from the
/// degenerate constraint rows), and not exploding.
#[test]
fn redundant_loop_closure_on_multibody_links() {
    let anchor = Vector::new(1.0, 0.0, 0.0); // world (2, 0, 0) = rev2 pivot
    let mut w = build(true, anchor);
    let (max_vel, _) = simulate(&mut w, anchor);
    println!("multibody, redundant closure: max |linvel| = {max_vel}");
    assert!(max_vel < 50.0, "system exploded: {max_vel}");
    assert!(max_vel > 1.0, "system is frozen: {max_vel}");
}

/// Closure anchored away from rev2's pivot: together with rev2 it welds
/// link1 and link2, so the assembly swings as a rigid compound pendulum.
/// The closure must hold (anchors stay coincident) without exploding.
#[test]
fn welding_loop_closure_on_multibody_links() {
    let anchor = Vector::new(2.0, 0.0, 0.0); // world (3, 0, 0)
    let mut w = build(true, anchor);
    let (max_vel, max_err) = simulate(&mut w, anchor);
    println!("multibody, welding closure: max |linvel| = {max_vel}, max anchor err = {max_err}");
    assert!(max_vel < 50.0, "system exploded: {max_vel}");
    assert!(max_vel > 1.0, "system is frozen: {max_vel}");
    assert!(max_err < 1.0e-2, "loop closure not enforced: {max_err}");
}

/// Same scenes built with impulse joints everywhere (no multibody), as a
/// sanity reference for the regular two-body solver path.
#[test]
fn welding_loop_closure_on_rigid_bodies() {
    let anchor = Vector::new(2.0, 0.0, 0.0);
    let mut w = build(false, anchor);
    let (max_vel, max_err) = simulate(&mut w, anchor);
    println!("rigid bodies, welding closure: max |linvel| = {max_vel}, max anchor err = {max_err}");
    assert!(max_vel < 50.0, "system exploded: {max_vel}");
    assert!(max_vel > 1.0, "system is frozen: {max_vel}");
    assert!(max_err < 1.0e-2, "loop closure not enforced: {max_err}");
}
