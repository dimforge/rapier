//! `Multibody` DoF couplings (`q2 = coeff·q1 + offset`) enforced as velocity
//! constraints — the mechanism behind MuJoCo's `<equality><joint>` (e.g. the
//! robotiq gripper's two driver joints moving together).

use rapier3d::dynamics::MultibodyDofCoupling;
use rapier3d::prelude::*;

/// Two sibling hinges (both children of a fixed base, rotating about Z in
/// place), coupled `q2 = coeff·q1 + offset`. A position motor drives `q1` to
/// `target`; after settling, `q2` should track `coeff·target + offset`.
fn run_coupling(coeff: Real, offset: Real, target: Real) -> (Real, Real) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    let inertia = Vector::new(0.1, 0.1, 0.1);
    let base = bodies.insert(RigidBodyBuilder::fixed());
    let link1 = bodies.insert(
        RigidBodyBuilder::dynamic()
            .additional_mass_properties(MassProperties::new(Vector::ZERO, 1.0, inertia)),
    );
    let link2 = bodies.insert(
        RigidBodyBuilder::dynamic()
            .additional_mass_properties(MassProperties::new(Vector::ZERO, 1.0, inertia)),
    );

    let h1 = multibody_joints
        .insert(base, link1, RevoluteJointBuilder::new(Vector::Z), true)
        .unwrap();
    let h2 = multibody_joints
        .insert(base, link2, RevoluteJointBuilder::new(Vector::Z), true)
        .unwrap();

    let link1_id = multibody_joints.get(h1).unwrap().1;
    let link2_id = multibody_joints.get(h2).unwrap().1;

    {
        let (mb, _) = multibody_joints.get_mut(h1).unwrap();
        // Couple q2 = coeff·q1 + offset (AngX is spatial axis 3, the hinge DoF).
        mb.add_dof_coupling(MultibodyDofCoupling {
            link1: link1_id,
            dof1: 0,
            axis1: 3,
            link2: link2_id,
            dof2: 0,
            axis2: 3,
            coeff,
            offset,
        });
        // Drive link1's hinge to `target`.
        let link1_joint = &mut mb.links_mut().nth(link1_id).unwrap().joint.data;
        link1_joint.set_motor_position(JointAxis::AngX, target, 20.0, 2.0);
    }

    let gravity = Vector::ZERO;
    let integration_parameters = IntegrationParameters::default();
    let mut pipeline = PhysicsPipeline::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut ccd = CCDSolver::new();
    for _ in 0..300 {
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
            &(),
            &(),
        );
    }
    let q1 = bodies[link1].rotation().to_scaled_axis().z;
    let q2 = bodies[link2].rotation().to_scaled_axis().z;
    (q1, q2)
}

#[test]
fn one_to_one_coupling_makes_joints_track() {
    // q2 = q1: the robotiq case. Drive q1 to 0.5; q2 must follow.
    let (q1, q2) = run_coupling(1.0, 0.0, 0.5);
    assert!((q1 - 0.5).abs() < 0.05, "driven joint didn't reach target: q1 = {q1}");
    assert!((q2 - q1).abs() < 0.02, "coupling not enforced: q1 = {q1}, q2 = {q2}");
}

#[test]
fn scaled_and_offset_coupling() {
    // q2 = -0.5·q1 + 0.2: a gear ratio with an offset.
    let (q1, q2) = run_coupling(-0.5, 0.2, 0.6);
    let expected = -0.5 * q1 + 0.2;
    assert!((q1 - 0.6).abs() < 0.05, "driven joint didn't reach target: q1 = {q1}");
    assert!(
        (q2 - expected).abs() < 0.02,
        "coupling q2 = -0.5·q1 + 0.2 not enforced: q1 = {q1}, q2 = {q2}, expected {expected}"
    );
}
