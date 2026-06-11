//! Armature is a per-DoF reflected rotor inertia added to the diagonal of a
//! multibody's generalized mass matrix (MuJoCo `<joint armature>` semantics).
//!
//! These tests pin the two properties that define it:
//!  1. It resists joint acceleration — a heavier rotor makes the joint spin up
//!     more slowly under the same torque — and behaves like a genuine inertia
//!     (no `dt` scaling, no velocity-proportional force, unlike damping).
//!  2. It is a joint-space quantity only: it does NOT modify the link's spatial
//!     (3x3) inertia tensor.

use rapier3d::prelude::*;

/// Build a fixed base + one dynamic link joined by a revolute multibody joint,
/// with the link's center of mass offset from the hinge so gravity applies a
/// torque. Optionally set `armature` on the joint DoF. Returns the joint's
/// angular velocity magnitude after `steps` steps.
fn spin_up(armature: Real, steps: usize) -> Real {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    let base = bodies.insert(RigidBodyBuilder::fixed());
    // COM offset 1m along +x from the hinge (hinge about Z at the origin).
    let link = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(1.0, 0.0, 0.0))
            .additional_mass_properties(MassProperties::new(
                Vector::ZERO,
                1.0,
                Vector::new(0.01, 0.01, 0.01),
            )),
    );
    let rev = RevoluteJointBuilder::new(Vector::Z)
        .local_anchor1(Vector::new(0.0, 0.0, 0.0))
        .local_anchor2(Vector::new(-1.0, 0.0, 0.0));
    let handle = multibody_joints.insert(base, link, rev, true).unwrap();

    let gravity = Vector::new(0.0, -9.81, 0.0);
    let integration_parameters = IntegrationParameters::default();
    let mut pipeline = PhysicsPipeline::new();
    let mut islands = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut ccd = CCDSolver::new();

    let mut step_once = |bodies: &mut RigidBodySet, mbj: &mut MultibodyJointSet| {
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

    // Warm up one step so the root joint type settles (a fixed base starts as
    // a 6-DoF free root and collapses to 0 DoFs on the first step, which
    // re-indexes the generalized vectors). Both branches take this identical
    // step, so it doesn't bias the comparison.
    step_once(&mut bodies, &mut multibody_joints);

    if armature != 0.0 {
        let (mb, _) = multibody_joints.get_mut(handle).unwrap();
        // Root is now fixed (0 DoFs); the single revolute DoF is all that's
        // left in the generalized vectors.
        let v = mb.armature_mut();
        assert_eq!(v.len(), 1, "expected one DoF after the root settled");
        v[0] = armature;
    }

    for _ in 0..steps {
        step_once(&mut bodies, &mut multibody_joints);
    }
    bodies[link].angvel().length()
}

#[test]
fn armature_resists_joint_acceleration() {
    // The link's own inertia about the hinge is dominated by m·r² = 1·1² = 1.
    // An armature of 4 roughly quintuples the effective rotor inertia, so the
    // joint should spin up much more slowly under the same gravity torque.
    let w_no = spin_up(0.0, 30);
    let w_arm = spin_up(4.0, 30);

    assert!(w_no > 0.0, "control case didn't move: {w_no}");
    assert!(
        w_arm < w_no * 0.5,
        "armature didn't slow the joint enough: no_armature={w_no}, armature={w_arm}"
    );
}

#[test]
fn armature_does_not_change_spatial_inertia() {
    // Adding armature must not touch the link's spatial mass properties — it's
    // purely a generalized-mass-matrix diagonal term.
    let mut bodies = RigidBodySet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let base = bodies.insert(RigidBodyBuilder::fixed());
    let inertia = Vector::new(0.01, 0.02, 0.03);
    let link = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(1.0, 0.0, 0.0))
            .additional_mass_properties(MassProperties::new(Vector::ZERO, 1.0, inertia)),
    );
    let rev = RevoluteJointBuilder::new(Vector::Z).local_anchor2(Vector::new(-1.0, 0.0, 0.0));
    let handle = multibody_joints.insert(base, link, rev, true).unwrap();

    let before = bodies[link].mass_properties().local_mprops.principal_inertia();

    let (mb, _) = multibody_joints.get_mut(handle).unwrap();
    mb.armature_mut()[0] = 5.0;

    let after = bodies[link].mass_properties().local_mprops.principal_inertia();
    assert_eq!(before, after, "armature must not alter the spatial inertia tensor");
}
