//! `Multibody::dof_inverse_inertia` returns MuJoCo's `dof_invweight0` —
//! `diag(M⁻¹)` of the joint-space inertia (armature included). These tests pin
//! the two regimes that matter: a single leaf joint (where it's just the link's
//! own inertia about the axis), and a chain (where the articulated coupling
//! makes the apparent inertia at a parent DoF depend on its children).

use rapier3d::prelude::*;

#[test]
fn leaf_joint_inverse_inertia_is_link_inertia() {
    // Fixed base, one dynamic link rotating in place about Z. The joint-space
    // inertia is just the link's inertia about Z + armature, so the inverse
    // inertia is 1 / (Izz + armature).
    let mut bodies = RigidBodySet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let base = bodies.insert(RigidBodyBuilder::fixed());
    let izz = 0.3;
    let link = bodies.insert(RigidBodyBuilder::dynamic().additional_mass_properties(
        MassProperties::new(Vector::ZERO, 1.0, Vector::new(0.1, 0.2, izz)),
    ));
    let h = multibody_joints
        .insert(base, link, RevoluteJointBuilder::new(Vector::Z), true)
        .unwrap();

    // `additional_mass_properties` only lands in `local_mprops` after a mass
    // recompute (otherwise the mass matrix is zero before the first step).
    let colliders = ColliderSet::new();
    bodies
        .get_mut(link)
        .unwrap()
        .recompute_mass_properties_from_colliders(&colliders);

    // Add a rotor armature on the joint DoF.
    let armature = 0.05;
    {
        let (mb, _) = multibody_joints.get_mut(h).unwrap();
        // forward_kinematics happens inside dof_inverse_inertia; the free root
        // collapses to 0 DoFs, leaving the single revolute DoF.
        let inv = mb.dof_inverse_inertia(&bodies);
        // Before setting armature: 1 / Izz.
        assert_eq!(inv.len(), 1);
        assert!(
            (inv[0] - 1.0 / izz).abs() < 1e-4,
            "inv = {}, want {}",
            inv[0],
            1.0 / izz
        );
        // Now set armature and recheck.
        let n = mb.armature().len();
        mb.armature_mut()[n - 1] = armature;
    }
    let (mb, _) = multibody_joints.get_mut(h).unwrap();
    let inv = mb.dof_inverse_inertia(&bodies);
    let expected = 1.0 / (izz + armature);
    assert!(
        (inv[0] - expected).abs() < 1e-4,
        "with armature: inv = {}, want {}",
        inv[0],
        expected
    );
}

#[test]
fn parent_joint_inverse_inertia_accounts_for_children() {
    // Base→link1(hinge Z)→link2(hinge Z), all rotating about the same world Z
    // through the origin so the joint-space inertia is diagonal and easy to
    // reason about: M[0,0] = Izz1 + Izz2 (link1's DoF carries both links),
    // M[1,1] = Izz2. The off-diagonal is Izz2 (shared axis), so M is NOT
    // diagonal and diag(M⁻¹) differs from 1/diag(M) — this is the articulated
    // coupling we want.
    let mut bodies = RigidBodySet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let base = bodies.insert(RigidBodyBuilder::fixed());
    let (i1, i2) = (0.4, 0.25);
    let link1 = bodies.insert(RigidBodyBuilder::dynamic().additional_mass_properties(
        MassProperties::new(Vector::ZERO, 1.0, Vector::new(0.1, 0.1, i1)),
    ));
    let link2 = bodies.insert(RigidBodyBuilder::dynamic().additional_mass_properties(
        MassProperties::new(Vector::ZERO, 1.0, Vector::new(0.1, 0.1, i2)),
    ));
    multibody_joints
        .insert(base, link1, RevoluteJointBuilder::new(Vector::Z), true)
        .unwrap();
    let h = multibody_joints
        .insert(link1, link2, RevoluteJointBuilder::new(Vector::Z), true)
        .unwrap();

    let colliders = ColliderSet::new();
    for b in [link1, link2] {
        bodies
            .get_mut(b)
            .unwrap()
            .recompute_mass_properties_from_colliders(&colliders);
    }

    let (mb, _) = multibody_joints.get_mut(h).unwrap();
    let inv = mb.dof_inverse_inertia(&bodies);
    assert_eq!(inv.len(), 2);

    // Both links rotate about the same axis through the origin, so:
    //   M = [[i1+i2, i2], [i2, i2]]
    // and its inverse has diagonal [1/i1, (i1+i2)/(i1*i2)].
    let inv0_expected = 1.0 / i1;
    let inv1_expected = (i1 + i2) / (i1 * i2);
    assert!(
        (inv[0] - inv0_expected).abs() < 1e-3,
        "dof0 inv inertia = {}, want {}",
        inv[0],
        inv0_expected
    );
    assert!(
        (inv[1] - inv1_expected).abs() < 1e-3,
        "dof1 inv inertia = {}, want {} (must reflect coupling, not 1/i2={})",
        inv[1],
        inv1_expected,
        1.0 / i2
    );
}
