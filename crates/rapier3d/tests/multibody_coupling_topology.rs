//! Multibody DoF couplings and the self-contacts flag across topology changes (link insertion,
//! multibody merges and splits), and the coupling removal API.

use rapier3d::dynamics::MultibodyDofCoupling;
use rapier3d::prelude::*;

fn insert_link(world: &mut PhysicsWorld, x: Real) -> RigidBodyHandle {
    world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(x, 0.0, 0.0))
            .additional_mass_properties(MassProperties::new(Vector::ZERO, 1.0, Vector::splat(0.1))),
    )
}

fn hinge(world: &mut PhysicsWorld, parent: RigidBodyHandle, child: RigidBodyHandle) {
    world
        .insert_multibody_joint(parent, child, RevoluteJointBuilder::new(Vector::Z))
        .unwrap();
}

fn link_id(world: &PhysicsWorld, body: RigidBodyHandle) -> usize {
    world.multibody_joints.rigid_body_link(body).unwrap().id
}

fn multibody_of(world: &PhysicsWorld, body: RigidBodyHandle) -> &Multibody {
    let link = world.multibody_joints.rigid_body_link(body).unwrap();
    world
        .multibody_joints
        .get_multibody(link.multibody)
        .unwrap()
}

fn multibody_of_mut(world: &mut PhysicsWorld, body: RigidBodyHandle) -> &mut Multibody {
    let link = *world.multibody_joints.rigid_body_link(body).unwrap();
    world
        .multibody_joints
        .get_multibody_mut(link.multibody)
        .unwrap()
}

/// Couples the hinges of `body1` and `body2`: `q2 = coeff·q1 + offset`.
fn couple(world: &mut PhysicsWorld, body1: RigidBodyHandle, body2: RigidBodyHandle, coeff: Real) {
    let coupling = MultibodyDofCoupling {
        link1: link_id(world, body1),
        dof1: 0,
        axis1: 3,
        link2: link_id(world, body2),
        dof2: 0,
        axis2: 3,
        coeff,
        offset: 0.0,
    };
    multibody_of_mut(world, body1).add_dof_coupling(coupling);
}

/// The couplings of the multibody containing `body`, as `(body1, body2, coeff)`.
fn couplings(
    world: &PhysicsWorld,
    body: RigidBodyHandle,
) -> Vec<(RigidBodyHandle, RigidBodyHandle, Real)> {
    let mb = multibody_of(world, body);
    mb.couplings()
        .iter()
        .map(|c| {
            assert_eq!((c.dof1, c.dof2, c.axis1, c.axis2), (0, 0, 3, 3));
            (
                mb.link(c.link1).unwrap().rigid_body_handle(),
                mb.link(c.link2).unwrap().rigid_body_handle(),
                c.coeff,
            )
        })
        .collect()
}

#[test]
fn couplings_and_self_contacts_survive_topology_changes() {
    let mut world = PhysicsWorld::new();

    // Multibody A: a fixed base with two coupled sibling hinges.
    let base = world.insert_body(RigidBodyBuilder::fixed());
    let a1 = insert_link(&mut world, 1.0);
    let a2 = insert_link(&mut world, 2.0);
    hinge(&mut world, base, a1);
    hinge(&mut world, base, a2);
    couple(&mut world, a1, a2, 1.0);

    // Link insertion: a new link hanging from `a2`.
    let a3 = insert_link(&mut world, 3.0);
    hinge(&mut world, a2, a3);
    assert_eq!(couplings(&world, a1), vec![(a1, a2, 1.0)]);

    // Multibody B: a dynamic root with two coupled hinges and self-contacts disabled.
    let b0 = insert_link(&mut world, 10.0);
    let b1 = insert_link(&mut world, 11.0);
    let b2 = insert_link(&mut world, 12.0);
    hinge(&mut world, b0, b1);
    hinge(&mut world, b1, b2);
    couple(&mut world, b1, b2, -0.5);
    multibody_of_mut(&mut world, b0).set_self_contacts_enabled(false);
    assert!(multibody_of(&world, a1).self_contacts_enabled());

    // Merge: B's root becomes a child of `a3`.
    world
        .multibody_joints
        .insert(a3, b0, RevoluteJointBuilder::new(Vector::Z), true)
        .unwrap();
    assert_eq!(couplings(&world, a1), vec![(a1, a2, 1.0), (b1, b2, -0.5)]);
    assert!(!multibody_of(&world, a1).self_contacts_enabled());

    // Split: detaching B again keeps each coupling on its side.
    let (b0_joint, _, _) = world.multibody_joints.joint_between(a3, b0).unwrap();
    world.multibody_joints.remove(b0_joint, true);
    assert_eq!(couplings(&world, a1), vec![(a1, a2, 1.0)]);
    assert_eq!(couplings(&world, b1), vec![(b1, b2, -0.5)]);
    assert!(!multibody_of(&world, b1).self_contacts_enabled());

    // Detaching `a2` removes the joint owning one of the coupled DoFs: the coupling is dropped.
    let (a2_joint, _, _) = world.multibody_joints.joint_between(base, a2).unwrap();
    world.multibody_joints.remove(a2_joint, true);
    assert!(couplings(&world, a1).is_empty());
    assert!(couplings(&world, a2).is_empty());

    // The simulation runs fine with the remapped couplings.
    for _ in 0..10 {
        world.step();
    }
}

#[test]
fn remove_dof_couplings() {
    let mut world = PhysicsWorld::new();
    let base = world.insert_body(RigidBodyBuilder::fixed());
    let l1 = insert_link(&mut world, 1.0);
    let l2 = insert_link(&mut world, 2.0);
    let l3 = insert_link(&mut world, 3.0);
    hinge(&mut world, base, l1);
    hinge(&mut world, base, l2);
    hinge(&mut world, base, l3);
    couple(&mut world, l1, l2, 1.0);
    couple(&mut world, l1, l3, 2.0);
    couple(&mut world, l2, l3, 3.0);

    let mb = multibody_of_mut(&mut world, l1);
    assert_eq!(mb.remove_dof_coupling(1).unwrap().coeff, 2.0);
    assert!(mb.remove_dof_coupling(2).is_none());
    assert_eq!(couplings(&world, l1), vec![(l1, l2, 1.0), (l2, l3, 3.0)]);

    let mb = multibody_of_mut(&mut world, l1);
    mb.retain_dof_couplings(|c| c.coeff > 2.0);
    assert_eq!(couplings(&world, l1), vec![(l2, l3, 3.0)]);

    multibody_of_mut(&mut world, l1).clear_dof_couplings();
    assert!(couplings(&world, l1).is_empty());
}

/// A coupling preserved across a split (removing an unrelated link) is still enforced.
#[test]
fn preserved_coupling_is_enforced() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let base = world.insert_body(RigidBodyBuilder::fixed());
    let l1 = insert_link(&mut world, 1.0);
    let l2 = insert_link(&mut world, 2.0);
    hinge(&mut world, base, l1);
    hinge(&mut world, base, l2);
    couple(&mut world, l1, l2, 1.0);

    // Topology changes after the coupling was declared.
    let l3 = insert_link(&mut world, 3.0);
    hinge(&mut world, l2, l3);
    let (l3_joint, _, _) = world.multibody_joints.joint_between(l2, l3).unwrap();
    world.multibody_joints.remove(l3_joint, true);
    assert_eq!(couplings(&world, l1), vec![(l1, l2, 1.0)]);

    let l1_id = link_id(&world, l1);
    let mb = multibody_of_mut(&mut world, l1);
    let joint = &mut mb.links_mut().nth(l1_id).unwrap().joint.data;
    joint.set_motor_position(JointAxis::AngX, 0.5, 100.0, 20.0);

    for _ in 0..300 {
        world.step();
    }
    let q1 = world.bodies[l1].rotation().to_scaled_axis().z;
    let q2 = world.bodies[l2].rotation().to_scaled_axis().z;
    assert!((q1 - 0.5).abs() < 0.05, "q1 = {q1}");
    assert!((q2 - q1).abs() < 0.02, "q1 = {q1}, q2 = {q2}");
}
