//! Regression tests for reported multibody bugs.
//!
//! Each test reproduces a specific issue from the GitHub tracker. The crash
//! tests assert that a known-bad scenario steps (or mutates the joint set)
//! without panicking; the behavior tests assert that the simulation result
//! matches what the multibody solver is supposed to produce.

#[cfg(feature = "dim3")]
use crate::alloc_prelude::*;
use crate::prelude::*;

/// Regression test for <https://github.com/dimforge/rapier/issues/927> (Bug 1).
///
/// Removing a joint that leaves one of its bodies as an isolated single-link
/// multibody must clean up that body's `rb2mb` entry. Otherwise the entry keeps
/// a stale `MultibodyIndex` pointing at a freed arena slot, and the next call
/// to `MultibodyJointSet::iter()` panics with "No element at index".
#[test]
#[cfg(feature = "dim3")]
fn issue_927_remove_isolating_single_link_keeps_iter_valid() {
    let mut world = PhysicsWorld::new();

    let a = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 5.0, 0.0))
            .additional_mass(1.0),
    );
    let b = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(1.0, 5.0, 0.0))
            .additional_mass(1.0),
    );
    let c = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(2.0, 5.0, 0.0))
            .additional_mass(1.0),
    );

    let joint = || RevoluteJointBuilder::new(Vector::new(0.0, 0.0, 1.0));
    world.insert_multibody_joint(a, b, joint());
    let handle_bc = world.insert_multibody_joint(b, c, joint()).unwrap();

    // Removing B->C isolates C as a single-link multibody.
    world.remove_multibody_joint(handle_bc);

    // Used to panic: "No element at index".
    let count = world.multibody_joints.iter().count();
    assert_eq!(count, 1, "only the A->B joint should remain");
}

/// Regression test for <https://github.com/dimforge/rapier/issues/927> (Bug 2).
///
/// Building multi-link sub-chains separately and then connecting them to a
/// common parent forces `Multibody::append()` to merge subtrees that have more
/// than one link. A bad `internal_id` rebase used to make `fill_jacobians`
/// read `body_jacobians` out of bounds during the velocity solve.
#[test]
#[cfg(feature = "dim3")]
fn issue_927_branching_multibody_tree_steps_without_panic() {
    let mut world = PhysicsWorld::new();

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0),
    );

    let make_body = |world: &mut PhysicsWorld, pos: Vector| {
        world
            .insert(
                RigidBodyBuilder::dynamic()
                    .translation(pos)
                    .additional_mass(5.0),
                ColliderBuilder::ball(0.3),
            )
            .0
    };

    let (chassis, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 1.5, 0.0))
            .additional_mass(50.0),
        ColliderBuilder::cuboid(2.0, 0.3, 1.0),
    );

    let joint = || RevoluteJointBuilder::new(Vector::new(1.0, 0.0, 0.0));

    // Two front sub-chains with three links each (so `append()` merges a
    // subtree with more than one link), and two rear sub-chains with two links.
    let axle_fl = make_body(&mut world, Vector::new(-1.5, 1.2, 1.2));
    let mangueta_fl = make_body(&mut world, Vector::new(-1.5, 1.0, 1.5));
    let wheel_fl = make_body(&mut world, Vector::new(-1.5, 0.5, 1.8));
    world.insert_multibody_joint(axle_fl, mangueta_fl, joint());
    world.insert_multibody_joint(mangueta_fl, wheel_fl, joint());

    let axle_fr = make_body(&mut world, Vector::new(1.5, 1.2, 1.2));
    let mangueta_fr = make_body(&mut world, Vector::new(1.5, 1.0, 1.5));
    let wheel_fr = make_body(&mut world, Vector::new(1.5, 0.5, 1.8));
    world.insert_multibody_joint(axle_fr, mangueta_fr, joint());
    world.insert_multibody_joint(mangueta_fr, wheel_fr, joint());

    let axle_rl = make_body(&mut world, Vector::new(-1.5, 1.2, -1.2));
    let wheel_rl = make_body(&mut world, Vector::new(-1.5, 0.5, -1.5));
    world.insert_multibody_joint(axle_rl, wheel_rl, joint());

    let axle_rr = make_body(&mut world, Vector::new(1.5, 1.2, -1.2));
    let wheel_rr = make_body(&mut world, Vector::new(1.5, 0.5, -1.5));
    world.insert_multibody_joint(axle_rr, wheel_rr, joint());

    // Connecting each sub-chain to the chassis triggers `append()` on a
    // multi-link subtree.
    world.insert_multibody_joint(chassis, axle_fl, joint());
    world.insert_multibody_joint(chassis, axle_fr, joint());
    world.insert_multibody_joint(chassis, axle_rl, joint());
    world.insert_multibody_joint(chassis, axle_rr, joint());

    for _ in 0..30 {
        world.step();
    }

    assert!(world.bodies[chassis].translation().y.is_finite());
}

/// Regression test for <https://github.com/dimforge/rapier/issues/906>.
///
/// Inserting a multibody joint *after* the pipeline has already stepped at
/// least once used to overflow a subtraction in `Multibody::append()`.
/// Inserting joints only during initialization, or using impulse joints, did
/// not trigger it.
#[test]
#[cfg(feature = "dim3")]
fn issue_906_insert_multibody_joint_between_steps() {
    const SHIFT: Real = 1.15;

    let mut world = PhysicsWorld::new();

    let (root, _) = world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );

    let joint = || {
        RevoluteJointBuilder::new(Vector::new(0.0, 1.0, 0.0))
            .local_anchor1(Vector::new(0.0, -SHIFT, 0.0))
    };

    // Build a short chain before the first step (this path always worked).
    let mut last = root;
    for i in 1..4 {
        let (body, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(0.0, -SHIFT * i as Real, 0.0)),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        );
        world.insert_multibody_joint(last, body, joint());
        last = body;
    }

    world.step();

    // Now extend the chain dynamically, between steps. This used to panic with
    // "attempt to subtract with overflow" in `Multibody::append()`.
    for i in 4..10 {
        let (body, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(0.0, -SHIFT * i as Real, 0.0)),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        );
        world.insert_multibody_joint(last, body, joint());
        last = body;

        world.step();
    }

    assert!(world.bodies[last].translation().y.is_finite());
}

/// Regression test for <https://github.com/dimforge/rapier/issues/907>.
///
/// A multibody-jointed body and a free rigid body both falling onto the same
/// ground used to crash the velocity solver's warm-start with a nalgebra
/// "Matrix slicing out of bounds" panic in `GenericRhs::apply_impulse`.
#[test]
#[cfg(feature = "dim3")]
fn issue_907_body_colliding_with_multibody() {
    let mut world = PhysicsWorld::new();

    let (ground, _) = world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0),
    );

    // A spinning body attached to the ground with a revolute multibody joint.
    let (attached, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 3.0, 0.0))
            .angvel(Vector::new(0.0, 1.0, 0.0)),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    world.insert_multibody_joint(
        ground,
        attached,
        RevoluteJointBuilder::new(Vector::new(0.0, 1.0, 0.0))
            .local_anchor1(Vector::new(0.0, 3.0, 0.0)),
    );

    // A free body that falls onto the ground next to the multibody.
    let (free, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(3.0, 3.0, 0.0)),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );

    for _ in 0..400 {
        world.step();
    }

    assert!(world.bodies[free].translation().y.is_finite());
}

/// Regression test for <https://github.com/dimforge/rapier/issues/908>.
///
/// Removing the links of a multibody chain one by one, while a free body rests
/// in contact with the chain's fixed root, used to crash the contact solver
/// with "No element at index" in `GenericContactConstraintBuilder::generate`.
#[test]
#[cfg(feature = "dim3")]
fn issue_908_remove_body_from_multibody_chain() {
    let mut world = PhysicsWorld::new();

    let (root, _) = world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );

    // A chain of spherical-jointed bodies hanging from the root.
    let mut chain = vec![root];
    for i in 1..4 {
        let (body, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(0.0, -2.0 * i as Real, 0.0)),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        );
        world.insert_multibody_joint(
            *chain.last().unwrap(),
            body,
            SphericalJointBuilder::new().local_anchor1(Vector::new(0.0, -2.0, 0.0)),
        );
        chain.push(body);
    }

    // A free body resting on the root. The crash only happened with it present.
    let (free, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.0, 0.0)),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );

    // Let everything settle, then peel the chain off one body at a time,
    // stepping in-between. Removing a link used to panic.
    for _ in 0..32 {
        world.step();
    }

    while chain.len() > 1 {
        let body = chain.pop().unwrap();
        world.remove_body(body);

        for _ in 0..32 {
            world.step();
        }
    }

    assert!(world.bodies[free].translation().y.is_finite());
}

/// Regression test for <https://github.com/dimforge/rapier/issues/400>.
///
/// A dynamic body attached to a *fixed* parent through a `MultibodyJoint`,
/// plus a free dynamic body colliding with that fixed parent, used to crash
/// the moment the free body touched the parent.
#[test]
#[cfg(feature = "dim3")]
fn issue_400_multibody_joint_fixed_parent_dynamic_collision() {
    let mut world = PhysicsWorld::new();

    // The fixed "table".
    let (table, _) = world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(1.0, 0.1, 1.0),
    );

    // A "flipper": dynamic body attached to the fixed table by a multibody
    // revolute joint, with a velocity motor.
    let (flipper, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(-0.5, 0.3, -0.5)),
        ColliderBuilder::cuboid(0.1, 0.1, 0.1),
    );
    world.insert_multibody_joint(
        table,
        flipper,
        RevoluteJointBuilder::new(Vector::new(0.0, 1.0, 0.0))
            .local_anchor1(Vector::new(-0.5, 0.3, -0.5))
            .motor_velocity(-1.0, 1.0),
    );

    // The "ball" that falls onto the table.
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.0, 0.0)),
        ColliderBuilder::ball(0.01).friction(0.0),
    );

    for _ in 0..200 {
        world.step();
    }

    assert!(world.bodies[ball].translation().y.is_finite());
}

/// Regression test for <https://github.com/dimforge/rapier/issues/656>.
///
/// A free-floating multibody whose root collides off-center with a ledge must
/// pick up angular velocity, just like the same setup built with an impulse
/// joint does. The bug left the multibody root unable to react to the
/// collision torque.
#[test]
#[cfg(feature = "dim2")]
fn issue_656_multibody_root_reacts_to_collision_torque() {
    let impulse_angvel = run_issue_656_scenario(false);
    let multibody_angvel = run_issue_656_scenario(true);

    // Sanity check: the off-center landing must spin the body in the
    // (known-good) impulse-joint case.
    assert!(
        impulse_angvel > 0.1,
        "scenario should induce rotation; impulse-joint baseline = {impulse_angvel}"
    );

    // Regression check: the multibody root must react to the collision torque
    // comparably. With the bug its angular velocity stays near zero.
    assert!(
        multibody_angvel > impulse_angvel * 0.25,
        "multibody root barely rotated: multibody = {multibody_angvel}, impulse = {impulse_angvel}"
    );
}

/// Builds the issue-656 scenario (two co-located dynamic bodies joined at their
/// center, free-falling onto a ledge they only partly overlap) and returns the
/// largest angular velocity magnitude the root body reaches.
#[cfg(feature = "dim2")]
fn run_issue_656_scenario(use_multibody: bool) -> Real {
    let mut world = PhysicsWorld::new();

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -2.0)),
        ColliderBuilder::cuboid(0.5, 0.5),
    );

    // Both bodies start at the same place; the joint pins them at that point.
    let start = Vector::new(0.70, 0.0);
    let (root, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(start),
        ColliderBuilder::cuboid(0.3, 0.1).mass(0.1),
    );
    let (child, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(start),
        ColliderBuilder::cuboid(0.05, 0.05).mass(0.1),
    );

    let joint = RevoluteJointBuilder::new()
        .local_anchor1(Vector::new(0.0, 0.0))
        .local_anchor2(Vector::new(0.0, 0.0))
        .contacts_enabled(false);
    if use_multibody {
        world.insert_multibody_joint(root, child, joint);
    } else {
        world.insert_impulse_joint(root, child, joint);
    }

    let mut max_angvel: Real = 0.0;
    for _ in 0..600 {
        world.step();
        max_angvel = max_angvel.max(world.bodies[root].angvel().abs());
    }
    max_angvel
}

/// Regression test for <https://github.com/dimforge/rapier/issues/379>.
///
/// Angular limits set on a ball (spherical) joint must be enforced by the
/// multibody constraint solver. The bug ignored them entirely, so the child
/// swung freely as if no limit had been set.
#[test]
#[cfg(feature = "dim3")]
fn issue_379_multibody_ball_joint_respects_angular_limits() {
    const LIMIT: Real = 0.3;

    let mut world = PhysicsWorld::new();

    // Fixed root at the origin.
    let root = world.insert_body(RigidBodyBuilder::fixed());

    // A child whose left end is pinned to the origin, extending along +X. Under
    // gravity it tries to swing down (rotating about the Z axis).
    let (child, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(1.0, 0.0, 0.0))
            .additional_mass(1.0),
        ColliderBuilder::cuboid(1.0, 0.1, 0.1),
    );

    world.insert_multibody_joint(
        root,
        child,
        SphericalJointBuilder::new()
            .local_anchor1(Vector::new(0.0, 0.0, 0.0))
            .local_anchor2(Vector::new(-1.0, 0.0, 0.0))
            .limits(JointAxis::AngZ, [-LIMIT, LIMIT]),
    );

    let mut lowest: Real = 0.0;
    for _ in 0..600 {
        world.step();
        lowest = lowest.min(world.bodies[child].translation().y);
    }

    // Gravity pulls the child down until the `AngZ` limit stops it, so the
    // child center settles at about `y = -sin(LIMIT)` (~ -0.30):
    //  * if the limit is ignored, it keeps swinging down towards y = -1.0;
    //  * if the child somehow never swings, it stays at y = 0.
    // Bracketing `lowest` around the expected stop rejects both failure modes.
    let max_drop = -LIMIT.sin();
    assert!(
        lowest > max_drop - 0.15,
        "ball joint ignored its angular limit: child dropped to y = {lowest}, \
         but the {LIMIT} rad limit should keep it above {max_drop}"
    );
    assert!(
        lowest < max_drop + 0.1,
        "child never reached the joint limit (y = {lowest}); the test is not \
         exercising the limit"
    );
}

/// Companion to [`issue_906_insert_multibody_joint_between_steps`], covering the
/// variant that motivated the question in issue #906's discussion: attaching a
/// body with a **fixed** multibody joint (`joint.ndofs() == 0`) to the
/// **non-dynamic root** of an existing multibody, between steps.
///
/// This is the same `Multibody::append()` underflow as #906 — the old
/// `self.velocities.len() - rhs_root_ndofs` subtraction underflows whenever
/// `self` has a fixed (small-`ndofs`) root, regardless of the connecting
/// joint's dof count. The fix's per-link arithmetic is underflow-free for any
/// `joint.ndofs()`, including 0.
#[test]
#[cfg(feature = "dim3")]
fn issue_906_append_fixed_joint_to_fixed_multibody_root() {
    let mut world = PhysicsWorld::new();

    // A multibody with a fixed (non-dynamic) root and one revolute child.
    let (root, _) = world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    let (child, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, -2.0, 0.0))
            .additional_mass(1.0),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    world.insert_multibody_joint(
        root,
        child,
        RevoluteJointBuilder::new(Vector::new(0.0, 0.0, 1.0))
            .local_anchor1(Vector::new(0.0, -2.0, 0.0)),
    );

    // Step once so `update_root_type` recognizes the root as non-dynamic.
    world.step();

    // Attach a new body to the (now non-dynamic) root with a *fixed* joint.
    let (attached, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(2.0, 0.0, 0.0))
            .additional_mass(1.0),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    let handle = world.insert_multibody_joint(
        root,
        attached,
        FixedJointBuilder::new().local_anchor1(Vector::new(2.0, 0.0, 0.0)),
    );
    assert!(
        handle.is_some(),
        "attaching a fixed joint to a fixed multibody root should be allowed"
    );

    let start = world.bodies[attached].translation();
    for _ in 0..60 {
        world.step();
    }
    let end = world.bodies[attached].translation();

    // A fixed joint to a fixed root contributes zero dofs: the attached body
    // must stay rigidly in place despite gravity.
    let drift = (end.x - start.x).abs() + (end.y - start.y).abs() + (end.z - start.z).abs();
    assert!(
        drift < 1.0e-3,
        "body fixed-jointed to a fixed root should not move, but drifted by {drift}"
    );
    // The pre-existing revolute branch must still simulate.
    assert!(world.bodies[child].translation().y.is_finite());
    assert_eq!(world.multibody_joints.iter().count(), 2);
}

/// Regression test for an issue #907 variant uncovered while reviewing that
/// fix. The #907 contact-side fix only treats a multibody's *fixed root* as a
/// plain fixed body — but a body attached to that root (here with a fixed
/// joint) is a *non-root link*, so it is still handled as a multibody link.
///
/// Such a link sits in a branch joined to the rest of the multibody only
/// *through* the fixed root. Island traversals never cross fixed bodies, so the
/// branch used to land in a separate island from the multibody's solver
/// representative — and a contact against it hit the same empty
/// `generic_solver_vels` out-of-bounds as #907. The fix keeps every link of a
/// multibody in a single island (island-merge anchoring in `do_insert` +
/// whole-multibody island traversal in `push_linked_bodies`).
#[test]
#[cfg(feature = "dim3")]
fn issue_907_contact_with_branch_off_fixed_root() {
    let mut world = PhysicsWorld::new();

    // A multibody with a fixed root and a revolute child.
    let (root, _) = world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    let (child, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, -3.0, 0.0))
            .additional_mass(1.0),
        ColliderBuilder::ball(0.5),
    );
    world.insert_multibody_joint(
        root,
        child,
        RevoluteJointBuilder::new(Vector::new(0.0, 0.0, 1.0))
            .local_anchor1(Vector::new(0.0, -3.0, 0.0)),
    );

    // Step once so the root is recognized as non-dynamic.
    world.step();

    // A second branch off the (non-dynamic) root, far from `child` — it lands
    // in a different island unless the whole multibody is kept together.
    let (branch, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(5.0, 0.0, 0.0))
            .additional_mass(1.0),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    world.insert_multibody_joint(
        root,
        branch,
        FixedJointBuilder::new().local_anchor1(Vector::new(5.0, 0.0, 0.0)),
    );

    // A free body that falls onto `branch`: the contact used to crash the
    // velocity solver's warm-start with an out-of-bounds matrix slice.
    let (free, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(5.0, 3.0, 0.0))
            .additional_mass(1.0),
        ColliderBuilder::cuboid(0.3, 0.3, 0.3),
    );

    for _ in 0..200 {
        world.step();
    }

    // The free body must come to rest on top of `branch` (`branch` top at
    // y = 0.5, free half-height 0.3 → free center settles around y = 0.8).
    let free_y = world.bodies[free].translation().y;
    assert!(
        (free_y - 0.8).abs() < 0.15,
        "free body should rest on the fixed-jointed branch, but settled at y = {free_y}"
    );
    assert!(world.bodies[child].translation().y.is_finite());
}
