//! Regression tests for the persistent joint constraint assembly: the solver
//! recycles joint builders across steps, so runtime changes to joints or their
//! attached bodies must correctly invalidate the cached assembly.

#[cfg(feature = "dim2")]
use rapier2d::prelude::*;
#[cfg(feature = "dim3")]
use rapier3d::prelude::*;

fn pendulum(
    world: &mut PhysicsWorld,
    anchor_pos: Vector,
) -> (RigidBodyHandle, RigidBodyHandle, ImpulseJointHandle) {
    let anchor = world
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(anchor_pos));
    let bob = world
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(anchor_pos - Vector::Y * 2.0));
    world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(0.2), bob, &mut world.bodies);
    #[cfg(feature = "dim2")]
    let joint = RevoluteJointBuilder::new().local_anchor2(Vector::Y * 2.0);
    #[cfg(feature = "dim3")]
    let joint = SphericalJointBuilder::new().local_anchor2(Vector::Y * 2.0);
    let j = world.impulse_joints.insert(anchor, bob, joint, true);
    (anchor, bob, j)
}

/// A joint removed after the assembly was cached must stop constraining.
#[test]
fn removed_joint_stops_constraining() {
    let mut world = PhysicsWorld::new();
    let (_, bob, joint) = pendulum(&mut world, Vector::ZERO);

    for _ in 0..30 {
        world.step();
    }
    let y_held = world.bodies[bob].translation().y;
    assert!(y_held > -2.5, "joint should hold the bob (y = {y_held})");

    world.impulse_joints.remove(joint, true);
    for _ in 0..60 {
        world.step();
    }
    let y_free = world.bodies[bob].translation().y;
    assert!(
        y_free < -4.0,
        "bob should free-fall after joint removal (y = {y_free})"
    );
}

/// Mutating a joint through `get_mut` after the assembly was cached must take
/// effect on the next step.
#[test]
fn joint_mutation_invalidates_cached_assembly() {
    let mut world = PhysicsWorld::new();
    let (_, bob, joint) = pendulum(&mut world, Vector::ZERO);

    for _ in 0..30 {
        world.step();
    }

    // Re-anchor the bob 4.0 below the pivot instead of 2.0.
    world
        .impulse_joints
        .get_mut(joint, true)
        .unwrap()
        .data
        .set_local_anchor2(Vector::Y * 4.0);
    for _ in 0..120 {
        world.step();
    }
    let y = world.bodies[bob].translation().y;
    assert!(
        y < -3.4 && y > -4.6,
        "bob should hang ~4.0 below the pivot after re-anchoring (y = {y})"
    );
}

/// Moving a (fixed) body attached to a joint must invalidate the cached
/// assembly: the joint frame of a fixed body is baked into the cached builder.
#[test]
fn moved_fixed_anchor_invalidates_cached_assembly() {
    let mut world = PhysicsWorld::new();
    let (anchor, bob, _) = pendulum(&mut world, Vector::ZERO);

    for _ in 0..30 {
        world.step();
    }

    let shift = Vector::X * 5.0;
    let new_pos = world.bodies[anchor].translation() + shift;
    world.bodies[anchor].set_translation(new_pos, true);
    for _ in 0..300 {
        world.step();
    }
    // The bob swings, so assert the constraint itself: it must be pinned 2.0
    // away from the *new* anchor (a stale cached joint frame would keep it
    // pinned around the old anchor, 3.0..7.0 away from the new one).
    let dist = (world.bodies[bob].translation() - new_pos).length();
    assert!(
        (dist - 2.0).abs() < 0.3,
        "bob should be pinned 2.0 from the moved anchor (dist = {dist})"
    );
}
