//! Regression test for https://github.com/dimforge/rapier/issues/952
//!
//! The SIMD joint-constraint builder forgot to subtract the body's center of mass from the
//! joint's local frames. Solver bodies live in center-of-mass space, so a joint anchored on a
//! body whose CoM is offset from its origin was pinned at the CoM instead of at the anchor,
//! displacing the body by exactly the CoM offset.
//!
//! The SIMD joint path only kicks in once a single constraint color holds at least
//! `JOINT_BATCH * LAYOUT_REF_WORKERS / 2` (= 64) joints, so the scene must be big enough:
//! with fewer joints everything falls back to the (correct) scalar path and the bug hides.

use rapier2d::prelude::*;

/// Comfortably above the 64-joint SIMD batching threshold.
const PENDULUMS: usize = 128;
const SPACING: f32 = 20.0;
/// Distance between the body origin (where the joint is anchored) and the collider's center.
const COM_OFFSET: f32 = 1.0;

fn make_pendulum(world: &mut PhysicsWorld, x: f32) -> RigidBodyHandle {
    let base_pos = Vector::new(x, 0.0);
    let base = world
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(base_pos));
    let body = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(base_pos)
            .can_sleep(false),
    );
    // Collider offset from the body origin: the CoM is NOT at the body origin.
    world.colliders.insert_with_parent(
        ColliderBuilder::capsule_x(COM_OFFSET, 0.2).translation(Vector::new(COM_OFFSET, 0.0)),
        body,
        &mut world.bodies,
    );

    // Pins the body's origin (local anchor `ZERO`) to the base position.
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::ZERO)
        .contacts_enabled(false);
    world.impulse_joints.insert(base, body, joint, true);
    body
}

/// Runs `n` independent pendulums and returns the largest distance ever seen between a body's
/// joint anchor and the base it is pinned to.
fn max_anchor_error(n: usize) -> f32 {
    let mut world = PhysicsWorld::new();
    let bodies: Vec<_> = (0..n)
        .map(|i| make_pendulum(&mut world, i as f32 * SPACING))
        .collect();

    let mut max_err: f32 = 0.0;
    for _ in 0..120 {
        world.step();
        for (i, handle) in bodies.iter().enumerate() {
            let base_pos = Vector::new(i as f32 * SPACING, 0.0);
            let anchor_world = world.bodies[*handle].position() * Vector::ZERO;
            max_err = max_err.max((anchor_world - base_pos).length());
        }
    }
    max_err
}

#[test]
fn simd_joints_respect_offset_center_of_mass() {
    // Small scene: scalar path only. Establishes the reference accuracy.
    let scalar_err = max_anchor_error(8);
    assert!(
        scalar_err < 1.0e-2,
        "the scalar joint path itself is broken: anchor error {scalar_err}"
    );

    // Large scene: the joints get batched into SIMD constraint groups. Before the fix the
    // anchor error jumped to `COM_OFFSET` (1.0) here while staying tiny above.
    let simd_err = max_anchor_error(PENDULUMS);
    assert!(
        simd_err < 1.0e-2,
        "SIMD-batched joints ignore the center-of-mass offset: anchor error {simd_err} \
         (scalar path: {scalar_err})"
    );
}
