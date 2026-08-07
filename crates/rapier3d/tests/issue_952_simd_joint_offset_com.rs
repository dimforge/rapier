//! Regression test for https://github.com/dimforge/rapier/issues/952
//!
//! The SIMD joint-constraint builder forgot to subtract the body's center of mass from the
//! joint's local frames, so joints on CoM-offset bodies were solved incorrectly in SIMD
//! batches. Four batched pendulums and one scalar-lane pendulum must stay in agreement.

use rapier3d::prelude::*;

const SPACING: f32 = 20.0;

fn make_pendulum(world: &mut PhysicsWorld, x: f32) -> RigidBodyHandle {
    let base = world
        .bodies
        .insert(RigidBodyBuilder::fixed().translation(Vector::new(x, 0.0, 0.0)));
    let body = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(x, 0.0, 0.0))
            .can_sleep(false),
    );
    // Collider offset from the body origin: the CoM is NOT at the body origin.
    world.colliders.insert_with_parent(
        ColliderBuilder::capsule_x(1.0, 0.2).translation(Vector::new(1.0, 0.0, 0.0)),
        body,
        &mut world.bodies,
    );

    let joint = RevoluteJointBuilder::new(Vector::Z)
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::ZERO)
        .contacts_enabled(false);
    world.impulse_joints.insert(base, body, joint, true);
    body
}

#[test]
fn simd_and_scalar_joint_paths_agree_with_offset_com() {
    let mut world = PhysicsWorld::new();

    // 4 identical pendulums side by side: candidates for one full SIMD block.
    let batch: Vec<RigidBodyHandle> = (0..4)
        .map(|i| make_pendulum(&mut world, i as f32 * SPACING))
        .collect();
    // One isolated pendulum: lands in the scalar/remainder path.
    let isolated = make_pendulum(&mut world, 4.0 * SPACING);

    let mut max_dev: f32 = 0.0;
    for _ in 0..300 {
        world.step();

        // Compare each pendulum's position relative to its own base.
        let reference = world.bodies[isolated].translation() - Vector::new(4.0 * SPACING, 0.0, 0.0);
        for (i, handle) in batch.iter().enumerate() {
            let rel =
                world.bodies[*handle].translation() - Vector::new(i as f32 * SPACING, 0.0, 0.0);
            max_dev = max_dev.max((rel - reference).length());
        }
    }

    assert!(
        max_dev < 1.0e-3,
        "SIMD-batched pendulums deviate from the scalar one by {max_dev}"
    );
}
