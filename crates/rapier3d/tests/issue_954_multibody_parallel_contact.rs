//! Regression test for https://github.com/dimforge/rapier/issues/954
//!
//! Under the `parallel` feature, the first contact involving a multibody link used to panic
//! with "Matrix slicing out of bounds": the on-demand jacobian-buffer resize was gated on
//! `!cfg!(feature = "parallel")`, so the buffer stayed empty forever.

#![cfg(feature = "parallel")]

use rapier3d::prelude::*;

#[test]
fn multibody_chain_contact_does_not_panic_with_parallel() {
    let mut world = PhysicsWorld::new();

    // Free-standing fixed ground (NOT part of any multibody).
    world.insert(
        RigidBodyBuilder::fixed(),
        ColliderBuilder::cuboid(50.0, 0.5, 50.0),
    );

    // A 4-link dynamic chain on spherical joints, starting above the ground.
    let mut prev = None;
    for i in 0..4 {
        let (link, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vec3::new(i as f32 * 1.2, 4.0, 0.0)),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        );
        if let Some(p) = prev {
            world.insert_multibody_joint(
                p,
                link,
                SphericalJointBuilder::new().local_anchor1(Vec3::new(1.2, 0.0, 0.0)),
            );
        }
        prev = Some(link);
    }

    // Used to panic once the chain reached the ground.
    for _ in 0..400 {
        world.step();
    }

    for (_, rb) in world.rigid_bodies() {
        assert!(rb.translation().is_finite());
    }
}
