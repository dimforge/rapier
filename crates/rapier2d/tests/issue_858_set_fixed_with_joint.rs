//! Regression test for https://github.com/dimforge/rapier/issues/858
//!
//! Setting a jointed dynamic body's type to `Fixed` after a few steps used to
//! panic in the SIMD joint-grouping code (`interaction_groups.rs`: index out of
//! bounds) on the simd path.

use rapier2d::prelude::*;

#[test]
fn set_body_type_fixed_with_joint_does_not_panic() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, 9.8);

    let a = world.bodies.insert(RigidBodyBuilder::dynamic());
    let b = world.bodies.insert(RigidBodyBuilder::dynamic());
    world
        .impulse_joints
        .insert(a, b, RevoluteJointBuilder::new(), true);

    for _ in 0..5 {
        world.step();
    }

    world.bodies[b].set_body_type(RigidBodyType::Fixed, true);

    for _ in 0..10 {
        world.step();
    }
}
