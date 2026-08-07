//! Regression test for https://github.com/dimforge/rapier/issues/258
//!
//! Changing a body's type from dynamic to fixed while it has an attached joint
//! (and is asleep) used to panic with an index-out-of-bounds on stale
//! active-body indices.

use rapier2d::prelude::*;

#[test]
fn sleeping_jointed_body_set_fixed_does_not_panic() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;

    let a = world.bodies.insert(RigidBodyBuilder::dynamic());
    let b = world
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(2.0, 0.0)));
    world.impulse_joints.insert(
        a,
        b,
        RevoluteJointBuilder::new().local_anchor1(Vector::new(2.0, 0.0)),
        true,
    );

    // With zero gravity the pair is at rest and falls asleep naturally.
    let mut steps = 0;
    while !(world.bodies[a].is_sleeping() && world.bodies[b].is_sleeping()) {
        world.step();
        steps += 1;
        assert!(steps < 2000, "bodies never fell asleep");
    }

    world.bodies[b].set_body_type(RigidBodyType::Fixed, false);

    for _ in 0..10 {
        world.step();
    }
}
