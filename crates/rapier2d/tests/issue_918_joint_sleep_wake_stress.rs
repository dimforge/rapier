//! Regression test for https://github.com/dimforge/rapier/issues/918
//!
//! Manually sleeping/waking joint-connected bodies used to trip island assertions on 0.32.
//! Stresses the same pattern: sleep at insertion, then periodically sleep()/wake_up().

use rapier2d::prelude::*;

#[test]
fn joint_sleep_wake_stress_does_not_panic() {
    let mut world = PhysicsWorld::new();

    let a = world.bodies.insert(RigidBodyBuilder::dynamic());
    let b = world
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(2.0, 0.0)));
    world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(0.5), a, &mut world.bodies);
    world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(0.5), b, &mut world.bodies);
    world.impulse_joints.insert(
        a,
        b,
        RevoluteJointBuilder::new().local_anchor1(Vector::new(2.0, 0.0)),
        true,
    );

    // Sleep the bodies right at insertion (before the first step), like the
    // issue's "sleep initial bodies to prevent falling through dynamic terrain".
    world.bodies[a].sleep();
    world.bodies[b].sleep();

    for i in 0..500 {
        match i % 40 {
            10 => {
                world.bodies[a].sleep();
                world.bodies[b].sleep();
            }
            20 => {
                world.bodies[a].wake_up(true);
            }
            30 => {
                world.bodies[b].sleep();
            }
            35 => {
                world.bodies[b].wake_up(false);
            }
            _ => {}
        }
        world.step();
    }
}
