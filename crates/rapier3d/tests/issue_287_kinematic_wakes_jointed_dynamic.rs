//! Regression test for https://github.com/dimforge/rapier/issues/287
//!
//! A sleeping dynamic body jointed to a kinematic body was not woken when the kinematic
//! body moved. Joints are now first-class island links, so driving the kinematic body must
//! wake and drag the dynamic one.

use rapier3d::prelude::*;

#[test]
fn moving_kinematic_wakes_jointed_dynamic() {
    let mut world = PhysicsWorld::new();

    let kinematic = world
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based());
    let dynamic = world
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, -2.0, 0.0)));
    world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(0.5), dynamic, &mut world.bodies);

    // Revolute pendulum: the dynamic body hangs 2 units below the kinematic one.
    let joint = RevoluteJointBuilder::new(Vector::X)
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::new(0.0, 2.0, 0.0));
    world.impulse_joints.insert(kinematic, dynamic, joint, true);

    // Let the dynamic body come to rest and fall asleep.
    let mut steps = 0;
    while !world.bodies[dynamic].is_sleeping() {
        world.step();
        steps += 1;
        assert!(steps < 2000, "dynamic body never fell asleep");
    }

    // Drive the kinematic body sideways; the dynamic body must wake up and follow.
    let mut woke = false;
    let mut x = 0.0;
    for _ in 0..200 {
        x += 0.05;
        world.bodies[kinematic].set_next_kinematic_translation(Vector::new(x, 0.0, 0.0));
        world.step();
        woke = woke || !world.bodies[dynamic].is_sleeping();
    }

    assert!(woke, "dynamic body never woke up");
    let final_x = world.bodies[dynamic].translation().x;
    assert!(
        (final_x - x).abs() < 2.0,
        "dynamic body did not follow the kinematic body (x: {final_x}, expected ~{x})"
    );
}
