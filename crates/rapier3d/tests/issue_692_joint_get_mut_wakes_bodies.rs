//! Regression test for https://github.com/dimforge/rapier/issues/692
//!
//! Modifying a joint attached to sleeping bodies did not wake them, so the change had no
//! visible effect: `ImpulseJointSet::get_mut(handle, true)` must wake both bodies.

use rapier3d::prelude::*;

#[test]
fn joint_get_mut_wakes_sleeping_bodies() {
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

    let joint = RevoluteJointBuilder::new(Vector::X)
        .local_anchor1(Vector::ZERO)
        .local_anchor2(Vector::new(0.0, 2.0, 0.0));
    let joint_handle = world.impulse_joints.insert(kinematic, dynamic, joint, true);

    // Let the dynamic body come to rest and fall asleep.
    let mut steps = 0;
    while !world.bodies[dynamic].is_sleeping() {
        world.step();
        steps += 1;
        assert!(steps < 2000, "dynamic body never fell asleep");
    }

    // Modify the joint's motor on the sleeping pair (wake_up_connected_bodies=true).
    world
        .impulse_joints
        .get_mut(joint_handle, true)
        .unwrap()
        .data
        .set_motor_velocity(JointAxis::AngX, 2.0, 100.0);

    world.step();
    assert!(
        !world.bodies[dynamic].is_sleeping(),
        "modifying the joint via get_mut(handle, true) must wake the connected bodies"
    );

    // The motor must actually start moving the body.
    let mut moved = false;
    for _ in 0..50 {
        world.step();
        moved = moved || world.bodies[dynamic].angvel().length() > 0.1;
    }
    assert!(moved, "the motor change had no effect on the woken body");
}
