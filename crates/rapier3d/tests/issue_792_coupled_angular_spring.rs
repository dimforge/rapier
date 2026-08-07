//! Regression test for https://github.com/dimforge/rapier/issues/792
//!
//! A joint with coupled angular axes and spring-like motors, attached to a kinematic body,
//! used to panic on a `usize::MAX` index in the one-body joint solver. The coupled angular
//! motor is still a solver no-op, so this only guards against the crash.

use rapier3d::prelude::*;

#[test]
fn coupled_angular_spring_joint_does_not_panic() {
    let mut world = PhysicsWorld::new();

    let kinematic = world
        .bodies
        .insert(RigidBodyBuilder::kinematic_position_based());
    world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(1.0), kinematic, &mut world.bodies);

    let dynamic = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, -5.0, 0.0))
            .linvel(Vector::new(0.1, 0.0, 0.1)),
    );
    world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(1.0), dynamic, &mut world.bodies);

    let mut joint =
        GenericJoint::new(JointAxesMask::LIN_X | JointAxesMask::LIN_Y | JointAxesMask::LIN_Z);
    joint
        .set_local_anchor2(Vector::new(0.0, -5.0, 0.0))
        .set_motor(JointAxis::AngX, 0.0, 0.0, 0.0, 0.5)
        .set_motor(JointAxis::AngZ, 0.0, 0.0, 0.0, 0.5)
        .set_limits(JointAxis::AngX, [0.0, 0.5])
        .set_limits(JointAxis::AngZ, [0.0, 0.5]);
    joint.coupled_axes |= JointAxesMask::ANG_X | JointAxesMask::ANG_Z;

    world.impulse_joints.insert(kinematic, dynamic, joint, true);

    for i in 0..100 {
        if i == 50 {
            // The original repro pushed the dynamic body sideways after a while.
            world.bodies[dynamic].apply_impulse(Vector::new(5.0, 0.0, 0.0), true);
        }
        world.step();
    }
}
