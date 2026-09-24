//! Zero-length steps (`IntegrationParameters::dt == 0`, e.g. the first frame of a variable
//! timestep) with joint motors of infinite max force (as loaded from MJCF actuators): their
//! impulse bounds (`max_force * dt`) used to be NaN, panicking in the multibody motor constraint.

use rapier3d::prelude::*;

fn arm(world: &mut PhysicsWorld, x: Real, multibody: bool) -> RigidBodyHandle {
    let base = world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(x, 2.0, 0.0)));
    let (link, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(x + 1.0, 2.0, 0.0)),
        ColliderBuilder::cuboid(0.5, 0.1, 0.1),
    );
    let joint = RevoluteJointBuilder::new(Vector::Z)
        .local_anchor2(Vector::new(-1.0, 0.0, 0.0))
        .motor_velocity(1.0, 1000.0)
        .motor_max_force(Real::INFINITY);
    if multibody {
        world.insert_multibody_joint(base, link, joint).unwrap();
    } else {
        world.insert_impulse_joint(base, link, joint);
    }
    link
}

#[test]
fn zero_dt_step_with_infinite_motor_forces() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let multibody_link = arm(&mut world, 0.0, true);
    let impulse_link = arm(&mut world, 5.0, false);
    let before = [
        *world.bodies[multibody_link].position(),
        *world.bodies[impulse_link].position(),
    ];

    world.integration_parameters.dt = 0.0;
    world.step();
    world.step();
    // Nothing moves (up to the multibody forward kinematics' rounding).
    for (link, pose) in [multibody_link, impulse_link].into_iter().zip(before) {
        let rb = &world.bodies[link];
        assert!((rb.position().translation - pose.translation).length() < 1.0e-5);
        assert!(rb.position().rotation.abs_diff_eq(pose.rotation, 1.0e-5));
        assert_eq!(rb.angvel(), Vector::ZERO);
    }

    world.integration_parameters.dt = 1.0 / 60.0;
    for _ in 0..30 {
        world.step();
    }
    for link in [multibody_link, impulse_link] {
        let rb = &world.bodies[link];
        assert!(rb.position().translation.is_finite());
        assert!(rb.angvel().is_finite());
        // The motors drive the arms at their target velocity.
        assert!((rb.angvel().z - 1.0).abs() < 0.05, "{}", rb.angvel());
    }
}
