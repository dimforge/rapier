//! Regression test for https://github.com/dimforge/rapier/issues/309
//!
//! With `lock_rotations()` and a collider offset from the body origin, applied rotations
//! were reported to pivot about a collider corner instead of the center of mass. Spinning
//! in place must leave the world-space center of mass untouched.

use rapier3d::prelude::*;

#[test]
fn angvel_rotation_pivots_about_center_of_mass() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;

    let body = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .lock_rotations()
            .can_sleep(false),
    );
    // Collider offset from the body origin: CoM is at (1, 0, 0), not the origin.
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5).translation(Vector::new(1.0, 0.0, 0.0)),
        body,
        &mut world.bodies,
    );
    world.step();
    let initial_com = world.bodies[body].center_of_mass();

    for i in 0..100 {
        // Manually spin the body each frame (rotation is locked, so only the
        // manual angular velocity rotates it).
        world.bodies[body].set_angvel(Vector::new(0.0, 0.0, 3.0), true);
        world.step();

        let com = world.bodies[body].center_of_mass();
        assert!(
            (com - initial_com).length() < 1.0e-3,
            "manual rotation moved the CoM from {initial_com:?} to {com:?} at step {i}"
        );
    }
}

#[test]
fn set_rotation_does_not_inject_motion() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;

    let body = world.bodies.insert(
        RigidBodyBuilder::dynamic()
            .lock_rotations()
            .can_sleep(false),
    );
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5).translation(Vector::new(1.0, 0.0, 0.0)),
        body,
        &mut world.bodies,
    );

    for i in 0..100 {
        // `set_rotation` is a teleport of the orientation about the body origin;
        // it must not make the body drift or acquire velocity.
        world.bodies[body].set_rotation(Rotation::from_rotation_z(0.05 * i as f32), true);
        world.step();

        let rb = &world.bodies[body];
        let pos = rb.translation();
        assert!(
            pos.length() < 1.0e-4 && rb.linvel().length() < 1.0e-4,
            "manual set_rotation injected motion: pos {pos:?}, linvel {:?} at step {i}",
            rb.linvel()
        );
    }
}
