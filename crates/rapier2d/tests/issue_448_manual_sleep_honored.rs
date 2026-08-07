//! Regression test for https://github.com/dimforge/rapier/issues/448
//!
//! Repositioning a moving body with `set_translation(_, false)` and then calling `sleep()`
//! used to not stick: the body stayed awake and kept moving.

use rapier2d::prelude::*;

#[test]
fn manual_sleep_after_reposition_is_honored() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81);

    let body = world
        .bodies
        .insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0)));
    world
        .colliders
        .insert_with_parent(ColliderBuilder::ball(0.5), body, &mut world.bodies);

    // Let it fall for a while so it's clearly awake and moving.
    for _ in 0..10 {
        world.step();
    }
    assert!(!world.bodies[body].is_sleeping());
    assert!(world.bodies[body].linvel().length() > 0.1);

    // Reset and stop it: move it back up (without waking) and put it to sleep.
    let rb = &mut world.bodies[body];
    rb.set_translation(Vector::new(0.0, 20.0), false);
    rb.set_rotation(Rotation::from_angle(0.0), false);
    rb.sleep();

    for _ in 0..10 {
        world.step();
        let rb = &world.bodies[body];
        assert!(rb.is_sleeping(), "manually slept body woke up");
        let pos = rb.translation();
        assert!(
            (pos - Vector::new(0.0, 20.0)).length() < 1.0e-6,
            "sleeping body moved to {pos:?}"
        );
    }
}
