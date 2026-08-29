//! Regression test for https://github.com/dimforge/rapier/issues/985.
//!
//! A fixed-target cache built by one CCD pass must not retain a collider that is
//! removed during a later step where no body is fast enough to run CCD.

use rapier2d::prelude::*;

#[test]
fn fixed_target_removed_without_active_ccd_is_not_reused_by_later_sweep() {
    let mut world = PhysicsWorld::default();
    world.gravity = Vector::new(0.0, -20.0);

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(50.0, 0.5),
    );

    let (obstacle, _) = world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(10.0, 5.0)),
        ColliderBuilder::cuboid(1.0, 1.0),
    );

    world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-10.0, 10.0))
            .linvel(Vector::new(0.0, -30.0)),
        ColliderBuilder::ball(0.25).density(8.0),
    );

    let mut falling_ball = None;
    for tick in 0..=202 {
        if tick == 120 {
            assert!(
                world
                    .bodies
                    .iter()
                    .filter(|(_, body)| body.is_dynamic())
                    .all(|(_, body)| !body.is_ccd_active()),
                "the fixed target must be removed on a step with no active CCD body"
            );
            assert!(world.remove_body(obstacle).is_some());
        }
        if tick == 121 {
            let ball = world
                .insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(10.0, 25.0)),
                    ColliderBuilder::ball(0.25).density(8.0),
                )
                .0;
            assert!(!world.bodies[ball].is_ccd_active());
            falling_ball = Some(ball);
        }

        world.step();
    }

    let falling_ball = falling_ball.unwrap();
    assert!(
        world.bodies[falling_ball].is_ccd_active(),
        "the later body must become fast enough to exercise the fixed-target sweep"
    );
    assert!(
        world.bodies[falling_ball].translation().y.is_finite(),
        "the later CCD sweep should complete with only live fixed-target handles"
    );
}
