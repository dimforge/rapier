//! Regression test for https://github.com/dimforge/rapier/issues/396
//!
//! A `HalfSpace` collider used to stall the old SAP broad phase in rapier3d-f64, whose
//! region grid looped over the whole `i32` range on the first step. The BVH broad phase has
//! no such discretization.

use rapier3d_f64::na::Unit;
use rapier3d_f64::prelude::*;

#[test]
fn halfspace_ground_does_not_stall_broad_phase() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);

    // Fixed body with a half-space ground, exactly like the issue's setup.
    let ground = world.insert_body(RigidBodyBuilder::fixed());
    world.insert_collider(
        ColliderBuilder::halfspace(Unit::new_unchecked(Vector::new(0.0, 1.0, 0.0))),
        Some(ground),
    );

    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 3.0, 0.0)),
        ColliderBuilder::ball(0.5),
    );

    // The original bug hung forever inside the first `step`; a bounded number of
    // steps completing at all is the regression check.
    for _ in 0..100 {
        world.step();
    }

    // And the half-space must actually act as a ground: the ball rests on it.
    let y = world.bodies[ball].translation().y;
    assert!(
        y > 0.0 && y < 1.0,
        "ball did not rest on the half-space: y = {y}"
    );
}
