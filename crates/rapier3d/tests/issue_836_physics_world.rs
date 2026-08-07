//! Regression test for #836: a convenience struct wrapping all of rapier's
//! setup code exists (`PhysicsWorld`) — build, step and query a scene through
//! it alone.

use rapier3d::prelude::*;

#[test]
fn physics_world_builds_steps_and_queries_a_scene() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);

    // Ground.
    let ground = world.insert_body(RigidBodyBuilder::fixed());
    world.insert_collider(ColliderBuilder::cuboid(100.0, 0.1, 100.0), Some(ground));

    // Bouncing ball.
    let (ball, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 10.0, 0.0)),
        ColliderBuilder::ball(0.5).restitution(0.7),
    );

    let mut min_y = Real::MAX;
    for _ in 0..200 {
        world.step();
        min_y = min_y.min(world.bodies[ball].translation().y);
    }

    // The ball fell under gravity and was stopped by the ground collider.
    let final_y = world.bodies[ball].translation().y;
    assert!(final_y < 9.0, "ball did not fall (y = {final_y})");
    assert!(
        min_y > 0.4,
        "ball tunneled through the ground (min_y = {min_y})"
    );

    // Queries work directly through the world.
    let ray = Ray::new(
        Vector::new(0.0, 20.0, 0.0).into(),
        Vector::new(0.0, -1.0, 0.0),
    );
    let hit = world.cast_ray(&ray, 100.0, true, QueryFilter::default());
    assert!(hit.is_some(), "raycast through PhysicsWorld found no hit");
}
