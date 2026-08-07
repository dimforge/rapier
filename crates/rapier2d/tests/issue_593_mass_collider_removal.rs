//! Regression test for https://github.com/dimforge/rapier/issues/593
//!
//! Removing colliders spread across broad-phase regions occasionally underflowed
//! `subproper_proxy_count` in the old SAP broad phase. That broad phase is gone; this pins
//! that mass removals step without panicking.

use rapier2d::prelude::*;

#[test]
fn removing_many_colliders_across_regions_does_not_panic() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81);

    // A wide grid of fixed colliders (spanning many of the old SAP regions).
    let mut fixed = Vec::new();
    for i in 0..20 {
        for j in 0..20 {
            let handle = world.insert_collider(
                ColliderBuilder::cuboid(0.5, 0.5).translation(Vector::new(
                    i as f32 * 50.0 - 500.0,
                    j as f32 * 50.0 - 500.0,
                )),
                None,
            );
            fixed.push(handle);
        }
    }

    // Some dynamic bodies moving around, including one that gets teleported
    // (the reported trigger involved resetting a body's position).
    let mut dynamics = Vec::new();
    for i in 0..10 {
        let (body, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(i as f32 * 40.0 - 200.0, 100.0)),
            ColliderBuilder::ball(1.0),
        );
        dynamics.push(body);
    }

    for _ in 0..10 {
        world.step();
    }

    // Remove every other fixed collider, teleport a body, keep stepping.
    for (n, handle) in fixed.iter().enumerate() {
        if n % 2 == 0 {
            world.remove_collider(*handle);
        }
    }
    world
        .bodies
        .get_mut(dynamics[0])
        .unwrap()
        .set_translation(Vector::new(0.0, -10.0), true);
    for _ in 0..10 {
        world.step();
    }

    // Remove the rest (removed handles are ignored), plus some dynamic bodies.
    for handle in fixed {
        world.remove_collider(handle);
    }
    for body in dynamics.into_iter().take(5) {
        world.remove_body(body);
    }
    for _ in 0..10 {
        world.step();
    }
}
