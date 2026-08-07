//! Regression test for https://github.com/dimforge/rapier/issues/783
//!
//! Disabling a body while removing an overlapping one between two steps used to trip the
//! old SAP broad phase's `point_key` assert. That broad phase is gone; this pins that the
//! sequence steps without panicking.

use rapier2d::prelude::*;

#[test]
fn disable_body_and_remove_overlapping_body_does_not_panic() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, 0.0);

    // Several "player vs projectile" pairs far apart from each other. Each player
    // has a main collider plus sensor colliders; each projectile has a collider
    // plus a sensor, overlapping the player — mirroring the issue's setup.
    let mut pairs = Vec::new();
    for i in 0..10 {
        let pos = Vector::new(i as f32 * 1000.0, 0.0);
        let player = world.insert_body(RigidBodyBuilder::dynamic().translation(pos));
        world.insert_collider(ColliderBuilder::ball(0.5), Some(player));
        world.insert_collider(ColliderBuilder::ball(1.0).sensor(true), Some(player));
        world.insert_collider(ColliderBuilder::cuboid(1.5, 1.5).sensor(true), Some(player));

        let projectile = world.insert_body(RigidBodyBuilder::dynamic().translation(pos));
        world.insert_collider(ColliderBuilder::ball(0.2), Some(projectile));
        world.insert_collider(ColliderBuilder::ball(0.4).sensor(true), Some(projectile));

        pairs.push((player, projectile));
    }

    for _ in 0..5 {
        world.step();
    }

    // Between two steps: disable each player and despawn the overlapping
    // projectile at the same instant.
    for (player, projectile) in pairs.clone() {
        world.bodies.get_mut(player).unwrap().set_enabled(false);
        world.remove_body(projectile);
        world.step();
    }

    for _ in 0..5 {
        world.step();
    }

    // Re-enable and remove the players too, still stepping.
    for (player, _) in pairs {
        world.bodies.get_mut(player).unwrap().set_enabled(true);
        world.step();
        world.remove_body(player);
        world.step();
    }
}
