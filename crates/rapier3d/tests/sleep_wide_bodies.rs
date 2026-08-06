//! A body that has stopped moving must fall asleep however far its shape reaches.
//!
//! Deriving the rotation chord from `sqrt(1 − dot²)` turns dot-product rounding into a drift
//! floor that scales with `max_extent`, so wide still bodies could never sleep.

use rapier3d::prelude::*;

/// A `U`-shaped compound (a wide bar plus two uprights), as in `stress_tests/compound3`:
/// `max_extent` is ~4, well past the point where the old floor swamped the allowance.
fn wide_compound_world() -> (PhysicsWorld, Vec<RigidBodyHandle>) {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.1, 0.0)),
        ColliderBuilder::cuboid(50.0, 0.1, 50.0),
    );

    let rad = 0.2;
    let mut handles = Vec::new();
    // Varied yaws: a single orientation can miss the rounding floor by luck; a spread cannot.
    for i in 0..8 {
        for k in 0..8 {
            let (handle, _) = world.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(i as f32 * 6.0, rad + 0.01, k as f32 * 6.0))
                    .rotation(Vector::Y * (i as f32 * 0.11 + k as f32 * 0.037)),
                ColliderBuilder::cuboid(rad * 10.0, rad, rad),
            );
            world.insert_collider(
                ColliderBuilder::cuboid(rad, rad * 10.0, rad).translation(Vector::new(
                    rad * 10.0,
                    rad * 10.0,
                    0.0,
                )),
                Some(handle),
            );
            world.insert_collider(
                ColliderBuilder::cuboid(rad, rad * 10.0, rad).translation(Vector::new(
                    -rad * 10.0,
                    rad * 10.0,
                    0.0,
                )),
                Some(handle),
            );
            handles.push(handle);
        }
    }
    (world, handles)
}

#[test]
fn wide_bodies_at_rest_fall_asleep() {
    let (mut world, handles) = wide_compound_world();
    // At rest within a few steps + 30 steps of sleep timer; 300 is a wide margin that still
    // fails outright when the drift floor scales with `max_extent`.
    for _ in 0..300 {
        world.step();
    }

    let asleep = handles
        .iter()
        .filter(|h| world.bodies[**h].is_sleeping())
        .count();
    assert_eq!(
        asleep,
        handles.len(),
        "{} of {} wide bodies stayed awake at rest",
        handles.len() - asleep,
        handles.len(),
    );
}

/// The floor is a property of the drift math, not of any particular scene: a body pinned to one
/// pose must report a drift of zero no matter how far its shape reaches.
#[test]
fn still_wide_body_reports_no_drift() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::ZERO;
    let handle = world.insert_body(
        // An orientation whose pose dot product rounds rather than landing exactly on 1.0.
        RigidBodyBuilder::dynamic().rotation(Vector::new(0.3, -0.7, 0.15)),
    );
    world.insert_collider(ColliderBuilder::cuboid(0.2, 8.0, 0.2), Some(handle));

    let pose = *world.bodies[handle].position();
    for _ in 0..200 {
        world.step();
    }

    let body = &world.bodies[handle];
    assert_eq!(
        *body.position(),
        pose,
        "a force-free body should not have moved at all"
    );
    assert!(
        body.is_sleeping(),
        "a motionless body with a far-reaching shape never slept",
    );
}
