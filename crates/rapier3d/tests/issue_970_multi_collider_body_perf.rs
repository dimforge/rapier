//! Regression test for https://github.com/dimforge/rapier/issues/970
//!
//! A single dynamic body with thousands of colliders reportedly kept the step slow forever
//! (26 ms/step in rapier.js 0.19 for 2200 colliders), even though same-parent colliders
//! never collide. Timing-based, so `#[ignore]`d: run manually in release with
//! `-- --ignored --nocapture`.

use rapier3d::prelude::*;
use std::time::{Duration, Instant};

#[test]
#[ignore = "timing-based perf check; run manually in release"]
fn multi_collider_body_steps_fast_and_sleeps() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);

    let _ground = world.insert_collider(ColliderBuilder::cuboid(100.0, 0.5, 100.0), None);

    // One dynamic body with 2000 colliders (a 50x40 slab of unit boxes),
    // resting just above the ground.
    let body =
        world.insert_body(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 1.05, 0.0)));
    for i in 0..50 {
        for j in 0..40 {
            world.insert_collider(
                ColliderBuilder::cuboid(0.5, 0.5, 0.5).translation(Vector::new(
                    i as f32 - 25.0,
                    0.0,
                    j as f32 - 20.0,
                )),
                Some(body),
            );
        }
    }

    let mut step_times = Vec::with_capacity(400);
    let total = Instant::now();
    for _ in 0..400 {
        let t = Instant::now();
        world.step();
        step_times.push(t.elapsed());
    }
    let total = total.elapsed();

    let avg = |s: &[Duration]| s.iter().sum::<Duration>() / s.len() as u32;
    let first_10 = avg(&step_times[..10]);
    let last_10 = avg(&step_times[390..]);
    println!("total: {total:?}, first 10 steps avg: {first_10:?}, last 10 steps avg: {last_10:?}");

    assert!(
        world.bodies[body].is_sleeping(),
        "multi-collider body never fell asleep"
    );
    // The reported bug was a *persistently* slow step; once asleep it must be
    // orders of magnitude below the reported 26ms — 3ms is still generous.
    assert!(
        last_10 < Duration::from_millis(3),
        "step still slow once asleep: {last_10:?}"
    );
    assert!(
        total < Duration::from_secs(30),
        "runaway total time: {total:?}"
    );
}
