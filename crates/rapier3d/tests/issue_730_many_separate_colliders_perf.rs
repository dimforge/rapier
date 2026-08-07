//! Regression test for https://github.com/dimforge/rapier/issues/730
//!
//! Many separate colliders on one fixed body used to get slower and slower over a long
//! simulation (old SAP region churn), while a compound collider stayed flat. Timing-based,
//! so `#[ignore]`d: run manually in release with `-- --ignored --nocapture`.

use rapier3d::prelude::*;
use std::time::{Duration, Instant};

#[test]
#[ignore = "timing-based perf check; run manually in release"]
fn separate_colliders_step_time_stays_flat() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);

    // 400 primitives (boxes, cylinders, capsules) attached to a single fixed body.
    let ground = world.insert_body(RigidBodyBuilder::fixed());
    let mut n = 0;
    'grid: for i in 0..20 {
        for j in 0..20 {
            let pos = Vector::new(i as f32 - 10.0, 0.0, j as f32 - 10.0);
            let builder = match n % 3 {
                0 => ColliderBuilder::cuboid(0.5, 0.5, 0.5),
                1 => ColliderBuilder::cylinder(0.5, 0.4),
                _ => ColliderBuilder::capsule_y(0.3, 0.4),
            };
            world.insert_collider(builder.translation(pos), Some(ground));
            n += 1;
            if n == 400 {
                break 'grid;
            }
        }
    }

    // 500 dynamic balls raining on top.
    for k in 0..500 {
        let x = (k % 10) as f32 - 5.0;
        let z = ((k / 10) % 10) as f32 - 5.0;
        let y = 3.0 + (k / 100) as f32 * 1.5;
        world.insert(
            RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z)),
            ColliderBuilder::ball(0.4),
        );
    }

    let mut step_times = Vec::with_capacity(100);
    let total = Instant::now();
    for _ in 0..100 {
        let t = Instant::now();
        world.step();
        step_times.push(t.elapsed());
    }
    let total = total.elapsed();

    let avg = |s: &[Duration]| s.iter().sum::<Duration>() / s.len() as u32;
    let first_10 = avg(&step_times[..10]);
    let last_10 = avg(&step_times[90..]);
    println!("total: {total:?}, first 10 steps avg: {first_10:?}, last 10 steps avg: {last_10:?}");

    // Generous bounds: the old bug was a runaway (monotonic creep), not a constant factor.
    assert!(
        total < Duration::from_secs(60),
        "runaway total time: {total:?}"
    );
    assert!(
        last_10 < first_10 * 3,
        "step time creeping up: first 10 avg {first_10:?}, last 10 avg {last_10:?}"
    );
}
