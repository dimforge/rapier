//! Regression test for https://github.com/dimforge/rapier/issues/617
//!
//! The old owned `QueryPipeline`'s QBVH grew linearly under spawn/despawn churn (removed
//! leaves were marked dirty, never reclaimed). It is now a zero-copy view over the
//! broad-phase BVH; this pins that the serialized size plateaus under the same churn.

#![cfg(feature = "serde-serialize")]

use rapier3d::prelude::*;
use std::collections::VecDeque;

#[test]
fn broad_phase_size_plateaus_under_churn() {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);

    // Deterministic pseudo-random positions (LCG), mirroring the issue's setup:
    // spawn one cuboid body per tick, keep at most 10 alive.
    let mut rng_state = 0x12345678u64;
    let mut rand01 = move || {
        rng_state = rng_state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((rng_state >> 33) as f32) / ((1u64 << 31) as f32)
    };

    let mut handles = VecDeque::new();
    let mut size_at_100 = 0;
    let mut max_size_after_100 = 0;

    for i in 0..400 {
        let body = world.insert_body(RigidBodyBuilder::dynamic().translation(Vector::new(
            rand01() * 100.0,
            rand01() * 100.0,
            rand01() * 100.0,
        )));
        world.insert_collider(ColliderBuilder::cuboid(1.0, 1.0, 1.0), Some(body));
        handles.push_back(body);

        while handles.len() > 10 {
            let remove = handles.pop_front().unwrap();
            world.remove_body(remove);
        }

        world.step();

        let bytes = bincode::serialized_size(&world.broad_phase).unwrap();
        if i == 100 {
            size_at_100 = bytes;
        } else if i > 100 {
            max_size_after_100 = max_size_after_100.max(bytes);
        }
    }

    // The old bug grew by a fixed amount per tick (linear, unbounded). A healthy
    // structure stays within a small constant factor of its steady-state size.
    assert!(size_at_100 > 0);
    assert!(
        max_size_after_100 <= size_at_100 * 3 / 2,
        "broad-phase serialized size still growing: {} bytes at tick 100, up to {} bytes afterwards",
        size_at_100,
        max_size_after_100,
    );
}
