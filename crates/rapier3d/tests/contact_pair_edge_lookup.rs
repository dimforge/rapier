//! Adding or removing the contact pair between two colliders looks the pair's edge up in the
//! contact graph. That lookup used to walk the first collider's whole edge list, and pairs
//! with the ground list the ground first, so a box leaving or landing on a ground touched by
//! thousands of other shapes cost as much as all of them. It now walks the shorter of the two
//! lists. The test is timing-based, so `#[ignore]`d: run it manually in release with
//! `-- --ignored --nocapture`.

use rapier3d::prelude::*;
use std::time::{Duration, Instant};

#[test]
#[ignore = "timing-based perf check, run manually in release"]
fn leaving_a_crowded_ground_does_not_walk_its_contacts() {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(500.0, 0.5, 500.0),
    );
    // 20,000 boxes resting apart on the ground, each with one contact pair with it.
    let mut handles = Vec::new();
    for i in 0..20_000 {
        let at = Vector::new((i % 141) as Real * 2.0, 0.5, (i / 141) as Real * 2.0);
        let (h, _) = world.insert(
            RigidBodyBuilder::dynamic().translation(at),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        );
        handles.push(h);
    }
    for _ in 0..600 {
        world.step();
        if world.active_bodies().next().is_none() {
            break;
        }
    }

    // A slow push up (5 m/s for this 1 kg box): the box leaves the ground, so its pair with
    // the ground is removed.
    world.bodies[handles[0]].apply_impulse(Vector::new(0.0, 5.0, 0.0), true);
    let mut steps = Vec::new();
    for _ in 0..3 {
        let t = Instant::now();
        world.step();
        steps.push(t.elapsed());
    }
    println!("the steps after the push: {steps:?}");

    // The pair is removed in the second step. Walking the ground's 20,000 edges took about
    // 0.1-0.3 ms there on a desktop CPU (single-threaded), and walking the box's one edge
    // takes nothing.
    assert!(
        steps[1] < Duration::from_micros(40),
        "the step that removed one pair cost {:?}",
        steps[1]
    );
}
