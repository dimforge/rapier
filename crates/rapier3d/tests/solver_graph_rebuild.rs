//! `NarrowPhase::maintain_solver_contact_graph` rebuilds the solver contact graph whenever
//! the set of awake bodies changes. The rebuild used to visit every contact pair in the world,
//! sleeping ones included, so waking one body among many sleeping ones cost as much as the
//! whole world. It now visits only the pairs that can be solver-active and the ones already in
//! the graph. These tests pin that the graph stays consistent while islands keep sleeping,
//! waking and losing bodies (checked in debug builds), and that the wake step no longer grows
//! with the number of sleeping pairs. The timing test is `#[ignore]`d, so run it manually in
//! release with `-- --ignored --nocapture`.

use rapier3d::prelude::*;
use std::time::{Duration, Instant};

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn unit(&mut self) -> Real {
        (self.next() % 10_000) as Real / 10_000.0
    }
}

fn world_with_ground() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(500.0, 0.5, 500.0),
    );
    world
}

fn insert_box(world: &mut PhysicsWorld, at: Vector) -> RigidBodyHandle {
    world
        .insert(
            RigidBodyBuilder::dynamic().translation(at),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        )
        .0
}

/// Small heaps of touching boxes set apart from each other, so each heap is its own island
/// and they fall asleep and wake up one at a time.
fn insert_heaps(world: &mut PhysicsWorld, side: usize) -> Vec<RigidBodyHandle> {
    let mut handles = Vec::new();
    for gx in 0..side {
        for gz in 0..side {
            for i in 0..8 {
                let (x, z, y) = (i % 2, (i / 2) % 2, i / 4);
                let at = Vector::new(
                    gx as Real * 5.0 + x as Real * 1.001,
                    0.5 + y as Real * 1.001,
                    gz as Real * 5.0 + z as Real * 1.001,
                );
                handles.push(insert_box(world, at));
            }
        }
    }
    handles
}

#[test]
fn graph_stays_consistent_under_sleep_wake_churn() {
    let mut world = world_with_ground();
    let side = 4;
    let mut handles = insert_heaps(&mut world, side);
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);

    for step in 0..900 {
        if step >= 240 {
            // A box dropped on a sleeping heap wakes its island.
            if step % 30 == 0 {
                let gx = (rng.next() % side as u64) as Real;
                let gz = (rng.next() % side as u64) as Real;
                handles.push(insert_box(
                    &mut world,
                    Vector::new(gx * 5.0 + 0.5, 3.5, gz * 5.0 + 0.5),
                ));
            }
            // A pushed box slides away from its neighbours, so its contact pairs are removed
            // while other islands fall asleep around it.
            if step % 45 == 0 {
                let h = handles[(rng.next() % handles.len() as u64) as usize];
                if let Some(body) = world.bodies.get_mut(h) {
                    body.apply_impulse(
                        Vector::new(rng.unit() * 8.0 - 4.0, 2.0, rng.unit() * 8.0 - 4.0),
                        true,
                    );
                }
            }
            // Removing a body removes its collider and moves contact-graph edges around.
            if step % 110 == 0 {
                let i = (rng.next() % handles.len() as u64) as usize;
                let h = handles.swap_remove(i);
                world.remove_body(h);
            }
        }
        world.step();
    }

    for _ in 0..600 {
        world.step();
    }
    assert_eq!(
        world.active_bodies().count(),
        0,
        "the heaps should all be asleep again"
    );
    for &h in &handles {
        let y = world.bodies[h].translation().y;
        assert!(y > 0.0 && y < 4.0, "a box left the heaps: y = {y}");
    }
}

#[test]
#[ignore = "timing-based perf check, run manually in release"]
fn wake_step_does_not_grow_with_sleeping_pairs() {
    let mut world = world_with_ground();
    // 20,000 boxes resting apart on the ground, each with one sleeping pair.
    let mut handles = Vec::new();
    for i in 0..20_000 {
        let at = Vector::new((i % 141) as Real * 2.0, 0.5, (i / 141) as Real * 2.0);
        handles.push(insert_box(&mut world, at));
    }
    for _ in 0..600 {
        world.step();
        if world.active_bodies().next().is_none() {
            break;
        }
    }
    assert_eq!(
        world.active_bodies().count(),
        0,
        "the boxes should be asleep"
    );

    let mut quiet = Vec::new();
    for _ in 0..20 {
        let t = Instant::now();
        world.step();
        quiet.push(t.elapsed());
    }
    // A slow push (5 m/s for this 1 kg box), so the CCD never needs to act.
    world.bodies[handles[0]].apply_impulse(Vector::new(0.0, 5.0, 0.0), true);
    let mut woken = Vec::new();
    for _ in 0..3 {
        let t = Instant::now();
        world.step();
        woken.push(t.elapsed());
    }
    let quiet = quiet.iter().sum::<Duration>() / quiet.len() as u32;
    println!("a quiet step: {quiet:?}, the steps after the wake: {woken:?}");

    // Visiting all 20,000 pairs took about 0.2 ms on a desktop CPU, and visiting only the
    // awake ones takes about 0.02 ms, which leaves plenty of room for slower machines.
    assert!(
        woken[0] < Duration::from_micros(100),
        "the step that woke one body cost {:?}",
        woken[0]
    );
}
