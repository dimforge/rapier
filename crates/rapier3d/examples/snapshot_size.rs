//! Snapshot size and step-timing harness for the snapshot-size reduction work
//! (see `docs/snapshot-size-plan.md`).
//!
//! Steps a few representative scenes, then prints the serialized size of each
//! `PhysicsWorld` part plus the average step time, so every phase of the plan can
//! record its effect in the ledger:
//!
//! ```text
//! cargo run -p rapier3d --release --features serde-serialize --example snapshot_size
//! ```
//!
//! Pass `--big` to also run the 50-layer pyramid stress scene (minutes, >100 MiB).

use rapier3d::prelude::*;
use std::time::Instant;

/// The pile scene from `tests/snapshot_roundtrip.rs`: boxes on a ground, either kept
/// awake or allowed to settle asleep (with a few permanent movers).
fn pile(sleepy: bool) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(20.0, 0.5, 20.0),
    );

    for i in 0..8 {
        for j in 0..3 {
            for k in 0..8 {
                let jitter = (i as Real * 0.013 + k as Real * 0.017) % 0.05;
                world.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(
                            i as Real * 1.05 - 4.0 + jitter,
                            j as Real * 1.05 + 0.55,
                            k as Real * 1.05 - 4.0 - jitter,
                        ))
                        .can_sleep(sleepy),
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5),
                );
            }
        }
    }

    if sleepy {
        for i in 0..4 {
            world.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(-9.0 + i as Real * 0.7, 3.4, -3.0 + i as Real))
                    .linvel(Vector::new(4.0, 0.0, 0.3 * i as Real))
                    .can_sleep(false),
                ColliderBuilder::ball(0.4),
            );
        }
    }

    world
}

/// The scaled `pyramid3` stress scene from `tests/snapshot_roundtrip.rs`.
fn pyramid(height: i32) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0, 0.0)),
        ColliderBuilder::cuboid(100.0, 1.0, 100.0),
    );

    let box_size = 2.0;
    let h = 0.5 * box_size - 0.025;
    for i in 0..height {
        let brick = if i & 1 != 0 { 0.5 * box_size } else { 0.0 };
        let y = 1.0 + (box_size + 0.5) * i as Real;
        for j in i / 2..height - (i + 1) / 2 {
            for k in i / 2..height - (i + 1) / 2 {
                let x = -(height as Real) + (box_size + 0.25) * j as Real + brick;
                let z = -(height as Real) + (box_size + 0.25) * k as Real + brick;
                world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(x, y, z)),
                    ColliderBuilder::cuboid(h, h, h).density(1000.0),
                );
            }
        }
    }
    world
}

/// The mixed scene from `tests/snapshot_portability.rs`: box pile, impulse-joint chain,
/// multibody articulation, sensor, CCD bullet.
fn mixed() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81, 0.0);
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(20.0, 0.5, 20.0),
    );

    for i in 0..6 {
        for j in 0..3 {
            for k in 0..6 {
                let jitter = (i as Real * 0.013 + k as Real * 0.017) % 0.05;
                world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(
                        i as Real * 1.05 - 3.0 + jitter,
                        j as Real * 1.05 + 0.55,
                        k as Real * 1.05 - 3.0 - jitter,
                    )),
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5),
                );
            }
        }
    }

    let mut prev =
        world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(0.0, 7.0, 0.0)));
    for i in 0..4 {
        let rb = world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(0.6 * (i + 1) as Real, 7.0, 0.0))
                .can_sleep(false),
        );
        world.insert_collider(ColliderBuilder::ball(0.25), Some(rb));
        world.insert_impulse_joint(
            prev,
            rb,
            SphericalJointBuilder::new()
                .local_anchor1(Vector::X * 0.3)
                .local_anchor2(Vector::X * -0.3),
        );
        prev = rb;
    }

    let size = 0.4;
    let mut last =
        world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(6.0, 4.0, 0.0)));
    for i in 0..6 {
        let rb = world.insert_body(RigidBodyBuilder::dynamic().can_sleep(false));
        world.insert_collider(
            ColliderBuilder::cuboid(size / 8.0, size / 2.0, size / 8.0).density(1.0),
            Some(rb),
        );
        let joint = SphericalJointBuilder::new()
            .local_anchor1(Vector::new(
                0.0,
                size / 2.0 * (i != 0) as usize as Real,
                0.0,
            ))
            .local_anchor2(Vector::new(0.0, -size / 2.0, 0.0))
            .build()
            .data;
        world.insert_multibody_joint(last, rb, joint);
        last = rb;
    }

    world.insert_collider(
        ColliderBuilder::cuboid(3.0, 0.5, 3.0)
            .translation(Vector::new(0.0, 4.0, 0.0))
            .sensor(true),
        None,
    );
    world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-8.0, 2.0, 0.5))
            .linvel(Vector::new(90.0, -20.0, 1.0))
            .ccd_enabled(true)
            .can_sleep(false),
        ColliderBuilder::ball(0.15),
    );

    world
}

/// A pile of boxes sharing a single trimesh ground plus instanced convex shapes, to
/// expose the shape-duplication cost that box-only scenes never show.
fn instanced_shapes() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    // One bumpy trimesh ground, referenced by a grid of fixed colliders (tiles).
    let n = 20usize;
    let mut vtx = Vec::new();
    let mut idx = Vec::new();
    for i in 0..=n {
        for j in 0..=n {
            let x = i as Real / n as Real * 4.0 - 2.0;
            let z = j as Real / n as Real * 4.0 - 2.0;
            let y = 0.08 * ((i * 7 + j * 3) % 5) as Real;
            vtx.push(Vector::new(x, y, z));
        }
    }
    for i in 0..n {
        for j in 0..n {
            let a = (i * (n + 1) + j) as u32;
            let b = a + 1;
            let c = a + (n + 1) as u32;
            let d = c + 1;
            idx.push([a, b, c]);
            idx.push([b, d, c]);
        }
    }
    let tile = ColliderBuilder::trimesh(vtx, idx).unwrap().build();
    for tx in 0..4 {
        for tz in 0..4 {
            let mut c = tile.clone();
            c.set_translation(Vector::new(
                tx as Real * 4.0 - 8.0,
                0.0,
                tz as Real * 4.0 - 8.0,
            ));
            world.insert_collider(c, None);
        }
    }

    // Instanced dynamic convex hulls, all sharing one shape.
    let pts: Vec<_> = (0..12)
        .map(|i| {
            let a = i as Real * 0.524;
            Vector::new(a.cos() * 0.3, ((i % 4) as Real - 1.5) * 0.2, a.sin() * 0.3)
        })
        .collect();
    let hull = ColliderBuilder::convex_hull(&pts).unwrap().build();
    for i in 0..10 {
        for k in 0..10 {
            let rb = world.insert_body(RigidBodyBuilder::dynamic().translation(Vector::new(
                i as Real * 0.8 - 4.0,
                1.0 + ((i + k) % 3) as Real * 0.7,
                k as Real * 0.8 - 4.0,
            )));
            world.insert_collider(hull.clone(), Some(rb));
        }
    }

    world
}

const PART_NAMES: [&str; 9] = [
    "gravity",
    "integration_parameters",
    "islands",
    "broad_phase",
    "narrow_phase",
    "bodies",
    "colliders",
    "impulse_joints",
    "multibody_joints",
];

fn part_sizes(world: &PhysicsWorld) -> [u64; 9] {
    [
        bincode::serialized_size(&world.gravity).unwrap(),
        bincode::serialized_size(&world.integration_parameters).unwrap(),
        bincode::serialized_size(&world.islands).unwrap(),
        bincode::serialized_size(&world.broad_phase).unwrap(),
        bincode::serialized_size(&world.narrow_phase).unwrap(),
        bincode::serialized_size(&world.bodies).unwrap(),
        bincode::serialized_size(&world.colliders).unwrap(),
        bincode::serialized_size(&world.impulse_joints).unwrap(),
        bincode::serialized_size(&world.multibody_joints).unwrap(),
    ]
}

fn run(name: &str, mut world: PhysicsWorld, warmup: usize, timed: usize) {
    for _ in 0..warmup {
        world.step();
    }

    let t0 = Instant::now();
    for _ in 0..timed {
        world.step();
    }
    let ms_per_step = t0.elapsed().as_secs_f64() * 1000.0 / timed as f64;

    let t0 = Instant::now();
    let bytes = bincode::serialize(&world).unwrap();
    let ser_ms = t0.elapsed().as_secs_f64() * 1000.0;
    let parts = part_sizes(&world);

    println!(
        "\n=== {name}: {} bodies, {} colliders ===",
        world.bodies.len(),
        world.colliders.len()
    );
    println!("  total: {:>12} bytes (serialize {ser_ms:.1} ms)", bytes.len());
    for (part, size) in PART_NAMES.iter().zip(parts.iter()) {
        println!("  {part:>22}: {size:>12}");
    }
    println!("  step: {ms_per_step:.3} ms avg over {timed} steps (after {warmup} warmup)");

    // Per-tick semantic deltas into a receiver restored from a snapshot at the baseline.
    world.enable_change_tracking();
    let mut acked = Some(world.change_tick().unwrap());
    let mut receiver: PhysicsWorld =
        bincode::deserialize(&bincode::serialize(&world).unwrap()).unwrap();
    receiver.set_delta_baseline(acked.unwrap());
    let ticks = 20;
    let (mut size, mut max_size, mut encode, mut apply, mut step) = (0, 0, 0.0, 0.0, 0.0);
    let mut breakdown = String::new();
    for _ in 0..ticks {
        let t0 = Instant::now();
        world.step();
        step += t0.elapsed().as_secs_f64() * 1000.0;
        let t0 = Instant::now();
        let (delta, tick) = world.delta_since(acked).unwrap();
        let delta_bytes = bincode::serialize(&delta).unwrap();
        encode += t0.elapsed().as_secs_f64() * 1000.0;
        let t0 = Instant::now();
        receiver
            .apply_delta(bincode::deserialize(&delta_bytes).unwrap())
            .unwrap();
        apply += t0.elapsed().as_secs_f64() * 1000.0;
        acked = Some(tick);
        let delta: rapier3d::pipeline::WorldDelta = bincode::deserialize(&delta_bytes).unwrap();
        breakdown = format!(
            "islands {} bp {} np {} bodies {} colliders {} joints {} mb {} soft {}",
            bincode::serialized_size(&delta.islands).unwrap(),
            bincode::serialized_size(&delta.broad_phase).unwrap(),
            bincode::serialized_size(&delta.narrow_phase).unwrap(),
            bincode::serialized_size(&delta.bodies).unwrap(),
            bincode::serialized_size(&delta.colliders).unwrap(),
            bincode::serialized_size(&delta.impulse_joints).unwrap(),
            bincode::serialized_size(&delta.multibody_joints).unwrap(),
            bincode::serialized_size(&delta.soft_bodies).unwrap(),
        );
        size += delta_bytes.len();
        max_size = max_size.max(delta_bytes.len());
    }
    let n = ticks as f64;
    println!(
        "  tick delta: {:>10} bytes avg, {max_size} max ({:.2}% of full); encode {:.3} ms, apply {:.3} ms; tracked step {:.3} ms",
        size / ticks,
        (size as f64 / n) * 100.0 / bytes.len() as f64,
        encode / n,
        apply / n,
        step / n,
    );
    println!("  last delta by component: {breakdown}");
}

fn main() {
    let big = std::env::args().any(|a| a == "--big");

    run("pile_awake", pile(false), 100, 100);
    run("pile_sleepy", pile(true), 100, 100);
    run("mixed", mixed(), 60, 60);
    run("instanced_shapes", instanced_shapes(), 100, 100);
    run("pyramid20", pyramid(20), 100, 100);
    run("pyramid30", pyramid(30), 100, 100);
    if big {
        run("pyramid50", pyramid(50), 100, 20);
    }
}
