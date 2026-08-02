//! Snapshot round-trip determinism (2D).
//!
//! The 2D counterpart of `rapier3d/tests/snapshot_roundtrip.rs`, same contract:
//! serializing a [`PhysicsWorld`], restoring it, and stepping on must continue the
//! simulation exactly as not snapshotting at all would have, and re-serializing at every
//! step after the restore must yield **the same bytes** as the run it resumed.
//!
//! Worth its own copy rather than trusting the 3D suite: the serialized state is mostly
//! dimension-generic, but the scenes that fill it are not — 2D contact manifolds carry two
//! points instead of up to eight, `Rot`/angular velocity are scalars, and the solver's
//! constraint layout differs. A field skipped only on the 2D path would go unseen here
//! otherwise.
#![cfg(feature = "serde-serialize")]

use rapier2d::prelude::*;

/// Serializes the world — what a user's save file holds: the simulation inputs plus the
/// seven structures, with the pipeline and CCD workspace left out.
fn save(world: &PhysicsWorld) -> Vec<u8> {
    bincode::serialize(world).expect("snapshot serialization")
}

/// Restores into a fresh world, the way loading a save file does.
fn restore(bytes: &[u8]) -> PhysicsWorld {
    bincode::deserialize(bytes).expect("snapshot deserialization")
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

/// Per-structure digests, used only to name the culprit when the bytes differ.
fn parts(world: &PhysicsWorld) -> [u64; 9] {
    let fnv = |bytes: Vec<u8>| {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in bytes {
            h ^= b as u64;
            h = h.wrapping_mul(0x100_0000_01b3);
        }
        h
    };
    [
        fnv(bincode::serialize(&world.gravity).unwrap()),
        fnv(bincode::serialize(&world.integration_parameters).unwrap()),
        fnv(bincode::serialize(&world.islands).unwrap()),
        fnv(bincode::serialize(&world.broad_phase).unwrap()),
        fnv(bincode::serialize(&world.narrow_phase).unwrap()),
        fnv(bincode::serialize(&world.bodies).unwrap()),
        fnv(bincode::serialize(&world.colliders).unwrap()),
        fnv(bincode::serialize(&world.impulse_joints).unwrap()),
        fnv(bincode::serialize(&world.multibody_joints).unwrap()),
    ]
}

/// One body per line, bit-faithfully (`Debug` on floats round-trips exactly). Only used to
/// report whether the physics moved when the bytes did.
fn state(world: &PhysicsWorld) -> Vec<String> {
    let mut out: Vec<_> = world
        .bodies
        .iter()
        .map(|(h, rb)| {
            format!(
                "{} {:?} {:?} {:?} {:?} {}",
                h.into_raw_parts().0,
                rb.translation(),
                rb.rotation(),
                rb.linvel(),
                rb.angvel(),
                rb.is_sleeping()
            )
        })
        .collect();
    out.sort();
    out
}

/// What one step of a run produces: the snapshot bytes, plus diagnostics for the message.
struct Step {
    bytes: Vec<u8>,
    parts: [u64; 9],
    state: Vec<String>,
}

fn record(world: &PhysicsWorld) -> Step {
    Step {
        bytes: save(world),
        parts: parts(world),
        state: state(world),
    }
}

fn assert_same(step: usize, want: &Step, got: &Step, what: &str) {
    if want.bytes == got.bytes {
        return;
    }
    let differing: Vec<_> = (0..9)
        .filter(|i| want.parts[*i] != got.parts[*i])
        .map(|i| PART_NAMES[i])
        .collect();
    let body = want
        .state
        .iter()
        .zip(got.state.iter())
        .find(|(a, b)| a != b)
        .map(|(a, b)| format!("\n  uninterrupted: {a}\n  restored:      {b}"))
        .unwrap_or_else(|| {
            " none — the stored layout moved without the simulation moving".to_string()
        });
    panic!(
        "{what}: the snapshot is not byte-identical {} step(s) after the restore.\n  \
         sizes {} vs {} bytes\n  differing structures: {differing:?}\n  \
         first differing body:{body}",
        step + 1,
        want.bytes.len(),
        got.bytes.len(),
    );
}

/// Steps `before`, snapshots, then runs the original world and a restored one in
/// lockstep for `after` steps, requiring their snapshots to be byte-identical at every
/// step.
///
/// `edit` runs before each step with the absolute step index and that world's own spawn
/// list, so structural changes are applied identically to both continuations.
/// `keep_pipeline` restores into the live pipeline (the testbed's pattern) instead of a
/// fresh one.
fn check_roundtrip(
    what: &str,
    mut world: PhysicsWorld,
    before: usize,
    after: usize,
    keep_pipeline: bool,
    mut edit: impl FnMut(&mut PhysicsWorld, usize, &mut Vec<RigidBodyHandle>),
) {
    let mut spawned = Vec::new();
    for step in 0..before {
        edit(&mut world, step, &mut spawned);
        world.step();
    }
    let snapshot = save(&world);
    let mut spawned_restored = spawned.clone();

    let mut restored = restore(&snapshot);
    if keep_pipeline {
        // The testbed's Save/Restore: the world is replaced, the live pipeline keeps
        // stepping. Moving them over leaves `world` with fresh ones, which is fine — it is
        // the *restored* side whose pipeline state is under test here.
        restored.physics_pipeline = core::mem::take(&mut world.physics_pipeline);
        restored.ccd_solver = core::mem::take(&mut world.ccd_solver);
    }

    for step in 0..after {
        edit(&mut world, before + step, &mut spawned);
        world.step();
        edit(&mut restored, before + step, &mut spawned_restored);
        restored.step();
        assert_same(step, &record(&world), &record(&restored), what);
    }
}

fn no_edits(_: &mut PhysicsWorld, _: usize, _: &mut Vec<RigidBodyHandle>) {}

/// A pile of boxes on a ground. `sleepy` lets it settle and fall asleep — which switches
/// the broad phase to SAH re-insertion and the narrow phase to its sparse-awake path —
/// and adds a few permanent movers so the scene is not simply frozen.
fn pile(sleepy: bool) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(20.0, 0.5),
    );

    for i in 0..10 {
        for j in 0..6 {
            let jitter = (i as Real * 0.013 + j as Real * 0.017) % 0.05;
            world.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(
                        i as Real * 1.05 - 5.0 + jitter,
                        j as Real * 1.05 + 0.55,
                    ))
                    .can_sleep(sleepy),
                ColliderBuilder::cuboid(0.5, 0.5),
            );
        }
    }

    if sleepy {
        for i in 0..4 {
            world.insert(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new(-9.0 + i as Real * 0.7, 3.4 + i as Real))
                    .linvel(Vector::new(4.0, 0.3 * i as Real))
                    .can_sleep(false),
                ColliderBuilder::ball(0.4),
            );
        }
    }

    world
}

/// A hanging chain of revolute joints, plus a sensor it swings through.
fn with_joints_and_sensor(world: &mut PhysicsWorld) {
    let anchor = world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(0.0, 7.0)));
    let mut prev = anchor;
    for i in 0..4 {
        let rb = world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(0.6 * (i + 1) as Real, 7.0))
                .can_sleep(false),
        );
        world.insert_collider(ColliderBuilder::ball(0.25), Some(rb));
        world.insert_impulse_joint(
            prev,
            rb,
            RevoluteJointBuilder::new()
                .local_anchor1(Vector::X * 0.3)
                .local_anchor2(Vector::X * -0.3),
        );
        prev = rb;
    }
    world.insert_collider(
        ColliderBuilder::cuboid(3.0, 0.5)
            .translation(Vector::new(0.0, 4.0))
            .sensor(true),
        None,
    );
}

/// The `large_pyramids2` stress scene (`examples2d/stress_tests/large_pyramids2.rs`), same
/// geometry: `count` piles with a base row of `base` boxes each, sleeping disabled.
///
/// Size is the point: at full size (8 piles of 1,540 boxes) it crosses the bulk-path
/// thresholds — batched leaf updates, chunked pair filtering, blocked contact update, the
/// solver-graph counting-sort rebuild — that the smaller scenes never reach.
fn pyramids(count: usize, base: usize) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    let rad = 0.5;
    let gap = 10.0;
    let pyramid_width = base as Real * 2.0 * rad;
    let total_width = count as Real * (pyramid_width + gap);

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0)),
        ColliderBuilder::cuboid(total_width, 1.0),
    );

    let shift = rad * 2.0;
    for p in 0..count {
        let x0 = p as Real * (pyramid_width + gap) - 0.5 * total_width;
        for i in 0..base {
            for j in i..base {
                let x = x0 + (i as Real * shift / 2.0) + (j - i) as Real * shift;
                let y = i as Real * shift * 1.001 + rad;
                world.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(x, y))
                        .can_sleep(false),
                    ColliderBuilder::cuboid(rad, rad),
                );
            }
        }
    }
    world
}

/// Every leaf moves every step: the broad phase stays in its bulk in-place-update regime
/// and the periodic tree optimizer keeps firing.
#[test]
fn awake_pile() {
    check_roundtrip("awake pile", pile(false), 100, 100, false, no_edits);
}

/// Few leaves move: SAH re-insertion, sparse-awake narrow phase, sleeping islands.
#[test]
fn sleeping_pile() {
    check_roundtrip("sleeping pile", pile(true), 100, 100, false, no_edits);
}

/// A pyramid big enough to reach the bulk paths, small enough for CI.
#[test]
fn pyramid_stress_scene() {
    check_roundtrip("pyramids", pyramids(2, 20), 100, 40, false, no_edits);
}

/// The real thing: 8 piles of 1,540 boxes, 12,320 bodies. Slow and memory-hungry, so it is
/// `#[ignore]`d and run explicitly:
///
/// ```text
/// cargo test -p rapier2d --release --features serde-serialize \
///     --test snapshot_roundtrip -- --ignored --nocapture
/// ```
#[test]
#[ignore = "full-size stress scene: 12,320 bodies, minutes to run"]
fn pyramid_stress_scene_full() {
    let world = pyramids(8, 55);
    println!("large_pyramids2: {} bodies", world.bodies.len());
    check_roundtrip("large_pyramids2", world, 100, 20, false, no_edits);
}

/// Fast CCD-enabled bodies. The CCD solver holds a cache that snapshots do not carry, so
/// this checks that a restore does not depend on it.
#[test]
fn ccd_bodies() {
    let mut world = pile(true);
    for i in 0..8 {
        world.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(-6.0 + i as Real * 1.5, 4.0 + i as Real * 0.3))
                .linvel(Vector::new(90.0, -45.0 + 3.0 * i as Real))
                .ccd_enabled(true)
                .can_sleep(false),
            ColliderBuilder::ball(0.15),
        );
    }
    check_roundtrip("ccd", world, 100, 100, false, no_edits);
}

/// Joints and a sensor: the solver's joint assembly is cached across steps, and a restore
/// necessarily rebuilds it.
#[test]
fn joints_and_sensor() {
    let mut world = pile(true);
    with_joints_and_sensor(&mut world);
    check_roundtrip("joints", world, 100, 100, false, no_edits);
}

/// An articulation: a chain of multibody joints over a settled pile.
///
/// Multibodies exercise two things nothing else does. Their topology epoch is stored by
/// the narrow phase alongside its solver contact graph, so the two must agree across a
/// restore or the graph is rebuilt into a different (equally valid) layout — that is what
/// broke every failing scene in the 3D example sweep, the 2D IK scene included. And their
/// chain links populate `PersistentIslands::joint_link_locs`, a hash map whose iteration
/// order is insertion-history dependent: serializing it as-is made two snapshots of the
/// same state differ.
#[test]
fn multibody_articulation() {
    let mut world = pile(true);

    let segments = 10;
    let size = 0.4;
    let mut last = world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(6.0, 4.0)));
    for i in 0..segments {
        let rb = world.insert_body(RigidBodyBuilder::dynamic().can_sleep(false));
        world.insert_collider(
            ColliderBuilder::cuboid(size / 8.0, size / 2.0).density(1.0),
            Some(rb),
        );
        let joint = RevoluteJointBuilder::new()
            .local_anchor1(Vector::new(0.0, size / 2.0 * (i != 0) as usize as Real))
            .local_anchor2(Vector::new(0.0, -size / 2.0))
            .build()
            .data;
        world.insert_multibody_joint(last, rb, joint);
        last = rb;
    }

    check_roundtrip("multibody", world, 100, 60, false, no_edits);
}

/// The testbed's Save/Restore pattern: the world is replaced but the same
/// [`PhysicsPipeline`] keeps stepping, carrying its workspace across the restore.
#[test]
fn restoring_into_a_live_pipeline() {
    check_roundtrip("live pipeline", pile(true), 100, 100, true, no_edits);
    check_roundtrip("live pipeline, awake", pile(false), 100, 60, true, no_edits);
}

/// Structural churn on both sides of the snapshot: pair creation and deletion, island
/// merges and splits, arena and edge-id recycling — with a joint chain and sensor present.
#[test]
fn structural_changes() {
    let mut world = pile(true);
    with_joints_and_sensor(&mut world);

    check_roundtrip(
        "structural",
        world,
        100,
        100,
        false,
        |world, step, spawned| {
            if step % 17 == 0 {
                let rb = world.insert_body(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(
                            (step % 5) as Real - 2.0,
                            6.0 + (step % 3) as Real,
                        ))
                        .can_sleep(false),
                );
                world.insert_collider(ColliderBuilder::ball(0.45), Some(rb));
                spawned.push(rb);
            }
            if step % 23 == 0 && !spawned.is_empty() {
                let rb = spawned.remove(0);
                world.remove_body(rb);
            }
        },
    );
}

/// Replacing the pipeline mid-run, with no snapshot involved: [`PhysicsPipeline`] is
/// documented as holding workspace only, and a restored world always starts from a fresh
/// one. The joint chain is the point — the solver's joint coloring is cached across steps
/// and only survives a rebuild because joints persist their color.
#[test]
fn replacing_the_pipeline_changes_nothing() {
    let mut a = pile(true);
    with_joints_and_sensor(&mut a);
    let mut b = pile(true);
    with_joints_and_sensor(&mut b);

    for _ in 0..100 {
        a.step();
        b.step();
    }
    b.physics_pipeline = PhysicsPipeline::new();
    b.ccd_solver = CCDSolver::new();

    for step in 0..100 {
        a.step();
        b.step();
        assert_same(step, &record(&a), &record(&b), "fresh pipeline");
    }
}
