//! Snapshot round-trip determinism.
//!
//! Serializing a [`PhysicsWorld`], restoring it, and stepping on must continue the
//! simulation exactly as not snapshotting at all would have — and the proof is the
//! snapshot itself: at every step after the restore, re-serializing the world must yield
//! **the same bytes** as the run it resumed produces at that step.
//!
//! Byte equality is the contract because it is the only check that sees stored *layout* —
//! container order, free lists, map iteration order, incrementally-maintained indices.
//! Derived views (body poses, contact impulses) can agree for many steps while a layout
//! difference waits to be read; the pyramid scene below was found exactly that way.
#![cfg(feature = "serde-serialize")]

use rapier3d::prelude::*;
use rapier3d::data::{ChangeTick, Delta, SnapshotDelta};
use rapier3d::pipeline::{WorldChangeTick, WorldDelta};

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
/// Lockstep rather than record-then-replay so memory stays bounded: the pyramid scene's
/// snapshot is ~140 MiB, which would be ruinous to keep one copy of per step.
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

/// A hanging chain of spherical joints, plus a sensor it swings through.
fn with_joints_and_sensor(world: &mut PhysicsWorld) {
    let anchor =
        world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(0.0, 7.0, 0.0)));
    let mut prev = anchor;
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
    world.insert_collider(
        ColliderBuilder::cuboid(3.0, 0.5, 3.0)
            .translation(Vector::new(0.0, 4.0, 0.0))
            .sensor(true),
        None,
    );
}

/// The `pyramid3` stress scene (`examples3d/stress_tests/pyramid3.rs`), scaled down but
/// geometrically faithful: shrunken 1.95-cubes on a 2.25 pitch, odd layers brick-offset by
/// 1.0 so every box rests on four unequal corner patches.
///
/// Size is the point: at ~4,900 bodies and ~17,900 pairs it crosses the bulk-path
/// thresholds (batched leaf updates, chunked pair filtering, blocked contact update, the
/// solver-graph counting-sort rebuild) that the smaller scenes never reach.
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

/// The real thing: 50 layers, ~43k bodies. Slow (minutes) and memory-hungry — its
/// snapshot alone is well over 100 MiB — so it is `#[ignore]`d and run explicitly:
///
/// ```text
/// cargo test -p rapier3d --release --features serde-serialize \
///     --test snapshot_roundtrip -- --ignored --nocapture
/// ```
#[test]
#[ignore = "full-size stress scene: ~43k bodies, minutes to run"]
fn pyramid_stress_scene_full() {
    let world = pyramid(50);
    println!("pyramid3: {} bodies", world.bodies.len());
    check_roundtrip("pyramid50", world, 100, 20, false, no_edits);
}

/// Shape sharing: many colliders referencing the same `SharedShape` `Arc` (a trimesh and
/// an instanced convex hull). The collider serialization interns shapes by `Arc`
/// identity, and a restore must rebuild the same sharing structure or the next snapshot
/// re-serializes with a different table.
/// A trimesh tiled four times and a convex hull on 25 bodies, all sharing their `Arc`s.
fn shared_shapes_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    let n = 8usize;
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
            idx.push([a, b, c]);
            idx.push([b, c + 1, c]);
        }
    }
    let tile = ColliderBuilder::trimesh(vtx, idx).unwrap().build();
    for tx in 0..2 {
        for tz in 0..2 {
            let mut c = tile.clone();
            c.set_translation(Vector::new(
                tx as Real * 4.0 - 4.0,
                0.0,
                tz as Real * 4.0 - 4.0,
            ));
            world.insert_collider(c, None);
        }
    }

    let pts: Vec<_> = (0..12)
        .map(|i| {
            let a = i as Real * 0.524;
            Vector::new(a.cos() * 0.3, ((i % 4) as Real - 1.5) * 0.2, a.sin() * 0.3)
        })
        .collect();
    let hull = ColliderBuilder::convex_hull(&pts).unwrap().build();
    for i in 0..5 {
        for k in 0..5 {
            let rb = world.insert_body(RigidBodyBuilder::dynamic().translation(Vector::new(
                i as Real * 0.8 - 2.0,
                1.0 + ((i + k) % 3) as Real * 0.7,
                k as Real * 0.8 - 2.0,
            )));
            world.insert_collider(hull.clone(), Some(rb));
        }
    }

    world
}

#[test]
fn shared_shapes() {
    let world = shared_shapes_world();
    // Structural churn so the shape table meets slot recycling: spawned balls share one
    // more `Arc`'d shape through `spawned`-driven insert/remove cycles.
    let ball = ColliderBuilder::ball(0.3).build();
    check_roundtrip(
        "shared shapes",
        world,
        60,
        60,
        false,
        move |world, step, spawned| {
            if step % 11 == 0 {
                let rb = world.insert_body(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(
                            (step % 3) as Real - 1.0,
                            4.0,
                            (step % 5) as Real - 2.0,
                        ))
                        .can_sleep(false),
                );
                world.insert_collider(ball.clone(), Some(rb));
                spawned.push(rb);
            }
            if step % 13 == 0 && !spawned.is_empty() {
                let rb = spawned.remove(0);
                world.remove_body(rb);
            }
        },
    );
}

/// The stored-solver-contacts fallback paths of the snapshot format: with contact
/// recycling disabled, pairs carry no recycle state, and colliders attached away from
/// their body's origin have non-replayable anchors — both serialize their solver
/// contacts verbatim instead of the compact replayed form.
#[test]
fn stored_solver_contacts() {
    let mut world = pile(true);
    world.integration_parameters.contact_recycling = false;
    check_roundtrip("no recycling", world, 100, 100, false, no_edits);

    let mut world = pile(true);
    for i in 0..6 {
        let rb = world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(-4.0 + i as Real * 1.4, 4.0, 6.0))
                .can_sleep(false),
        );
        // Two colliders offset from the body origin: identity `pos_wrt_parent` does
        // not hold, so their pairs use the verbatim fallback.
        world.insert_collider(
            ColliderBuilder::cuboid(0.4, 0.3, 0.4).translation(Vector::new(0.45, 0.0, 0.0)),
            Some(rb),
        );
        world.insert_collider(
            ColliderBuilder::ball(0.3).translation(Vector::new(-0.45, 0.1, 0.0)),
            Some(rb),
        );
    }
    check_roundtrip("offset colliders", world, 100, 100, false, no_edits);
}

/// Fast CCD-enabled bodies. The CCD solver holds a cache that snapshots do not include, so
/// this checks that a restore does not depend on it.
#[test]
fn ccd_bodies() {
    let mut world = pile(true);
    for i in 0..8 {
        world.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(
                    -6.0 + i as Real * 1.5,
                    4.0 + i as Real * 0.3,
                    0.0,
                ))
                .linvel(Vector::new(90.0, -45.0, 3.0 * i as Real))
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
/// broke every failing scene in the example sweep (3D articulations, IK, angular motors,
/// 2D IK). And their chain links populate `PersistentIslands::joint_link_locs`, a hash map
/// whose iteration order is insertion-history dependent: serializing it as-is made two
/// snapshots of the same state differ.
#[test]
fn multibody_articulation() {
    let mut world = if std::env::var("MB_AWAKE").is_ok() {
        pile(false)
    } else if std::env::var("MB_ALONE").is_ok() {
        PhysicsWorld::new()
    } else {
        pile(true)
    };

    let segments = 10;
    let mut last =
        world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(6.0, 4.0, 0.0)));
    for i in 0..segments {
        let size = 0.4;
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

    check_roundtrip("multibody", world, 100, 60, false, no_edits);
}

/// The testbed's Save/Restore pattern: the world is replaced but the same
/// [`PhysicsPipeline`] keeps stepping, keeping its workspace across the restore.
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
                            (step % 7) as Real - 3.0,
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

/// A snapshot taken between a joint insertion and the next step: the joint's pending
/// persistent-island link events are part of the state.
/// A box and a stack of four resting on a ground, so their contact pairs recycle.
fn resting_boxes() -> (PhysicsWorld, RigidBodyHandle) {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(5.0, 0.5, 5.0),
    );
    let (body, _) = world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.5, 0.0))
            .can_sleep(false),
        ColliderBuilder::cuboid(0.5, 0.5, 0.5),
    );
    for i in 0..4 {
        world.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(2.0, 0.5 + i as Real, 0.0))
                .can_sleep(false),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        );
    }
    (world, body)
}

#[test]
fn center_of_mass_change_while_recycling() {
    let (world, body) = resting_boxes();
    check_roundtrip("center of mass change", world, 62, 30, false, |w, step, _| {
        if step == 60 {
            let props = MassProperties::new(Vector::new(0.05, 0.0, 0.0), 1.0, Vector::splat(0.2));
            w.bodies[body].set_additional_mass_properties(props, true);
        }
    });
}

#[test]
fn contact_recycling_disabled_at_runtime() {
    let (world, _) = resting_boxes();
    check_roundtrip("recycling disabled", world, 80, 30, false, |w, step, _| {
        if step == 60 {
            w.integration_parameters.contact_recycling = false;
        }
    });
}

#[test]
fn snapshot_between_joint_insert_and_step() {
    for multibody in [false, true] {
        let mut world = pile(true);
        for _ in 0..80 {
            world.step();
        }
        let dynamic: Vec<_> = world
            .bodies
            .iter()
            .filter(|(_, b)| b.is_dynamic())
            .map(|(h, _)| h)
            .collect();
        let (b0, b1) = (dynamic[3], dynamic[40]);
        let joint = SphericalJointBuilder::new().build().data;
        if multibody {
            world.insert_multibody_joint(b0, b1, joint);
        } else {
            world.insert_impulse_joint(b0, b1, joint);
        }
        let snapshot = save(&world);
        let mut restored = restore(&snapshot);
        for step in 0..60 {
            world.step();
            restored.step();
            assert_same(step, &record(&world), &record(&restored), "joint insert then snapshot");
        }
    }
}

/*
 * Snapshot deltas: worlds and individual components are replicated into receivers through
 * serialized deltas, which must leave each receiver byte-identical to the sender after every
 * sync, and simulating identically afterwards.
 */


#[path = "common/delta_replication.rs"]
mod delta_replication;
use delta_replication::check_delta_replication;

#[test]
fn delta_awake_pile() {
    check_delta_replication("delta awake pile", pile(false), 30, 60, no_edits);
}

#[test]
fn delta_sleeping_pile() {
    let (max_delta, snapshot) =
        check_delta_replication("delta sleeping pile", pile(true), 200, 60, no_edits);
    // Only the four permanent movers and their contacts change once the pile sleeps.
    assert!(
        max_delta * 10 < snapshot,
        "a {max_delta}-byte delta for a {snapshot}-byte snapshot"
    );
}

#[test]
fn delta_shared_shapes_and_churn() {
    check_delta_replication("delta structural churn", pile(true), 20, 120, |world, step, spawned| {
        if step % 7 == 0 {
            let rb = world.insert_body(
                RigidBodyBuilder::dynamic()
                    .translation(Vector::new((step % 5) as Real - 2.0, 6.0, 0.0)),
            );
            world.insert_collider(ColliderBuilder::ball(0.3), Some(rb));
            spawned.push(rb);
        }
        if step % 11 == 0 && spawned.len() > 2 {
            let rb = spawned.remove(0);
            world.remove_body(rb);
        }
    });
}

/// Heavy shapes by content key: instanced hulls spawned and removed, shapes replaced, and a
/// trimesh deformed in place (same `Arc`, new content).
#[test]
fn delta_heavy_shapes() {
    let pts: Vec<_> = (0..10)
        .map(|i| {
            let a = i as Real * 0.7;
            Vector::new(a.cos() * 0.25, ((i % 3) as Real - 1.0) * 0.2, a.sin() * 0.25)
        })
        .collect();
    let hull = ColliderBuilder::convex_hull(&pts).unwrap().build();
    let other_hull = ColliderBuilder::convex_hull(&pts[..7]).unwrap().build();
    check_delta_replication(
        "delta heavy shapes",
        shared_shapes_world(),
        10,
        120,
        move |world, step, spawned| {
            if step % 9 == 0 {
                let rb = world.insert_body(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new((step % 4) as Real - 1.5, 5.0, 0.0)),
                );
                world.insert_collider(hull.clone(), Some(rb));
                spawned.push(rb);
            }
            if step % 13 == 0 && spawned.len() > 1 {
                let rb = spawned.remove(0);
                world.remove_body(rb);
            }
            if step % 17 == 5 {
                // Replace a hull by another (new content), then by the original again.
                if let Some(&rb) = spawned.last() {
                    let co = world.bodies[rb].colliders()[0];
                    let shape = if step % 2 == 0 { &other_hull } else { &hull };
                    world.colliders[co].set_shape(shape.shared_shape().clone());
                }
            }
            if step % 19 == 3 {
                // Deform a trimesh tile in place.
                let tile = world
                    .colliders
                    .iter()
                    .find(|(_, co)| co.shape().as_trimesh().is_some())
                    .map(|(h, _)| h)
                    .unwrap();
                world.colliders[tile]
                    .shape_mut()
                    .as_trimesh_mut()
                    .unwrap()
                    .update_vertices(|vtx| vtx[0].y += 0.01);
            }
        },
    );
}

/// A chain of multibody links lying on the ground (it settles and sleeps), plus impulse
/// joints and a sensor.
fn joints_world() -> PhysicsWorld {
    let mut world = pile(true);
    with_joints_and_sensor(&mut world);
    let mut last = world.insert_body(RigidBodyBuilder::dynamic().translation(Vector::new(6.0, 0.06, 0.0)));
    world.insert_collider(ColliderBuilder::cuboid(0.2, 0.05, 0.05), Some(last));
    for i in 1..6 {
        let rb = world.insert_body(
            RigidBodyBuilder::dynamic().translation(Vector::new(6.0 + i as Real * 0.4, 0.06, 0.0)),
        );
        world.insert_collider(ColliderBuilder::cuboid(0.2, 0.05, 0.05), Some(rb));
        let joint = RevoluteJointBuilder::new(Vector::Z)
            .local_anchor1(Vector::new(0.2, 0.0, 0.0))
            .local_anchor2(Vector::new(-0.2, 0.0, 0.0))
            .build()
            .data;
        world.insert_multibody_joint(last, rb, joint);
        last = rb;
    }
    world
}

#[test]
fn delta_joints() {
    check_delta_replication("delta joints", joints_world(), 10, 150, |world, step, spawned| {
        // Joint churn: link and unlink pairs of pile bodies, impulse and multibody alike.
        let dynamic: Vec<_> = world
            .bodies
            .iter()
            .filter(|(_, b)| b.is_dynamic())
            .map(|(h, _)| h)
            .collect();
        if step % 23 == 4 {
            let (a, b) = (dynamic[step % 30], dynamic[(step % 30) + 40]);
            world.insert_impulse_joint(a, b, SphericalJointBuilder::new().build().data);
        }
        if step % 31 == 7 {
            let (a, b) = (dynamic[(step % 20) + 2], dynamic[(step % 20) + 60]);
            if let Some(h) = world.insert_multibody_joint(a, b, SphericalJointBuilder::new().build().data) {
                let _ = h;
                spawned.push(a);
            }
        }
        if step % 37 == 11 {
            let first = world.impulse_joints.iter().next().map(|(h, _)| h);
            if let Some(h) = first {
                world.remove_impulse_joint(h);
            }
        }
    });
}

/// Soft bodies (a tetrahedral cube and a pinned rope) in contact with rigid ones, settling
/// to sleep.
fn soft_world() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(20.0, 0.5, 20.0),
    );
    let cube = SoftBodyBuilder::cuboid(Vector::new(0.0, 0.6, 0.0), Vector::splat(0.6), 4, 4, 4)
        .particle_mass(0.2);
    world.insert_soft_body(cube);
    let rope = SoftBodyBuilder::rope(Vector::new(2.0, 2.0, 0.0), Vector::new(4.0, 2.0, 0.0), 10)
        .pinned_particles([0]);
    world.insert_soft_body(rope);
    world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 2.0, 0.0)),
        ColliderBuilder::cuboid(0.2, 0.2, 0.2),
    );
    world
}

/// Byte equality needs `enhanced-determinism` here: parry serializes the soft surfaces'
/// composite contact workspaces in hash-map order, which depends on the map's growth history,
/// and a receiver rebuilds the maps of every pair it patches (the content is identical).
#[cfg(feature = "enhanced-determinism")]
#[test]
fn delta_soft_bodies() {
    check_delta_replication("delta soft bodies", soft_world(), 5, 240, no_edits);
}


/// Early in the soft scene most broad-phase leaves are pending (soft colliders resynced at
/// the end of each step), leaving one or two plain leaves for the tree rebuild.
#[test]
fn soft_bodies_early_snapshot() {
    check_roundtrip("soft bodies early", soft_world(), 5, 60, false, no_edits);
}

/// Re-saving a restored world before stepping it yields the bytes it was restored from:
/// pairs awaiting their post-restore solver-contact replay keep their compact encoding.
#[test]
fn restore_then_save_is_identical() {
    for (name, mut world, steps) in [
        ("awake pile", pile(false), 40),
        ("sleeping pile", pile(true), 150),
        ("shared shapes", shared_shapes_world(), 60),
        ("joints", joints_world(), 60),
        ("soft bodies", soft_world(), 30),
    ] {
        for _ in 0..steps {
            world.step();
        }
        let bytes = save(&world);
        let restored = restore(&bytes);
        assert!(save(&restored) == bytes, "{name}: re-saving a restored world changed its bytes");
    }
}


/// Impulse joints added at runtime next to a multibody: the multibody links' poses are
/// rewritten by forward kinematics before their colliders follow, so the solver contacts of
/// their pairs must not be replayed from the collider poses.
#[test]
fn multibody_with_joint_churn() {
    check_roundtrip("multibody with joint churn", joints_world(), 160, 30, false, |world, step, spawned| {
        // Joint churn: link and unlink pairs of pile bodies, impulse and multibody alike.
        let dynamic: Vec<_> = world
            .bodies
            .iter()
            .filter(|(_, b)| b.is_dynamic())
            .map(|(h, _)| h)
            .collect();
        if step % 23 == 4 {
            let (a, b) = (dynamic[step % 30], dynamic[(step % 30) + 40]);
            world.insert_impulse_joint(a, b, SphericalJointBuilder::new().build().data);
        }
        if step % 31 == 7 {
            let (a, b) = (dynamic[(step % 20) + 2], dynamic[(step % 20) + 60]);
            if let Some(h) = world.insert_multibody_joint(a, b, SphericalJointBuilder::new().build().data) {
                let _ = h;
                spawned.push(a);
            }
        }
        if step % 37 == 11 {
            let first = world.impulse_joints.iter().next().map(|(h, _)| h);
            if let Some(h) = first {
                world.remove_impulse_joint(h);
            }
        }
    });
}

/// A collider moving with its body resends its record, not its (unchanged) heavy shape, even
/// when no other collider holds that shape.
#[test]
fn delta_moving_heavy_shape_is_not_resent() {
    let n = 24usize;
    let mut vtx = Vec::new();
    let mut idx = Vec::new();
    for i in 0..=n {
        for j in 0..=n {
            let y = 0.05 * ((i * 7 + j * 3) % 5) as Real;
            vtx.push(Vector::new(i as Real * 0.1, y, j as Real * 0.1));
        }
    }
    for i in 0..n {
        for j in 0..n {
            let a = (i * (n + 1) + j) as u32;
            idx.push([a, a + 1, a + (n + 1) as u32]);
            idx.push([a + 1, a + (n + 2) as u32, a + (n + 1) as u32]);
        }
    }
    let mesh = ColliderBuilder::trimesh(vtx, idx).unwrap().build();
    let mesh_bytes = bincode::serialize(mesh.shared_shape()).unwrap().len();

    let mut world = PhysicsWorld::new();
    let rb = world.insert_body(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 50.0, 0.0))
            .additional_mass(1.0),
    );
    world.insert_collider(mesh, Some(rb));
    world.enable_change_tracking();
    let mut acked = Some(world.colliders.change_tick().unwrap());
    let mut receiver = restore(&save(&world));
    receiver.colliders.set_delta_baseline(acked.unwrap());
    for step in 0..20 {
        world.step();
        let (delta, tick) = world.colliders.delta_since(acked).unwrap();
        let bytes = bincode::serialize(&delta).unwrap();
        assert!(
            bytes.len() * 10 < mesh_bytes,
            "step {step}: a {}-byte collider delta for a moving {mesh_bytes}-byte mesh",
            bytes.len()
        );
        receiver
            .colliders
            .apply_delta(bincode::deserialize(&bytes).unwrap())
            .unwrap();
        acked = Some(tick);
        assert!(
            bincode::serialize(&world.colliders).unwrap()
                == bincode::serialize(&receiver.colliders).unwrap()
        );
    }
}

/// Change tracking must not hold heavy shapes, which would make every `shape_mut` copy them.
#[test]
fn tracking_does_not_copy_edited_heavy_shapes() {
    let vtx = vec![
        Vector::new(0.0, 0.0, 0.0),
        Vector::new(1.0, 0.0, 0.0),
        Vector::new(0.0, 0.0, 1.0),
        Vector::new(1.0, 0.0, 1.0),
    ];
    let mesh = ColliderBuilder::trimesh(vtx, vec![[0, 1, 2], [1, 3, 2]]).unwrap();
    let mut world = PhysicsWorld::new();
    let co = world.insert_collider(mesh, None);
    world.enable_change_tracking();
    world.colliders.delta_since(None).unwrap();

    let before = std::sync::Arc::as_ptr(&world.colliders[co].shared_shape().0) as *const ();
    world.colliders[co]
        .shape_mut()
        .as_trimesh_mut()
        .unwrap()
        .update_vertices(|vtx| vtx[0].y += 0.01);
    let after = std::sync::Arc::as_ptr(&world.colliders[co].shared_shape().0) as *const ();
    assert_eq!(before, after, "shape_mut copied the shape");
}

/// An incremental delta needs the receiver to hold its baseline.
#[test]
fn incremental_deltas_need_their_baseline() {
    use rapier3d::data::DeltaError;

    let mut server = RigidBodySet::new();
    server.enable_change_tracking();
    server.insert(RigidBodyBuilder::dynamic());
    let (full, tick) = server.delta_since(None).unwrap();
    server.insert(RigidBodyBuilder::fixed());
    let (delta, _) = server.delta_since(Some(tick)).unwrap();

    // A receiver that never saw the baseline.
    let mut fresh = RigidBodySet::new();
    assert_eq!(
        fresh.apply_delta(delta.clone()).unwrap_err(),
        DeltaError::BaselineMismatch
    );
    fresh.apply_delta(full).unwrap();
    fresh.apply_delta(delta).unwrap();
    assert_eq!(fresh.len(), 2);

    // A clone issues its own ticks: the original's are unknown to it.
    let mut clone = server.clone();
    assert_eq!(
        clone.delta_since(Some(tick)).unwrap_err(),
        DeltaError::UnknownBaseline
    );
}

/// A receiver that stepped on its own no longer holds its baseline: it recovers with a full
/// delta, which also drops the workspaces it built while running ahead.
#[test]
fn stepped_receiver_recovers_with_a_full_delta() {
    use rapier3d::data::DeltaError;

    let mut world = pile(true);
    for _ in 0..150 {
        world.step();
    }
    world.enable_change_tracking();
    let tick = world.change_tick().unwrap();
    let mut receiver = restore(&save(&world));
    receiver.set_delta_baseline(tick);

    world.step();
    let (delta, acked) = world.delta_since(Some(tick)).unwrap();
    receiver.apply_delta(delta).unwrap();

    // Prediction: the receiver runs ahead of the sender.
    for _ in 0..40 {
        receiver.step();
    }
    world.step();
    let (delta, _) = world.delta_since(Some(acked)).unwrap();
    assert_eq!(
        receiver.apply_delta(delta).unwrap_err(),
        DeltaError::BaselineMismatch
    );

    let (full, _) = world.delta_since(None).unwrap();
    receiver.apply_delta(full).unwrap();
    assert_same(0, &record(&world), &record(&receiver), "full delta after prediction");
    for step in 0..60 {
        world.step();
        receiver.step();
        assert_same(step, &record(&world), &record(&receiver), "after the full delta");
    }
}
