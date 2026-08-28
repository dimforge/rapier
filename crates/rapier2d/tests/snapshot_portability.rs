//! Cross-target snapshot portability (2D).
//!
//! A snapshot taken on one machine must be byte-identical to one taken on another, so a
//! server and a browser client (or two peers in a lockstep game) can exchange and compare
//! them. That only holds with `enhanced-determinism` — which pins the floating-point
//! results — plus a stored layout that has no target-dependent encoding left in it. This
//! test pins the second half: it steps a fixed scene and asserts the snapshot's size and
//! digest against a committed constant, so the *same* test binary compiled for a different
//! target must reproduce the *same* number.
//!
//! It is a portability test only when it is actually run on more than one target. The CI
//! `wasm-determinism` job runs it under `wasm32-wasip1` (32-bit pointers, a different
//! libm, a different codegen backend) against the same golden the native jobs check; run
//! it locally the same way with:
//!
//! ```text
//! cargo test -p rapier2d --release --features enhanced-determinism,serde-serialize \
//!     --target wasm32-wasip1 --test snapshot_portability
//! ```
//!
//! (with `CARGO_TARGET_WASM32_WASIP1_RUNNER` pointing at wasmtime or an equivalent).
//!
//! What it has caught: `usize` sentinels. `RigidBodyIds`' active-set ids and the multibody
//! `IndexSequence`'s `first_to_remove` all stored `usize::MAX` to mean "none", which
//! bincode writes as `0xFFFF_FFFF_FFFF_FFFF` on a 64-bit target and `0xFFFF_FFFF` on a
//! 32-bit one — the same state, different bytes. They are `u32` now.
#![cfg(all(feature = "serde-serialize", feature = "enhanced-determinism"))]

use rapier2d::prelude::*;

/// Snapshot size in bytes, and its FNV-1a digest. Both, because the size alone localizes a
/// failure: a differing size means a container's *encoding* changed, an equal size with a
/// differing digest means the values did.
const GOLDEN: (usize, u64) = (88_572, 0x9e84_4370_55e1_8c88);

const STEPS: usize = 60;

fn digest(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= *b as u64;
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h
}

/// Deliberately mixed: box contacts (some settling into sleep), an impulse-joint chain, a
/// multibody articulation, a sensor and a CCD body — so the snapshot covers the island,
/// broad-phase, narrow-phase, solver-graph and multibody containers rather than just body
/// poses.
fn scene() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.gravity = Vector::new(0.0, -9.81);
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5)),
        ColliderBuilder::cuboid(20.0, 0.5),
    );

    for i in 0..10 {
        for j in 0..4 {
            let jitter = (i as Real * 0.013 + j as Real * 0.017) % 0.05;
            world.insert(
                RigidBodyBuilder::dynamic().translation(Vector::new(
                    i as Real * 1.05 - 5.0 + jitter,
                    j as Real * 1.05 + 0.55,
                )),
                ColliderBuilder::cuboid(0.5, 0.5),
            );
        }
    }

    // Impulse-joint chain: exercises the solver's persisted joint coloring.
    let mut prev = world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(0.0, 7.0)));
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

    // Multibody chain: the augmented-mass dof permutation and the topology epoch.
    let size = 0.4;
    let mut last = world.insert_body(RigidBodyBuilder::fixed().translation(Vector::new(6.0, 4.0)));
    for i in 0..6 {
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

    // A sensor to swing through, and a bullet to sweep across the pile.
    world.insert_collider(
        ColliderBuilder::cuboid(3.0, 0.5)
            .translation(Vector::new(0.0, 4.0))
            .sensor(true),
        None,
    );
    world.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(-8.0, 2.0))
            .linvel(Vector::new(90.0, -20.0))
            .ccd_enabled(true)
            .can_sleep(false),
        ColliderBuilder::ball(0.15),
    );

    world
}

#[test]
fn snapshot_is_target_independent() {
    let mut world = scene();
    for _ in 0..STEPS {
        world.step();
    }
    let bytes = bincode::serialize(&world).expect("snapshot serialization");
    let got = (bytes.len(), digest(&bytes));

    assert_eq!(
        got,
        GOLDEN,
        "\nsnapshot differs from the golden on this target \
         ({}-bit pointers).\n  golden: {:?}\n  got:    {:?}\n\
         Either a target-dependent encoding crept back into the stored state (a `usize` \
         sentinel is the usual one), or the simulation itself changed — in which case \
         re-mint GOLDEN and re-verify it on wasm32 before committing.\n",
        core::mem::size_of::<usize>() * 8,
        GOLDEN,
        got,
    );
}
