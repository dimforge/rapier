//! The `parallel`-off ≡ `parallel`-on contract, end to end.
//!
//! Building with the `parallel` feature must not change what the engine computes — only
//! who computes it. The realistic deployment is a native server built with `parallel` in
//! lockstep with a single-threaded wasm client built without it: those two binaries must
//! agree bit for bit.
//!
//! The checksum covers the *serialized* broad-phase and narrow-phase, so it pins stored
//! container order (map iteration order, per-collider adjacency lists, graph edge order),
//! not just float values — which is where a work-distribution difference actually shows
//! up. Body poses and velocities are folded in too, so a divergence that has not yet
//! reached the manifolds still fails the test.
//!
//! That contract spans separate binaries, so it cannot be asserted inside one test
//! process: this pins a golden hash instead, and CI runs the file under both feature
//! sets. A divergence fails whichever build drifted.
//!
//! Only meaningful under `enhanced-determinism`, which is also what makes the hash
//! reproducible across platforms: parry's hash map is an `IndexMap` there, so
//! `BroadPhaseBvh::pairs` serializes in insertion order rather than in an order that
//! depends on the target's hashbrown control-group width. `simd8` is excluded by the
//! `enhanced-determinism` compile error, so this is the 4-lane domain.
#![cfg(all(feature = "enhanced-determinism", feature = "serde-serialize"))]

use rapier3d::prelude::*;

/// Golden hash of [`run`]. Identical in every build; re-mint (with a note saying why)
/// only when a change is *meant* to alter the simulation.
const GOLDEN: u64 = 0xa8b2_0bad_6b09_7e3b;

/// FNV-1a.
struct Fnv(u64);

impl Fnv {
    fn new() -> Self {
        Self(0xcbf29ce484222325)
    }

    fn eat_bytes(&mut self, bytes: &[u8]) {
        for b in bytes {
            self.0 ^= *b as u64;
            self.0 = self.0.wrapping_mul(0x100000001b3);
        }
    }

    fn eat(&mut self, f: Real) {
        self.eat_bytes(&f.to_bits().to_le_bytes());
    }
}

/// Hashes the serialized broad/narrow-phase plus every body's pose and velocity.
fn checksum(world: &PhysicsWorld) -> u64 {
    let mut h = Fnv::new();

    h.eat_bytes(&bincode::serialize(&world.broad_phase).expect("broad-phase serialization"));
    h.eat_bytes(&bincode::serialize(&world.narrow_phase).expect("narrow-phase serialization"));

    let mut handles: Vec<_> = world.bodies.iter().map(|(handle, _)| handle).collect();
    handles.sort_by_key(|h| h.into_raw_parts().0);
    for handle in handles {
        let rb = &world.bodies[handle];
        for c in rb.translation().to_array() {
            h.eat(c);
        }
        for c in rb.rotation().to_array() {
            h.eat(c);
        }
        for c in rb.linvel().to_array() {
            h.eat(c);
        }
        for c in rb.angvel().to_array() {
            h.eat(c);
        }
    }

    h.0
}

fn spawn_cluster(world: &mut PhysicsWorld, seed: usize, height: Real) {
    for i in 0..12 {
        // Jitter so clusters never land in a symmetric configuration that would hide
        // ordering effects.
        let a = (seed * 7 + i * 13) as Real * 0.011;
        let b = (seed * 11 + i * 5) as Real * 0.017;
        world.insert(
            RigidBodyBuilder::dynamic()
                .translation(Vector::new(
                    (i as Real % 4.0) * 1.1 - 2.2 + a,
                    height + (i / 4) as Real * 1.1,
                    b % 3.0 - 1.5,
                ))
                // A sideways kick: these pairs stop overlapping while their colliders
                // keep moving, which is what exercises stale-pair detection.
                .linvel(Vector::new(a % 1.5 - 0.75, 0.0, b % 1.5 - 0.75)),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        );
    }
}

/// A pile that settles and falls asleep, then repeated drops onto it.
///
/// Each ingredient targets one of the divergences this contract covers:
/// - drops arrive while the pile's leaves also move, so one broad-phase update mixes
///   in-place leaf updates with structural insertions (leaf-update ordering);
/// - the pile sleeps between drops, so the narrow phase takes its sparse-awake branch,
///   where the update-candidate order is per-collider adjacency rather than ascending
///   edge id (dirty-list ordering);
/// - the kicked bodies separate again, ageing pairs out of overlap (stale-pair
///   detection, pair timestamps);
/// - the run is long enough for deferred BVH optimization passes to fire.
fn run() -> u64 {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)),
        ColliderBuilder::cuboid(30.0, 0.5, 30.0),
    );

    for i in 0..14 {
        for j in 0..2 {
            for k in 0..14 {
                let jitter = (i as Real * 0.013 + k as Real * 0.017) % 0.05;
                world.insert(
                    RigidBodyBuilder::dynamic().translation(Vector::new(
                        i as Real * 1.05 - 7.0 + jitter,
                        j as Real * 1.05 + 0.55,
                        k as Real * 1.05 - 7.0 - jitter,
                    )),
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5),
                );
            }
        }
    }

    // Settle and sleep.
    for _ in 0..220 {
        world.step();
    }

    // Drops onto the (mostly sleeping) pile.
    for round in 0..10 {
        spawn_cluster(&mut world, round, 6.0);
        for _ in 0..40 {
            world.step();
        }
    }

    checksum(&world)
}

#[test]
fn parallel_and_sequential_builds_agree() {
    let hash = run();
    assert_eq!(
        hash, GOLDEN,
        "\nbroad/narrow-phase checksum drifted: got {hash:#018x}, expected {GOLDEN:#018x}.\n\
         This build's work distribution changed what it computes. The `parallel` feature \
         must only decide *who* runs a chunk — never how work is split, in what order \
         results are merged, or which algorithm runs. If the change was intentional, \
         re-mint GOLDEN and say why in the commit.\n"
    );
}
