//! The cross-platform determinism contract, end to end.
//!
//! The solver is 4-lane AoSoA over `wide`'s `WideF32x4`, whose per-platform
//! intrinsics are bitwise identical lane for lane to the portable scalar
//! reference (pinned per-operation by `simd_backend_parity`). A simulation must
//! therefore come out the same, bit for bit, whichever intrinsics `wide` picks.
//!
//! That contract spans separate binaries, so it cannot be asserted inside one
//! test process: this pins a golden hash instead, and CI runs the file on every
//! target. A divergence fails whichever build drifted.
//!
//! Only meaningful under `enhanced-determinism` (which pins the transcendentals
//! through libm on every backend and forces glam's scalar core). `simd8` is
//! excluded on purpose: 8 lanes bundle constraints differently and are their own
//! determinism domain.
#![cfg(all(feature = "enhanced-determinism", not(feature = "simd8")))]

use rapier3d::prelude::*;

/// FNV-1a over the raw bit patterns of every body's pose and velocity.
struct Fnv(u64);

impl Fnv {
    fn new() -> Self {
        Self(0xcbf29ce484222325)
    }

    fn eat(&mut self, f: Real) {
        for b in f.to_bits().to_le_bytes() {
            self.0 ^= b as u64;
            self.0 = self.0.wrapping_mul(0x100000001b3);
        }
    }
}

fn state_hash(bodies: &RigidBodySet) -> u64 {
    let mut h = Fnv::new();
    let mut handles: Vec<_> = bodies.iter().map(|(handle, _)| handle).collect();
    handles.sort_by_key(|h| h.into_raw_parts().0);

    for handle in handles {
        let rb = &bodies[handle];
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

/// A pile (many same-color contacts, so the parallel color stages and the SIMD
/// chunking genuinely engage) plus a joint chain, stepped long enough to settle.
fn run(steps: usize) -> u64 {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut broad_phase = BroadPhaseBvh::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut islands = IslandManager::new();
    let mut ccd = CCDSolver::new();
    let params = IntegrationParameters::default();

    let ground = bodies.insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)));
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(20.0, 0.5, 20.0),
        ground,
        &mut bodies,
    );

    for i in 0..12 {
        for j in 0..3 {
            for k in 0..12 {
                // Jitter so the pile settles asymmetrically instead of landing
                // in a symmetric configuration that hides ordering effects.
                let jitter = (i as Real * 0.013 + k as Real * 0.017) % 0.05;
                let rb = bodies.insert(RigidBodyBuilder::dynamic().translation(Vector::new(
                    i as Real * 1.05 - 6.0 + jitter,
                    j as Real * 1.05 + 0.55,
                    k as Real * 1.05 - 6.0 - jitter,
                )));
                colliders.insert_with_parent(
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5),
                    rb,
                    &mut bodies,
                );
            }
        }
    }

    let anchor = bodies.insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, 8.0, 0.0)));
    let mut prev = anchor;
    for i in 0..4 {
        let rb = bodies.insert(RigidBodyBuilder::dynamic().translation(Vector::new(
            0.6 * (i + 1) as Real,
            8.0,
            0.0,
        )));
        colliders.insert_with_parent(ColliderBuilder::ball(0.25), rb, &mut bodies);
        impulse_joints.insert(
            prev,
            rb,
            SphericalJointBuilder::new()
                .local_anchor1(Vector::X * 0.3)
                .local_anchor2(Vector::X * -0.3),
            true,
        );
        prev = rb;
    }

    for _ in 0..steps {
        pipeline.step(
            Vector::Y * -9.81,
            &params,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &(),
            &(),
        );
    }
    state_hash(&bodies)
}

#[test]
fn golden_state_hash_is_backend_independent() {
    // Regenerate by running this test on every supported target: they must all
    // print the same value. If they don't, the backends have diverged and
    // `simd_backend_parity` should say on which operation.
    const GOLDEN: u64 = 0x5ba1_cfe5_3d4d_6aef;
    let hash = run(120);
    assert_eq!(
        hash, GOLDEN,
        "state hash drifted: got {hash:#018x}, expected {GOLDEN:#018x}"
    );
}
