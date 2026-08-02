//! The `parallel` feature's determinism contract: results must be bitwise
//! identical for any rayon pool size (and therefore identical across machines
//! with different core counts). See `LAYOUT_REF_WORKERS` and
//! `NarrowPhase::apply_pair_transitions` for the mechanisms under test.
#![cfg(feature = "parallel")]

use rapier3d::geometry::ContactPair;
use rapier3d::pipeline::EventHandler;
use rapier3d::prelude::*;
use std::sync::Mutex;

/// Records the collision-event sequence: the emission ORDER is part of the
/// determinism contract (it is far more sensitive to scheduling leaks than the
/// body states, which are often permutation-invariant).
#[derive(Default)]
struct EventLog(Mutex<Vec<String>>);

impl EventHandler for EventLog {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
        self.0.lock().unwrap().push(format!("{event:?}"));
    }

    fn handle_contact_force_event(
        &self,
        _dt: Real,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        _contact_pair: &ContactPair,
        _total_force_magnitude: Real,
    ) {
    }
}

/// One bit-faithful snapshot per body, sorted by handle. Float `Debug` output is
/// round-trip exact, so identical strings ⟺ identical bit patterns (modulo NaN
/// payloads, which must not occur here anyway).
fn snapshot(bodies: &RigidBodySet) -> Vec<(u32, String)> {
    let mut out: Vec<_> = bodies
        .iter()
        .map(|(h, rb)| {
            (
                h.into_raw_parts().0,
                format!(
                    "{:?} {:?} {:?} {:?} {}",
                    rb.translation(),
                    rb.rotation(),
                    rb.linvel(),
                    rb.angvel(),
                    rb.is_sleeping()
                ),
            )
        })
        .collect();
    out.sort_by_key(|e| e.0);
    out
}

/// FNV-1a over the serialized broad-phase + narrow-phase.
///
/// The body snapshots above are float state; this is stored *order and layout* —
/// map iteration order, per-collider adjacency lists, graph edge order — which is
/// where a work-distribution leak shows up first, often a few steps before it moves
/// a float. Sampled rather than computed every step: serializing both structures is
/// far more expensive than the step itself in a debug build.
#[cfg(feature = "serde-serialize")]
fn phase_checksum(bf: &BroadPhaseBvh, nf: &NarrowPhase) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    let mut eat = |bytes: &[u8]| {
        for b in bytes {
            h ^= *b as u64;
            h = h.wrapping_mul(0x100000001b3);
        }
    };
    eat(&bincode::serialize(bf).expect("broad-phase serialization"));
    eat(&bincode::serialize(nf).expect("narrow-phase serialization"));
    h
}

#[cfg(not(feature = "serde-serialize"))]
fn phase_checksum(_bf: &BroadPhaseBvh, _nf: &NarrowPhase) -> u64 {
    0
}

/// Steps a moderately chaotic scene (pile + joint chain + sensor + mid-run
/// collider removal and wake impulse, to exercise pair creation/deletion,
/// touching transitions, sleeping and island edits) and records a snapshot
/// after every step.
type StepRecord = (Vec<(u32, String)>, Vec<String>, u64);

fn run_sim(num_threads: usize, num_steps: usize) -> Vec<StepRecord> {
    let pool = rapier3d::rayon::ThreadPoolBuilder::new()
        .num_threads(num_threads)
        .build()
        .unwrap();

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut bf = BroadPhaseBvh::new();
    let mut nf = NarrowPhase::new();
    let mut islands = IslandManager::new();
    let mut ccd = CCDSolver::new();
    let params = IntegrationParameters::default();
    let gravity = Vector::Y * -9.81;

    let ground = bodies.insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, -0.5, 0.0)));
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(20.0, 0.5, 20.0),
        ground,
        &mut bodies,
    );

    // A 12x3x12 jittered pile: enough same-color contacts that some colors clear
    // `min_color_chunks` and the parallel color stages genuinely engage.
    let mut cubes = Vec::new();
    for i in 0..12 {
        for j in 0..3 {
            for k in 0..12 {
                let jitter = (i as f32 * 0.013 + k as f32 * 0.017) % 0.05;
                let pos = Vector::new(
                    i as f32 * 1.05 - 6.0 + jitter,
                    j as f32 * 1.05 + 0.55,
                    k as f32 * 1.05 - 6.0 - jitter,
                );
                let rb = bodies.insert(RigidBodyBuilder::dynamic().translation(pos));
                colliders.insert_with_parent(
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5)
                        .active_events(ActiveEvents::COLLISION_EVENTS),
                    rb,
                    &mut bodies,
                );
                cubes.push(rb);
            }
        }
    }

    // A swinging chain of spherical joints anchored above the pile.
    let anchor = bodies.insert(RigidBodyBuilder::fixed().translation(Vector::new(0.0, 8.0, 0.0)));
    let mut prev = anchor;
    for i in 0..4 {
        let rb = bodies.insert(RigidBodyBuilder::dynamic().translation(Vector::new(
            0.6 * (i + 1) as f32,
            8.0,
            0.0,
        )));
        colliders.insert_with_parent(ColliderBuilder::ball(0.25), rb, &mut bodies);
        let joint = SphericalJointBuilder::new()
            .local_anchor1(Vector::X * 0.3)
            .local_anchor2(Vector::X * -0.3);
        impulse_joints.insert(prev, rb, joint, true);
        prev = rb;
    }

    // A sensor volume the pile settles through (intersection events path).
    colliders.insert(
        ColliderBuilder::cuboid(3.0, 0.5, 3.0)
            .translation(Vector::new(0.0, 1.0, 0.0))
            .sensor(true)
            .active_events(ActiveEvents::COLLISION_EVENTS),
    );

    let events = EventLog::default();
    let mut snapshots = Vec::new();
    for step in 0..num_steps {
        if step == 60 {
            // Remove a cube mid-pile: broad-phase pair deletions + island edits.
            bodies.remove(
                cubes[7],
                &mut islands,
                &mut colliders,
                &mut impulse_joints,
                &mut multibody_joints,
                true,
            );
        }
        if step == 100 {
            // Wake part of the (possibly sleeping) pile.
            bodies[cubes[20]].apply_impulse(Vector::new(2.0, 3.0, 1.0), true);
        }

        pool.install(|| {
            pipeline.step(
                gravity,
                &params,
                &mut islands,
                &mut bf,
                &mut nf,
                &mut bodies,
                &mut colliders,
                &mut impulse_joints,
                &mut multibody_joints,
                &mut ccd,
                &(),
                &events,
            );
        });
        let step_events = core::mem::take(&mut *events.0.lock().unwrap());
        let checksum = if step % 10 == 0 || step + 1 == num_steps {
            phase_checksum(&bf, &nf)
        } else {
            0
        };
        snapshots.push((snapshot(&bodies), step_events, checksum));
    }
    snapshots
}

#[test]
fn identical_results_for_any_worker_count() {
    const STEPS: usize = 160;
    let base = run_sim(1, STEPS);
    for num_threads in [2, 8] {
        let run = run_sim(num_threads, STEPS);
        for step in 0..STEPS {
            let ((base_state, base_events, base_sum), (run_state, run_events, run_sum)) =
                (&base[step], &run[step]);
            assert_eq!(
                base_events, run_events,
                "event-sequence divergence at step {step}, {num_threads} threads vs 1"
            );
            assert_eq!(
                base_sum, run_sum,
                "broad/narrow-phase checksum divergence at step {step}, {num_threads} threads \
                 vs 1: the two pool sizes stored the same simulation in a different layout"
            );
            if base_state != run_state {
                let (h, s) = base_state
                    .iter()
                    .zip(run_state.iter())
                    .find(|(a, b)| a != b)
                    .map(|(a, b)| {
                        (
                            a.0,
                            format!("1 thread: {}\n{num_threads} threads: {}", a.1, b.1),
                        )
                    })
                    .unwrap();
                panic!(
                    "state divergence at step {step}, body {h}, {num_threads} threads vs 1:\n{s}"
                );
            }
        }
    }
}
