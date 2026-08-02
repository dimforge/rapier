//! Regression test for the deferred BVH optimization on a single-worker pool.
//!
//! The broad-phase defers its (quality-only) optimization pass in every build, so the
//! tree this step's pair traversal walked stays un-optimized until the join point —
//! that is what keeps a `parallel` build and a non-`parallel` build on the same tree.
//! Only the *execution* needs a spare worker.
//!
//! `step()` runs inside the dedicated pool (`PhysicsPipeline::step`), so with a
//! single-worker pool the step body *is* the pool's only thread. Handing the pass to
//! `rayon::spawn` there queues it behind the `recv` that waits for it, and the step
//! never returns. The fix routes those to `join_deferred_bvh_optimize`, which runs
//! them inline.
//!
//! Symptom when it regresses: `step()` hangs on the first step that defers a pass —
//! not the first step of the run, since the optimizer only fires once enough
//! quality-degrading changes accumulate.
// The pool API this drives is compiled out by `unsync-callbacks`.
#![cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]

use rapier3d::prelude::*;
use std::sync::mpsc;
use std::time::Duration;

/// The scene below runs in milliseconds when healthy; a deadlocked step never returns,
/// so this only has to separate "running" from "wedged".
const TIMEOUT: Duration = Duration::from_secs(60);
const STEPS: usize = 120;

/// Falling balls over a ground: every body moves every step, so leaf updates accrue
/// the optimizer debt that makes `update` defer a pass.
fn scene() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0, 0.0)),
        ColliderBuilder::cuboid(100.0, 1.0, 100.0),
    );

    let n = 12i32;
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                world.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(
                            i as f32 * 1.5 - 9.0,
                            j as f32 * 1.5 + 1.0,
                            k as f32 * 1.5 - 9.0,
                        ))
                        .can_sleep(false),
                    ColliderBuilder::ball(0.5),
                );
            }
        }
    }
    world
}

/// Steps against a one-worker dedicated pool and requires the run to finish.
///
/// The stepping runs on a spawned thread so a regression fails the test instead of
/// hanging the harness forever.
#[test]
fn single_worker_pool_steps_to_completion() {
    let (tx, rx) = mpsc::channel();

    std::thread::spawn(move || {
        let mut world = scene();
        world
            .configure_thread_pool(1)
            .expect("failed to build the single-worker pool");

        for _ in 0..STEPS {
            world.step();
        }
        let _ = tx.send(());
    });

    match rx.recv_timeout(TIMEOUT) {
        Ok(()) => {}
        Err(mpsc::RecvTimeoutError::Timeout) => panic!(
            "step() made no progress for {TIMEOUT:?} on a single-worker pool. The \
             deferred BVH optimization was handed to `rayon::spawn` with no spare \
             worker to run it: the step blocks in `join_deferred_bvh_optimize`'s \
             `recv` while the task it waits for is queued behind that very step. \
             Check that `solve.rs` only spawns when `current_num_threads() > 1` and \
             otherwise leaves the task to `deferred_bvh_inline`."
        ),
        Err(mpsc::RecvTimeoutError::Disconnected) => panic!("stepping thread died"),
    }
}
