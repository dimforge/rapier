//! Forward-progress test for `step()` called from a thread that is NOT a member of
//! the pool it drives — the documented setup for `configure_thread_pool`, where
//! rayon injects the whole step into the pool as a foreign job.
//!
//! Historically this configuration livelocked. `step_with_hot_workers` spawned one
//! spinning helper per pool thread via `Scope::spawn_broadcast`, each looping until
//! the step body set `done`; a worker executing the injected step could pull its own
//! pending broadcast copy into a nested work-stealing wait, and spinning on `done`
//! from inside that frame was a circular wait — `done` is set only after the step
//! finishes, the step was blocked on the join, and the join could not return past the
//! spinner. Symptom: `step()` never returned, every worker burning CPU in
//! `yield_now`/`find_work`; one observed hang lasted 49 minutes on 4.9 s of work.
//!
//! The hot-worker machinery has since been removed outright (it bought ~0.13 ms/step
//! in a narrow body-count band and cost ~0.1 ms on large 3D scenes — see
//! `docs/parallel-caps-and-cuboid-fastpath-benchmark.md`), so that circular wait is
//! now structurally impossible. The test stays because the injection path it drives
//! is still live and still the one most likely to wedge.
// The pool API this drives is compiled out by `unsync-callbacks`.
#![cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]

use rapier3d::prelude::*;
use std::sync::mpsc;
use std::time::{Duration, Instant};

/// Wall-clock budget for one batch of steps. The batch below takes tens of
/// milliseconds when healthy; a livelocked step never returns at all, so this
/// only has to separate "running" from "wedged", not measure anything.
const BATCH_TIMEOUT: Duration = Duration::from_secs(60);

const WORKERS: usize = 8;
const BATCHES: usize = 40;
const STEPS_PER_BATCH: usize = 25;

/// A grid of boxes with sleeping disabled, large enough that the step's parallel
/// regions actually engage every worker rather than running their small-domain
/// serial fallbacks.
fn scene() -> PhysicsWorld {
    let mut world = PhysicsWorld::new();

    world.insert(
        RigidBodyBuilder::fixed().translation(Vector::new(0.0, -1.0, 0.0)),
        ColliderBuilder::cuboid(100.0, 1.0, 100.0),
    );

    let n = 10i32; // 1000 dynamic bodies
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                world.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(
                            i as f32 * 2.0 - 10.0,
                            j as f32 * 2.0 + 1.0,
                            k as f32 * 2.0 - 10.0,
                        ))
                        // Keeps every body in the active set for the whole run, so
                        // the parallel regions stay engaged on every step.
                        .can_sleep(false),
                    ColliderBuilder::cuboid(1.0, 1.0, 1.0),
                );
            }
        }
    }

    world
}

/// Steps from a non-pool thread against a dedicated pool — the configuration that
/// used to livelock — and requires steady forward progress.
///
/// The stepping runs on a spawned thread so a regression fails the test instead of
/// hanging the harness forever: the wedged thread is abandoned and the process
/// tears it down on exit.
#[test]
fn external_caller_with_dedicated_pool_makes_progress() {
    let (tx, rx) = mpsc::channel();

    std::thread::spawn(move || {
        for batch in 0..BATCHES {
            // A fresh pool per batch re-exercises the injection path, which is
            // where the stale broadcast copy gets picked up.
            let mut world = scene();
            world
                .configure_thread_pool(WORKERS)
                .expect("failed to build the rayon pool");

            for _ in 0..STEPS_PER_BATCH {
                world.step();
            }

            // A closed channel just means the test already failed; stop quietly.
            if tx.send(batch).is_err() {
                return;
            }
        }
    });

    let start = Instant::now();
    for expected in 0..BATCHES {
        match rx.recv_timeout(BATCH_TIMEOUT) {
            Ok(batch) => assert_eq!(batch, expected, "batches must arrive in order"),
            Err(mpsc::RecvTimeoutError::Timeout) => panic!(
                "step() made no progress for {:?} on batch {expected}/{BATCHES} \
                 ({WORKERS} workers, external caller). Something in the step is \
                 waiting on work that cannot complete while the injected step job \
                 holds the worker running it — look for a rayon scope, broadcast or \
                 join whose completion depends on a task the same worker must first \
                 finish. See this file's header for the original instance.",
                BATCH_TIMEOUT
            ),
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                panic!("stepping thread died on batch {expected}/{BATCHES}")
            }
        }
    }

    eprintln!(
        "{BATCHES} batches x {STEPS_PER_BATCH} steps at {WORKERS} workers in {:?}",
        start.elapsed()
    );
}
