//! The `unsync-callbacks` contract: a `PhysicsHooks` / `EventHandler` that is **not** `Sync`
//! compiles and runs, whatever `parallel` is doing.
//!
//! Most of the value here is that this file compiles at all: the hook and the handler
//! below hold a `Cell`, so `Sync` is not satisfied and any `Sync` bound creeping back onto
//! either trait turns into a build error. The assertions then confirm the callbacks were
//! actually reached rather than silently skipped.
//!
//! Run with `--features parallel,unsync-callbacks` for the combination that matters — the
//! engine has to keep the callbacks on the thread driving the step while the solver stays
//! threaded.
#![cfg(feature = "unsync-callbacks")]

use rapier3d::prelude::*;
use std::cell::Cell;

/// Counts its own invocations through a `Cell`, which is `Send` but not `Sync`.
#[derive(Default)]
struct UnsyncHooks {
    filtered: Cell<usize>,
}

impl PhysicsHooks for UnsyncHooks {
    fn filter_contact_pair(&self, _: &PairFilterContext) -> Option<SolverFlags> {
        self.filtered.set(self.filtered.get() + 1);
        Some(SolverFlags::COMPUTE_IMPULSES)
    }
}

#[derive(Default)]
struct UnsyncEvents {
    collisions: Cell<usize>,
}

impl EventHandler for UnsyncEvents {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        _event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
        self.collisions.set(self.collisions.get() + 1);
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

/// Enough falling boxes to push the step onto its parallel paths where `parallel` is on.
fn scene() -> (RigidBodySet, ColliderSet, IslandManager) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();

    colliders.insert(
        ColliderBuilder::cuboid(100.0, 1.0, 100.0)
            .translation(Vector::new(0.0, -1.0, 0.0))
            .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );

    for i in 0..8 {
        for j in 0..8 {
            for k in 0..8 {
                let rb = bodies.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(Vector::new(
                            i as Real * 1.1 - 4.0,
                            j as Real * 1.1 + 1.0,
                            k as Real * 1.1 - 4.0,
                        ))
                        .build(),
                );
                colliders.insert_with_parent(
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5)
                        .active_hooks(ActiveHooks::FILTER_CONTACT_PAIRS)
                        .active_events(ActiveEvents::COLLISION_EVENTS)
                        .build(),
                    rb,
                    &mut bodies,
                );
            }
        }
    }

    (bodies, colliders, IslandManager::new())
}

#[test]
fn non_sync_hooks_and_events_are_invoked() {
    let (mut bodies, mut colliders, mut islands) = scene();
    let mut pipeline = PhysicsPipeline::new();
    let mut broad_phase = BroadPhaseBvh::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let params = IntegrationParameters::default();

    let hooks = UnsyncHooks::default();
    let events = UnsyncEvents::default();

    for _ in 0..60 {
        pipeline.step(
            Vector::new(0.0, -9.81, 0.0),
            &params,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd_solver,
            &hooks,
            &events,
        );
    }

    assert!(
        hooks.filtered.get() > 0,
        "filter_contact_pair was never called"
    );
    assert!(
        events.collisions.get() > 0,
        "handle_collision_event was never called"
    );
}

/// A dedicated thread pool is still usable under `unsync-callbacks` — the caller enters it
/// themselves, since the pipeline's own pool API is compiled out along with the bound.
///
/// `ThreadPool::install` runs its body on the calling thread whenever that thread is already
/// a member of the same pool (rayon's `Registry::in_worker` only migrates for outsiders), and
/// every nested rayon construct then targets that pool. So stepping from inside the pool
/// gives the pool's threads *and* keeps the callbacks on one known thread. The callback is
/// built inside the closure because `install` needs its body to be `Send`, which a
/// thread-affine callback is not.
#[cfg(feature = "parallel")]
#[test]
fn dedicated_pool_is_usable_when_the_caller_enters_it() {
    let pool = rapier3d::rayon::ThreadPoolBuilder::new()
        .num_threads(3)
        .build()
        .unwrap();

    let (hook_calls, workers) = pool.install(|| {
        // Built here: nothing non-`Send` crosses into the pool.
        let hooks = UnsyncHooks::default();
        let events = UnsyncEvents::default();
        let (mut bodies, mut colliders, mut islands) = scene();
        let mut pipeline = PhysicsPipeline::new();
        let mut broad_phase = BroadPhaseBvh::new();
        let mut narrow_phase = NarrowPhase::new();
        let mut impulse_joints = ImpulseJointSet::new();
        let mut multibody_joints = MultibodyJointSet::new();
        let mut ccd_solver = CCDSolver::new();
        let params = IntegrationParameters::default();

        for _ in 0..30 {
            pipeline.step(
                Vector::new(0.0, -9.81, 0.0),
                &params,
                &mut islands,
                &mut broad_phase,
                &mut narrow_phase,
                &mut bodies,
                &mut colliders,
                &mut impulse_joints,
                &mut multibody_joints,
                &mut ccd_solver,
                &hooks,
                &events,
            );
        }
        // Reports the pool this thread currently belongs to — which is what we want to
        // observe: the step's parallel regions went to our pool, not the global one.
        (hooks.filtered.get(), rapier3d::rayon::current_num_threads())
    });

    assert_eq!(workers, 3, "the step ran against the global pool, not ours");
    assert!(hook_calls > 0, "filter_contact_pair was never called");
}
