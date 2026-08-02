//! Physics pipeline structures.

use crate::alloc_prelude::*;

use crate::counters::Counters;
use crate::dynamics::{
    CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
    RigidBodySet,
};
use crate::geometry::{
    BroadPhaseBvh, BroadPhasePairEvent, ColliderHandle, ColliderSet, ContactManifoldIndex,
    NarrowPhase,
};
use crate::math::Vector;
use crate::pipeline::{EventHandler, PhysicsHooks};

mod quarantine;
pub use quarantine::Quarantine;
mod solve;
mod substep;
#[cfg(test)]
mod test;
#[cfg(test)]
mod test_staged;

/// The main physics simulation engine that runs your physics world forward in time.
///
/// Think of this as the "game loop" for your physics simulation. Each frame, you call
/// [`PhysicsPipeline::step`] to advance the simulation by one timestep. This structure
/// handles all the complex physics calculations: detecting collisions between objects,
/// resolving contacts so objects don't overlap, and updating positions and velocities.
///
/// ## Performance note
/// This structure only contains temporary working memory (scratch buffers). You can create
/// a new one anytime, but it's more efficient to reuse the same instance across frames
/// since Rapier can reuse allocated memory.
///
/// ## How it works (simplified)
/// Rapier uses a time-stepping approach where each step involves:
/// 1. **Collision detection**: Find which objects are touching or overlapping
/// 2. **Constraint solving**: Calculate forces to prevent overlaps and enforce joint constraints
/// 3. **Integration**: Update object positions and velocities based on forces and gravity
/// 4. **Position correction**: Fix any remaining overlaps that might have occurred
// NOTE: this contains only workspace data, so there is no point in making this serializable.
pub struct PhysicsPipeline {
    /// Counters used for benchmarking only.
    pub counters: Counters,
    joint_constraint_indices: Vec<ContactManifoldIndex>,
    /// Whether [`Self::joint_constraint_indices`] has been filled by this pipeline yet.
    /// The joint set memoizes its selection against the buffer the caller keeps, so a
    /// pipeline that just came into existence must invalidate that memo before its first
    /// selection — otherwise it reuses a buffer it never filled.
    joint_selection_primed: bool,
    broad_phase_events: Vec<BroadPhasePairEvent>,
    /// Colliders moved by the last `advance_to_final_positions` with their fresh broad-phase
    /// AABBs, fed to the broad-phase refresh without the user-modification tracking. AABBs are
    /// computed inside the advance loop while body/collider are in cache.
    end_step_collider_aabbs: Vec<(ColliderHandle, crate::geometry::Aabb)>,
    /// Non-finite state detected and neutralized during the last step.
    quarantine: Quarantine,
    /// Scratch buffer holding the active body handles (parallel body update).
    #[cfg(feature = "parallel")]
    active_body_handles: Vec<crate::dynamics::RigidBodyHandle>,
    /// Scratch: per-active-body sleep observations `(persistent island id, eligible)`,
    /// run-length compressed by the fused traversal, consumed by `IslandManager::
    /// update_islands`'s whole-island sleep decision — which never re-touches the body arena.
    sleep_observations: Vec<(u32, bool)>,
    /// The single, unified solver: the awake island is solved by its colored,
    /// staged workers. On a non-parallel (or wasm) build it runs with one worker
    /// inline on the calling thread.
    staged_solver: crate::dynamics::StagedIslandSolver,
    /// Handle on the BVH optimization pass running concurrently with the narrow
    /// phase and solver (the `Mutex` only exists to keep the pipeline `Sync`; it is
    /// never contended).
    #[cfg(feature = "parallel")]
    deferred_bvh:
        std::sync::Mutex<Option<std::sync::mpsc::Receiver<crate::geometry::DeferredBvhOptimize>>>,
    /// Deferred BVH optimization that had no spare worker to run on (single-threaded
    /// pool, or `parallel` off): run inline by `join_deferred_bvh_optimize`, i.e. at
    /// the same point of the step where the concurrent one is joined.
    deferred_bvh_inline: Option<crate::geometry::DeferredBvhOptimize>,
    /// Pool running the parallel parts of the step (see [`Self::configure_thread_pool`]).
    /// `None` uses whichever pool the calling thread is in.
    #[cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]
    thread_pool: Option<std::sync::Arc<rayon::ThreadPool>>,
}

impl Default for PhysicsPipeline {
    fn default() -> Self {
        PhysicsPipeline::new()
    }
}

#[allow(dead_code)]
fn check_pipeline_send_sync() {
    fn do_test<T: Sync>() {}
    do_test::<PhysicsPipeline>();
}

impl PhysicsPipeline {
    /// Creates a new physics pipeline.
    ///
    /// Call this once when setting up your physics world. The pipeline can be reused
    /// across multiple frames for better performance.
    pub fn new() -> PhysicsPipeline {
        PhysicsPipeline {
            counters: Counters::new(true),
            #[cfg(feature = "parallel")]
            active_body_handles: vec![],
            sleep_observations: Vec::new(),
            staged_solver: crate::dynamics::StagedIslandSolver::new(),
            #[cfg(feature = "parallel")]
            deferred_bvh: std::sync::Mutex::new(None),
            deferred_bvh_inline: None,
            #[cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]
            thread_pool: None,
            joint_constraint_indices: vec![],
            joint_selection_primed: false,
            broad_phase_events: vec![],
            end_step_collider_aabbs: vec![],
            quarantine: Quarantine::default(),
        }
    }

    /// Completes the BVH optimization pass deferred by the last broad-phase update (if
    /// any) and puts the optimized tree back into the broad-phase. Must be called before
    /// anything uses the broad-phase tree again.
    ///
    /// Waits for the concurrent pass when one was spawned; otherwise runs it here. Both
    /// paths leave the same tree behind, so the build and the pool size don't change what
    /// the rest of the step sees.
    fn join_deferred_bvh_optimize(&mut self, broad_phase: &mut BroadPhaseBvh) {
        #[cfg(feature = "parallel")]
        if let Some(rx) = self.deferred_bvh.get_mut().unwrap().take() {
            let task = rx.recv().expect("the deferred BVH optimization task died");
            broad_phase.finish_deferred_optimize(task);
            return;
        }

        if let Some(mut task) = self.deferred_bvh_inline.take() {
            task.run();
            broad_phase.finish_deferred_optimize(task);
        }
    }

    /// Advances the physics simulation by one timestep.
    ///
    /// This is the main function you'll call every frame in your game loop. It performs all
    /// physics calculations: collision detection, constraint solving, and updating object positions.
    ///
    /// # Parameters
    ///
    /// * `gravity` - The gravity vector applied to all dynamic bodies (e.g., `vector![0.0, -9.81, 0.0]` for Earth gravity pointing down)
    /// * `integration_parameters` - Controls the simulation quality and timestep size (typically 60 Hz = 1/60 second per step)
    /// * `islands` - Internal system that groups connected objects together for efficient solving (automatically managed)
    /// * `broad_phase` - Fast collision detection phase that filters out distant object pairs (automatically managed)
    /// * `narrow_phase` - Precise collision detection that computes exact contact points (automatically managed)
    /// * `bodies` - Your collection of rigid bodies (the physical objects that move and collide)
    /// * `colliders` - The collision shapes attached to your bodies (boxes, spheres, meshes, etc.)
    /// * `impulse_joints` - Regular joints connecting bodies (hinges, sliders, etc.)
    /// * `multibody_joints` - Articulated joints for robot-like structures (optional, can be empty)
    /// * `ccd_solver` - Continuous collision detection to prevent fast objects from tunneling through thin walls
    /// * `hooks` - Optional callbacks to customize collision filtering and contact modification
    /// * `events` - Optional handler to receive collision events (when objects start/stop touching)
    ///
    /// # Example
    ///
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut bodies = RigidBodySet::new();
    /// # let mut colliders = ColliderSet::new();
    /// # let mut impulse_joints = ImpulseJointSet::new();
    /// # let mut multibody_joints = MultibodyJointSet::new();
    /// # let mut islands = IslandManager::new();
    /// # let mut broad_phase = BroadPhaseBvh::new();
    /// # let mut narrow_phase = NarrowPhase::new();
    /// # let mut ccd_solver = CCDSolver::new();
    /// # let mut physics_pipeline = PhysicsPipeline::new();
    /// # let integration_parameters = IntegrationParameters::default();
    /// // In your game loop:
    /// physics_pipeline.step(
    ///     Vector::new(0.0, -9.81, 0.0),  // Gravity pointing down
    ///     &integration_parameters,
    ///     &mut islands,
    ///     &mut broad_phase,
    ///     &mut narrow_phase,
    ///     &mut bodies,
    ///     &mut colliders,
    ///     &mut impulse_joints,
    ///     &mut multibody_joints,
    ///     &mut ccd_solver,
    ///     &(),  // No custom hooks
    ///     &(),  // No event handler
    /// );
    /// ```
    pub fn step(
        &mut self,
        gravity: Vector,
        integration_parameters: &IntegrationParameters,
        islands: &mut IslandManager,
        broad_phase: &mut BroadPhaseBvh,
        narrow_phase: &mut NarrowPhase,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
        ccd_solver: &mut CCDSolver,
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
    ) {
        // With a dedicated pool configured, run the whole step inside it.
        #[cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]
        if let Some(pool) = self.thread_pool.clone() {
            return pool.install(|| {
                self.step_inner(
                    gravity,
                    integration_parameters,
                    islands,
                    broad_phase,
                    narrow_phase,
                    bodies,
                    colliders,
                    impulse_joints,
                    multibody_joints,
                    ccd_solver,
                    hooks,
                    events,
                )
            });
        }

        self.step_inner(
            gravity,
            integration_parameters,
            islands,
            broad_phase,
            narrow_phase,
            bodies,
            colliders,
            impulse_joints,
            multibody_joints,
            ccd_solver,
            hooks,
            events,
        )
    }
}

#[cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]
impl PhysicsPipeline {
    /// Configures a dedicated thread pool for this pipeline's parallel work (default:
    /// whichever pool the calling thread is in — usually the global one or the one
    /// setup with `ThreadPool::install`.
    pub fn configure_thread_pool(
        &mut self,
        num_threads: usize,
    ) -> Result<(), rayon::ThreadPoolBuildError> {
        let builder = rayon::ThreadPoolBuilder::new()
            .num_threads(num_threads)
            .thread_name(|i| alloc::format!("rapier-worker-{i}"));

        self.thread_pool = Some(std::sync::Arc::new(builder.build()?));
        Ok(())
    }

    /// The thread-pool used by this physics pipeline, if it was configured.
    pub fn thread_pool(&self) -> Option<std::sync::Arc<rayon::ThreadPool>> {
        self.thread_pool.clone()
    }

    /// Sets (or clears) the thread pool running this pipeline's parallel work.
    ///
    /// Unlike [`Self::configure_thread_pool`], this takes an existing pool.
    pub fn set_thread_pool(&mut self, pool: Option<std::sync::Arc<rayon::ThreadPool>>) {
        self.thread_pool = pool;
    }

    /// Removes the dedicated thread pool: the parallel parts of the step run on whichever
    /// pool the calling thread is in again.
    pub fn clear_thread_pool(&mut self) {
        self.thread_pool = None;
    }

    /// The number of workers this pipeline's parallel work runs on: the size of its
    /// dedicated thread pool, or of the pool the calling thread is in if it has none.
    pub fn num_threads(&self) -> Option<usize> {
        self.thread_pool
            .as_ref()
            .map(|pool| pool.current_num_threads())
    }
}
