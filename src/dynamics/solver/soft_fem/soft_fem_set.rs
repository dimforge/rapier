//! The FEM systems of the soft bodies solved by the staged island solver: the per-substep
//! predict stage's bodies, and the responses of the step's constraints.

use super::system::SoftFemSystem;
use crate::alloc_prelude::*;
use crate::dynamics::solver::soft_constraint::SoftConstraintsSet;
use crate::dynamics::solver::solver_body::SolverBodies;
use crate::dynamics::{SoftBody, SoftBodyHandle, SoftBodySet, SoftFemParameters};
use crate::math::Real;
use core::ops::Range;
use parry::utils::hashmap::HashMap;

/// One awake soft body solved by the FEM path, during one island solve.
struct ActiveFem {
    /// The soft body, valid for the solve's scope (same contract as `AwakeSoftBody::ptr`).
    body: *mut SoftBody,
    /// Its system, owned by `SoftFemSet::systems` and boxed so the pointer stays valid.
    system: *mut SoftFemSystem,
    /// The substep length of the body's solve group.
    dt: Real,
}

// SAFETY: the pointers are only dereferenced during the solve, where the stage discipline gives
// exactly one worker access to a given index (same contract as `AwakeSoftBody`).
unsafe impl Send for ActiveFem {}
unsafe impl Sync for ActiveFem {}

/// The FEM systems of every soft body that selected the FEM solver, looked up by handle and kept
/// across steps (the sparsity pattern and element tables are the expensive part; values refill
/// every substep); `active` holds the awake ones of the island being solved, group-major.
#[derive(Default)]
pub(crate) struct SoftFemSet {
    systems: HashMap<SoftBodyHandle, Box<SoftFemSystem>>,
    active: Vec<ActiveFem>,
    /// Slice of `active` belonging to each substep group.
    groups: Vec<Range<usize>>,
}

impl SoftFemSet {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_empty(&self) -> bool {
        self.active.is_empty()
    }

    /// The `active` range of a substep group.
    pub fn group(&self, group: usize) -> Range<usize> {
        self.groups
            .get(group)
            .cloned()
            .unwrap_or(self.active.len()..self.active.len())
    }

    /// Collects the island's awake FEM soft bodies (after `SoftConstraintsSet::assemble` laid out
    /// the particle slots) and updates their systems' step-constant state; the factorization runs
    /// in the parallel response stage ([`Self::compute_responses`]).
    pub fn assemble(
        &mut self,
        soft_constraints: &SoftConstraintsSet,
        soft_bodies: &SoftBodySet,
        num_groups: usize,
        group_dt: impl Fn(usize) -> Real,
    ) {
        self.active.clear();
        self.groups.clear();
        self.groups.resize(num_groups.max(1), 0..0);
        if soft_constraints.is_empty() {
            // Bodies whose soft body was removed would otherwise keep their system forever; the
            // sweep is only worth its cost once the map has grown well past what is in use.
            if self.systems.len() > 64 {
                self.systems.retain(|h, _| soft_bodies.get(*h).is_some());
            }
            return;
        }

        let mut group_starts = vec![0usize; self.groups.len() + 1];
        for awake in &soft_constraints.awake {
            // SAFETY: read-only access during assembly, same contract as the constraint assembly.
            let sb = unsafe { &*awake.ptr };
            if !awake.frozen && sb.uses_fem() {
                group_starts[awake.group as usize + 1] += 1;
            }
        }
        for k in 1..group_starts.len() {
            group_starts[k] += group_starts[k - 1];
        }
        for (g, range) in self.groups.iter_mut().enumerate() {
            *range = group_starts[g]..group_starts[g + 1];
        }

        // `awake` is already sorted by group, so a single pass keeps the group-major order.
        for awake in &soft_constraints.awake {
            let sb = unsafe { &*awake.ptr };
            if awake.frozen || !sb.uses_fem() {
                continue;
            }
            let slots =
                &soft_constraints.slots[awake.slot_start..awake.slot_start + awake.num_particles];
            let system = self.systems.entry(awake.handle).or_default();
            system.prepare(sb, slots);
            self.active.push(ActiveFem {
                body: awake.ptr,
                system: &mut **system as *mut SoftFemSystem,
                dt: group_dt(awake.group as usize),
            });
        }
    }

    /// The implicit elastic step of `active[index]` (one substep).
    /// # Safety
    /// The caller must guarantee exclusive access to this index (one claiming worker per index
    /// per stage) and that the body's solver-body slots are not written concurrently.
    pub unsafe fn predict(
        &self,
        index: usize,
        bodies: &mut SolverBodies,
        params: &SoftFemParameters,
    ) {
        let entry = &self.active[index];
        let system = unsafe { &mut *entry.system };
        system.predict(
            bodies,
            entry.dt,
            params.linear_tolerance,
            params.max_linear_iterations,
        );
    }

    /// Records `active[index]`'s velocities at the start of a solve pass.
    ///
    /// # Safety
    /// Same contract as [`Self::predict`].
    pub unsafe fn snapshot(&self, index: usize, bodies: &SolverBodies) {
        let entry = &self.active[index];
        let system = unsafe { &mut *entry.system };
        system.snapshot(bodies);
    }

    /// Propagates the impulses `active[index]` received during a solve pass through its
    /// elasticity.
    ///
    /// # Safety
    /// Same contract as [`Self::predict`].
    pub unsafe fn propagate(
        &self,
        index: usize,
        bodies: &mut SolverBodies,
        params: &SoftFemParameters,
    ) {
        let entry = &self.active[index];
        let system = unsafe { &mut *entry.system };
        system.propagate(
            bodies,
            params.linear_tolerance,
            params.max_linear_iterations,
        );
    }

    /// Writes the per-cell state kept across steps back to the soft bodies.
    ///
    /// # Safety
    /// Same contract as [`Self::predict`].
    pub unsafe fn writeback(&self, index: usize, step_dt: Real) {
        let entry = &self.active[index];
        let system = unsafe { &*entry.system };
        let sb = unsafe { &mut *entry.body };
        system.writeback(sb, step_dt);
    }

    pub fn num_active(&self) -> usize {
        self.active.len()
    }
}
