//! Persistent solver-facing structures: per-pair solver hints, the per-color
//! solver contact graph and its incremental maintenance, and the
//! contact-force-event pair list.

use super::{NarrowPhase, PAIR_HINT_COUNT_MASK, PAIR_HINT_DYN_BIT};
use crate::alloc_prelude::*;
use crate::dynamics::solver::manifold_store::ManifoldStoreParts;
use crate::dynamics::solver::solver_contact_graph::{
    ContactRef, GENERIC_BUCKET, GraphPos, SolverContactGraph, bucket_id,
};
use crate::dynamics::{IslandManager, MultibodyJointSet, RigidBodySet};
use crate::geometry::{
    Collider, ColliderHandle, ColliderSet, ContactManifold, ContactManifoldData, ContactPair,
    InteractionGraph, PairEventStatus, SolverFlags,
};
use crate::math::Real;
#[cfg(feature = "alloc")]
use crate::pipeline::EventHandler;

impl NarrowPhase {
    /// Count-clears the solver hints of a just-asleep body's pairs so they stop
    /// reaching the solver (whole-island sleep: every touching partner sleeps too,
    /// so no pair of a sleeping body may stay solver-active).
    pub(crate) fn clear_asleep_pair_solver_hint_counts_of(&mut self, collider: ColliderHandle) {
        if let Some(gid) = self.graph_indices.get(collider.0) {
            if !InteractionGraph::<ColliderHandle, ContactPair>::is_graph_index_valid(
                gid.contact_graph_index,
            ) {
                return;
            }

            // Falling asleep drops these pairs from the selection and shifts the awake
            // set's solver-body indexing — neither goes through a full contact update,
            // so the persistent solver contact graph must be rebuilt next step.
            self.solver_graph_valid = false;

            let hints = &mut self.pair_solver_hints;
            let force_list = &mut self.force_event_pairs;
            let force_pos = &mut self.force_event_pos;
            for edge in self.contact_graph.graph.edges(gid.contact_graph_index) {
                if let Some(hint) = hints.get_mut(edge.id().index()) {
                    *hint &= PAIR_HINT_DYN_BIT;
                }
                // Count-cleared pairs leave the solver selection, so they
                // leave the force-event list too (no full contact update will
                // follow while both sides sleep).
                Self::force_event_remove(force_list, force_pos, edge.id().index() as u32);
            }
        }
    }

    /// Rebuilds [`Self::body_qualify_info`], the dense per-body table resolving
    /// `(is-dynamic-awake, solver-body index)` without fetching `RigidBody` structs. Low bit:
    /// `is_dynamic`; high 32: `active_set_id` (or frontier slot); `u64::MAX` = missing/fixed/kinematic.
    fn rebuild_body_qualify_info(&mut self, islands: &IslandManager, bodies: &RigidBodySet) {
        let max_body_index = islands
            .active_bodies()
            .map(|h| h.into_raw_parts().0 as usize)
            .max()
            .map(|m| m + 1)
            .unwrap_or(0);
        self.body_qualify_info.clear();
        self.body_qualify_info.resize(max_body_index, u64::MAX);

        // Rebuilt on every sleep/wake epoch bump (every step on churn-heavy scenes):
        // the O(active bodies) scatter is worth parallelizing (distinct handles map
        // to distinct slots, so the writes are disjoint).
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            let info_ptr = &crate::utils::SyncPtr(self.body_qualify_info.as_mut_ptr());
            for slice in islands.active_body_slices() {
                slice.par_chunks(1024).for_each(|chunk| {
                    for handle in chunk {
                        if let Some(rb) = bodies.get(*handle) {
                            // SAFETY: active handles are distinct, so the slots are disjoint.
                            unsafe {
                                *info_ptr.add(handle.into_raw_parts().0 as usize) =
                                    ((rb.ids.active_set_id as u64) << 32)
                                        | rb.body_type.is_dynamic() as u64;
                            }
                        }
                    }
                });
            }
        }

        #[cfg(not(feature = "parallel"))]
        for handle in islands.active_bodies() {
            if let Some(rb) = bodies.get(handle) {
                self.body_qualify_info[handle.into_raw_parts().0 as usize] =
                    ((rb.ids.active_set_id as u64) << 32) | rb.body_type.is_dynamic() as u64;
            }
        }
    }

    /// The manifold's solver-body indices if solver-active (at least one dynamic awake
    /// side), else `None` — the qualification rule shared by the full-rebuild and
    /// incremental maintenance paths (both resolve through the same table).
    #[inline]
    fn qualify_manifold_bqi(
        body_qualify_info: &[u64],
        manifold: &ContactManifold,
    ) -> Option<[u32; 2]> {
        let lookup = |h: Option<crate::dynamics::RigidBodyHandle>| -> u64 {
            h.and_then(|h| {
                body_qualify_info
                    .get(h.into_raw_parts().0 as usize)
                    .copied()
            })
            .unwrap_or(u64::MAX)
        };
        let entry1 = lookup(manifold.data.rigid_body1);
        let entry2 = lookup(manifold.data.rigid_body2);
        let (info1, info2) = (entry1 as u32, entry2 as u32);
        let solver_body_ids = [(entry1 >> 32) as u32, (entry2 >> 32) as u32];
        let dynamic_awake1 = info1 != u32::MAX && (info1 & 1) != 0;
        let dynamic_awake2 = info2 != u32::MAX && (info2 & 1) != 0;
        if dynamic_awake1 || dynamic_awake2 {
            Some(solver_body_ids)
        } else {
            None
        }
    }

    /// Maintains the persistent per-color [`SolverContactGraph`]:
    /// full rebuild when the awake set shifts (epoch bump),
    /// else only this step's fully-updated pairs reconcile — unchanged manifolds keep their slots.
    pub(crate) fn maintain_solver_contact_graph(
        &mut self,
        islands: &IslandManager,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        multibody_joints: &MultibodyJointSet,
    ) {
        let epoch = islands.active_set_epoch;
        let mb_epoch = multibody_joints.topology_epoch;
        let full_rebuild = !self.solver_graph_valid
            || epoch != self.solver_graph_epoch
            || mb_epoch != self.solver_graph_mb_epoch;

        // The body qualification table stays exact while the active set is unchanged: every
        // membership change bumps the island epoch or clears `solver_graph_valid` (both folded
        // into `full_rebuild`), so it rebuilds on exactly the solver graph's rebuild condition.
        if full_rebuild || self.body_qualify_info.is_empty() {
            self.rebuild_body_qualify_info(islands, bodies);
        }
        #[cfg(debug_assertions)]
        self.debug_validate_body_qualify_info(islands, bodies);
        let has_multibodies = multibody_joints.iter().next().is_some();

        let bqi = &self.body_qualify_info;
        let hints = &self.pair_solver_hints;
        let graph = &mut self.solver_contact_graph;
        let edges_ptr = self.contact_graph.graph.edges.as_mut_ptr();

        if full_rebuild {
            // Buckets are cleared, so every stored `graph_pos` is stale: reconcile in "fresh" mode
            // (insert the qualified, reset the rest, never trust a prior position). Large parallel scenes
            // rebuild with two rayon passes (count -> prefix-sum -> scatter): a serial walk on every epoch bump would dwarf the collection stage.
            let num_edges = self.contact_graph.graph.edges.len();

            #[cfg(feature = "parallel")]
            {
                use crate::dynamics::solver::solver_contact_graph::NUM_BUCKETS_WITH_GENERIC;
                use rayon::prelude::*;

                // Small enough that mid-size scenes (10-20k edges) split across every
                // worker, large enough that the per-chunk bucket-count vectors stay cheap.
                const CHUNK: usize = 1024;
                let num_chunks = num_edges.div_ceil(CHUNK);

                let edges_ptr_sync = crate::utils::SyncPtr(edges_ptr);
                let edges_ptr_sync = &edges_ptr_sync;

                // Pass 1: per-chunk per-bucket counts (read-only).
                let counts: Vec<Vec<u32>> = (0..num_chunks)
                    .into_par_iter()
                    .map(|c| {
                        let mut counts = alloc::vec![0u32; NUM_BUCKETS_WITH_GENERIC];
                        for edge in (c * CHUNK)..((c + 1) * CHUNK).min(num_edges) {
                            let hint = hints.get(edge).copied().unwrap_or(0);
                            // SAFETY: each edge read by exactly one chunk.
                            unsafe {
                                Self::for_each_desired_manifold(
                                    edges_ptr_sync.0,
                                    bqi,
                                    hint,
                                    edge as u32,
                                    has_multibodies,
                                    multibody_joints,
                                    |_, desired| {
                                        if let Some((bucket, _, _)) = desired {
                                            counts[bucket as usize] += 1;
                                        }
                                    },
                                )
                            };
                        }
                        counts
                    })
                    .collect();

                // Serial prefix sums: per-(chunk, bucket) write bases + final lens.
                let mut lens = alloc::vec![0u32; NUM_BUCKETS_WITH_GENERIC];
                let mut bases: Vec<Vec<u32>> = Vec::with_capacity(num_chunks);
                for c in &counts {
                    let mut base = alloc::vec![0u32; NUM_BUCKETS_WITH_GENERIC];
                    for ((base, len), count) in base.iter_mut().zip(lens.iter_mut()).zip(c.iter()) {
                        *base = *len;
                        *len += *count;
                    }
                    bases.push(base);
                }

                let bucket_ptrs: Vec<crate::utils::SyncPtr<ContactRef>> = graph
                    .resize_for_bulk_rebuild(&lens)
                    .into_iter()
                    .map(crate::utils::SyncPtr)
                    .collect();
                let bucket_ptrs = &bucket_ptrs;

                // Pass 2: scatter the refs at their exact offsets and stamp the manifolds
                // (graph_pos, solver-body ids, color); layout identical to the serial walk's.
                bases
                    .into_par_iter()
                    .enumerate()
                    .for_each(|(c, mut cursors)| {
                        for edge in (c * CHUNK)..((c + 1) * CHUNK).min(num_edges) {
                            let hint = hints.get(edge).copied().unwrap_or(0);
                            // SAFETY: each edge (and thus each manifold and each
                            //         precomputed bucket range) is written by exactly
                            //         one chunk.
                            unsafe {
                                let pair: *mut ContactPair =
                                    &mut (*edges_ptr_sync.0.add(edge)).weight;
                                let sm = (*pair).solver_manifolds_mut();
                                let sm_ptr = sm.as_mut_ptr();
                                Self::for_each_desired_manifold(
                                    edges_ptr_sync.0,
                                    bqi,
                                    hint,
                                    edge as u32,
                                    has_multibodies,
                                    multibody_joints,
                                    |ordinal, desired| {
                                        let mdata: *mut ContactManifoldData =
                                            &mut (*sm_ptr.add(ordinal as usize)).data;
                                        match desired {
                                            Some((bucket, solver_body_ids, color)) => {
                                                let local = cursors[bucket as usize];
                                                cursors[bucket as usize] += 1;
                                                *bucket_ptrs[bucket as usize].add(local as usize) =
                                                    ContactRef {
                                                        edge: edge as u32,
                                                        manifold: ordinal,
                                                    };
                                                (*mdata).graph_pos = GraphPos::new(bucket, local);
                                                (*mdata).solver_body_ids = solver_body_ids;
                                                (*mdata).solver_color = color;
                                            }
                                            None => {
                                                (*mdata).graph_pos = GraphPos::NONE;
                                            }
                                        }
                                    },
                                );
                            }
                        }
                    });
            }

            #[cfg(not(feature = "parallel"))]
            {
                graph.clear();
                for edge in 0..num_edges as u32 {
                    let hint = hints.get(edge as usize).copied().unwrap_or(0);
                    // SAFETY: single-threaded; each edge visited once; `reconcile_pair`
                    //         never aliases a live reference across a graph mutation.
                    unsafe {
                        Self::reconcile_pair(
                            graph,
                            edges_ptr,
                            bqi,
                            hint,
                            edge,
                            true,
                            has_multibodies,
                            multibody_joints,
                        )
                    };
                }
            }
            self.solver_graph_epoch = epoch;
            self.solver_graph_mb_epoch = mb_epoch;
            self.solver_graph_valid = true;
            // `solver_graph_dirty` is intentionally NOT cleared: the force-event reconcile
            // below still needs this step's dirty edges, and the next contact update
            // clears the list at its start anyway.
        } else {
            let dirty = core::mem::take(&mut self.solver_graph_dirty);
            for &edge in &dirty {
                let hint = hints.get(edge as usize).copied().unwrap_or(0);
                // SAFETY: as above; `dirty` is the deduplicated set of edges fully
                //         updated this step.
                unsafe {
                    Self::reconcile_pair(
                        graph,
                        edges_ptr,
                        bqi,
                        hint,
                        edge,
                        false,
                        has_multibodies,
                        multibody_joints,
                    )
                };
            }
            self.solver_graph_dirty = dirty;
        }

        // Reconcile the persistent force-event pair list. Maintained incrementally through every
        // membership transition (dirty list, inline sleep/removal drops, flagged user changes), so
        // it needs no epoch full rebuilds — only a deserialized/degenerate state triggers the scan.
        {
            let force_list_was_valid = self.force_list_valid;
            self.force_list_valid = true;
            let num_edges = self.contact_graph.graph.edges.len();
            let list = &mut self.force_event_pairs;
            let pos = &mut self.force_event_pos;
            let hints = &self.pair_solver_hints;
            let edges = &self.contact_graph.graph.edges;
            if !force_list_was_valid {
                list.clear();
                pos.clear();
                pos.resize(num_edges, u32::MAX);
                for edge in 0..num_edges as u32 {
                    let hint = hints.get(edge as usize).copied().unwrap_or(0);
                    Self::reconcile_force_event_pair(list, pos, colliders, edges, hint, edge);
                }
            } else {
                if pos.len() < num_edges {
                    pos.resize(num_edges, u32::MAX);
                }
                for &edge in self
                    .solver_graph_dirty
                    .iter()
                    .chain(self.force_event_flagged.iter())
                {
                    let hint = hints.get(edge as usize).copied().unwrap_or(0);
                    Self::reconcile_force_event_pair(list, pos, colliders, edges, hint, edge);
                }
            }
            self.force_event_flagged.clear();
        }

        #[cfg(debug_assertions)]
        self.debug_validate_solver_graph(full_rebuild, multibody_joints);
        #[cfg(debug_assertions)]
        self.debug_validate_force_event_pairs(colliders);
    }

    /// Debug-only: proves the incrementally-maintained force-event pair list
    /// (and its per-edge back-ref mirror) equals a from-scratch recomputation.
    #[cfg(debug_assertions)]
    fn debug_validate_force_event_pairs(&self, colliders: &ColliderSet) {
        let edges = &self.contact_graph.graph.edges;
        debug_assert_eq!(self.force_event_pos.len(), edges.len());
        let mut expected: Vec<u32> = Vec::new();
        for (edge, hint) in self.pair_solver_hints.iter().enumerate() {
            let selectable = hint & PAIR_HINT_DYN_BIT != 0 && hint & PAIR_HINT_COUNT_MASK != 0;
            if !selectable {
                continue;
            }
            let pair = &edges[edge].weight;
            let threshold = |h: ColliderHandle| {
                colliders
                    .get(h)
                    .map(|co| co.effective_contact_force_event_threshold())
                    .unwrap_or(Real::MAX)
            };
            if threshold(pair.collider1).min(threshold(pair.collider2)) < Real::MAX {
                expected.push(edge as u32);
            }
        }
        let mut actual = self.force_event_pairs.clone();
        for (i, &edge) in self.force_event_pairs.iter().enumerate() {
            debug_assert_eq!(
                self.force_event_pos[edge as usize], i as u32,
                "force-event back-ref desync"
            );
        }
        expected.sort_unstable();
        actual.sort_unstable();
        debug_assert_eq!(actual, expected, "force-event pair list diverged");
    }

    /// Removes `edge`'s membership from the force-event pair list (O(1) via the
    /// back-ref; no-op for non-members / out-of-range ids).
    fn force_event_remove(list: &mut Vec<u32>, pos: &mut [u32], edge: u32) {
        if let Some(cur) = pos.get(edge as usize).copied() {
            if cur != u32::MAX {
                list.swap_remove(cur as usize);
                if (cur as usize) < list.len() {
                    pos[list[cur as usize] as usize] = cur;
                }
                pos[edge as usize] = u32::MAX;
            }
        }
    }

    /// Reconciles one pair's membership in the persistent force-event pair list:
    /// a member is solver-selectable (hint gate) and has at least one collider
    /// with contact-force events enabled. O(1) via the per-edge back-reference.
    fn reconcile_force_event_pair(
        list: &mut Vec<u32>,
        pos: &mut [u32],
        colliders: &ColliderSet,
        edges: &[crate::data::graph::Edge<ContactPair>],
        hint: u16,
        edge: u32,
    ) {
        // A user-change-flagged edge id can go stale if a pair removal swapped edges after
        // the flagging; reconciling whatever pair lives there now is harmless (idempotent
        // true-up), and out-of-range ids are skipped.
        let Some(edge_ref) = edges.get(edge as usize) else {
            return;
        };
        let selectable = hint & PAIR_HINT_DYN_BIT != 0 && hint & PAIR_HINT_COUNT_MASK != 0;
        let want = selectable && {
            let pair = &edge_ref.weight;
            let threshold = |h: ColliderHandle| {
                colliders
                    .get(h)
                    .map(|co| co.effective_contact_force_event_threshold())
                    .unwrap_or(Real::MAX)
            };
            threshold(pair.collider1).min(threshold(pair.collider2)) < Real::MAX
        };
        let cur = pos[edge as usize];
        if want && cur == u32::MAX {
            pos[edge as usize] = list.len() as u32;
            list.push(edge);
        } else if !want && cur != u32::MAX {
            list.swap_remove(cur as usize);
            if (cur as usize) < list.len() {
                pos[list[cur as usize] as usize] = cur;
            }
            pos[edge as usize] = u32::MAX;
        }
    }

    /// The solver-active pairs with contact-force events enabled — the exact set
    /// the pipeline's post-solve force-event pass must inspect.
    /// Emits the contact force events of the solver-active pairs that request them, and
    /// updates each pair's above-threshold status (from which
    /// [`crate::geometry::ContactForceEvent::started`] is derived).
    ///
    /// The narrow-phase maintains the exact set of solver-active pairs with force events
    /// enabled, so scenes without them pay nothing here.
    #[cfg(feature = "alloc")]
    pub(crate) fn emit_contact_force_events(
        &mut self,
        dt: Real,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        events: &dyn EventHandler,
    ) {
        let inv_dt = crate::utils::inv(dt);
        for i in 0..self.force_event_pairs.len() {
            let edge = self.force_event_pairs[i] as usize;
            let pair = &mut self.contact_graph.graph.edges[edge].weight;
            let threshold = |h| {
                colliders
                    .get(h)
                    .map(|co: &Collider| co.effective_contact_force_event_threshold())
                    .unwrap_or(Real::MAX)
            };
            let threshold = threshold(pair.collider1).min(threshold(pair.collider2));

            if threshold < Real::MAX {
                let total_magnitude = pair.total_impulse_magnitude() * inv_dt;

                // NOTE: the strict inequality is important here, so we don’t
                //       trigger an event if the force is 0.0 and the threshold is 0.0.
                if total_magnitude > threshold {
                    // The handler runs before the status update, so the event can read
                    // the previous step's status to derive `started`.
                    events.handle_contact_force_event(dt, bodies, colliders, pair, total_magnitude);
                    pair.event_status
                        .insert(PairEventStatus::INITIAL_FORCE_THRESHOLD_EVENT_EMITTED);
                } else {
                    pair.event_status
                        .remove(PairEventStatus::INITIAL_FORCE_THRESHOLD_EVENT_EMITTED);
                }
            }
        }
    }

    /// Raw parts of the solver-facing `ManifoldStore` view: the contact graph's edge-array
    /// pointer and length, type-erased so they stay holdable across a later exclusive
    /// narrow-phase borrow.
    /// A store built from these is only valid while the graph is unmutated (this step's solver scope).
    pub(crate) fn manifold_store_parts(&mut self) -> ManifoldStoreParts {
        ManifoldStoreParts::new(
            self.contact_graph.graph.edges.as_mut_ptr(),
            self.contact_graph.graph.edges.len(),
        )
    }

    /// Walks one pair's solver manifolds, reporting each ordinal's *desired* membership —
    /// `Some((bucket, solver_body_ids, color))` or `None`. Mirrors [`Self::reconcile_pair`]'s
    /// qualification exactly (the shadow validator proves both against the same spec).
    /// Safety: same contract as `reconcile_pair`, but read-only (one caller per edge at a time).
    #[cfg(feature = "parallel")]
    #[allow(clippy::too_many_arguments)]
    unsafe fn for_each_desired_manifold(
        edges_ptr: *mut crate::data::graph::Edge<ContactPair>,
        bqi: &[u64],
        hint: u16,
        edge: u32,
        has_multibodies: bool,
        multibody_joints: &MultibodyJointSet,
        mut f: impl FnMut(u32, Option<(u16, [u32; 2], u8)>),
    ) {
        use crate::geometry::contact_pair::{SOLVER_COLOR_OVERFLOW, SOLVER_COLOR_UNCOLORED};

        let selectable = hint & PAIR_HINT_DYN_BIT != 0 && hint & PAIR_HINT_COUNT_MASK != 0;
        let pair: *const ContactPair = unsafe { &(*edges_ptr.add(edge as usize)).weight };
        let pair_color = unsafe { (*pair).solver_color };
        let sm = unsafe { (*pair).solver_manifolds() };

        let mut first_of_pair = true;
        for (ordinal, manifold) in sm.iter().enumerate() {
            let qualifies = selectable
                && manifold
                    .data
                    .solver_flags
                    .contains(SolverFlags::COMPUTE_IMPULSES)
                && manifold.data.num_active_contacts() != 0;
            let desired = if qualifies {
                Self::qualify_manifold_bqi(bqi, manifold).map(|solver_body_ids| {
                    let mut color = pair_color;
                    if !first_of_pair || color == SOLVER_COLOR_UNCOLORED {
                        color = SOLVER_COLOR_OVERFLOW;
                    }
                    first_of_pair = false;
                    let is_generic = has_multibodies
                        && (manifold
                            .data
                            .rigid_body1
                            .and_then(|h| multibody_joints.rigid_body_link(h))
                            .is_some()
                            || manifold
                                .data
                                .rigid_body2
                                .and_then(|h| multibody_joints.rigid_body_link(h))
                                .is_some());
                    let bucket = if is_generic {
                        GENERIC_BUCKET
                    } else {
                        bucket_id(color)
                    };
                    (bucket, solver_body_ids, color)
                })
            } else {
                None
            };
            f(ordinal as u32, desired);
        }
    }

    /// Reconciles one pair's solver manifolds with their color buckets, storing each
    /// solver-active manifold's [`GraphPos`] back on its [`ContactManifoldData`]. `full` =
    /// buckets just cleared, prior positions ignored (insert-only); else diff against current.
    /// Safety: `edges_ptr` = the contact graph's edge array, no other live borrow (single-threaded).
    #[allow(clippy::too_many_arguments)]
    unsafe fn reconcile_pair(
        graph: &mut SolverContactGraph,
        edges_ptr: *mut crate::data::graph::Edge<ContactPair>,
        bqi: &[u64],
        hint: u16,
        edge: u32,
        full: bool,
        has_multibodies: bool,
        multibody_joints: &MultibodyJointSet,
    ) {
        use crate::geometry::contact_pair::{SOLVER_COLOR_OVERFLOW, SOLVER_COLOR_UNCOLORED};

        // First gate of the solver selection. Honoring the hint (not the live
        // body/manifold state) keeps the graph exactly in step with the selection across
        // sleep/wake transitions, where the hint lags the raw awake-state by design.
        let selectable = hint & PAIR_HINT_DYN_BIT != 0 && hint & PAIR_HINT_COUNT_MASK != 0;

        let pair: *mut ContactPair = unsafe { &mut (*edges_ptr.add(edge as usize)).weight };
        let pair_color = unsafe { (*pair).solver_color };
        let (sm_ptr, num): (*mut ContactManifold, usize) = unsafe {
            let sm = (*pair).solver_manifolds_mut();
            (sm.as_mut_ptr(), sm.len())
        };

        let mut first_of_pair = true;
        for ordinal in 0..num {
            // Only transient references are taken, all dropped before any graph
            // mutation, so no live `&mut` aliases the raw edge pointer.
            let mdata: *mut ContactManifoldData = unsafe { &mut (*sm_ptr.add(ordinal)).data };
            let qualifies = selectable
                && unsafe {
                    (*mdata)
                        .solver_flags
                        .contains(SolverFlags::COMPUTE_IMPULSES)
                        && (*mdata).num_active_contacts() != 0
                };
            let desired = if qualifies {
                match Self::qualify_manifold_bqi(bqi, unsafe { &*sm_ptr.add(ordinal) }) {
                    Some(solver_body_ids) => {
                        let mut color = pair_color;
                        if !first_of_pair || color == SOLVER_COLOR_UNCOLORED {
                            color = SOLVER_COLOR_OVERFLOW;
                        }
                        first_of_pair = false;
                        // Multibody-involved manifolds are solved by the scalar generic
                        // path, so they live in the generic list, not the two-body buckets
                        // (they still consume the pair color so the selection is unaffected).
                        let is_generic = has_multibodies
                            && unsafe {
                                (*mdata)
                                    .rigid_body1
                                    .and_then(|h| multibody_joints.rigid_body_link(h))
                                    .is_some()
                                    || (*mdata)
                                        .rigid_body2
                                        .and_then(|h| multibody_joints.rigid_body_link(h))
                                        .is_some()
                            };
                        Some((color, solver_body_ids, is_generic))
                    }
                    None => None,
                }
            } else {
                None
            };

            let current = unsafe { (*mdata).graph_pos };
            let contact = ContactRef {
                edge,
                manifold: ordinal as u32,
            };
            match desired {
                Some((color, solver_body_ids, is_generic)) => {
                    unsafe {
                        (*mdata).solver_body_ids = solver_body_ids;
                        (*mdata).solver_color = color;
                    }
                    let target_bucket = if is_generic {
                        GENERIC_BUCKET
                    } else {
                        bucket_id(color)
                    };
                    if full || !current.is_some() || current.bucket() != target_bucket {
                        if !full && current.is_some() {
                            unsafe { Self::remove_and_fixup(graph, edges_ptr, current) };
                        }
                        let pos = if is_generic {
                            graph.insert_generic(contact)
                        } else {
                            graph.insert(color, contact)
                        };
                        unsafe { (*mdata).graph_pos = pos };
                    }
                    // Same bucket: the position is still valid, nothing to do.
                }
                None => {
                    if full {
                        unsafe { (*mdata).graph_pos = GraphPos::NONE };
                    } else if current.is_some() {
                        unsafe { Self::remove_and_fixup(graph, edges_ptr, current) };
                        unsafe { (*mdata).graph_pos = GraphPos::NONE };
                    }
                }
            }
        }
    }

    /// Swap-removes the entry at `pos` and repairs the moved entry's back-reference.
    /// Safety: same contract as [`Self::reconcile_pair`].
    #[inline]
    pub(super) unsafe fn remove_and_fixup(
        graph: &mut SolverContactGraph,
        edges_ptr: *mut crate::data::graph::Edge<ContactPair>,
        pos: GraphPos,
    ) {
        if let Some(moved) = graph.remove(pos) {
            let pair = unsafe { &mut (*edges_ptr.add(moved.edge as usize)).weight };
            pair.solver_manifolds_mut()[moved.manifold as usize]
                .data
                .graph_pos = pos;
        }
    }

    /// Debug-only: proves the (possibly cached) body qualification table equals
    /// a from-scratch rebuild — guards the epoch/validity reasoning that lets
    /// [`Self::maintain_solver_contact_graph`] skip the per-step rebuild.
    #[cfg(debug_assertions)]
    fn debug_validate_body_qualify_info(&mut self, islands: &IslandManager, bodies: &RigidBodySet) {
        let cached = self.body_qualify_info.clone();
        self.rebuild_body_qualify_info(islands, bodies);
        debug_assert!(
            cached == self.body_qualify_info,
            "stale cached body qualification table"
        );
    }

    /// Debug-only shadow validator: recomputes the exact solver-active manifold set and
    /// asserts the persistent graph holds precisely that set in the right buckets —
    /// proves the incremental maintenance stays exact.
    #[cfg(debug_assertions)]
    fn debug_validate_solver_graph(
        &self,
        full_rebuild: bool,
        multibody_joints: &MultibodyJointSet,
    ) {
        use crate::geometry::contact_pair::{SOLVER_COLOR_OVERFLOW, SOLVER_COLOR_UNCOLORED};

        let bqi = &self.body_qualify_info;
        let edges = &self.contact_graph.graph.edges;

        let has_multibodies = multibody_joints.iter().next().is_some();

        // Expected: what a from-scratch selection would emit, as
        // (edge << 32 | ordinal, color, is_generic).
        let mut expected: Vec<(u64, u8, bool)> = Vec::new();
        for (pair_id, hint) in self.pair_solver_hints.iter().enumerate() {
            let effective = if hint & PAIR_HINT_DYN_BIT != 0 {
                (hint & PAIR_HINT_COUNT_MASK) as usize
            } else {
                0
            };
            if effective == 0 {
                continue;
            }
            let pair = &edges[pair_id].weight;
            let pair_color = pair.solver_color;
            let mut first_of_pair = true;
            for (ordinal, manifold) in pair.solver_manifolds().iter().enumerate() {
                if !manifold
                    .data
                    .solver_flags
                    .contains(SolverFlags::COMPUTE_IMPULSES)
                    || manifold.data.num_active_contacts() == 0
                {
                    continue;
                }
                if Self::qualify_manifold_bqi(bqi, manifold).is_none() {
                    continue;
                }
                let mut color = pair_color;
                if !first_of_pair || color == SOLVER_COLOR_UNCOLORED {
                    color = SOLVER_COLOR_OVERFLOW;
                }
                first_of_pair = false;
                let is_generic = has_multibodies
                    && (manifold
                        .data
                        .rigid_body1
                        .and_then(|h| multibody_joints.rigid_body_link(h))
                        .is_some()
                        || manifold
                            .data
                            .rigid_body2
                            .and_then(|h| multibody_joints.rigid_body_link(h))
                            .is_some());
                expected.push((((pair_id as u64) << 32) | ordinal as u64, color, is_generic));
            }
        }

        // Actual: what the persistent graph currently holds.
        let mut actual: Vec<(u64, u8, bool)> = Vec::new();
        for (color, refs) in self.solver_contact_graph.buckets() {
            for c in refs {
                actual.push((((c.edge as u64) << 32) | c.manifold as u64, color, false));
            }
        }
        for c in self.solver_contact_graph.generic() {
            let pair = &edges[c.edge as usize].weight;
            let manifold = &pair.solver_manifolds()[c.manifold as usize];
            actual.push((
                ((c.edge as u64) << 32) | c.manifold as u64,
                manifold.data.solver_color,
                true,
            ));
        }

        // A full rebuild must lay every bucket out in ascending (edge, manifold) order.
        // Both rebuild variants produce exactly that — the serial walk visits edges in
        // order and appends, and the counting sort's prefix sums run over ascending
        // chunks — which is what makes them interchangeable, and therefore what makes a
        // `parallel` build and a non-`parallel` build agree on the solve order. The
        // membership comparison below sorts both sides, so it would not catch a
        // reordering. (The incremental path appends and swap-removes as edges qualify,
        // so its buckets are in insertion-history order: nothing to check there.)
        if full_rebuild {
            let ordered = |refs: &[ContactRef]| {
                refs.windows(2)
                    .all(|w| (w[0].edge, w[0].manifold) < (w[1].edge, w[1].manifold))
            };
            for (color, refs) in self.solver_contact_graph.buckets() {
                debug_assert!(
                    ordered(refs),
                    "solver contact graph bucket {color} is not in ascending (edge, manifold) \
                     order after a full rebuild: the two rebuild variants no longer agree on \
                     the layout, so the solve order now depends on the build"
                );
            }
            debug_assert!(
                ordered(self.solver_contact_graph.generic()),
                "the generic solver-contact list is not in ascending (edge, manifold) order \
                 after a full rebuild"
            );
        }

        expected.sort_unstable();
        actual.sort_unstable();
        debug_assert_eq!(
            actual.len(),
            expected.len(),
            "solver contact graph size {} != selection size {} (full_rebuild={full_rebuild})",
            actual.len(),
            expected.len()
        );
        debug_assert!(
            actual == expected,
            "solver contact graph diverged from the from-scratch selection (full_rebuild={full_rebuild})"
        );
    }

    /// The persistent, incrementally-maintained per-color solver
    /// contact graph, consumed directly by the single-threaded solver's
    /// constraint assembly (see [`Self::maintain_solver_contact_graph`]).
    pub(crate) fn solver_graph(&self) -> &SolverContactGraph {
        &self.solver_contact_graph
    }
}
