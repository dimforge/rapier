//! The broad-phase update pass: BVH leaf maintenance (refit-free re-inserts,
//! partial/full refits, deferred optimization scheduling) plus the pair
//! creation and stale-pair removal bookkeeping.

use super::{BroadPhaseBvh, BvhOptimizationStrategy};
use crate::alloc_prelude::*;
use crate::dynamics::{IntegrationParameters, RigidBodySet, RigidBodyType};
use crate::geometry::Collider;
use crate::geometry::{
    Aabb, BroadPhasePairEvent, ColliderChanges, ColliderHandle, ColliderPair, ColliderSet,
};
use crate::math::Real;
use parry::partitioning::BvhLeafUpdateStatus;

impl BroadPhaseBvh {
    /// Updates the broad-phase.
    ///
    /// The results are output through the `events` struct. The broad-phase algorithm is only
    /// required to generate new events (i.e. no need to re-send an `AddPair` event if it was already
    /// sent previously and no `RemovePair` happened since then). Sending redundant events is allowed
    /// but can result in a slight computational overhead.
    ///
    /// # Parameters
    /// - `params`: the integration parameters governing the simulation.
    /// - `colliders`: the set of colliders. Change detection with `collider.needs_broad_phase_update()`
    ///   can be relied on at this stage.
    /// - `modified_colliders`: colliders that are know to be modified since the last update.
    /// - `removed_colliders`: colliders that got removed since the last update. Any associated data
    ///   in the broad-phase should be removed by this call to `update`.
    /// - `events`: the broad-phase’s output. They indicate what collision pairs need to be created
    ///   and what pairs need to be removed. It is OK to create pairs for colliders that don’t
    ///   actually collide (though this can increase computational overhead in the narrow-phase)
    ///   but it is important not to indicate removal of a collision pair if the underlying colliders
    ///   are still touching or closer than `prediction_distance`.
    pub fn update(
        &mut self,
        params: &IntegrationParameters,
        colliders: &ColliderSet,
        bodies: &RigidBodySet,
        modified_colliders: &[ColliderHandle],
        removed_colliders: &[ColliderHandle],
        events: &mut Vec<BroadPhasePairEvent>,
    ) {
        self.frame_index = self.frame_index.overflowing_add(1).0;

        // If the previous update requested a deferred optimization but nothing ran it
        // (e.g. the broad-phase is driven without the physics pipeline), run it now.
        if self.deferred_optimize_pending {
            self.deferred_optimize_pending = false;
            super::run_bvh_optimize(&mut self.tree, &mut self.workspace);
        }

        // Removals must be handled first, in case another collider in
        // `modified_colliders` shares the same index.
        for handle in removed_colliders {
            self.tree.remove(handle.into_raw_parts().0);
        }

        let first_pass = self.tree.is_empty();

        self.updated_colliders.clear();
        self.curr_updated_leaves.clear();

        // Colliders updated through `set_aabb` since the last update already have an
        // up-to-date tree leaf, but must still be taken into account for change-flag
        // resolution and stale-pair detection.
        for handle in self.pending_set_aabb.drain(..) {
            if colliders.contains(handle) {
                self.updated_colliders.push(handle);
                self.curr_updated_leaves.push(handle.into_raw_parts().0);
            }
        }

        // Colliders whose pair-filter inputs may have flipped (re-parented / parent type changed) get
        // their leaf removed so the loop below re-inserts it as brand-new: the traversal then re-reports
        // every pair involving them, re-creating pairs the filter suppressed under the previous type. Skipped for leaves not in the tree yet.
        let mut forced_reinsertion = false;
        for handle in modified_colliders {
            if let Some(co) = colliders.get(*handle) {
                let leaf_index = handle.into_raw_parts().0;
                if co.is_enabled()
                    && co.changes.intersects(
                        ColliderChanges::PARENT | ColliderChanges::PARENT_EFFECTIVE_DOMINANCE,
                    )
                    && self.tree.leaf_node(leaf_index).is_some()
                {
                    self.tree.remove(leaf_index);
                    forced_reinsertion = true;
                }
            }
        }

        // The AABB (and margin) computation is the expensive part of the leaf-update
        // loop; precompute it in parallel and keep only the tree writes sequential.
        let mut update_scratch = core::mem::take(&mut self.update_scratch);
        update_scratch.clear();

        let compute_update = |modified: &ColliderHandle| -> Option<(ColliderHandle, Aabb, Real)> {
            let collider = colliders.get(*modified)?;
            // `PARENT_EFFECTIVE_DOMINANCE` is NF-only in general, but the forced
            // leaf-removal pre-pass above targets exactly these colliders: they MUST
            // be re-inserted here or their leaf would be lost.
            if !collider.is_enabled()
                || !(collider.changes.needs_broad_phase_update()
                    || collider
                        .changes
                        .contains(ColliderChanges::PARENT_EFFECTIVE_DOMINANCE))
            {
                return None;
            }

            let aabb = collider.compute_broad_phase_aabb(params, bodies);
            // A non-finite AABB would corrupt the tree (NaN breaks the partitioning
            // invariants); skip it and let the pipeline's end-of-step quarantine handle it.
            if !(aabb.mins.is_finite() && aabb.maxs.is_finite()) {
                return None;
            }
            let change_detection_skin = self.change_detection_skin(params, &aabb);

            Some((*modified, aabb, change_detection_skin))
        };

        #[cfg(feature = "parallel")]
        {
            // TODO(PERF): avoid the systematic Vec<Vec<_>> allocation?
            use rayon::prelude::*;
            let precomputed: Vec<Vec<_>> = modified_colliders
                .par_chunks(1024)
                .map(|chunk| chunk.iter().filter_map(compute_update).collect())
                .collect();
            update_scratch.extend(precomputed.into_iter().flatten());
        }
        #[cfg(not(feature = "parallel"))]
        update_scratch.extend(modified_colliders.iter().filter_map(compute_update));

        // Small change volumes relocate moved leaves via SAH re-insertion (O(log n) per leaf):
        // tree quality maintains itself and the periodic O(tree) optimizer never runs — what keeps
        // huge mostly-static scenes free of multi-ms spikes. Bulk volumes keep O(1) in-place updates.
        let leaf_count = self.tree.leaf_count() as usize;
        let use_reinsert =
            self.reinsert_leaf_updates && update_scratch.len() * 16 < leaf_count && !first_pass;

        // In-place leaf updates apply in parallel; the change-flag bookkeeping
        // below stays sequential (it's a cheap push per *changed* leaf).
        #[cfg(feature = "parallel")]
        let parallel_leaf_updates = !use_reinsert;
        #[cfg(feature = "parallel")]
        if parallel_leaf_updates {
            self.update_batch_scratch.clear();
            self.update_batch_scratch.extend(
                update_scratch
                    .iter()
                    .map(|(handle, aabb, skin)| (*aabb, handle.into_raw_parts().0, *skin)),
            );
            self.tree.insert_or_update_batch_partially_parallel(
                &self.update_batch_scratch,
                &mut self.update_batch_statuses,
            );

            for ((modified, _, _), status) in
                update_scratch.iter().zip(self.update_batch_statuses.iter())
            {
                let leaf_index = modified.into_raw_parts().0;
                match status {
                    BvhLeafUpdateStatus::Unchanged => {}
                    BvhLeafUpdateStatus::UpdatedInPlace | BvhLeafUpdateStatus::Inserted => {
                        if *status == BvhLeafUpdateStatus::UpdatedInPlace {
                            self.changes_since_optimize =
                                self.changes_since_optimize.saturating_add(1);
                        }
                        self.updated_colliders.push(*modified);
                        self.curr_updated_leaves.push(leaf_index);
                    }
                }
            }
        }

        #[cfg(feature = "parallel")]
        let sequential_leaf_updates = !parallel_leaf_updates;
        #[cfg(not(feature = "parallel"))]
        let sequential_leaf_updates = true;

        #[allow(clippy::collapsible_if)]
        if sequential_leaf_updates {
            // Two passes, mirroring `insert_or_update_batch_partially_parallel`: every
            // existing leaf is updated before any structural insertion, so an insertion's
            // SAH descent (and the rotations it applies) sees all of this step's AABBs
            // rather than a half-updated tree. The batch path cannot interleave the two
            // (its updates run concurrently), so this one must not either — a step that
            // mixes moved colliders with newly added ones would otherwise build a
            // different tree here than it does there.
            let mut deferred_inserts: Vec<usize> = Vec::new();
            for (i, (modified, aabb, change_detection_skin)) in update_scratch.iter().enumerate() {
                let leaf_index = modified.into_raw_parts().0;
                // `..._if_present` reports a missing leaf through the lookup it already
                // performs, so deferring insertions costs no extra probe.
                let status = if use_reinsert {
                    self.tree.reinsert_or_update_if_present(
                        *aabb,
                        leaf_index,
                        *change_detection_skin,
                    )
                } else {
                    self.tree
                        .update_partially_if_present(*aabb, leaf_index, *change_detection_skin)
                };
                let Some(status) = status else {
                    deferred_inserts.push(i);
                    continue;
                };
                match status {
                    // New AABB still inside the leaf's fattened AABB: tree untouched. No
                    // refit needed, no new pairs possible (traversal only visits changed
                    // leaves), no pair invalidation (deletion requires a changed leaf).
                    BvhLeafUpdateStatus::Unchanged => {}
                    BvhLeafUpdateStatus::UpdatedInPlace | BvhLeafUpdateStatus::Inserted => {
                        // Only in-place updates degrade the tree quality
                        // (re-insertions self-optimize, fresh insertions pick their
                        // spot by SAH descent).
                        if !use_reinsert && status == BvhLeafUpdateStatus::UpdatedInPlace {
                            self.changes_since_optimize =
                                self.changes_since_optimize.saturating_add(1);
                        }
                        self.updated_colliders.push(*modified);
                        self.curr_updated_leaves.push(leaf_index);
                    }
                }
            }

            // Pass 2: the structural insertions, in `update_scratch` order. Fresh
            // insertions pick their spot by SAH descent, so they never count toward the
            // optimizer's debt, and their status is always `Inserted`.
            for i in deferred_inserts {
                let (modified, aabb, change_detection_skin) = &update_scratch[i];
                let leaf_index = modified.into_raw_parts().0;
                let status =
                    self.tree
                        .insert_or_update_partially(*aabb, leaf_index, *change_detection_skin);
                debug_assert_eq!(status, BvhLeafUpdateStatus::Inserted);
                self.updated_colliders.push(*modified);
                self.curr_updated_leaves.push(leaf_index);
            }
        }

        self.update_scratch = update_scratch;

        // The incremental optimizer (and its O(tree) full refit) only runs when enough quality-degrading
        // changes accumulated: every frame under bulk volumes, every 8th for moderate, never for small
        // ones (SAH re-insertion accrues no debt — mostly-static scenes stay O(moving set)).
        let num_updated = self.updated_colliders.len();
        // Hysteresis for the re-insertion regime: `set_aabb` calls arriving before
        // the next `update` need the decision upfront, so it is based on this
        // step's change volume.
        self.reinsert_leaf_updates = num_updated * 16 < self.tree.leaf_count() as usize;
        self.changes_since_optimize = self
            .changes_since_optimize
            .saturating_add(removed_colliders.len() as u32);
        let run_optimizer = self.changes_since_optimize > 0
            && (num_updated * 16 >= leaf_count
                || (self.frame_index % 8 == 0
                    && self.changes_since_optimize as usize * 64 >= leaf_count))
            && self.optimization_strategy == BvhOptimizationStrategy::SubtreeOptimizer;

        // The optimizer is quality-only: defer it (plus its flag-preserving refit) to overlap the narrow
        // phase and solver; inline only when a full refit is needed anyway. Insertions need
        // NO full refit (`Bvh` maintains ancestor AABBs/counts; `refit_partial` resolves their flags — vital for huge mostly-static scenes); removals do (flag raw-merge into ancestors + orphaned wide nodes).
        let must_full_refit = first_pass || !removed_colliders.is_empty() || forced_reinsertion;
        let defer_optimize = run_optimizer && !must_full_refit;

        if run_optimizer {
            // The deferred pass is scheduled to run before the next update, so both
            // cases leave the tree freshly optimized.
            self.changes_since_optimize = 0;
        }

        if run_optimizer && !defer_optimize {
            self.tree.optimize_incremental(&mut self.workspace);
        }

        // NOTE: refit runs after optimization (skips internal-node updates there; allows the depth-first
        // cache-friendly reorder). Full refit is O(node count); with only leaf updates/insertions/relocations,
        // a partial refit visits just the ancestors of BOTH frames' changed leaves (flag clearing) — serial, so full is cheaper when most leaves changed.
        let partial_refit_too_expensive =
            (num_updated + self.prev_updated_leaves.len()) * 16 >= self.tree.leaf_count() as usize;
        let full_refit =
            must_full_refit || (run_optimizer && !defer_optimize) || partial_refit_too_expensive;
        if full_refit {
            #[cfg(feature = "parallel")]
            self.tree.refit_parallel(&mut self.workspace);
            #[cfg(not(feature = "parallel"))]
            self.tree.refit(&mut self.workspace);
        } else {
            self.tree
                .refit_partial(&self.prev_updated_leaves, &self.curr_updated_leaves);
        }
        core::mem::swap(&mut self.prev_updated_leaves, &mut self.curr_updated_leaves);

        self.deferred_optimize_pending |= defer_optimize;

        // The tree walk dominates the pair traversal, so walk it in parallel when there are
        // threads for it — parry pins the parallel walk to the sequential walk's exact pair
        // order — then pre-filter the reported pairs with read-only map probes so the
        // sequential tail only pays for genuinely new pairs.
        //
        // The probe is read-only in every build. The alternative (a sequential collector
        // refreshing each visited pair's timestamp, so stale-pair detection could skip it
        // with an integer compare) cannot run concurrently, and its map writes are part of
        // the serialized broad-phase state: keeping it would make the two builds' snapshots
        // differ even on an identical simulation.
        #[cfg(feature = "parallel")]
        let candidates = self
            .tree
            .traverse_bvtt_single_tree_parallel::<{ Self::CHANGE_DETECTION_ENABLED }>();

        #[cfg(not(feature = "parallel"))]
        let candidates = {
            // Reused across steps: the sequential walk reports through a closure, so
            // collecting it into the same shape the parallel walk returns costs nothing
            // beyond the (amortized) buffer.
            let mut candidates = core::mem::take(&mut self.candidates_scratch);
            candidates.clear();
            self.tree
                .traverse_bvtt_single_tree::<{ Self::CHANGE_DETECTION_ENABLED }>(
                    &mut self.workspace,
                    &mut |co1, co2| candidates.push((co1, co2)),
                );
            candidates
        };

        {
            let filter_new =
                |&(co1, co2): &(u32, u32)| -> Option<(ColliderHandle, ColliderHandle)> {
                    debug_assert_ne!(co1, co2);
                    let (mut collider1, mut handle1) = colliders.get_unknown_gen(co1)?;
                    let (mut collider2, mut handle2) = colliders.get_unknown_gen(co2)?;

                    if co1 > co2 {
                        core::mem::swap(&mut handle1, &mut handle2);
                        core::mem::swap(&mut collider1, &mut collider2);
                    }

                    if self.pairs.contains_key(&(handle1, handle2)) {
                        return None;
                    }

                    // Never create a pair the narrow phase's `ActiveCollisionTypes` filter
                    // would drop anyway (keeps big static environments from flooding the contact
                    // graph); later filter-input changes re-discover via the forced re-insertion pre-pass.
                    let rb_type = |co: &Collider| {
                        co.parent
                            .and_then(|p| bodies.get(p.handle))
                            .map(|rb| rb.body_type)
                            .unwrap_or(RigidBodyType::Fixed)
                    };
                    let rb_type1 = rb_type(collider1);
                    let rb_type2 = rb_type(collider2);
                    if !collider1
                        .flags
                        .active_collision_types
                        .test(rb_type1, rb_type2)
                        && !collider2
                            .flags
                            .active_collision_types
                            .test(rb_type1, rb_type2)
                    {
                        return None;
                    }

                    Some((handle1, handle2))
                };

            // rayon's ordered collect keeps the new pairs in traversal order, so the pair
            // set, adjacency lists and emitted events stay deterministic — and identical to
            // the sequential filter below.
            // TODO(perf): avoid systematic `Vec` allocation.
            #[cfg(feature = "parallel")]
            let new_pairs: Vec<(ColliderHandle, ColliderHandle)> = {
                use rayon::prelude::*;
                candidates
                    .par_chunks(512)
                    .flat_map_iter(|chunk| chunk.iter().filter_map(filter_new))
                    .collect()
            };
            #[cfg(not(feature = "parallel"))]
            let new_pairs: Vec<(ColliderHandle, ColliderHandle)> =
                candidates.iter().filter_map(filter_new).collect();

            for (handle1, handle2) in new_pairs {
                let prev = self.pairs.insert((handle1, handle2), self.frame_index);
                debug_assert!(prev.is_none());
                self.pair_adjacency
                    .ensure_element_exist(handle1.0, Vec::new())
                    .push(handle2);
                self.pair_adjacency
                    .ensure_element_exist(handle2.0, Vec::new())
                    .push(handle1);
                events.push(BroadPhasePairEvent::AddPair(ColliderPair::new(
                    handle1, handle2,
                )));
            }
        }

        #[cfg(not(feature = "parallel"))]
        {
            self.candidates_scratch = candidates;
        }

        /*
         *
         * Stale pairs handling (+ pairs removed events).
         *
         */
        // TODO(refactor): looks more complex than it could be.

        // Find outdated entries. A pair can only stop overlapping if one of its colliders
        // changed in the tree, so only pairs adjacent to updated/removed colliders are
        // checked. (A linear scan of the whole pair map used to be the fallback when most
        // colliders moved, but it was only worth it thanks to a per-pair timestamp
        // refreshed by the sequential pair collector — a map write the parallel collector
        // cannot do, and part of the serialized broad-phase state. One scan for every
        // build is both simpler and what the parallel build already did.)
        self.stale_pairs.clear();

        // Pairs involving a removed collider are always dropped (without emitting an
        // event, matching the behavior of the narrow-phase which handles removed
        // colliders on its own).
        for handle in removed_colliders {
            if let Some(mut others) = self.pair_adjacency.remove(handle.0, Vec::new()) {
                for other in others.drain(..) {
                    self.stale_pairs.push((*handle, other, false));
                }
            }
        }

        // Adjacency scan: pure read-only lookups, stale candidates flattened in `updated_colliders`
        // order (deterministic). No map probe: adjacency membership implies map membership, and the
        // sequential application tolerates duplicates via `pairs.remove`. No timestamp fast-path —
        // the tree's geometry is the only input, which is what lets it run concurrently.
        // Each pair sits in both its colliders' adjacency lists, so a pair whose two
        // sides both moved would be examined twice — the common case in a dense scene,
        // and the work the old timestamp fast-path used to hide. Visit those from the
        // lower-index side only. Pairs with one static side keep being visited from their
        // moving side. (Duplicates were harmless — the application loop dedups through
        // `pairs.remove` — so dropping them changes no result.)
        self.updated_mask.clear();
        self.updated_mask.resize(
            self.updated_colliders
                .iter()
                .map(|h| h.into_raw_parts().0 as usize + 1)
                .max()
                .unwrap_or(0),
            false,
        );
        for handle in &self.updated_colliders {
            self.updated_mask[handle.into_raw_parts().0 as usize] = true;
        }

        {
            let tree = &self.tree;
            let pair_adjacency = &self.pair_adjacency;
            let updated_mask = &self.updated_mask;
            let scan =
                |handle: &ColliderHandle, out: &mut Vec<(ColliderHandle, ColliderHandle, bool)>| {
                    let Some(others) = pair_adjacency.get(handle.0) else {
                        return;
                    };

                    // Fetched once for the whole adjacency list: the per-pair lookups
                    // are random accesses into the node array, and half of them are this
                    // same leaf. Both tests below are symmetric, so pulling one side out
                    // does not change the outcome.
                    let node_self = tree.leaf_node(handle.into_raw_parts().0);

                    let self_index = handle.into_raw_parts().0;

                    for other in others {
                        let other_index = other.into_raw_parts().0;
                        if self_index > other_index
                            && updated_mask
                                .get(other_index as usize)
                                .copied()
                                .unwrap_or(false)
                        {
                            // Both sides moved: this pair is visited from `other`.
                            continue;
                        }

                        let (h0, h1) = if self_index > other_index {
                            (*other, *handle)
                        } else {
                            (*handle, *other)
                        };

                        let Some(node0) = node_self else {
                            out.push((h0, h1, false));
                            continue;
                        };
                        let Some(node1) = tree.leaf_node(other_index) else {
                            out.push((h0, h1, false));
                            continue;
                        };

                        if (!Self::CHANGE_DETECTION_ENABLED
                            || node0.is_changed()
                            || node1.is_changed())
                            && !node0.intersects(node1)
                        {
                            out.push((h0, h1, true));
                        }
                    }
                };

            // rayon's ordered `par_extend` appends the chunks in order, so the flattened
            // result matches the sequential scan below element for element.
            #[cfg(feature = "parallel")]
            {
                use rayon::prelude::*;
                let mut stale_pairs = core::mem::take(&mut self.stale_pairs);
                stale_pairs.par_extend(self.updated_colliders.par_chunks(256).flat_map_iter(
                    |chunk| {
                        // TODO(perf): avoid these Vec allocations?
                        let mut out = Vec::new();
                        for handle in chunk {
                            scan(handle, &mut out);
                        }
                        out
                    },
                ));
                self.stale_pairs = stale_pairs;
            }

            #[cfg(not(feature = "parallel"))]
            {
                let mut stale_pairs = core::mem::take(&mut self.stale_pairs);
                for handle in &self.updated_colliders {
                    scan(handle, &mut stale_pairs);
                }
                self.stale_pairs = stale_pairs;
            }
        }

        // Canonical order: which detection variant ran (and its iteration order —
        // hash-map order for the full scan) must not leak into the `DeletePair`
        // sequence, which decides contact-graph edge-id reuse.
        self.stale_pairs
            .sort_unstable_by_key(|&(h0, h1, emit_event)| {
                let a = h0.into_raw_parts().0;
                let b = h1.into_raw_parts().0;
                (a.min(b), a.max(b), emit_event)
            });

        for i in 0..self.stale_pairs.len() {
            let (h0, h1, emit_event) = self.stale_pairs[i];
            let (h0, h1) = if h0.into_raw_parts().0 > h1.into_raw_parts().0 {
                (h1, h0)
            } else {
                (h0, h1)
            };

            // The `remove` check also deduplicates: the same pair can be pushed twice
            // if both its colliders changed this frame.
            if crate::utils::hashmap_remove(&mut self.pairs, &(h0, h1)).is_some() {
                for (ha, hb) in [(h0, h1), (h1, h0)] {
                    if let Some(others) = self.pair_adjacency.get_mut(ha.0) {
                        if let Some(pos) = others.iter().position(|h| *h == hb) {
                            others.swap_remove(pos);
                        }
                    }
                }

                if emit_event {
                    events.push(BroadPhasePairEvent::DeletePair(ColliderPair::new(h0, h1)));
                }
            }
        }
    }
}
