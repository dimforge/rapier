use crate::alloc_prelude::*;
use crate::data::Coarena;
use crate::data::change_stamps::ChangeStamps;
use crate::dynamics::IntegrationParameters;
use crate::geometry::{Aabb, ColliderHandle};
use crate::math::Real;
use parry::partitioning::{Bvh, BvhLeafUpdateStatus, BvhWorkspace};
use parry::utils::hashmap::HashMap;

mod update;

/// The broad-phase collision detector that quickly filters out distant object pairs.
///
/// The broad-phase is the "first pass" of collision detection. It uses a hierarchical
/// bounding volume tree (BVH) to quickly identify which collider pairs are close enough
/// to potentially collide, avoiding expensive narrow-phase checks for distant objects.
///
/// Think of it as a "spatial index" that answers: "Which objects are near each other?"
///
/// You typically don't interact with this directly - it's managed by [`PhysicsPipeline`](crate::pipeline::PhysicsPipeline).
/// However, you can use it to create a [`QueryPipeline`](crate::pipeline::QueryPipeline) for spatial queries.
#[derive(Default, Clone)]
pub struct BroadPhaseBvh {
    pub(crate) tree: Bvh,
    workspace: BvhWorkspace,
    pairs: HashMap<(ColliderHandle, ColliderHandle), u32>,
    /// For each collider, the other colliders it currently forms a pair with. Lets
    /// stale-pair detection examine only pairs adjacent to changed colliders instead of
    /// re-scanning the whole `pairs` map (a pair can only stop overlapping if one side changed).
    pair_adjacency: Coarena<Vec<ColliderHandle>>,
    /// Workspace buffer holding the colliders whose AABB was updated in the tree
    /// during the last `update` call.
    updated_colliders: Vec<ColliderHandle>,
    /// Workspace buffer holding the leaf pairs reported by the tree traversal. Only the
    /// sequential traversal needs it (it reports through a closure); the parallel one
    /// returns its own vector.
    #[cfg(not(feature = "parallel"))]
    candidates_workspace: Vec<(u32, u32)>,
    /// Workspace: per-collider "was updated this step" bit (collider arena index), so the
    /// stale-pair scan can visit a pair from one side only when both sides moved.
    updated_mask: Vec<bool>,
    /// Workspace buffer holding the stale pairs detected during `update`.
    ///
    /// The boolean indicates if a `DeletePair` event must be emitted for the pair.
    stale_pairs: Vec<(ColliderHandle, ColliderHandle, bool)>,
    /// Leaves updated at the previous `update` call — (a superset of) the leaves whose
    /// change flag the previous refit set; partial refitting needs it to clear those flags.
    ///
    /// Note that this needs to be serialized for determinism after snapshot restore.
    prev_updated_leaves: Vec<u32>,
    /// Leaves updated in the tree during the current `update` call.
    curr_updated_leaves: Vec<u32>,
    /// Colliders whose tree leaf was updated through [`Self::set_aabb`] since the last
    /// `update` call (e.g. by the physics pipeline at the end of the previous step).
    /// They count as changed colliders for the next `update` call.
    pending_set_aabb: Vec<ColliderHandle>,
    /// Quality-degrading tree changes (in-place leaf updates, removals) since the last
    /// incremental optimization; re-inserted leaves don't count (SAH re-insertion is
    /// self-optimizing). Periodic optimization is skipped while small relative to tree size.
    changes_since_optimize: u32,
    /// True when the previous `update` saw few leaves change: that regime relocates moved leaves via
    /// SAH re-insertion (tree quality without an O(tree) optimizer/refit pass); bulk regimes keep cheaper
    /// in-place updates + the periodic optimizer. One step of hysteresis: `set_aabb` runs between updates.
    reinsert_leaf_updates: bool,
    /// Workspace buffer for the precomputed leaf updates of [`Self::update`].
    update_workspace: Vec<(ColliderHandle, Aabb, Real)>,
    /// Workspace of the parallel leaf-update batches (the tree API takes raw
    /// leaf indices).
    #[cfg(feature = "parallel")]
    update_batch_workspace: Vec<(Aabb, u32, Real)>,
    #[cfg(feature = "parallel")]
    update_batch_statuses: Vec<BvhLeafUpdateStatus>,
    frame_index: u32,
    optimization_strategy: BvhOptimizationStrategy,
    /// If enabled, each tree leaf's change-detection margin adapts to the collider's size
    /// (12.5% of its smallest AABB extent, capped) instead of a fixed fraction of the
    /// length unit (default: `false`). Large shapes then keep their leaf and candidate
    /// pairs valid across bigger displacements — fewer tree updates and pair re-checks,
    /// at the cost of slightly fatter AABBs (more candidate pairs for the narrow-phase).
    pub adaptive_change_detection_margin: bool,
    /// True when the last `update` deferred its (quality-only) BVH optimization pass
    /// so the physics pipeline can run it concurrently with the narrow phase and
    /// solver; consumed by [`Self::take_deferred_optimize`].
    deferred_optimize_pending: bool,
    /// Per-leaf (collider index) change stamps for snapshot deltas.
    leaf_stamps: ChangeStamps,
    /// Per-collider stamps of the pairs it takes part in.
    pair_stamps: ChangeStamps,
    #[cfg(feature = "serde-serialize")]
    change_tracker: crate::data::delta::ChangeTracker,
}

// TODO: would be interesting to try out:
// "Fast Insertion-Based Optimization of Bounding Volume Hierarchies"
// by Bittner et al.
/// Selection of strategies to maintain through time the broad-phase BVH in shape that remains
/// efficient for collision-detection and scene queries.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Default, PartialEq, Eq, Copy, Clone)]
pub enum BvhOptimizationStrategy {
    /// Different sub-trees of the BVH will be optimized at each frame.
    #[default]
    SubtreeOptimizer,
    /// Disables incremental BVH optimization (discouraged).
    ///
    /// This should not be used except for debugging purpose.
    None,
}

/// Runs the deferred (quality-only) optimization pass on `tree`.
///
/// The parallel and sequential refits produce the same nodes (parry pins that in
/// `refit_parallel_matches_sequential`), so which one runs is a pure execution choice.
pub(crate) fn run_bvh_optimize(tree: &mut Bvh, workspace: &mut BvhWorkspace) {
    tree.optimize_incremental(workspace);
    // Flag-preserving refit: the change-detection flags were already resolved by
    // the partial refit that ran before this step's pair traversal, and the next
    // step's traversal must see them untouched.
    #[cfg(feature = "parallel")]
    tree.refit_without_resolve_parallel(workspace);
    #[cfg(not(feature = "parallel"))]
    tree.refit_without_resolve(workspace);
}

/// A pending (quality-only) BVH optimization pass, extracted from the broad-phase so
/// it can run on another thread while the rest of the step doesn't touch the tree.
///
/// Deferred in every build, so every consumer sees the same tree at the same point of
/// the step: with a spare worker the pass runs concurrently, otherwise it runs inline
/// at the join point (see `PhysicsPipeline::join_deferred_bvh_optimize`). Running it
/// eagerly instead would optimize the tree *before* this step's pair traversal rather
/// than after — a different tree, hence different pairs.
pub(crate) struct DeferredBvhOptimize {
    tree: Bvh,
    workspace: BvhWorkspace,
}

impl DeferredBvhOptimize {
    pub(crate) fn run(&mut self) {
        run_bvh_optimize(&mut self.tree, &mut self.workspace);
    }
}

/// Custom serialization of the broad phase: the tree is stored as its sorted leaves
/// (collider index, fattened AABB) and rebuilt on restore, and the pair adjacency is
/// rebuilt from the sorted pair map.
///
/// The tree's topology, slot free lists, and optimizer cursor are a rebuildable cache:
/// pair *discovery* depends only on the per-leaf fattened AABBs (traversal reports the
/// geometric overlap set of changed leaves), and every order the tree's internals used
/// to leak into the simulation is canonically sorted at the source (the `AddPair`
/// batch, the `DeletePair` batch, and this serialization). Restoring builds a
/// different but equally valid tree; only spatial-query iteration order (never part of
/// the determinism contract) can differ from the uninterrupted run.
#[cfg(feature = "serde-serialize")]
mod snapshot_serde {
    use super::*;
    use parry::partitioning::BvhBuildStrategy;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    /// The broad-phase fields snapshots and deltas carry verbatim (the tree and the pairs are
    /// stored separately).
    #[derive(Clone, Serialize, Deserialize)]
    pub(super) struct BvhState {
        prev_updated_leaves: Vec<u32>,
        pending_set_aabb: Vec<ColliderHandle>,
        changes_since_optimize: u32,
        reinsert_leaf_updates: bool,
        frame_index: u32,
        optimization_strategy: BvhOptimizationStrategy,
        adaptive_change_detection_margin: bool,
    }

    impl BvhState {
        pub(super) fn pending_set_aabb(&self) -> &[ColliderHandle] {
            &self.pending_set_aabb
        }
    }

    impl BroadPhaseBvh {
        /// The verbatim state. Destructures `self` exhaustively, so a field added later must be
        /// classified here.
        pub(super) fn verbatim_state(&self) -> BvhState {
            let BroadPhaseBvh {
                // Stored as sorted leaves and pairs.
                tree: _,
                pairs: _,
                prev_updated_leaves,
                pending_set_aabb,
                changes_since_optimize,
                reinsert_leaf_updates,
                frame_index,
                optimization_strategy,
                adaptive_change_detection_margin,
                // Derived from the pairs, quality-only, delta bookkeeping, or workspaces.
                pair_adjacency: _,
                deferred_optimize_pending: _,
                leaf_stamps: _,
                pair_stamps: _,
                change_tracker: _,
                workspace: _,
                updated_colliders: _,
                #[cfg(not(feature = "parallel"))]
                    candidates_workspace: _,
                updated_mask: _,
                stale_pairs: _,
                curr_updated_leaves: _,
                update_workspace: _,
                #[cfg(feature = "parallel")]
                    update_batch_workspace: _,
                #[cfg(feature = "parallel")]
                    update_batch_statuses: _,
            } = self;
            BvhState {
                prev_updated_leaves: prev_updated_leaves.clone(),
                pending_set_aabb: pending_set_aabb.clone(),
                changes_since_optimize: *changes_since_optimize,
                reinsert_leaf_updates: *reinsert_leaf_updates,
                frame_index: *frame_index,
                optimization_strategy: *optimization_strategy,
                adaptive_change_detection_margin: *adaptive_change_detection_margin,
            }
        }

        pub(super) fn set_verbatim_state(&mut self, state: BvhState) {
            let BvhState {
                prev_updated_leaves,
                pending_set_aabb,
                changes_since_optimize,
                reinsert_leaf_updates,
                frame_index,
                optimization_strategy,
                adaptive_change_detection_margin,
            } = state;
            self.prev_updated_leaves = prev_updated_leaves;
            self.pending_set_aabb = pending_set_aabb;
            self.changes_since_optimize = changes_since_optimize;
            self.reinsert_leaf_updates = reinsert_leaf_updates;
            self.frame_index = frame_index;
            self.optimization_strategy = optimization_strategy;
            self.adaptive_change_detection_margin = adaptive_change_detection_margin;
        }
    }

    #[derive(Serialize)]
    struct ReprSer<'a> {
        leaves: Vec<(u32, Aabb, bool)>,
        pairs: Vec<(&'a (ColliderHandle, ColliderHandle), &'a u32)>,
        state: BvhState,
    }

    #[derive(Deserialize)]
    struct ReprDe {
        leaves: Vec<(u32, Aabb, bool)>,
        pairs: Vec<((ColliderHandle, ColliderHandle), u32)>,
        state: BvhState,
    }

    impl Serialize for BroadPhaseBvh {
        fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            // Per leaf: index, exact fattened AABB, and the resolved change flag (it
            // gates the next traversal and stale-pair scan, so it is state).
            let mut leaves: Vec<(u32, Aabb, bool)> = self
                .tree
                .leaves(|_| true)
                .filter_map(|key| {
                    let node = self.tree.leaf_node(key)?;
                    Some((key, node.aabb(), node.is_changed()))
                })
                .collect();
            leaves.sort_unstable_by_key(|(index, ..)| *index);

            let mut pairs: Vec<_> = self.pairs.iter().collect();
            pairs.sort_unstable_by_key(|((a, b), _)| (a.into_raw_parts(), b.into_raw_parts()));

            ReprSer {
                leaves,
                pairs,
                state: self.verbatim_state(),
            }
            .serialize(serializer)
        }
    }

    /// Builds the tree holding `leaves` (index, fattened AABB, resolved change flag), with the
    /// leaves of `pending_set_aabb` carrying a pending change flag.
    pub(super) fn build_tree(leaves: &[(u32, Aabb, bool)], pending_set_aabb: &[ColliderHandle]) -> Bvh {
        // Leaves touched by an end-of-step `set_aabb` carry a pending change flag
        // (`pending_set_aabb` records exactly the escaped updates), which the next
        // refit resolves to changed. Pending is not reproducible through an
        // existing leaf, but a fresh insert creates its leaf pending with verbatim
        // bounds: build the tree without those leaves, insert them after.
        let pending: std::collections::HashSet<u32> =
            pending_set_aabb.iter().map(|h| h.0.into_raw_parts().0).collect();
        let built: Vec<(usize, Aabb)> = leaves
            .iter()
            .filter(|(index, ..)| !pending.contains(index))
            .map(|(index, aabb, _)| (*index as usize, *aabb))
            .collect();
        let mut tree = if built.len() > 2 {
            Bvh::from_iter(BvhBuildStrategy::Binned, built)
        } else {
            // Parry's bulk build assumes leaf ids 0 and 1 for one or two leaves: insert
            // them instead (pending), then resolve them to changed like a bulk build.
            let mut tree = Bvh::new();
            for (index, aabb) in &built {
                tree.insert(*aabb, *index as u32);
            }
            let ids: Vec<u32> = built.iter().map(|(index, _)| *index as u32).collect();
            tree.refit_partial(&[], &ids);
            tree
        };
        // A fresh build marks every leaf changed; resolve the flag away for the
        // leaves that were not flagged in the serialized state (`refit_partial`'s
        // previously-changed pass clears a non-pending flag).
        let unflagged: Vec<u32> = leaves
            .iter()
            .filter(|(index, _, changed)| !changed && !pending.contains(index))
            .map(|(index, ..)| *index)
            .collect();
        tree.refit_partial(&unflagged, &[]);
        for (index, aabb, _) in leaves.iter().filter(|(index, ..)| pending.contains(index)) {
            tree.insert(*aabb, *index);
        }
        tree
    }

    impl<'de> Deserialize<'de> for BroadPhaseBvh {
        fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            let repr = ReprDe::deserialize(deserializer)?;
            let tree = build_tree(&repr.leaves, &repr.state.pending_set_aabb);

            let mut pair_adjacency = Coarena::new();
            for ((h1, h2), _) in &repr.pairs {
                pair_adjacency
                    .ensure_element_exist(h1.0, Vec::new())
                    .push(*h2);
                pair_adjacency
                    .ensure_element_exist(h2.0, Vec::new())
                    .push(*h1);
            }

            let mut bvh = BroadPhaseBvh {
                tree,
                pairs: repr.pairs.into_iter().collect(),
                pair_adjacency,
                ..Default::default()
            };
            bvh.set_verbatim_state(repr.state);
            Ok(bvh)
        }
    }
}

/// Snapshot deltas of the broad phase: the stamped leaves (by collider index), the whole pair
/// list of every collider whose pairs changed, and the small per-update state.
#[cfg(feature = "serde-serialize")]
mod snapshot_delta {
    use super::*;
    use crate::data::delta::{ChangeTracker, DeltaError, DeltaComponent};
    use serde::{Deserialize, Serialize};

    /// The pairs a collider takes part in, with their insertion frames.
    type ColliderPairs = Vec<((ColliderHandle, ColliderHandle), u32)>;

    /// The content of a [`BroadPhaseBvh`] delta.
    #[doc(hidden)]
    #[derive(Clone, Serialize, Deserialize)]
    pub struct BroadPhaseBvhDelta {
        /// Whether this delta replaces every leaf and pair (no baseline).
        full: bool,
        /// Per stamped leaf: its fattened AABB and resolved change flag, `None` if absent.
        leaves: Vec<(u32, Option<(Aabb, bool)>)>,
        /// Per collider (index) whose pairs changed: every pair it takes part in.
        pairs: Vec<(u32, ColliderPairs)>,
        state: snapshot_serde::BvhState,
    }

    impl BroadPhaseBvh {
        /// The pairs collider `index` takes part in.
        fn pairs_of(&self, index: u32) -> ColliderPairs {
            let Some(generation) = self.pair_adjacency.get_gen(index).filter(|g| *g != u32::MAX)
            else {
                return Vec::new();
            };
            let handle = ColliderHandle::from_raw_parts(index, generation);
            let Some(others) = self.pair_adjacency.get_unknown_gen(index) else {
                return Vec::new();
            };
            others
                .iter()
                .filter_map(|other| {
                    [(handle, *other), (*other, handle)]
                        .into_iter()
                        .find_map(|key| self.pairs.get(&key).map(|v| (key, *v)))
                })
                .collect()
        }

        /// Drops every pair collider `index` takes part in.
        fn remove_pairs_of(&mut self, index: u32) {
            let generation = self.pair_adjacency.get_gen(index).filter(|g| *g != u32::MAX);
            let others = self
                .pair_adjacency
                .get_mut_unknown_gen(index)
                .map(core::mem::take)
                .unwrap_or_default();
            if let Some(generation) = generation {
                let handle = ColliderHandle::from_raw_parts(index, generation);
                for other in others {
                    let _ = crate::utils::hashmap_remove(&mut self.pairs, &(handle, other));
                    let _ = crate::utils::hashmap_remove(&mut self.pairs, &(other, handle));
                    if let Some(list) = self.pair_adjacency.get_mut(other.0) {
                        if let Some(pos) = list.iter().position(|h| *h == handle) {
                            list.swap_remove(pos);
                        }
                    }
                }
                let _ = self.pair_adjacency.remove(handle.0, Vec::new());
            }
            self.pair_stamps.mark(index as usize);
        }
    }

    impl DeltaComponent for BroadPhaseBvh {
        type Payload = BroadPhaseBvhDelta;

        fn change_tracker_mut(&mut self) -> &mut ChangeTracker {
            &mut self.change_tracker
        }

        fn change_tracker(&self) -> &ChangeTracker {
            &self.change_tracker
        }

        fn set_stamp_tick(&mut self, now: u32) {
            self.leaf_stamps.set_now(now);
            self.pair_stamps.set_now(now);
        }


        fn collect_delta(&mut self, since: Option<u32>) -> BroadPhaseBvhDelta {
            let leaf = |index: u32| {
                self.tree
                    .leaf_node(index)
                    .map(|node| (node.aabb(), node.is_changed()))
            };
            let (leaves, pairs) = match since {
                None => {
                    let mut leaves: Vec<_> = self
                        .tree
                        .leaves(|_| true)
                        .map(|index| (index, leaf(index)))
                        .collect();
                    leaves.sort_unstable_by_key(|(index, _)| *index);
                    let mut pairs: Vec<_> = self.pairs.iter().map(|(k, v)| (*k, *v)).collect();
                    pairs.sort_unstable_by_key(|((a, b), _)| (a.into_raw_parts(), b.into_raw_parts()));
                    (leaves, vec![(u32::MAX, pairs)])
                }
                Some(since) => (
                    self.leaf_stamps
                        .changed_since(since)
                        .map(|index| (index as u32, leaf(index as u32)))
                        .collect(),
                    self.pair_stamps
                        .changed_since(since)
                        .map(|index| (index as u32, self.pairs_of(index as u32)))
                        .collect(),
                ),
            };
            BroadPhaseBvhDelta {
                full: since.is_none(),
                leaves,
                pairs,
                state: self.verbatim_state(),
            }
        }

        fn apply_delta_payload(&mut self, delta: BroadPhaseBvhDelta) -> Result<(), DeltaError> {
            // Leaves: patch the current set, then rebuild the tree like a deserialization (the
            // current tree is just as valid when no leaf changed).
            let rebuild = delta.full
                || !delta.leaves.is_empty()
                || delta.state.pending_set_aabb() != &self.pending_set_aabb[..];
            if rebuild {
                let mut leaves: Vec<(u32, Aabb, bool)> = if delta.full {
                    Vec::new()
                } else {
                    self.tree
                        .leaves(|_| true)
                        .filter_map(|index| {
                            let node = self.tree.leaf_node(index)?;
                            Some((index, node.aabb(), node.is_changed()))
                        })
                        .collect()
                };
                leaves.sort_unstable_by_key(|(index, ..)| *index);
                for (index, leaf) in delta.leaves {
                    let pos = leaves.binary_search_by_key(&index, |(i, ..)| *i);
                    match (pos, leaf) {
                        (Ok(pos), Some((aabb, changed))) => leaves[pos] = (index, aabb, changed),
                        (Err(pos), Some((aabb, changed))) => leaves.insert(pos, (index, aabb, changed)),
                        (Ok(pos), None) => {
                            let _ = leaves.remove(pos);
                        }
                        (Err(_), None) => {}
                    }
                    self.leaf_stamps.mark(index as usize);
                }
                self.tree = snapshot_serde::build_tree(&leaves, delta.state.pending_set_aabb());
            }

            // Pairs: each listed collider's pairs replace its current ones.
            if delta.full {
                self.pairs.clear();
                self.pair_adjacency = Coarena::new();
            }
            for (index, pairs) in delta.pairs {
                if index != u32::MAX {
                    self.remove_pairs_of(index);
                }
                for ((h1, h2), value) in pairs {
                    if self.pairs.insert((h1, h2), value).is_none() {
                        self.pair_adjacency
                            .ensure_element_exist(h1.0, Vec::new())
                            .push(h2);
                        self.pair_adjacency
                            .ensure_element_exist(h2.0, Vec::new())
                            .push(h1);
                    }
                    self.pair_stamps.mark(h1.into_raw_parts().0 as usize);
                    self.pair_stamps.mark(h2.into_raw_parts().0 as usize);
                }
            }

            self.set_verbatim_state(delta.state);
            self.deferred_optimize_pending = false;
            Ok(())
        }
    }
}

impl BroadPhaseBvh {
    const CHANGE_DETECTION_ENABLED: bool = true;
    // Fraction of the length unit each tree leaf is fattened by (movement within the skin
    // leaves tree and pairs untouched; pairs appear up to `2 * factor` early). 0.04 keeps
    // broad-phase cost low without a measurable narrow-phase hit.
    const CHANGE_DETECTION_FACTOR: Real = 4.0e-2;
    /// Upper bound of the adaptive change-detection margin, as a fraction of the
    /// length unit (see [`Self::adaptive_change_detection_margin`]).
    const ADAPTIVE_CHANGE_DETECTION_CAP: Real = 0.25;

    /// Initializes a new empty broad-phase.
    pub fn new() -> Self {
        Self::default()
    }

    /// The change-detection margin (fat-AABB skin) for a leaf with the given AABB:
    /// a fixed fraction of the length unit, or, with
    /// [`Self::adaptive_change_detection_margin`], proportional to the shape's
    /// smallest extent and kept within [fixed margin, cap].
    fn change_detection_skin(&self, params: &IntegrationParameters, aabb: &Aabb) -> Real {
        if !Self::CHANGE_DETECTION_ENABLED {
            0.0
        } else if self.adaptive_change_detection_margin {
            let min_extent = aabb.extents().min_element();
            (min_extent * 0.125).clamp(
                Self::CHANGE_DETECTION_FACTOR * params.length_unit,
                Self::ADAPTIVE_CHANGE_DETECTION_CAP * params.length_unit,
            )
        } else {
            Self::CHANGE_DETECTION_FACTOR * params.length_unit
        }
    }

    /// Initializes a new empty broad-phase with the specified strategy for incremental
    /// BVH optimization.
    pub fn with_optimization_strategy(optimization_strategy: BvhOptimizationStrategy) -> Self {
        Self {
            optimization_strategy,
            ..Default::default()
        }
    }

    /// Extracts the deferred BVH optimization pass requested by the last [`Self::update`],
    /// if any, moving the tree out of the broad-phase. The tree MUST be handed back through
    /// [`Self::finish_deferred_optimize`] before anything else uses this broad-phase.
    pub(crate) fn take_deferred_optimize(&mut self) -> Option<DeferredBvhOptimize> {
        self.deferred_optimize_pending.then(|| {
            self.deferred_optimize_pending = false;
            DeferredBvhOptimize {
                tree: core::mem::replace(&mut self.tree, Bvh::new()),
                workspace: core::mem::take(&mut self.workspace),
            }
        })
    }

    /// Puts back the tree extracted by [`Self::take_deferred_optimize`].
    pub(crate) fn finish_deferred_optimize(&mut self, task: DeferredBvhOptimize) {
        self.tree = task.tree;
        self.workspace = task.workspace;
    }

    /// Sets the AABB associated to the given collider.
    ///
    /// The change is immediately applied and propagated through the underlying BVH;
    /// change detection accounts for it during the next broad-phase update.
    pub fn set_aabb(&mut self, params: &IntegrationParameters, handle: ColliderHandle, aabb: Aabb) {
        let change_detection_skin = self.change_detection_skin(params, &aabb);
        let leaf_index = handle.into_raw_parts().0;
        // Same regime split as the `update` loop: small change volumes relocate
        // moved leaves through self-optimizing SAH re-insertion, bulk volumes use
        // in-place updates (and count toward the periodic optimizer).
        let status = if self.reinsert_leaf_updates {
            self.tree.reinsert_or_update_with_change_detection(
                aabb,
                leaf_index,
                change_detection_skin,
            )
        } else {
            self.tree
                .insert_with_change_detection(aabb, leaf_index, change_detection_skin)
        };
        match status {
            // The new AABB stayed within the leaf's fattened AABB: the tree was left
            // untouched, so the next `update` has nothing to refit or re-check for
            // this collider.
            BvhLeafUpdateStatus::Unchanged => {}
            BvhLeafUpdateStatus::UpdatedInPlace | BvhLeafUpdateStatus::Inserted => {
                if !self.reinsert_leaf_updates && status == BvhLeafUpdateStatus::UpdatedInPlace {
                    self.changes_since_optimize = self.changes_since_optimize.saturating_add(1);
                }
                self.leaf_stamps.mark(leaf_index as usize);
                self.pending_set_aabb.push(handle);
            }
        }
    }
}

#[cfg(test)]
#[cfg(all(feature = "dim3", feature = "f32"))]
mod test {
    #[allow(unused_imports)]
    use crate::alloc_prelude::*;
    use crate::math::Vector;
    use crate::prelude::{
        CCDSolver, ColliderBuilder, ColliderSet, DefaultBroadPhase, ImpulseJointSet,
        IntegrationParameters, IslandManager, MultibodyJointSet, NarrowPhase, PhysicsPipeline,
        RigidBodyBuilder, RigidBodySet, SoftBodySet,
    };

    /// With the adaptive change-detection margin enabled, collisions must still be
    /// detected and resolved like with the fixed margin (the margin only affects how
    /// often tree leaves are updated, not which pairs eventually collide).
    #[test]
    fn adaptive_change_detection_margin_smoke() {
        let mut final_ys = [0.0; 2];

        for (i, adaptive) in [false, true].into_iter().enumerate() {
            let mut bodies = RigidBodySet::new();
            let mut colliders = ColliderSet::new();
            let mut impulse_joints = ImpulseJointSet::new();
            let mut multibody_joints = MultibodyJointSet::new();
            let mut soft_bodies = SoftBodySet::new();
            let mut pipeline = PhysicsPipeline::new();
            let mut islands = IslandManager::new();
            let mut broad_phase = DefaultBroadPhase::new();
            broad_phase.adaptive_change_detection_margin = adaptive;
            let mut narrow_phase = NarrowPhase::new();
            let mut ccd = CCDSolver::new();

            colliders.insert(ColliderBuilder::cuboid(10.0, 0.5, 10.0));
            let ball =
                bodies.insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 4.0, 0.0)));
            colliders.insert_with_parent(ColliderBuilder::ball(0.5), ball, &mut bodies);

            let params = IntegrationParameters::default();
            for _ in 0..200 {
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
                    &mut soft_bodies,
                    &mut ccd,
                    &(),
                    &(),
                );
            }

            final_ys[i] = bodies[ball].translation().y;
        }

        // Both must rest on the floor (0.5 half-thickness + 0.5 radius).
        for y in final_ys {
            assert!(
                (y - 1.0).abs() < 0.02,
                "ball did not rest on the floor: y = {y}"
            );
        }
    }
}

