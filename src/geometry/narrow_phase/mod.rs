//! Narrow-phase collision detection: contact and intersection pair management
//! between colliders whose broad-phase AABBs overlap, plus the persistent
//! solver-facing bookkeeping maintained across steps.

mod contacts;
mod intersections;
mod pair_management;
mod pair_update;
mod queries;
mod solver_graph;
#[cfg(test)]
#[cfg(feature = "f32")]
#[cfg(feature = "dim3")]
mod test;

use crate::alloc_prelude::*;
use crate::data::Coarena;
use crate::dynamics::solver::solver_contact_graph::{
    GENERIC_BUCKET, SolverContactGraph, bucket_id,
};
use crate::dynamics::{IslandManager, RigidBodySet};
use crate::geometry::{
    ColliderGraphIndex, ColliderHandle, ColliderSet, ContactData, ContactManifoldData, ContactPair,
    InteractionGraph, IntersectionPair, SolverFlags,
};
use alloc::sync::Arc;
use parry::query::{DefaultQueryDispatcher, PersistentQueryDispatcher};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
struct ColliderGraphIndices {
    contact_graph_index: ColliderGraphIndex,
    intersection_graph_index: ColliderGraphIndex,
}

impl ColliderGraphIndices {
    fn invalid() -> Self {
        Self {
            contact_graph_index: InteractionGraph::<(), ()>::invalid_graph_index(),
            intersection_graph_index: InteractionGraph::<(), ()>::invalid_graph_index(),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum PairRemovalMode {
    FromContactGraph,
    FromIntersectionGraph,
    Auto,
}

/// Strong-wakes whichever of the two bodies is a sleeping dynamic body.
fn strong_wake_sleeping_side(
    islands: &mut IslandManager,
    bodies: &mut RigidBodySet,
    h1: Option<crate::dynamics::RigidBodyHandle>,
    h2: Option<crate::dynamics::RigidBodyHandle>,
) {
    for h in [h1, h2].into_iter().flatten() {
        let sleeping_dyn = bodies
            .get(h)
            .is_some_and(|rb| rb.is_dynamic() && rb.activation.sleeping);
        if sleeping_dyn {
            islands.wake_up(bodies, h, true);
        }
    }
}

/// Packs a coloring body descriptor `(arena index, is_fixed)` into a `u32` for the
/// deferred-coloring scratch list: `u32::MAX` = no body, else `(id << 1) | is_fixed`.
fn pack_color_body_info(info: Option<(u32, bool)>) -> u32 {
    match info {
        None => u32::MAX,
        Some((id, fixed)) => (id << 1) | fixed as u32,
    }
}

/// Inverse of [`pack_color_body_info`].
fn unpack_color_body_info(packed: u32) -> Option<(u32, bool)> {
    if packed == u32::MAX {
        None
    } else {
        Some((packed >> 1, packed & 1 != 0))
    }
}

/// Assigns a persistent solver graph color to a newly-active pair: first color used by
/// neither body. Dynamic/dynamic pairs search from color 0 up, pairs with a non-dynamic
/// body from 127 down; when no color is free, the pair takes the sequential overflow color.
fn assign_pair_solver_color(
    masks: &mut Vec<u128>,
    pair: &mut ContactPair,
    body1: Option<(u32, bool)>, // (arena index, is_fixed)
    body2: Option<(u32, bool)>,
) {
    use crate::geometry::contact_pair::{
        SOLVER_COLOR_OVERFLOW, SOLVER_COLOR_UNCOLORED, SOLVER_DYNAMIC_COLOR_COUNT,
    };

    if pair.solver_color != SOLVER_COLOR_UNCOLORED {
        return;
    }

    let conflicting1 = body1.filter(|(_, fixed)| !fixed).map(|(id, _)| id);
    let conflicting2 = body2.filter(|(_, fixed)| !fixed).map(|(id, _)| id);
    let max_id = conflicting1.max(conflicting2).map(|id| id as usize);

    if let Some(max_id) = max_id {
        if masks.len() <= max_id {
            masks.resize(max_id + 1, 0);
        }
    }

    let (color, bodies) = match (conflicting1, conflicting2) {
        (Some(i1), Some(i2)) => {
            // Dynamic-vs-dynamic: pack from the low colors, but never into the top band reserved
            // for dynamic-vs-fixed contacts (so those stay strictly last). If the low colors are
            // exhausted the pair overflows (solved sequentially) rather than encroaching.
            let mask = masks[i1 as usize] | masks[i2 as usize];
            let dynamic_free = !mask & ((1u128 << SOLVER_DYNAMIC_COLOR_COUNT) - 1);
            (dynamic_free.trailing_zeros(), [i1, i2])
        }
        (Some(i1), None) => {
            let mask = masks[i1 as usize];
            (127u32.wrapping_sub((!mask).leading_zeros()), [i1, u32::MAX])
        }
        (None, Some(i2)) => {
            let mask = masks[i2 as usize];
            (127u32.wrapping_sub((!mask).leading_zeros()), [i2, u32::MAX])
        }
        (None, None) => {
            // No conflicting body: this pair never reaches the parallel solver.
            pair.solver_color = SOLVER_COLOR_OVERFLOW;
            pair.solver_color_bodies = [u32::MAX; 2];
            return;
        }
    };

    if color >= 128 {
        // The color space of at least one body is saturated.
        pair.solver_color = SOLVER_COLOR_OVERFLOW;
        pair.solver_color_bodies = [u32::MAX; 2];
        return;
    }

    for id in bodies {
        if id != u32::MAX {
            masks[id as usize] |= 1 << color;
        }
    }

    pair.solver_color = color as u8;
    pair.solver_color_bodies = bodies;
}

/// Releases the solver graph color held by a contact pair (no-op if it holds none).
fn clear_pair_solver_color(masks: &mut [u128], pair: &mut ContactPair) {
    use crate::geometry::contact_pair::{SOLVER_COLOR_OVERFLOW, SOLVER_COLOR_UNCOLORED};

    if pair.solver_color < SOLVER_COLOR_OVERFLOW {
        for id in pair.solver_color_bodies {
            if id != u32::MAX {
                if let Some(mask) = masks.get_mut(id as usize) {
                    *mask &= !(1u128 << pair.solver_color);
                }
            }
        }
    }

    pair.solver_color = SOLVER_COLOR_UNCOLORED;
    pair.solver_color_bodies = [u32::MAX; 2];
}

/// Clears a filtered-out pair's contacts, reporting whether a solver manifold still held
/// a live solver-graph entry: clearing destroys the `graph_pos` back-references the
/// incremental maintenance needs, so `true` must force a full rebuild (`OUTCOME_CLEARED_IN_GRAPH`).
fn clear_filtered_pair(pair: &mut ContactPair) -> bool {
    let in_graph = pair
        .solver_manifolds()
        .iter()
        .any(|m| m.data.graph_pos.is_some());
    pair.clear();
    in_graph
}

/// Bit of a pair's solver hint: at least one body is a dynamic *awake* body (no dynamic
/// awake side means the pair never reaches the solver — the hint must predict solver
/// qualification exactly). Repaired by the pair update when a sleeping side wakes.
const PAIR_HINT_DYN_BIT: u16 = 1 << 15;
/// Mask of a pair's solver hint holding its qualified solver-manifold count.
const PAIR_HINT_COUNT_MASK: u16 = PAIR_HINT_DYN_BIT - 1;

/// Whether a *single-manifold* pair's bucket membership drifted from its stored
/// `graph_pos` — the event-driven dirty predicate (bucket entries move only on
/// begin/end-touch). Only exact for single-manifold pairs; others always reconcile.
fn single_manifold_bucket_drift(pair: &ContactPair, selectable: bool) -> bool {
    use crate::geometry::contact_pair::{SOLVER_COLOR_OVERFLOW, SOLVER_COLOR_UNCOLORED};

    let manifold = &pair.solver_manifolds()[0];
    let qualifies = selectable
        && manifold
            .data
            .solver_flags
            .contains(SolverFlags::COMPUTE_IMPULSES)
        && manifold.data.num_active_contacts() != 0;
    let pos = manifold.data.graph_pos;
    if !qualifies {
        return pos.is_some();
    }
    if !pos.is_some() {
        return true;
    }
    // Generic (multibody) membership only changes with the multibody topology
    // epoch, which forces a full rebuild — count/color drift doesn't move it.
    if pos.bucket() == GENERIC_BUCKET {
        return false;
    }
    let mut color = pair.solver_color;
    if color == SOLVER_COLOR_UNCOLORED {
        color = SOLVER_COLOR_OVERFLOW;
    }
    pos.bucket() != bucket_id(color)
}

/// The number of this pair's solver manifolds that the constraint solver must see
/// (impulses to compute and at least one active contact).
fn pair_qualified_manifold_count(pair: &ContactPair) -> u16 {
    let solver_manifolds = if pair.solver_clusters.is_empty() {
        &pair.manifolds
    } else {
        &pair.solver_clusters
    };

    let mut count: u16 = 0;
    for manifold in solver_manifolds {
        if manifold
            .data
            .solver_flags
            .contains(SolverFlags::COMPUTE_IMPULSES)
            && manifold.data.num_active_contacts() != 0
        {
            count = count.saturating_add(1);
        }
    }
    count.min(PAIR_HINT_COUNT_MASK)
}

/// Collects into `candidates` the sorted, deduplicated indices of the graph edges adjacent
/// to a collider needing a narrow-phase update. Seeding from `modified_colliders` + active
/// bodies' colliders is exhaustive (pipeline-moved colliders leave the set; body is active).
fn collect_pairs_to_update<E>(
    candidates: &mut Vec<u32>,
    graph_indices: &Coarena<ColliderGraphIndices>,
    graph: &crate::data::graph::Graph<ColliderHandle, E>,
    islands: &IslandManager,
    bodies: &RigidBodySet,
    colliders: &ColliderSet,
    modified_colliders: &[ColliderHandle],
    select_graph_id: impl Fn(&ColliderGraphIndices) -> ColliderGraphIndex,
) {
    candidates.clear();

    if graph.edges.is_empty() {
        return;
    }

    // When most bodies are awake, walking the graph adjacency (pointer-chasing) and
    // sorting costs more than the linear edge scan it replaces: visit every edge and
    // let the per-edge change-flags check skip the few unchanged ones.
    let num_active = islands.active_bodies().count();
    if num_active * 2 >= bodies.len() {
        candidates.extend(0..graph.edges.len() as u32);
        return;
    }

    let mut push_edges_of = |handle: ColliderHandle, require_change_flags: bool| {
        let Some(co) = colliders.get(handle) else {
            return;
        };
        if require_change_flags && !co.changes.needs_narrow_phase_update() {
            return;
        }
        let Some(gid) = graph_indices.get(handle.0) else {
            return;
        };
        for edge in graph.edges(select_graph_id(gid)) {
            candidates.push(edge.id().index() as u32);
        }
    };

    for handle in modified_colliders {
        push_edges_of(*handle, true);
    }

    // Active bodies' colliders may have moved this step without carrying any
    // change flag (internal motion doesn't go through the user-modification
    // tracking), so their pairs are always candidates.
    for body_handle in islands.active_bodies() {
        if let Some(rb) = bodies.get(body_handle) {
            for co_handle in rb.colliders() {
                push_edges_of(*co_handle, false);
            }
        }
    }

    // Sort + dedup: each edge visited exactly once (it can be pushed once per seeded
    // collider), and the deterministic edge-index order of a full graph scan is preserved.
    candidates.sort_unstable();
    candidates.dedup();
}

/// The narrow-phase collision detector that computes precise contact points between colliders.
///
/// After the broad-phase quickly filters out distant object pairs, the narrow-phase performs
/// detailed geometric computations to find exact:
/// - Contact points (where surfaces touch)
/// - Contact normals (which direction surfaces face)
/// - Penetration depths (how much objects overlap)
///
/// You typically don't interact with this directly - it's managed by [`PhysicsPipeline::step`](crate::pipeline::PhysicsPipeline::step).
/// However, you can access it to query contact information or intersection state between specific colliders.
///
/// **For spatial queries** (raycasts, shape casts), use [`QueryPipeline`](crate::pipeline::QueryPipeline) instead.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct NarrowPhase {
    #[cfg_attr(
        feature = "serde-serialize",
        serde(skip, default = "crate::geometry::default_persistent_query_dispatcher")
    )]
    query_dispatcher: Arc<dyn PersistentQueryDispatcher<ContactManifoldData, ContactData>>,
    contact_graph: InteractionGraph<ColliderHandle, ContactPair>,
    intersection_graph: InteractionGraph<ColliderHandle, IntersectionPair>,
    graph_indices: Coarena<ColliderGraphIndices>,
    /// Scratch buffer holding the edge indices of pairs to process during a step, so
    /// the per-step loops don’t have to iterate on the whole interaction graphs.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    update_candidates: Vec<u32>,
    /// Solver graph coloring masks: per rigid-body (arena index), the set of solver colors
    /// used by its active contact pairs. Maintained incrementally on contact start/stop,
    /// so the solver never recolors its constraint graph from scratch.
    #[cfg_attr(feature = "serde-serialize", serde(default))]
    body_solver_color_masks: Vec<u128>,
    /// Scratch: per-body packed qualification info (rigid-body arena index), rebuilt during
    /// solver-graph maintenance. `u64::MAX` = missing/fixed/kinematic-or-sleeping;
    /// else `(active_set_id << 32) | is_dynamic`.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    body_qualify_info: Vec<u64>,
    /// Scratch: per-body awake bit (arena index), rebuilt each narrow-phase update.
    /// Internal motion carries no change flags, so "the parent body is awake" is the
    /// narrow-phase's it-may-have-moved signal for pair updates.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    awake_body_mask: Vec<bool>,
    /// Scratch: begin/end-touch transitions recorded during the serial pair update,
    /// kept as a field so its capacity is reused across steps.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pair_transitions: Vec<pair_update::PairTransition>,
    /// Per-pair solver-qualification hints (contact-graph edge index): bit 15 = has a
    /// dynamic body, low bits = qualified solver-manifold count. Maintained incrementally
    /// (count-cleared on sleep, mirrored on removals) so selection never re-walks every pair.
    pair_solver_hints: Vec<u16>,
    /// Persistent per-color buckets of the solver-active contact manifolds,
    /// maintained by [`Self::maintain_solver_contact_graph`];
    /// the solver consumes them directly — no re-selection, re-qualification, or counting sort.
    solver_contact_graph: SolverContactGraph,
    /// Whether [`Self::solver_contact_graph`] currently reflects the live contact
    /// set. `false` forces a full rebuild on the next maintenance pass (the very first
    /// step, or after a change that invalidates the whole graph).
    solver_graph_valid: bool,
    /// The [`IslandManager::active_set_epoch`] the solver contact graph was last
    /// (re)built at. A mismatch means the awake set / solver-body indices shifted
    /// (sleep, wake, body add/remove), so the graph is fully rebuilt.
    solver_graph_epoch: u32,
    /// The [`MultibodyJointSet::topology_epoch`] the solver contact graph was last built at.
    /// A mismatch means bodies may have joined/left a multibody (manifolds can switch
    /// between color buckets and the generic list), forcing a full rebuild.
    solver_graph_mb_epoch: u32,
    /// Scratch: edge indices fully updated this step (`OUTCOME_FULL`) — possible bucket
    /// membership change. Consumed by the incremental maintenance in
    /// [`Self::maintain_solver_contact_graph`] and the force-event list reconciliation.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    solver_graph_dirty: Vec<u32>,
    /// Persistent list of solver-active pairs (edge indices) with contact-force events
    /// enabled — exactly what the post-solve force-event pass must inspect. Maintained
    /// incrementally with the solver graph, so no-force-events scenes pay nothing per step.
    force_event_pairs: Vec<u32>,
    /// Per-edge back-reference into [`Self::force_event_pairs`] (`u32::MAX` = not a member):
    /// O(1) membership reconciliation. Edge-index shifts (pair/collider removal) are covered
    /// by the full rebuild those removals already force via `solver_graph_valid`.
    force_event_pos: Vec<u32>,
    /// Scratch: edges flagged for force-event membership reconciliation because a collider
    /// was user-modified this step (an `ActiveEvents`/threshold flip has no change flag
    /// and need not trigger a contact update, so it would otherwise go unnoticed mid-epoch).
    force_event_flagged: Vec<u32>,
    /// Whether the force-event pair list is intact. It is maintained incrementally
    /// through every transition, so it does NOT need epoch full rebuilds; `false`
    /// (a degenerate state) triggers the from-scratch scan.
    force_list_valid: bool,
    /// Scratch: begin-touch pairs deferred for greedy coloring in canonical
    /// `(min, max body id)` order (discovery-order independent: ≈ Δ colors instead of ≈ 2Δ).
    /// Entries are `(edge id, packed body infos)`, see [`pack_color_body_info`].
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    solver_color_todo: Vec<(u32, u32, u32)>,
    /// Pool of retired [`ContactPair`]s, reused by [`Self::add_pair`] so
    /// pair-churn-heavy scenes (hundreds of broad-phase add/delete events per
    /// step) skip the buffer reallocation of freshly constructed pairs.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    retired_pairs: Vec<ContactPair>,
}

pub(crate) type ContactManifoldIndex = usize;

impl Default for NarrowPhase {
    fn default() -> Self {
        Self::new()
    }
}

impl NarrowPhase {
    /// Creates a new empty narrow-phase.
    pub fn new() -> Self {
        Self::with_query_dispatcher(DefaultQueryDispatcher)
    }

    /// Creates a new empty narrow-phase with a custom query dispatcher.
    pub fn with_query_dispatcher<D>(d: D) -> Self
    where
        D: 'static + PersistentQueryDispatcher<ContactManifoldData, ContactData>,
    {
        Self {
            query_dispatcher: Arc::new(d),
            contact_graph: InteractionGraph::new(),
            intersection_graph: InteractionGraph::new(),
            graph_indices: Coarena::new(),
            update_candidates: Vec::new(),
            pair_transitions: Vec::new(),
            retired_pairs: Vec::new(),
            body_solver_color_masks: Vec::new(),
            body_qualify_info: Vec::new(),
            awake_body_mask: Vec::new(),
            pair_solver_hints: Vec::new(),
            solver_contact_graph: SolverContactGraph::new(),
            solver_graph_valid: false,
            solver_graph_epoch: 0,
            solver_graph_mb_epoch: 0,
            solver_graph_dirty: Vec::new(),
            force_event_pairs: Vec::new(),
            force_event_pos: Vec::new(),
            force_event_flagged: Vec::new(),
            force_list_valid: false,
            solver_color_todo: Vec::new(),
        }
    }

    fn refresh_awake_body_mask(&mut self, islands: &IslandManager) {
        self.awake_body_mask.clear();
        let len = islands
            .active_bodies()
            .map(|h| h.into_raw_parts().0 as usize)
            .max()
            .map(|m| m + 1)
            .unwrap_or(0);
        self.awake_body_mask.resize(len, false);
        for handle in islands.active_bodies() {
            self.awake_body_mask[handle.into_raw_parts().0 as usize] = true;
        }
    }
}
