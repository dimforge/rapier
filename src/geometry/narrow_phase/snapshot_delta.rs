//! Snapshot deltas of the narrow phase.
//!
//! Pairs (contact and intersection graph edges) are stamped where they are written. The small
//! per-edge and per-body side arrays are written from many places, some in parallel through
//! raw pointers: their changes are found by comparison with a copy (see [`ShadowStamps`]).

use super::*;
use crate::data::coarena::CoarenaDelta;
use crate::data::delta::{ChangeTracker, DeltaError, ShadowStamps, VecDelta, DeltaComponent};
use crate::data::graph::GraphDelta;
use crate::dynamics::solver::solver_contact_graph::ContactRef;

/// Per-phase delta bookkeeping (never serialized).
#[derive(Clone, Default)]
pub(super) struct NarrowPhaseDeltaState {
    tracker: ChangeTracker,
    body_solver_color_masks: ShadowStamps<u128>,
    pair_solver_hints: ShadowStamps<u16>,
    force_event_pos: ShadowStamps<u32>,
    solver_buckets: ShadowStamps<Vec<ContactRef>>,
}

/// The content of a [`NarrowPhase`] delta.
#[doc(hidden)]
#[derive(Clone, Serialize, Deserialize)]
pub struct NarrowPhaseDelta {
    contact_graph: GraphDelta<ColliderHandle, ContactPair>,
    intersection_graph: GraphDelta<ColliderHandle, IntersectionPair>,
    graph_indices: CoarenaDelta<ColliderGraphIndices>,
    body_solver_color_masks: VecDelta<u128>,
    pair_solver_hints: VecDelta<u16>,
    solver_buckets: VecDelta<Vec<ContactRef>>,
    solver_graph_valid: bool,
    solver_graph_epoch: u32,
    solver_graph_mb_epoch: u32,
    force_event_pairs: Vec<u32>,
    force_event_pos: VecDelta<u32>,
    force_list_valid: bool,
}

impl DeltaComponent for NarrowPhase {
    type Payload = NarrowPhaseDelta;

    fn change_tracker_mut(&mut self) -> &mut ChangeTracker {
        &mut self.delta_state.tracker
    }

    fn change_tracker(&self) -> &ChangeTracker {
        &self.delta_state.tracker
    }

    fn reset_for_full_delta(&mut self) {
        // The query dispatcher is configuration, not state: keep the receiver's own.
        let query_dispatcher = self.query_dispatcher.clone();
        *self = Self::default();
        self.query_dispatcher = query_dispatcher;
    }

    fn set_stamp_tick(&mut self, now: u32) {
        self.contact_graph.graph.set_stamp_tick(now);
        self.intersection_graph.graph.set_stamp_tick(now);
        self.graph_indices.stamps.set_now(now);
        let state = &mut self.delta_state;
        if now == 0 {
            let tracker = core::mem::take(&mut state.tracker);
            *state = NarrowPhaseDeltaState {
                tracker,
                ..Default::default()
            };
        } else {
            state.body_solver_color_masks.stamps.set_now(now);
            state.pair_solver_hints.stamps.set_now(now);
            state.force_event_pos.stamps.set_now(now);
            state.solver_buckets.stamps.set_now(now);
        }
    }

    fn collect_delta(&mut self, since: Option<u32>) -> NarrowPhaseDelta {
        let state = &mut self.delta_state;
        NarrowPhaseDelta {
            contact_graph: self.contact_graph.graph.collect_delta(since),
            intersection_graph: self.intersection_graph.graph.collect_delta(since),
            graph_indices: self.graph_indices.collect_delta(since),
            body_solver_color_masks: state
                .body_solver_color_masks
                .collect(&self.body_solver_color_masks, since),
            pair_solver_hints: state
                .pair_solver_hints
                .collect(&self.pair_solver_hints, since),
            solver_buckets: state
                .solver_buckets
                .collect(self.solver_contact_graph.raw_buckets(), since),
            solver_graph_valid: self.solver_graph_valid,
            solver_graph_epoch: self.solver_graph_epoch,
            solver_graph_mb_epoch: self.solver_graph_mb_epoch,
            force_event_pairs: self.force_event_pairs.clone(),
            force_event_pos: state.force_event_pos.collect(&self.force_event_pos, since),
            force_list_valid: self.force_list_valid,
        }
    }

    fn apply_delta_payload(&mut self, delta: NarrowPhaseDelta) -> Result<(), DeltaError> {
        self.contact_graph.graph.apply_delta(
            delta.contact_graph,
            ColliderHandle::invalid,
            ContactPair::default,
        )?;
        self.intersection_graph.graph.apply_delta(
            delta.intersection_graph,
            ColliderHandle::invalid,
            IntersectionPair::new,
        )?;
        self.graph_indices.apply_delta(delta.graph_indices)?;
        let state = &mut self.delta_state;
        state.body_solver_color_masks.apply(
            delta.body_solver_color_masks,
            &mut self.body_solver_color_masks,
            || 0,
        )?;
        state
            .pair_solver_hints
            .apply(delta.pair_solver_hints, &mut self.pair_solver_hints, || 0)?;
        state.solver_buckets.apply(
            delta.solver_buckets,
            self.solver_contact_graph.raw_buckets_mut(),
            Vec::new,
        )?;
        self.solver_graph_valid = delta.solver_graph_valid;
        self.solver_graph_epoch = delta.solver_graph_epoch;
        self.solver_graph_mb_epoch = delta.solver_graph_mb_epoch;
        self.force_event_pairs = delta.force_event_pairs;
        state
            .force_event_pos
            .apply(delta.force_event_pos, &mut self.force_event_pos, || u32::MAX)?;
        self.force_list_valid = delta.force_list_valid;

        // Like after a deserialization: the patched pairs replay their solver contacts at the
        // next update, and the per-step caches are rebuilt.
        self.pending_solver_replay = true;
        self.body_qualify_info.clear();
        self.retired_pairs.clear();
        self.soft_self.clear();
        Ok(())
    }
}
