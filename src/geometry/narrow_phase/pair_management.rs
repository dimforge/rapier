//! Contact/intersection pair lifecycle: collider removal, user-change handling,
//! pair insertion/removal in the interaction graphs (with persistent
//! solver-structure mirroring), and broad-phase event registration.

use super::{
    ColliderGraphIndices, NarrowPhase, PairRemovalMode, assign_pair_solver_color,
    clear_pair_solver_color,
};
use crate::alloc_prelude::*;
use crate::dynamics::solver::solver_contact_graph::GraphPos;
use crate::dynamics::{IslandManager, RigidBodySet};
use crate::geometry::{
    BroadPhasePairEvent, ColliderChanges, ColliderGraphIndex, ColliderHandle, ColliderPair,
    ColliderSet, CollisionEvent, ContactManifoldData, ContactPair, InteractionGraph,
    IntersectionPair, PairEventStatus,
};
use crate::pipeline::{ActiveEvents, EventHandler};
use crate::prelude::CollisionEventFlags;
use parry::utils::hashmap::HashMap;

impl NarrowPhase {
    /// Maintain the narrow-phase internal state by taking collider removal into account.
    #[profiling::function]
    pub fn handle_user_changes(
        &mut self,
        mut islands: Option<&mut IslandManager>,
        modified_colliders: &[ColliderHandle],
        removed_colliders: &[ColliderHandle],
        colliders: &mut ColliderSet,
        bodies: &mut RigidBodySet,
        events: &dyn EventHandler,
    ) {
        // TODO: avoid these hash-maps.
        // They are necessary to handle the swap-remove done internally
        // by the contact/intersection graphs when a node is removed.
        let mut prox_id_remap = HashMap::default();
        let mut contact_id_remap = HashMap::default();

        for collider in removed_colliders {
            // NOTE: if the collider does not have any graph indices currently, there is nothing
            // to remove in the narrow-phase for this collider.
            if let Some(graph_idx) = self
                .graph_indices
                .remove(collider.0, ColliderGraphIndices::invalid())
            {
                let intersection_graph_id = prox_id_remap
                    .get(collider)
                    .copied()
                    .unwrap_or(graph_idx.intersection_graph_index);
                let contact_graph_id = contact_id_remap
                    .get(collider)
                    .copied()
                    .unwrap_or(graph_idx.contact_graph_index);

                self.remove_collider(
                    intersection_graph_id,
                    contact_graph_id,
                    islands.as_deref_mut(),
                    colliders,
                    bodies,
                    &mut prox_id_remap,
                    &mut contact_id_remap,
                    events,
                );
            }
        }

        self.handle_user_changes_on_colliders(
            islands,
            modified_colliders,
            colliders,
            bodies,
            events,
        );
    }

    #[profiling::function]
    pub(crate) fn remove_collider(
        &mut self,
        intersection_graph_id: ColliderGraphIndex,
        contact_graph_id: ColliderGraphIndex,
        mut islands: Option<&mut IslandManager>,
        colliders: &mut ColliderSet,
        bodies: &mut RigidBodySet,
        prox_id_remap: &mut HashMap<ColliderHandle, ColliderGraphIndex>,
        contact_id_remap: &mut HashMap<ColliderHandle, ColliderGraphIndex>,
        events: &dyn EventHandler,
    ) {
        // Wake up every body in contact with the deleted collider and generate Stopped collision events.
        if let Some(islands) = islands.as_deref_mut() {
            for (a, b, pair) in self.contact_graph.interactions_with(contact_graph_id) {
                if let Some(parent) = colliders.get(a).and_then(|c| c.parent.as_ref()) {
                    islands.wake_up(bodies, parent.handle, true)
                }

                if let Some(parent) = colliders.get(b).and_then(|c| c.parent.as_ref()) {
                    islands.wake_up(bodies, parent.handle, true)
                }

                if pair
                    .event_status
                    .contains(PairEventStatus::START_EVENT_EMITTED)
                {
                    events.handle_collision_event(
                        bodies,
                        colliders,
                        CollisionEvent::Stopped(a, b, CollisionEventFlags::REMOVED),
                        Some(pair),
                    );
                }
            }
        } else {
            // If there is no island, don’t wake-up bodies, but do send the Stopped collision event.
            for (a, b, pair) in self.contact_graph.interactions_with(contact_graph_id) {
                if pair
                    .event_status
                    .contains(PairEventStatus::START_EVENT_EMITTED)
                {
                    events.handle_collision_event(
                        bodies,
                        colliders,
                        CollisionEvent::Stopped(a, b, CollisionEventFlags::REMOVED),
                        Some(pair),
                    );
                }
            }
        }

        // Generate Stopped collision events for intersections.
        for (a, b, pair) in self
            .intersection_graph
            .interactions_with(intersection_graph_id)
        {
            if pair.start_event_emitted {
                events.handle_collision_event(
                    bodies,
                    colliders,
                    CollisionEvent::Stopped(
                        a,
                        b,
                        CollisionEventFlags::REMOVED | CollisionEventFlags::SENSOR,
                    ),
                    None,
                );
            }
        }

        // We have to manage the fact that one other collider will
        // have its graph index changed because of the node's swap-remove.
        if let Some(replacement) = self.intersection_graph.remove_node(intersection_graph_id) {
            if let Some(replacement) = self.graph_indices.get_mut(replacement.0) {
                replacement.intersection_graph_index = intersection_graph_id;
            } else {
                prox_id_remap.insert(replacement, intersection_graph_id);
                // I feel like this should never happen now that the narrow-phase is the one owning
                // the graph_indices. Let's put an unreachable in there and see if anybody still manages
                // to reach it. If nobody does, we will remove this.
                unreachable!();
            }
        }

        // Node removal swap-removes contact-graph edges, shifting the edge indices the
        // solver graph's `ContactRef`s and the force-event list are keyed by (neither is
        // mirrored through node removal), so both must be rebuilt from scratch next step.
        self.solver_graph_valid = false;
        self.force_list_valid = false;
        let pair_solver_hints = &mut self.pair_solver_hints;
        // Persistent islands can't be rebuilt lazily like the tables above:
        // unlink each removed edge and mirror each swap-remove exactly.
        let mut edges_len = self.contact_graph.graph.edges.len();
        let mut pi = islands.map(|i| &mut i.persistent);
        if let Some(replacement) = self
            .contact_graph
            .remove_node_with(contact_graph_id, &mut |e| {
                // Mirror the edges vec's swap_remove on the solver hints (on length
                // mismatch — deserialized state — drop them; they rebuild lazily).
                if e.index() < pair_solver_hints.len() {
                    pair_solver_hints.swap_remove(e.index());
                } else {
                    pair_solver_hints.clear();
                }

                edges_len -= 1;
                if let Some(pi) = pi.as_deref_mut() {
                    pi.unlink_contact(e.index() as u32);
                    pi.contact_edge_removed(e.index() as u32, edges_len as u32);
                }
            })
        {
            if let Some(replacement) = self.graph_indices.get_mut(replacement.0) {
                replacement.contact_graph_index = contact_graph_id;
            } else {
                contact_id_remap.insert(replacement, contact_graph_id);
                // I feel like this should never happen now that the narrow-phase is the one owning
                // the graph_indices. Let's put an unreachable in there and see if anybody still manages
                // to reach it. If nobody does, we will remove this.
                unreachable!();
            }
        }
    }

    #[profiling::function]
    pub(crate) fn handle_user_changes_on_colliders(
        &mut self,
        mut islands: Option<&mut IslandManager>,
        modified_colliders: &[ColliderHandle],
        colliders: &ColliderSet,
        bodies: &mut RigidBodySet,
        events: &dyn EventHandler,
    ) {
        let mut pairs_to_remove = vec![];

        for handle in modified_colliders {
            // NOTE: we use `get` because the collider may no longer
            //       exist if it has been removed.
            if let Some(co) = colliders.get(*handle) {
                // Any user modification can flip the contact-force-event config
                // (`ActiveEvents`/threshold have no change flag), so flag its pairs for
                // force-event membership reconciliation at the next graph maintenance.
                if let Some(gid) = self.graph_indices.get(handle.0) {
                    if InteractionGraph::<ColliderHandle, ContactPair>::is_graph_index_valid(
                        gid.contact_graph_index,
                    ) {
                        for edge in self.contact_graph.graph.edges(gid.contact_graph_index) {
                            self.force_event_flagged.push(edge.id().index() as u32);
                        }
                    }
                }

                if !co.changes.needs_narrow_phase_update() {
                    // No flag relevant to the narrow-phase is enabled for this collider.
                    continue;
                }

                if let Some(gid) = self.graph_indices.get(handle.0) {
                    // For each modified colliders, we need to wake-up the bodies it is in contact with
                    // so that the narrow-phase properly takes into account the change in, e.g.,
                    // collision groups. Waking up the modified collider's parent isn't enough because
                    // it could be a fixed or kinematic body which don't propagate the wake-up state.
                    if let Some(islands) = islands.as_deref_mut() {
                        if let Some(co_parent) = &co.parent {
                            islands.wake_up(bodies, co_parent.handle, true);
                        }

                        for inter in self
                            .contact_graph
                            .interactions_with(gid.contact_graph_index)
                        {
                            let other_handle = if *handle == inter.0 { inter.1 } else { inter.0 };
                            let other_parent = colliders
                                .get(other_handle)
                                .and_then(|co| co.parent.as_ref());

                            if let Some(other_parent) = other_parent {
                                islands.wake_up(bodies, other_parent.handle, true);
                            }
                        }
                    }

                    // A parent or effective-dominance change (re-parenting, body type
                    // change) invalidates the solver graph colors of this collider's
                    // pairs: release and re-assign them with the current bodies.
                    if co.changes.intersects(
                        ColliderChanges::PARENT | ColliderChanges::PARENT_EFFECTIVE_DOMINANCE,
                    ) {
                        let mut edges_to_recolor = alloc::vec::Vec::new();
                        for edge in self.contact_graph.graph.edges(gid.contact_graph_index) {
                            edges_to_recolor.push(edge.id().index());
                        }

                        for edge_id in edges_to_recolor {
                            let pair = &mut self.contact_graph.graph.edges[edge_id].weight;
                            clear_pair_solver_color(&mut self.body_solver_color_masks, pair);

                            let touching = pair.has_any_active_contact();
                            if touching {
                                let body_info = |co: ColliderHandle| {
                                    colliders
                                        .get(co)
                                        .and_then(|co| co.parent.as_ref())
                                        .map(|p| {
                                            let rb = &bodies[p.handle];
                                            (p.handle.into_raw_parts().0, rb.is_fixed())
                                        })
                                };
                                let info1 = body_info(pair.collider1);
                                let info2 = body_info(pair.collider2);
                                assign_pair_solver_color(
                                    &mut self.body_solver_color_masks,
                                    pair,
                                    info1,
                                    info2,
                                );
                            }

                            // Persistent islands: after re-parenting or a body type change,
                            // a link recorded with the old endpoints may no longer describe
                            // connectivity (a link to a fixed body doesn't connect). Refresh it.
                            if let Some(islands) = islands.as_deref_mut() {
                                let parent = |co: ColliderHandle| {
                                    colliders.get(co).and_then(|c| c.parent.map(|p| p.handle))
                                };
                                let pair = &self.contact_graph.graph.edges[edge_id].weight;
                                let (co1, co2) = (pair.collider1, pair.collider2);
                                islands.persistent.unlink_contact(edge_id as u32);
                                if touching {
                                    islands.persistent.link_contact(
                                        bodies,
                                        edge_id as u32,
                                        parent(co1),
                                        parent(co2),
                                    );
                                }
                            }
                        }
                    }

                    // For each collider which had their sensor status modified, we need
                    // to transfer their contact/intersection graph edges to the intersection/contact graph.
                    // To achieve this we will remove the relevant contact/intersection pairs form the
                    // contact/intersection graphs, and then add them into the other graph.
                    if co.changes.intersects(ColliderChanges::TYPE) {
                        if co.is_sensor() {
                            // Find the contact pairs for this collider and
                            // push them to `pairs_to_remove`.
                            for inter in self
                                .contact_graph
                                .interactions_with(gid.contact_graph_index)
                            {
                                pairs_to_remove.push((
                                    ColliderPair::new(inter.0, inter.1),
                                    PairRemovalMode::FromContactGraph,
                                ));
                            }
                        } else {
                            // Find the contact pairs for this collider and
                            // push them to `pairs_to_remove` if both involved
                            // colliders are not sensors.
                            for inter in self
                                .intersection_graph
                                .interactions_with(gid.intersection_graph_index)
                                .filter(|(h1, h2, _)| {
                                    !colliders[*h1].is_sensor() && !colliders[*h2].is_sensor()
                                })
                            {
                                pairs_to_remove.push((
                                    ColliderPair::new(inter.0, inter.1),
                                    PairRemovalMode::FromIntersectionGraph,
                                ));
                            }
                        }
                    }

                    // NOTE: if a collider only changed parent, we don’t need to remove it from any
                    //       of the graphs as re-parenting doesn’t change the sensor status of a
                    //       collider. If needed, their collision/intersection data will be
                    //       updated/removed automatically in the contact or intersection update
                    //       functions.
                }
            }
        }

        // Remove the pair from the relevant graph.
        for pair in &pairs_to_remove {
            self.remove_pair(
                islands.as_deref_mut(),
                colliders,
                bodies,
                &pair.0,
                events,
                pair.1,
            );
        }

        // Add the removed pair to the relevant graph.
        for pair in pairs_to_remove {
            self.add_pair(colliders, &pair.0);
        }
    }

    #[profiling::function]
    fn remove_pair(
        &mut self,
        mut islands: Option<&mut IslandManager>,
        colliders: &ColliderSet,
        bodies: &mut RigidBodySet,
        pair: &ColliderPair,
        events: &dyn EventHandler,
        mode: PairRemovalMode,
    ) {
        if let (Some(co1), Some(co2)) =
            (colliders.get(pair.collider1), colliders.get(pair.collider2))
        {
            // TODO: could we just unwrap here?
            // Don't we have the guarantee that we will get a `AddPair` before a `DeletePair`?
            if let (Some(gid1), Some(gid2)) = (
                self.graph_indices.get(pair.collider1.0),
                self.graph_indices.get(pair.collider2.0),
            ) {
                if mode == PairRemovalMode::FromIntersectionGraph
                    || (mode == PairRemovalMode::Auto && (co1.is_sensor() || co2.is_sensor()))
                {
                    let intersection = self
                        .intersection_graph
                        .remove_edge(gid1.intersection_graph_index, gid2.intersection_graph_index);

                    // Emit an intersection lost event if we had an intersection before removing the edge.
                    if let Some(mut intersection) = intersection {
                        if intersection.intersecting
                            && (co1.flags.active_events | co2.flags.active_events)
                                .contains(ActiveEvents::COLLISION_EVENTS)
                        {
                            intersection.emit_stop_event(
                                bodies,
                                colliders,
                                pair.collider1,
                                pair.collider2,
                                events,
                            )
                        }
                    }
                } else {
                    // O(1) maintenance of the persistent solver structures through the edges vec's
                    // swap-remove (with stored-position fixup) — a global rebuild
                    // is O(all pairs). If a rebuild is pending, `ContactRef`/`graph_pos` are garbage: skip it.
                    let solver_graph_valid = self.solver_graph_valid;
                    let graph = &mut self.solver_contact_graph;
                    let force_list = &mut self.force_event_pairs;
                    let force_pos = &mut self.force_event_pos;
                    let pair_solver_hints = &mut self.pair_solver_hints;
                    let num_edges = self.contact_graph.graph.edges.len();
                    let edges_ptr = self.contact_graph.graph.edges.as_mut_ptr();
                    let mut removed_index = usize::MAX;
                    let mut consistent = true;
                    let contact_pair = self.contact_graph.remove_edge_with(
                        gid1.contact_graph_index,
                        gid2.contact_graph_index,
                        &mut |e| {
                            removed_index = e.index();
                            // Mirror the edges vec's swap_remove on the solver hints
                            // (on length mismatch — deserialized state — drop them
                            // and fall back to a full solver-graph rebuild).
                            if e.index() < pair_solver_hints.len() {
                                pair_solver_hints.swap_remove(e.index());
                            } else {
                                pair_solver_hints.clear();
                                consistent = false;
                            }

                            if e.index() >= num_edges {
                                consistent = false;
                                return;
                            }

                            // Pull the removed pair's manifolds out of the buckets. Raw ordinal
                            // access: the fixup may rewrite another manifold of this same pair via
                            // `edges_ptr`. SAFETY: runs before the swap_remove (indices live); single-threaded.
                            if solver_graph_valid {
                                unsafe {
                                    let pair: *mut ContactPair =
                                        &mut (*edges_ptr.add(e.index())).weight;
                                    let sm = (*pair).solver_manifolds_mut();
                                    let (sm_ptr, num) = (sm.as_mut_ptr(), sm.len());
                                    for ordinal in 0..num {
                                        let mdata: *mut ContactManifoldData =
                                            &mut (*sm_ptr.add(ordinal)).data;
                                        let pos = (*mdata).graph_pos;
                                        if pos.is_some() {
                                            Self::remove_and_fixup(graph, edges_ptr, pos);
                                            (*mdata).graph_pos = GraphPos::NONE;
                                        }
                                    }
                                }
                            }

                            // Force-event list: drop the removed pair's membership, then
                            // mirror the swap-remove on the back-ref array (grown at pair
                            // add so it matches the edges vec; anything else = deserialized).
                            if force_pos.len() == num_edges {
                                let cur = force_pos[e.index()];
                                if cur != u32::MAX {
                                    force_list.swap_remove(cur as usize);
                                    if (cur as usize) < force_list.len() {
                                        force_pos[force_list[cur as usize] as usize] = cur;
                                    }
                                    force_pos[e.index()] = u32::MAX;
                                }
                                force_pos.swap_remove(e.index());
                            } else {
                                force_pos.clear();
                                force_list.clear();
                                consistent = false;
                            }
                        },
                    );

                    if !consistent {
                        // Deserialized/degenerate bookkeeping: rebuild from scratch.
                        self.solver_graph_valid = false;
                        self.force_list_valid = false;
                    } else if removed_index < self.contact_graph.graph.edges.len() {
                        // A pair was swap-moved into the removed slot: rewrite the
                        // edge index its persistent entries are keyed by.
                        let new_edge = removed_index as u32;
                        let moved = &mut self.contact_graph.graph.edges[removed_index].weight;
                        let graph = &mut self.solver_contact_graph;
                        // Same rebuild-pending gate as the removal loop above:
                        // stale `graph_pos` back-refs must not drive bucket writes.
                        if solver_graph_valid {
                            for m in moved.solver_manifolds_mut() {
                                if m.data.graph_pos.is_some() {
                                    graph.rewrite_edge(m.data.graph_pos, new_edge);
                                }
                            }
                        }
                        let fpos = self
                            .force_event_pos
                            .get(removed_index)
                            .copied()
                            .unwrap_or(u32::MAX);
                        if fpos != u32::MAX {
                            self.force_event_pairs[fpos as usize] = new_edge;
                        }
                    }

                    // Persistent islands: unlink the removed pair (if it was
                    // touching) and mirror the edges-vec swap-remove on the
                    // link-location table.
                    if removed_index != usize::MAX && num_edges > 0 {
                        if let Some(islands) = islands.as_deref_mut() {
                            islands.persistent.unlink_contact(removed_index as u32);
                            islands
                                .persistent
                                .contact_edge_removed(removed_index as u32, num_edges as u32 - 1);
                        }
                    }

                    // Emit a contact stopped event if we had a contact before removing the edge.
                    // Also wake up the dynamic bodies that were in contact.
                    if let Some(mut ctct) = contact_pair {
                        clear_pair_solver_color(&mut self.body_solver_color_masks, &mut ctct);

                        if ctct.has_any_active_contact() {
                            if let Some(islands) = islands {
                                if let Some(co_parent1) = &co1.parent {
                                    islands.wake_up(bodies, co_parent1.handle, true);
                                }

                                if let Some(co_parent2) = co2.parent {
                                    islands.wake_up(bodies, co_parent2.handle, true);
                                }
                            }

                            if (co1.flags.active_events | co2.flags.active_events)
                                .contains(ActiveEvents::COLLISION_EVENTS)
                            {
                                ctct.emit_stop_event(bodies, colliders, events);
                            }
                        }

                        // Retire the pair for reuse by `add_pair` (bounded pool).
                        if self.retired_pairs.len() < 2048 {
                            self.retired_pairs.push(ctct);
                        }
                    }
                }
            }
        }
    }

    #[profiling::function]
    fn add_pair(&mut self, colliders: &ColliderSet, pair: &ColliderPair) {
        if let (Some(co1), Some(co2)) =
            (colliders.get(pair.collider1), colliders.get(pair.collider2))
        {
            // These colliders have no parents - continue.

            let (gid1, gid2) = self.graph_indices.ensure_pair_exists(
                pair.collider1.0,
                pair.collider2.0,
                ColliderGraphIndices::invalid(),
            );

            if co1.is_sensor() || co2.is_sensor() {
                // NOTE: the collider won't have a graph index as long
                // as it does not interact with anything.
                if !InteractionGraph::<(), ()>::is_graph_index_valid(gid1.intersection_graph_index)
                {
                    gid1.intersection_graph_index =
                        self.intersection_graph.graph.add_node(pair.collider1);
                }

                if !InteractionGraph::<(), ()>::is_graph_index_valid(gid2.intersection_graph_index)
                {
                    gid2.intersection_graph_index =
                        self.intersection_graph.graph.add_node(pair.collider2);
                }

                if self
                    .intersection_graph
                    .graph
                    .find_edge(gid1.intersection_graph_index, gid2.intersection_graph_index)
                    .is_none()
                {
                    let _ = self.intersection_graph.add_edge(
                        gid1.intersection_graph_index,
                        gid2.intersection_graph_index,
                        IntersectionPair::new(),
                    );
                }
            } else {
                // NOTE: same code as above, but for the contact graph.
                // TODO: refactor both pieces of code somehow?

                // NOTE: the collider won't have a graph index as long
                // as it does not interact with anything.
                if !InteractionGraph::<(), ()>::is_graph_index_valid(gid1.contact_graph_index) {
                    gid1.contact_graph_index = self.contact_graph.graph.add_node(pair.collider1);
                }

                if !InteractionGraph::<(), ()>::is_graph_index_valid(gid2.contact_graph_index) {
                    gid2.contact_graph_index = self.contact_graph.graph.add_node(pair.collider2);
                }

                if self
                    .contact_graph
                    .graph
                    .find_edge(gid1.contact_graph_index, gid2.contact_graph_index)
                    .is_none()
                {
                    let interaction = if let Some(mut retired) = self.retired_pairs.pop() {
                        retired.reset_for_reuse(pair.collider1, pair.collider2);
                        retired
                    } else {
                        ContactPair::new(pair.collider1, pair.collider2)
                    };
                    // Keep the solver hints aligned with the edges vec at all times: add/remove
                    // events interleave within a step, so deferred growth would desync the swap_remove
                    // mirroring. On mismatch (deserialized), drop them — they rebuild lazily.
                    if self.pair_solver_hints.len() == self.contact_graph.graph.edges.len() {
                        self.pair_solver_hints.push(0);
                    } else {
                        self.pair_solver_hints.clear();
                    }
                    // Same for the force-event back-ref array: the O(1) removal maintenance
                    // mirrors the edges vec's swap_remove on it, so it must track the edge
                    // count exactly (not lazily at the next maintenance).
                    if self.force_event_pos.len() == self.contact_graph.graph.edges.len() {
                        self.force_event_pos.push(u32::MAX);
                    } else {
                        self.force_event_pos.clear();
                        self.force_event_pairs.clear();
                        self.force_list_valid = false;
                    }
                    let _ = self.contact_graph.add_edge(
                        gid1.contact_graph_index,
                        gid2.contact_graph_index,
                        interaction,
                    );
                }
            }
        }
    }

    pub(crate) fn register_pairs(
        &mut self,
        mut islands: Option<&mut IslandManager>,
        colliders: &ColliderSet,
        bodies: &mut RigidBodySet,
        broad_phase_events: &[BroadPhasePairEvent],
        events: &dyn EventHandler,
    ) {
        for event in broad_phase_events {
            match event {
                BroadPhasePairEvent::AddPair(pair) => {
                    self.add_pair(colliders, pair);
                }
                BroadPhasePairEvent::DeletePair(pair) => {
                    self.remove_pair(
                        islands.as_deref_mut(),
                        colliders,
                        bodies,
                        pair,
                        events,
                        PairRemovalMode::Auto,
                    );
                }
            }
        }
    }
}
