#[cfg(feature = "parallel")]
use rayon::prelude::*;

use crate::data::graph::EdgeIndex;
use crate::data::Coarena;
use crate::dynamics::{
    CoefficientCombineRule, ImpulseJointSet, IslandManager, RigidBodyDominance, RigidBodySet,
    RigidBodyType,
};
use crate::geometry::{
    BroadPhasePairEvent, ColliderChanges, ColliderGraphIndex, ColliderHandle, ColliderPair,
    ColliderSet, CollisionEvent, ContactData, ContactManifold, ContactManifoldData, ContactPair,
    InteractionGraph, IntersectionPair, SolverContact, SolverFlags, TemporaryInteractionIndex,
};
use crate::math::{Real, Vector};
use crate::pipeline::{
    ActiveEvents, ActiveHooks, ContactModificationContext, EventHandler, PairFilterContext,
    PhysicsHooks,
};
use crate::prelude::{CollisionEventFlags, MultibodyJointSet};
use parry::query::{DefaultQueryDispatcher, PersistentQueryDispatcher};
use parry::utils::IsometryOpt;
use std::collections::HashMap;
use std::sync::Arc;

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

/// The narrow-phase responsible for computing precise contact information between colliders.
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
        }
    }

    /// The query dispatcher used by this narrow-phase to select the right collision-detection
    /// algorithms depending of the shape types.
    pub fn query_dispatcher(
        &self,
    ) -> &dyn PersistentQueryDispatcher<ContactManifoldData, ContactData> {
        &*self.query_dispatcher
    }

    /// The contact graph containing all contact pairs and their contact information.
    pub fn contact_graph(&self) -> &InteractionGraph<ColliderHandle, ContactPair> {
        &self.contact_graph
    }

    /// The intersection graph containing all intersection pairs and their intersection information.
    pub fn intersection_graph(&self) -> &InteractionGraph<ColliderHandle, IntersectionPair> {
        &self.intersection_graph
    }

    /// All the contacts involving the given collider.
    ///
    /// It is strongly recommended to use the [`NarrowPhase::contacts_with`] method instead. This
    /// method can be used if the generation number of the collider handle isn't known.
    pub fn contacts_with_unknown_gen(&self, collider: u32) -> impl Iterator<Item = &ContactPair> {
        self.graph_indices
            .get_unknown_gen(collider)
            .map(|id| id.contact_graph_index)
            .into_iter()
            .flat_map(move |id| self.contact_graph.interactions_with(id))
            .map(|pair| pair.2)
    }

    /// All the contacts involving the given collider.
    pub fn contacts_with<'a>(
        &self,
        collider: ColliderHandle,
    ) -> impl Iterator<Item = &ContactPair> {
        self.graph_indices
            .get(collider.0)
            .map(|id| id.contact_graph_index)
            .into_iter()
            .flat_map(move |id| self.contact_graph.interactions_with(id))
            .map(|pair| pair.2)
    }

    /// All the intersections involving the given collider.
    ///
    /// It is strongly recommended to use the [`NarrowPhase::intersections_with`]  method instead.
    /// This method can be used if the generation number of the collider handle isn't known.
    pub fn intersections_with_unknown_gen(
        &self,
        collider: u32,
    ) -> impl Iterator<Item = (ColliderHandle, ColliderHandle, bool)> + '_ {
        self.graph_indices
            .get_unknown_gen(collider)
            .map(|id| id.intersection_graph_index)
            .into_iter()
            .flat_map(move |id| {
                self.intersection_graph
                    .interactions_with(id)
                    .map(|e| (e.0, e.1, e.2.intersecting))
            })
    }

    /// All the intersections involving the given collider.
    pub fn intersections_with(
        &self,
        collider: ColliderHandle,
    ) -> impl Iterator<Item = (ColliderHandle, ColliderHandle, bool)> + '_ {
        self.graph_indices
            .get(collider.0)
            .map(|id| id.intersection_graph_index)
            .into_iter()
            .flat_map(move |id| {
                self.intersection_graph
                    .interactions_with(id)
                    .map(|e| (e.0, e.1, e.2.intersecting))
            })
    }

    /// Returns the contact pair at the given temporary index.
    pub fn contact_pair_at_index(&self, id: TemporaryInteractionIndex) -> &ContactPair {
        &self.contact_graph.graph.edges[id.index()].weight
    }

    /// The contact pair involving two specific colliders.
    ///
    /// It is strongly recommended to use the [`NarrowPhase::contact_pair`] method instead. This
    /// method can be used if the generation number of the collider handle isn't known.
    ///
    /// If this returns `None`, there is no contact between the two colliders.
    /// If this returns `Some`, then there may be a contact between the two colliders. Check the
    /// result [`ContactPair::has_any_active_contact`] method to see if there is an actual contact.
    pub fn contact_pair_unknown_gen(&self, collider1: u32, collider2: u32) -> Option<&ContactPair> {
        let id1 = self.graph_indices.get_unknown_gen(collider1)?;
        let id2 = self.graph_indices.get_unknown_gen(collider2)?;
        self.contact_graph
            .interaction_pair(id1.contact_graph_index, id2.contact_graph_index)
            .map(|c| c.2)
    }

    /// The contact pair involving two specific colliders.
    ///
    /// If this returns `None`, there is no contact between the two colliders.
    /// If this returns `Some`, then there may be a contact between the two colliders. Check the
    /// result [`ContactPair::has_any_active_contact`] method to see if there is an actual contact.
    pub fn contact_pair(
        &self,
        collider1: ColliderHandle,
        collider2: ColliderHandle,
    ) -> Option<&ContactPair> {
        let id1 = self.graph_indices.get(collider1.0)?;
        let id2 = self.graph_indices.get(collider2.0)?;
        self.contact_graph
            .interaction_pair(id1.contact_graph_index, id2.contact_graph_index)
            .map(|c| c.2)
    }

    /// The intersection pair involving two specific colliders.
    ///
    /// It is strongly recommended to use the [`NarrowPhase::intersection_pair`] method instead. This
    /// method can be used if the generation number of the collider handle isn't known.
    ///
    /// If this returns `None` or `Some(false)`, then there is no intersection between the two colliders.
    /// If this returns `Some(true)`, then there may be an intersection between the two colliders.
    pub fn intersection_pair_unknown_gen(&self, collider1: u32, collider2: u32) -> Option<bool> {
        let id1 = self.graph_indices.get_unknown_gen(collider1)?;
        let id2 = self.graph_indices.get_unknown_gen(collider2)?;
        self.intersection_graph
            .interaction_pair(id1.intersection_graph_index, id2.intersection_graph_index)
            .map(|c| c.2.intersecting)
    }

    /// The intersection pair involving two specific colliders.
    ///
    /// If this returns `None` or `Some(false)`, then there is no intersection between the two colliders.
    /// If this returns `Some(true)`, then there may be an intersection between the two colliders.
    pub fn intersection_pair(
        &self,
        collider1: ColliderHandle,
        collider2: ColliderHandle,
    ) -> Option<bool> {
        let id1 = self.graph_indices.get(collider1.0)?;
        let id2 = self.graph_indices.get(collider2.0)?;
        self.intersection_graph
            .interaction_pair(id1.intersection_graph_index, id2.intersection_graph_index)
            .map(|c| c.2.intersecting)
    }

    /// All the contact pairs maintained by this narrow-phase.
    pub fn contact_pairs(&self) -> impl Iterator<Item = &ContactPair> {
        self.contact_graph.interactions()
    }

    /// All the intersection pairs maintained by this narrow-phase.
    pub fn intersection_pairs(
        &self,
    ) -> impl Iterator<Item = (ColliderHandle, ColliderHandle, bool)> + '_ {
        self.intersection_graph
            .interactions_with_endpoints()
            .map(|e| (e.0, e.1, e.2.intersecting))
    }

    // #[cfg(feature = "parallel")]
    // pub(crate) fn contact_pairs_vec_mut(&mut self) -> &mut Vec<ContactPair> {
    //     &mut self.contact_graph.interactions
    // }

    /// Maintain the narrow-phase internal state by taking collider removal into account.
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
        let mut prox_id_remap = HashMap::new();
        let mut contact_id_remap = HashMap::new();

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

                if pair.start_event_emited {
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
                if pair.start_event_emited {
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
            if pair.start_event_emited {
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

        if let Some(replacement) = self.contact_graph.remove_node(contact_graph_id) {
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

                    // For each collider which had their sensor status modified, we need
                    // to transfer their contact/intersection graph edges to the intersection/contact graph.
                    // To achieve this we will remove the relevant contact/intersection pairs form the
                    // contact/intersection graphs, and then add them into the other graph.
                    if co.changes.contains(ColliderChanges::TYPE) {
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

        // Add the paid removed pair to the relevant graph.
        for pair in pairs_to_remove {
            self.add_pair(colliders, &pair.0);
        }
    }

    fn remove_pair(
        &mut self,
        islands: Option<&mut IslandManager>,
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
                        if intersection.intersecting {
                            if (co1.flags.active_events | co2.flags.active_events)
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
                    }
                } else {
                    let contact_pair = self
                        .contact_graph
                        .remove_edge(gid1.contact_graph_index, gid2.contact_graph_index);

                    // Emit a contact stopped event if we had a contact before removing the edge.
                    // Also wake up the dynamic bodies that were in contact.
                    if let Some(mut ctct) = contact_pair {
                        if ctct.has_any_active_contact {
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
                    }
                }
            }
        }
    }

    fn add_pair(&mut self, colliders: &ColliderSet, pair: &ColliderPair) {
        if let (Some(co1), Some(co2)) =
            (colliders.get(pair.collider1), colliders.get(pair.collider2))
        {
            if co1.parent.map(|p| p.handle) == co2.parent.map(|p| p.handle) {
                if co1.parent.is_some() {
                    // Same parents. Ignore collisions.
                    return;
                }

                // These colliders have no parents - continue.
            }

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
                    let interaction = ContactPair::new(pair.collider1, pair.collider2);
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

    pub(crate) fn compute_intersections(
        &mut self,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        modified_colliders: &[ColliderHandle],
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
    ) {
        if modified_colliders.is_empty() {
            return;
        }

        let nodes = &self.intersection_graph.graph.nodes;
        let query_dispatcher = &*self.query_dispatcher;

        // TODO: don't iterate on all the edges.
        par_iter_mut!(&mut self.intersection_graph.graph.edges).for_each(|edge| {
            let handle1 = nodes[edge.source().index()].weight;
            let handle2 = nodes[edge.target().index()].weight;
            let had_intersection = edge.weight.intersecting;
            let co1 = &colliders[handle1];
            let co2 = &colliders[handle2];

            // TODO: remove the `loop` once labels on blocks is stabilized.
            'emit_events: loop {
                if !co1.changes.needs_narrow_phase_update()
                    && !co2.changes.needs_narrow_phase_update()
                {
                    // No update needed for these colliders.
                    return;
                }

                // TODO: avoid lookup into bodies.
                let mut rb_type1 = RigidBodyType::Fixed;
                let mut rb_type2 = RigidBodyType::Fixed;

                if let Some(co_parent1) = &co1.parent {
                    rb_type1 = bodies[co_parent1.handle].body_type;
                }

                if let Some(co_parent2) = &co2.parent {
                    rb_type2 = bodies[co_parent2.handle].body_type;
                }

                // Filter based on the rigid-body types.
                if !co1.flags.active_collision_types.test(rb_type1, rb_type2)
                    && !co2.flags.active_collision_types.test(rb_type1, rb_type2)
                {
                    edge.weight.intersecting = false;
                    break 'emit_events;
                }

                // Filter based on collision groups.
                if !co1.flags.collision_groups.test(co2.flags.collision_groups) {
                    edge.weight.intersecting = false;
                    break 'emit_events;
                }

                let active_hooks = co1.flags.active_hooks | co2.flags.active_hooks;

                if active_hooks.contains(ActiveHooks::FILTER_INTERSECTION_PAIR) {
                    let context = PairFilterContext {
                        bodies,
                        colliders,
                        rigid_body1: co1.parent.map(|p| p.handle),
                        rigid_body2: co2.parent.map(|p| p.handle),
                        collider1: handle1,
                        collider2: handle2,
                    };

                    if !hooks.filter_intersection_pair(&context) {
                        // No intersection allowed.
                        edge.weight.intersecting = false;
                        break 'emit_events;
                    }
                }

                let pos12 = co1.pos.inv_mul(&co2.pos);
                edge.weight.intersecting = query_dispatcher
                    .intersection_test(&pos12, &*co1.shape, &*co2.shape)
                    .unwrap_or(false);
                break 'emit_events;
            }

            let active_events = co1.flags.active_events | co2.flags.active_events;

            if active_events.contains(ActiveEvents::COLLISION_EVENTS)
                && had_intersection != edge.weight.intersecting
            {
                if edge.weight.intersecting {
                    edge.weight
                        .emit_start_event(bodies, colliders, handle1, handle2, events);
                } else {
                    edge.weight
                        .emit_stop_event(bodies, colliders, handle1, handle2, events);
                }
            }
        });
    }

    pub(crate) fn compute_contacts(
        &mut self,
        prediction_distance: Real,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
        modified_colliders: &[ColliderHandle],
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
    ) {
        if modified_colliders.is_empty() {
            return;
        }

        let query_dispatcher = &*self.query_dispatcher;

        // TODO: don't iterate on all the edges.
        par_iter_mut!(&mut self.contact_graph.graph.edges).for_each(|edge| {
            let pair = &mut edge.weight;
            let had_any_active_contact = pair.has_any_active_contact;
            let co1 = &colliders[pair.collider1];
            let co2 = &colliders[pair.collider2];

            // TODO: remove the `loop` once labels on blocks are supported.
            'emit_events: loop {
                if !co1.changes.needs_narrow_phase_update()
                    && !co2.changes.needs_narrow_phase_update()
                {
                    // No update needed for these colliders.
                    return;
                }

                // TODO: avoid lookup into bodies.
                let mut rb_type1 = RigidBodyType::Fixed;
                let mut rb_type2 = RigidBodyType::Fixed;

                if let Some(co_parent1) = &co1.parent {
                    rb_type1 = bodies[co_parent1.handle].body_type;
                }

                if let Some(co_parent2) = &co2.parent {
                    rb_type2 = bodies[co_parent2.handle].body_type;
                }

                // Deal with contacts disabled between bodies attached by joints.
                if let (Some(co_parent1), Some(co_parent2)) = (&co1.parent, &co2.parent) {
                    for (_, joint) in
                        impulse_joints.joints_between(co_parent1.handle, co_parent2.handle)
                    {
                        if !joint.data.contacts_enabled {
                            pair.clear();
                            break 'emit_events;
                        }
                    }

                    if let Some((_, _, mb_link)) =
                        multibody_joints.joint_between(co_parent1.handle, co_parent2.handle)
                    {
                        if !mb_link.joint.data.contacts_enabled {
                            pair.clear();
                            break 'emit_events;
                        }
                    }
                }

                // Filter based on the rigid-body types.
                if !co1.flags.active_collision_types.test(rb_type1, rb_type2)
                    && !co2.flags.active_collision_types.test(rb_type1, rb_type2)
                {
                    pair.clear();
                    break 'emit_events;
                }

                // Filter based on collision groups.
                if !co1.flags.collision_groups.test(co2.flags.collision_groups) {
                    pair.clear();
                    break 'emit_events;
                }

                let active_hooks = co1.flags.active_hooks | co2.flags.active_hooks;

                let mut solver_flags = if active_hooks.contains(ActiveHooks::FILTER_CONTACT_PAIRS) {
                    let context = PairFilterContext {
                        bodies,
                        colliders,
                        rigid_body1: co1.parent.map(|p| p.handle),
                        rigid_body2: co2.parent.map(|p| p.handle),
                        collider1: pair.collider1,
                        collider2: pair.collider2,
                    };

                    if let Some(solver_flags) = hooks.filter_contact_pair(&context) {
                        solver_flags
                    } else {
                        // No contact allowed.
                        pair.clear();
                        break 'emit_events;
                    }
                } else {
                    SolverFlags::default()
                };

                if !co1.flags.solver_groups.test(co2.flags.solver_groups) {
                    solver_flags.remove(SolverFlags::COMPUTE_IMPULSES);
                }

                if co1.changes.contains(ColliderChanges::SHAPE)
                    || co2.changes.contains(ColliderChanges::SHAPE)
                {
                    // The shape changed so the workspace is no longer valid.
                    pair.workspace = None;
                }

                let pos12 = co1.pos.inv_mul(&co2.pos);
                let _ = query_dispatcher.contact_manifolds(
                    &pos12,
                    &*co1.shape,
                    &*co2.shape,
                    prediction_distance,
                    &mut pair.manifolds,
                    &mut pair.workspace,
                );

                let friction = CoefficientCombineRule::combine(
                    co1.material.friction,
                    co2.material.friction,
                    co1.material.friction_combine_rule as u8,
                    co2.material.friction_combine_rule as u8,
                );
                let restitution = CoefficientCombineRule::combine(
                    co1.material.restitution,
                    co2.material.restitution,
                    co1.material.restitution_combine_rule as u8,
                    co2.material.restitution_combine_rule as u8,
                );

                let zero = RigidBodyDominance(0); // The value doesn't matter, it will be MAX because of the effective groups.
                let dominance1 = co1
                    .parent
                    .map(|p1| bodies[p1.handle].dominance)
                    .unwrap_or(zero);
                let dominance2 = co2
                    .parent
                    .map(|p2| bodies[p2.handle].dominance)
                    .unwrap_or(zero);

                pair.has_any_active_contact = false;

                for manifold in &mut pair.manifolds {
                    let world_pos1 = manifold.subshape_pos1.prepend_to(&co1.pos);
                    manifold.data.solver_contacts.clear();
                    manifold.data.rigid_body1 = co1.parent.map(|p| p.handle);
                    manifold.data.rigid_body2 = co2.parent.map(|p| p.handle);
                    manifold.data.solver_flags = solver_flags;
                    manifold.data.relative_dominance = dominance1.effective_group(&rb_type1)
                        - dominance2.effective_group(&rb_type2);
                    manifold.data.normal = world_pos1 * manifold.local_n1;

                    // Generate solver contacts.
                    for (contact_id, contact) in manifold.points.iter().enumerate() {
                        assert!(
                            contact_id <= u8::MAX as usize,
                            "A contact manifold cannot contain more than 255 contacts currently."
                        );

                        if contact.dist < prediction_distance {
                            // Generate the solver contact.
                            let solver_contact = SolverContact {
                                contact_id: contact_id as u8,
                                point: world_pos1 * contact.local_p1
                                    + manifold.data.normal * contact.dist / 2.0,
                                dist: contact.dist,
                                friction,
                                restitution,
                                tangent_velocity: Vector::zeros(),
                                is_new: contact.data.impulse == 0.0,
                            };

                            manifold.data.solver_contacts.push(solver_contact);
                            pair.has_any_active_contact = true;
                        }
                    }

                    // Apply the user-defined contact modification.
                    if active_hooks.contains(ActiveHooks::MODIFY_SOLVER_CONTACTS) {
                        let mut modifiable_solver_contacts =
                            std::mem::replace(&mut manifold.data.solver_contacts, Vec::new());
                        let mut modifiable_user_data = manifold.data.user_data;
                        let mut modifiable_normal = manifold.data.normal;

                        let mut context = ContactModificationContext {
                            bodies,
                            colliders,
                            rigid_body1: co1.parent.map(|p| p.handle),
                            rigid_body2: co2.parent.map(|p| p.handle),
                            collider1: pair.collider1,
                            collider2: pair.collider2,
                            manifold,
                            solver_contacts: &mut modifiable_solver_contacts,
                            normal: &mut modifiable_normal,
                            user_data: &mut modifiable_user_data,
                        };

                        hooks.modify_solver_contacts(&mut context);

                        manifold.data.solver_contacts = modifiable_solver_contacts;
                        manifold.data.normal = modifiable_normal;
                        manifold.data.user_data = modifiable_user_data;
                    }
                }

                break 'emit_events;
            }

            let active_events = co1.flags.active_events | co2.flags.active_events;

            if pair.has_any_active_contact != had_any_active_contact {
                if active_events.contains(ActiveEvents::COLLISION_EVENTS) {
                    if pair.has_any_active_contact {
                        pair.emit_start_event(bodies, colliders, events);
                    } else {
                        pair.emit_stop_event(bodies, colliders, events);
                    }
                }
            }
        });
    }

    /// Retrieve all the interactions with at least one contact point, happening between two active bodies.
    // NOTE: this is very similar to the code from ImpulseJointSet::select_active_interactions.
    pub(crate) fn select_active_contacts<'a>(
        &'a mut self,
        islands: &IslandManager,
        bodies: &RigidBodySet,
        out_contact_pairs: &mut Vec<TemporaryInteractionIndex>,
        out_manifolds: &mut Vec<&'a mut ContactManifold>,
        out: &mut Vec<Vec<ContactManifoldIndex>>,
    ) {
        for out_island in &mut out[..islands.num_islands()] {
            out_island.clear();
        }

        // TODO: don't iterate through all the interactions.
        for (pair_id, inter) in self.contact_graph.graph.edges.iter_mut().enumerate() {
            let mut push_pair = false;

            for manifold in &mut inter.weight.manifolds {
                if manifold
                    .data
                    .solver_flags
                    .contains(SolverFlags::COMPUTE_IMPULSES)
                    && manifold.data.num_active_contacts() != 0
                {
                    let (active_island_id1, rb_type1, sleeping1) =
                        if let Some(handle1) = manifold.data.rigid_body1 {
                            let rb1 = &bodies[handle1];
                            (
                                rb1.ids.active_island_id,
                                rb1.body_type,
                                rb1.activation.sleeping,
                            )
                        } else {
                            (0, RigidBodyType::Fixed, true)
                        };

                    let (active_island_id2, rb_type2, sleeping2) =
                        if let Some(handle2) = manifold.data.rigid_body2 {
                            let rb2 = &bodies[handle2];
                            (
                                rb2.ids.active_island_id,
                                rb2.body_type,
                                rb2.activation.sleeping,
                            )
                        } else {
                            (0, RigidBodyType::Fixed, true)
                        };

                    if (rb_type1.is_dynamic() || rb_type2.is_dynamic())
                        && (!rb_type1.is_dynamic() || !sleeping1)
                        && (!rb_type2.is_dynamic() || !sleeping2)
                    {
                        let island_index = if !rb_type1.is_dynamic() {
                            active_island_id2
                        } else {
                            active_island_id1
                        };

                        out[island_index].push(out_manifolds.len());
                        out_manifolds.push(manifold);
                        push_pair = true;
                    }
                }
            }

            if push_pair {
                out_contact_pairs.push(EdgeIndex::new(pair_id as u32));
            }
        }
    }
}
