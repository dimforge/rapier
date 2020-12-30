#[cfg(feature = "parallel")]
use rayon::prelude::*;

use crate::data::pubsub::Subscription;
use crate::data::Coarena;
use crate::dynamics::RigidBodySet;
use crate::geometry::{
    BroadPhasePairEvent, ColliderGraphIndex, ColliderHandle, ContactData, ContactEvent,
    ContactManifoldData, ContactPairFilter, IntersectionEvent, PairFilterContext,
    ProximityPairFilter, RemovedCollider, SolverContact, SolverFlags,
};
use crate::geometry::{ColliderSet, ContactManifold, ContactPair, InteractionGraph};
use crate::math::Vector;
use crate::pipeline::EventHandler;
use cdl::query::{DefaultQueryDispatcher, PersistentQueryDispatcher, QueryDispatcher};
use std::collections::HashMap;
use std::sync::Arc;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct ColliderGraphIndices {
    contact_graph_index: ColliderGraphIndex,
    intersection_graph_index: ColliderGraphIndex,
}

impl ColliderGraphIndices {
    fn invalid() -> Self {
        Self {
            contact_graph_index: InteractionGraph::<ContactPair>::invalid_graph_index(),
            intersection_graph_index: InteractionGraph::<bool>::invalid_graph_index(),
        }
    }
}

/// The narrow-phase responsible for computing precise contact information between colliders.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct NarrowPhase {
    #[cfg_attr(
        feature = "serde-serialize",
        serde(skip, default = "default_query_dispatcher")
    )]
    query_dispatcher: Arc<dyn PersistentQueryDispatcher<ContactManifoldData, ContactData>>,
    contact_graph: InteractionGraph<ContactPair>,
    intersection_graph: InteractionGraph<bool>,
    graph_indices: Coarena<ColliderGraphIndices>,
    removed_colliders: Option<Subscription<RemovedCollider>>,
}

fn default_query_dispatcher() -> Arc<dyn PersistentQueryDispatcher<ContactManifoldData, ContactData>>
{
    Arc::new(DefaultQueryDispatcher)
}

pub(crate) type ContactManifoldIndex = usize;

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
            removed_colliders: None,
        }
    }

    /// The contact graph containing all contact pairs and their contact information.
    pub fn contact_graph(&self) -> &InteractionGraph<ContactPair> {
        &self.contact_graph
    }

    /// The intersection graph containing all intersection pairs and their intersection information.
    pub fn intersection_graph(&self) -> &InteractionGraph<bool> {
        &self.intersection_graph
    }

    /// All the contacts involving the given collider.
    pub fn contacts_with(
        &self,
        collider: ColliderHandle,
    ) -> Option<impl Iterator<Item = (ColliderHandle, ColliderHandle, &ContactPair)>> {
        let id = self.graph_indices.get(collider)?;
        Some(self.contact_graph.interactions_with(id.contact_graph_index))
    }

    /// All the intersections involving the given collider.
    pub fn intersections_with<'a>(
        &'a self,
        collider: ColliderHandle,
    ) -> Option<impl Iterator<Item = (ColliderHandle, ColliderHandle, bool)> + 'a> {
        let id = self.graph_indices.get(collider)?;
        Some(
            self.intersection_graph
                .interactions_with(id.intersection_graph_index)
                .map(|e| (e.0, e.1, *e.2)),
        )
    }

    /// The contact pair involving two specific colliders.
    ///
    /// If this returns `None`, there is no contact between the two colliders.
    /// If this returns `Some`, then there may be a contact between the two colliders. Check the
    /// result [`ContactPair::has_any_active_collider`] method to see if there is an actual contact.
    pub fn contact_pair(
        &self,
        collider1: ColliderHandle,
        collider2: ColliderHandle,
    ) -> Option<&ContactPair> {
        let id1 = self.graph_indices.get(collider1)?;
        let id2 = self.graph_indices.get(collider2)?;
        self.contact_graph
            .interaction_pair(id1.contact_graph_index, id2.contact_graph_index)
            .map(|c| c.2)
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
        let id1 = self.graph_indices.get(collider1)?;
        let id2 = self.graph_indices.get(collider2)?;
        self.intersection_graph
            .interaction_pair(id1.intersection_graph_index, id2.intersection_graph_index)
            .map(|c| *c.2)
    }

    /// All the contact pairs maintained by this narrow-phase.
    pub fn contact_pairs(&self) -> impl Iterator<Item = &ContactPair> {
        self.contact_graph.interactions()
    }

    /// All the intersection pairs maintained by this narrow-phase.
    pub fn intersection_pairs<'a>(
        &'a self,
    ) -> impl Iterator<Item = (ColliderHandle, ColliderHandle, bool)> + 'a {
        self.intersection_graph
            .interactions_with_endpoints()
            .map(|e| (e.0, e.1, *e.2))
    }

    // #[cfg(feature = "parallel")]
    // pub(crate) fn contact_pairs_vec_mut(&mut self) -> &mut Vec<ContactPair> {
    //     &mut self.contact_graph.interactions
    // }

    /// Maintain the narrow-phase internal state by taking collider removal into account.
    pub fn maintain(&mut self, colliders: &mut ColliderSet, bodies: &mut RigidBodySet) {
        // Ensure we already subscribed.
        if self.removed_colliders.is_none() {
            self.removed_colliders = Some(colliders.removed_colliders.subscribe());
        }

        let mut cursor = self.removed_colliders.take().unwrap();

        // TODO: avoid these hash-maps.
        // They are necessary to handle the swap-remove done internally
        // by the contact/intersection graphs when a node is removed.
        let mut prox_id_remap = HashMap::new();
        let mut contact_id_remap = HashMap::new();
        let mut i = 0;

        while let Some(collider) = colliders.removed_colliders.read_ith(&cursor, i) {
            // NOTE: if the collider does not have any graph indices currently, there is nothing
            // to remove in the narrow-phase for this collider.
            if let Some(graph_idx) = self.graph_indices.get(collider.handle) {
                let intersection_graph_id = prox_id_remap
                    .get(&collider.handle)
                    .copied()
                    .unwrap_or(graph_idx.intersection_graph_index);
                let contact_graph_id = contact_id_remap
                    .get(&collider.handle)
                    .copied()
                    .unwrap_or(graph_idx.contact_graph_index);

                self.remove_collider(
                    intersection_graph_id,
                    contact_graph_id,
                    colliders,
                    bodies,
                    &mut prox_id_remap,
                    &mut contact_id_remap,
                );
            }

            i += 1;
        }

        colliders.removed_colliders.ack(&mut cursor);
        self.removed_colliders = Some(cursor);
    }

    pub(crate) fn remove_collider<'a>(
        &mut self,
        intersection_graph_id: ColliderGraphIndex,
        contact_graph_id: ColliderGraphIndex,
        colliders: &mut ColliderSet,
        bodies: &mut RigidBodySet,
        prox_id_remap: &mut HashMap<ColliderHandle, ColliderGraphIndex>,
        contact_id_remap: &mut HashMap<ColliderHandle, ColliderGraphIndex>,
    ) {
        // Wake up every body in contact with the deleted collider.
        for (a, b, _) in self.contact_graph.interactions_with(contact_graph_id) {
            if let Some(parent) = colliders.get(a).map(|c| c.parent) {
                bodies.wake_up(parent, true)
            }

            if let Some(parent) = colliders.get(b).map(|c| c.parent) {
                bodies.wake_up(parent, true)
            }
        }

        // We have to manage the fact that one other collider will
        // have its graph index changed because of the node's swap-remove.
        if let Some(replacement) = self.intersection_graph.remove_node(intersection_graph_id) {
            if let Some(replacement) = self.graph_indices.get_mut(replacement) {
                replacement.intersection_graph_index = intersection_graph_id;
            } else {
                prox_id_remap.insert(replacement, intersection_graph_id);
            }
        }

        if let Some(replacement) = self.contact_graph.remove_node(contact_graph_id) {
            if let Some(replacement) = self.graph_indices.get_mut(replacement) {
                replacement.contact_graph_index = contact_graph_id;
            } else {
                contact_id_remap.insert(replacement, contact_graph_id);
            }
        }
    }

    pub(crate) fn register_pairs(
        &mut self,
        colliders: &mut ColliderSet,
        bodies: &mut RigidBodySet,
        broad_phase_events: &[BroadPhasePairEvent],
        events: &dyn EventHandler,
    ) {
        for event in broad_phase_events {
            match event {
                BroadPhasePairEvent::AddPair(pair) => {
                    if let (Some(co1), Some(co2)) =
                        (colliders.get(pair.collider1), colliders.get(pair.collider2))
                    {
                        if co1.parent == co2.parent {
                            // Same parents. Ignore collisions.
                            continue;
                        }

                        let (gid1, gid2) = self.graph_indices.ensure_pair_exists(
                            pair.collider1,
                            pair.collider2,
                            ColliderGraphIndices::invalid(),
                        );

                        if co1.is_sensor() || co2.is_sensor() {
                            // NOTE: the collider won't have a graph index as long
                            // as it does not interact with anything.
                            if !InteractionGraph::<bool>::is_graph_index_valid(
                                gid1.intersection_graph_index,
                            ) {
                                gid1.intersection_graph_index =
                                    self.intersection_graph.graph.add_node(pair.collider1);
                            }

                            if !InteractionGraph::<bool>::is_graph_index_valid(
                                gid2.intersection_graph_index,
                            ) {
                                gid2.intersection_graph_index =
                                    self.intersection_graph.graph.add_node(pair.collider2);
                            }

                            if self
                                .intersection_graph
                                .graph
                                .find_edge(
                                    gid1.intersection_graph_index,
                                    gid2.intersection_graph_index,
                                )
                                .is_none()
                            {
                                let _ = self.intersection_graph.add_edge(
                                    gid1.intersection_graph_index,
                                    gid2.intersection_graph_index,
                                    false,
                                );
                            }
                        } else {
                            // NOTE: same code as above, but for the contact graph.
                            // TODO: refactor both pieces of code somehow?

                            // NOTE: the collider won't have a graph index as long
                            // as it does not interact with anything.
                            if !InteractionGraph::<ContactPair>::is_graph_index_valid(
                                gid1.contact_graph_index,
                            ) {
                                gid1.contact_graph_index =
                                    self.contact_graph.graph.add_node(pair.collider1);
                            }

                            if !InteractionGraph::<ContactPair>::is_graph_index_valid(
                                gid2.contact_graph_index,
                            ) {
                                gid2.contact_graph_index =
                                    self.contact_graph.graph.add_node(pair.collider2);
                            }

                            if self
                                .contact_graph
                                .graph
                                .find_edge(gid1.contact_graph_index, gid2.contact_graph_index)
                                .is_none()
                            {
                                let interaction = ContactPair::new(*pair);
                                let _ = self.contact_graph.add_edge(
                                    gid1.contact_graph_index,
                                    gid2.contact_graph_index,
                                    interaction,
                                );
                            }
                        }
                    }
                }
                BroadPhasePairEvent::DeletePair(pair) => {
                    if let (Some(co1), Some(co2)) =
                        (colliders.get(pair.collider1), colliders.get(pair.collider2))
                    {
                        // TODO: could we just unwrap here?
                        // Don't we have the guarantee that we will get a `AddPair` before a `DeletePair`?
                        if let (Some(gid1), Some(gid2)) = (
                            self.graph_indices.get(pair.collider1),
                            self.graph_indices.get(pair.collider2),
                        ) {
                            if co1.is_sensor() || co2.is_sensor() {
                                let was_intersecting = self.intersection_graph.remove_edge(
                                    gid1.intersection_graph_index,
                                    gid2.intersection_graph_index,
                                );

                                // Emit an intersection lost event if we had an intersection before removing the edge.
                                if Some(true) == was_intersecting {
                                    let prox_event = IntersectionEvent::new(
                                        pair.collider1,
                                        pair.collider2,
                                        false,
                                    );
                                    events.handle_intersection_event(prox_event)
                                }
                            } else {
                                let contact_pair = self.contact_graph.remove_edge(
                                    gid1.contact_graph_index,
                                    gid2.contact_graph_index,
                                );

                                // Emit a contact stopped event if we had a contact before removing the edge.
                                // Also wake up the dynamic bodies that were in contact.
                                if let Some(ctct) = contact_pair {
                                    if ctct.has_any_active_contact() {
                                        bodies.wake_up(co1.parent, true);
                                        bodies.wake_up(co2.parent, true);

                                        events.handle_contact_event(ContactEvent::Stopped(
                                            pair.collider1,
                                            pair.collider2,
                                        ))
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub(crate) fn compute_intersections(
        &mut self,
        prediction_distance: f32,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        pair_filter: Option<&dyn ProximityPairFilter>,
        events: &dyn EventHandler,
    ) {
        let nodes = &self.intersection_graph.graph.nodes;
        let query_dispatcher = &*self.query_dispatcher;
        par_iter_mut!(&mut self.intersection_graph.graph.edges).for_each(|edge| {
            let handle1 = nodes[edge.source().index()].weight;
            let handle2 = nodes[edge.target().index()].weight;
            let co1 = &colliders[handle1];
            let co2 = &colliders[handle2];

            // FIXME: avoid lookup into bodies.
            let rb1 = &bodies[co1.parent];
            let rb2 = &bodies[co2.parent];

            if (rb1.is_sleeping() && rb2.is_static())
                || (rb2.is_sleeping() && rb1.is_static())
                || (rb1.is_sleeping() && rb2.is_sleeping())
            {
                // No need to update this intersection because nothing moved.
                return;
            }

            if !co1.collision_groups.test(co2.collision_groups) {
                // The intersection is not allowed.
                return;
            }

            if pair_filter.is_none() && !rb1.is_dynamic() && !rb2.is_dynamic() {
                // Default filtering rule: no intersection between two non-dynamic bodies.
                return;
            }

            if let Some(filter) = pair_filter {
                let context = PairFilterContext {
                    rigid_body1: rb1,
                    rigid_body2: rb2,
                    collider1: co1,
                    collider2: co2,
                };

                if !filter.filter_intersection_pair(&context) {
                    // No intersection allowed.
                    return;
                }
            }

            let pos12 = co1.position().inv_mul(co2.position());

            if let Ok(intersection) =
                query_dispatcher.intersection_test(&pos12, co1.shape(), co2.shape())
            {
                if intersection != edge.weight {
                    edge.weight = intersection;
                    events.handle_intersection_event(IntersectionEvent::new(
                        handle1,
                        handle2,
                        intersection,
                    ));
                }
            }
        });
    }

    pub(crate) fn compute_contacts(
        &mut self,
        prediction_distance: f32,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        pair_filter: Option<&dyn ContactPairFilter>,
        events: &dyn EventHandler,
    ) {
        let query_dispatcher = &*self.query_dispatcher;

        par_iter_mut!(&mut self.contact_graph.graph.edges).for_each(|edge| {
            let pair = &mut edge.weight;
            let co1 = &colliders[pair.pair.collider1];
            let co2 = &colliders[pair.pair.collider2];

            // FIXME: avoid lookup into bodies.
            let rb1 = &bodies[co1.parent];
            let rb2 = &bodies[co2.parent];

            if (rb1.is_sleeping() && rb2.is_static())
                || (rb2.is_sleeping() && rb1.is_static())
                || (rb1.is_sleeping() && rb2.is_sleeping())
            {
                // No need to update this contact because nothing moved.
                return;
            }

            if !co1.collision_groups.test(co2.collision_groups) {
                // The collision is not allowed.
                return;
            }

            if pair_filter.is_none() && !rb1.is_dynamic() && !rb2.is_dynamic() {
                // Default filtering rule: no contact between two non-dynamic bodies.
                return;
            }

            let mut solver_flags = if let Some(filter) = pair_filter {
                let context = PairFilterContext {
                    rigid_body1: rb1,
                    rigid_body2: rb2,
                    collider1: co1,
                    collider2: co2,
                };

                if let Some(solver_flags) = filter.filter_contact_pair(&context) {
                    solver_flags
                } else {
                    // No contact allowed.
                    return;
                }
            } else {
                SolverFlags::COMPUTE_IMPULSES
            };

            if !co1.solver_groups.test(co2.solver_groups) {
                solver_flags.remove(SolverFlags::COMPUTE_IMPULSES);
            }

            let pos12 = co1.position().inv_mul(co2.position());
            let _ = query_dispatcher.contact_manifolds(
                &pos12,
                co1.shape(),
                co2.shape(),
                prediction_distance,
                &mut pair.manifolds,
                &mut pair.workspace,
            );

            // TODO: don't write this everytime?
            for manifold in &mut pair.manifolds {
                manifold.data.solver_contacts.clear();
                manifold.data.set_from_colliders(co1, co2, solver_flags);
                manifold.data.normal = co1.position() * manifold.local_n1;

                for contact in &manifold.points[..manifold.num_active_contacts] {
                    let solver_contact = SolverContact {
                        point: co1.position() * contact.local_p1
                            + manifold.data.normal * contact.dist / 2.0,
                        dist: contact.dist,
                        friction: (co1.friction + co2.friction) / 2.0,
                        restitution: (co1.restitution + co2.restitution) / 2.0,
                        surface_velocity: Vector::zeros(),
                        data: contact.data,
                    };

                    manifold.data.solver_contacts.push(solver_contact);
                }
            }
        });
    }

    /// Retrieve all the interactions with at least one contact point, happening between two active bodies.
    // NOTE: this is very similar to the code from JointSet::select_active_interactions.
    pub(crate) fn sort_and_select_active_contacts<'a>(
        &'a mut self,
        bodies: &RigidBodySet,
        out_manifolds: &mut Vec<&'a mut ContactManifold>,
        out: &mut Vec<Vec<ContactManifoldIndex>>,
    ) {
        for out_island in &mut out[..bodies.num_islands()] {
            out_island.clear();
        }

        // FIXME: don't iterate through all the interactions.
        for inter in self.contact_graph.graph.edges.iter_mut() {
            for manifold in &mut inter.weight.manifolds {
                let rb1 = &bodies[manifold.data.body_pair.body1];
                let rb2 = &bodies[manifold.data.body_pair.body2];
                if manifold
                    .data
                    .solver_flags
                    .contains(SolverFlags::COMPUTE_IMPULSES)
                    && manifold.num_active_contacts() != 0
                    && (rb1.is_dynamic() || rb2.is_dynamic())
                    && (!rb1.is_dynamic() || !rb1.is_sleeping())
                    && (!rb2.is_dynamic() || !rb2.is_sleeping())
                {
                    let island_index = if !rb1.is_dynamic() {
                        rb2.active_island_id
                    } else {
                        rb1.active_island_id
                    };

                    out[island_index].push(out_manifolds.len());
                    out_manifolds.push(manifold);
                }
            }
        }
    }
}
