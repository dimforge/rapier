#[cfg(feature = "parallel")]
use rayon::prelude::*;

use crate::data::pubsub::Subscription;
use crate::data::Coarena;
use crate::dynamics::{BodyPair, CoefficientCombineRule, RigidBodySet};
use crate::geometry::{
    BroadPhasePairEvent, ColliderGraphIndex, ColliderHandle, ColliderSet, ContactData,
    ContactEvent, ContactManifold, ContactManifoldData, ContactPair, InteractionGraph,
    IntersectionEvent, RemovedCollider, SolverContact, SolverFlags,
};
use crate::math::{Real, Vector};
use crate::pipeline::{
    ContactModificationContext, EventHandler, PairFilterContext, PhysicsHooks, PhysicsHooksFlags,
};
use parry::query::{DefaultQueryDispatcher, PersistentQueryDispatcher};
use parry::utils::IsometryOpt;
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
            contact_graph_index: InteractionGraph::<(), ()>::invalid_graph_index(),
            intersection_graph_index: InteractionGraph::<(), ()>::invalid_graph_index(),
        }
    }
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
    intersection_graph: InteractionGraph<ColliderHandle, bool>,
    graph_indices: Coarena<ColliderGraphIndices>,
    removed_colliders: Option<Subscription<RemovedCollider>>,
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
    pub fn contact_graph(&self) -> &InteractionGraph<ColliderHandle, ContactPair> {
        &self.contact_graph
    }

    /// The intersection graph containing all intersection pairs and their intersection information.
    pub fn intersection_graph(&self) -> &InteractionGraph<ColliderHandle, bool> {
        &self.intersection_graph
    }

    /// All the contacts involving the given collider.
    pub fn contacts_with(
        &self,
        collider: ColliderHandle,
    ) -> Option<impl Iterator<Item = (ColliderHandle, ColliderHandle, &ContactPair)>> {
        let id = self.graph_indices.get(collider.0)?;
        Some(self.contact_graph.interactions_with(id.contact_graph_index))
    }

    /// All the intersections involving the given collider.
    pub fn intersections_with(
        &self,
        collider: ColliderHandle,
    ) -> Option<impl Iterator<Item = (ColliderHandle, ColliderHandle, bool)> + '_> {
        let id = self.graph_indices.get(collider.0)?;
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
        let id1 = self.graph_indices.get(collider1.0)?;
        let id2 = self.graph_indices.get(collider2.0)?;
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
        let id1 = self.graph_indices.get(collider1.0)?;
        let id2 = self.graph_indices.get(collider2.0)?;
        self.intersection_graph
            .interaction_pair(id1.intersection_graph_index, id2.intersection_graph_index)
            .map(|c| *c.2)
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

        let cursor = self.removed_colliders.take().unwrap();

        // TODO: avoid these hash-maps.
        // They are necessary to handle the swap-remove done internally
        // by the contact/intersection graphs when a node is removed.
        let mut prox_id_remap = HashMap::new();
        let mut contact_id_remap = HashMap::new();
        let mut i = 0;

        while let Some(collider) = colliders.removed_colliders.read_ith(&cursor, i) {
            // NOTE: if the collider does not have any graph indices currently, there is nothing
            // to remove in the narrow-phase for this collider.
            if let Some(graph_idx) = self.graph_indices.get(collider.handle.0) {
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

        colliders.removed_colliders.ack(&cursor);
        self.removed_colliders = Some(cursor);
    }

    pub(crate) fn remove_collider(
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
            if let Some(replacement) = self.graph_indices.get_mut(replacement.0) {
                replacement.intersection_graph_index = intersection_graph_id;
            } else {
                prox_id_remap.insert(replacement, intersection_graph_id);
            }
        }

        if let Some(replacement) = self.contact_graph.remove_node(contact_graph_id) {
            if let Some(replacement) = self.graph_indices.get_mut(replacement.0) {
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
                            pair.collider1.0,
                            pair.collider2.0,
                            ColliderGraphIndices::invalid(),
                        );

                        if co1.is_sensor() || co2.is_sensor() {
                            // NOTE: the collider won't have a graph index as long
                            // as it does not interact with anything.
                            if !InteractionGraph::<(), ()>::is_graph_index_valid(
                                gid1.intersection_graph_index,
                            ) {
                                gid1.intersection_graph_index =
                                    self.intersection_graph.graph.add_node(pair.collider1);
                            }

                            if !InteractionGraph::<(), ()>::is_graph_index_valid(
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
                            if !InteractionGraph::<(), ()>::is_graph_index_valid(
                                gid1.contact_graph_index,
                            ) {
                                gid1.contact_graph_index =
                                    self.contact_graph.graph.add_node(pair.collider1);
                            }

                            if !InteractionGraph::<(), ()>::is_graph_index_valid(
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
                            self.graph_indices.get(pair.collider1.0),
                            self.graph_indices.get(pair.collider2.0),
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
                                    if ctct.has_any_active_contact {
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
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
    ) {
        let nodes = &self.intersection_graph.graph.nodes;
        let query_dispatcher = &*self.query_dispatcher;
        let active_hooks = hooks.active_hooks();

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

            if !active_hooks.contains(PhysicsHooksFlags::FILTER_INTERSECTION_PAIR)
                && !rb1.is_dynamic()
                && !rb2.is_dynamic()
            {
                // Default filtering rule: no intersection between two non-dynamic bodies.
                return;
            }

            if active_hooks.contains(PhysicsHooksFlags::FILTER_INTERSECTION_PAIR) {
                let context = PairFilterContext {
                    rigid_body1: rb1,
                    rigid_body2: rb2,
                    collider_handle1: handle1,
                    collider_handle2: handle2,
                    collider1: co1,
                    collider2: co2,
                };

                if !hooks.filter_intersection_pair(&context) {
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
        prediction_distance: Real,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
    ) {
        let query_dispatcher = &*self.query_dispatcher;
        let active_hooks = hooks.active_hooks();

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

            if !active_hooks.contains(PhysicsHooksFlags::FILTER_CONTACT_PAIR)
                && !rb1.is_dynamic()
                && !rb2.is_dynamic()
            {
                // Default filtering rule: no contact between two non-dynamic bodies.
                return;
            }

            let mut solver_flags = if active_hooks.contains(PhysicsHooksFlags::FILTER_CONTACT_PAIR)
            {
                let context = PairFilterContext {
                    rigid_body1: rb1,
                    rigid_body2: rb2,
                    collider_handle1: pair.pair.collider1,
                    collider_handle2: pair.pair.collider2,
                    collider1: co1,
                    collider2: co2,
                };

                if let Some(solver_flags) = hooks.filter_contact_pair(&context) {
                    solver_flags
                } else {
                    // No contact allowed.
                    return;
                }
            } else {
                co1.solver_flags | co2.solver_flags
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

            let mut has_any_active_contact = false;

            let friction = CoefficientCombineRule::combine(
                co1.friction,
                co2.friction,
                co1.flags.friction_combine_rule_value(),
                co2.flags.friction_combine_rule_value(),
            );
            let restitution = CoefficientCombineRule::combine(
                co1.restitution,
                co2.restitution,
                co1.flags.restitution_combine_rule_value(),
                co2.flags.restitution_combine_rule_value(),
            );

            for manifold in &mut pair.manifolds {
                let world_pos1 = manifold.subshape_pos1.prepend_to(co1.position());
                manifold.data.solver_contacts.clear();
                manifold.data.body_pair = BodyPair::new(co1.parent(), co2.parent());
                manifold.data.solver_flags = solver_flags;
                manifold.data.relative_dominance =
                    rb1.effective_dominance_group() - rb2.effective_dominance_group();
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
                            data: contact.data,
                        };

                        manifold.data.solver_contacts.push(solver_contact);
                        has_any_active_contact = true;
                    }
                }

                // Apply the user-defined contact modification.
                if active_hooks.contains(PhysicsHooksFlags::MODIFY_SOLVER_CONTACTS)
                    && manifold
                        .data
                        .solver_flags
                        .contains(SolverFlags::MODIFY_SOLVER_CONTACTS)
                {
                    let mut modifiable_solver_contacts =
                        std::mem::replace(&mut manifold.data.solver_contacts, Vec::new());
                    let mut modifiable_user_data = manifold.data.user_data;
                    let mut modifiable_normal = manifold.data.normal;

                    let mut context = ContactModificationContext {
                        rigid_body1: rb1,
                        rigid_body2: rb2,
                        collider_handle1: pair.pair.collider1,
                        collider_handle2: pair.pair.collider2,
                        collider1: co1,
                        collider2: co2,
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

            if has_any_active_contact != pair.has_any_active_contact {
                if has_any_active_contact {
                    events.handle_contact_event(ContactEvent::Started(
                        pair.pair.collider1,
                        pair.pair.collider2,
                    ));
                } else {
                    events.handle_contact_event(ContactEvent::Stopped(
                        pair.pair.collider1,
                        pair.pair.collider2,
                    ));
                }

                pair.has_any_active_contact = has_any_active_contact;
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
                    && manifold.data.num_active_contacts() != 0
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
