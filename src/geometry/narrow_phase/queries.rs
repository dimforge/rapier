//! Read-only accessors and iterators over the narrow-phase's contact and
//! intersection pairs and their interaction graphs.

use super::NarrowPhase;
use crate::geometry::{
    ColliderHandle, ColliderSet, ContactData, ContactManifoldData, ContactPair, InteractionGraph,
    IntersectionPair, TemporaryInteractionIndex,
};
use parry::query::PersistentQueryDispatcher;

impl NarrowPhase {
    /// Per-body masks (indexed by rigid-body arena index) of the solver colors used
    /// by each body's active contact pairs. Read by the staged island solver to
    /// color joints in the same color space as the contacts.
    pub(crate) fn body_solver_color_masks(&self) -> &[u128] {
        &self.body_solver_color_masks
    }

    /// The query dispatcher used by this narrow-phase to select the right collision-detection
    /// algorithms depending on the shape types.
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
    /// It is strongly recommended to use the [`NarrowPhase::contact_pairs_with`] method instead. This
    /// method can be used if the generation number of the collider handle isn't known.
    pub fn contact_pairs_with_unknown_gen(
        &self,
        collider: u32,
    ) -> impl Iterator<Item = &ContactPair> {
        self.graph_indices
            .get_unknown_gen(collider)
            .map(|id| id.contact_graph_index)
            .into_iter()
            .flat_map(move |id| self.contact_graph.interactions_with(id))
            .map(|pair| pair.2)
    }

    /// All the contact pairs involving the given collider.
    ///
    /// The returned contact pairs identify pairs of colliders with intersecting bounding-volumes.
    /// To check if any geometric contact happened between the collider shapes, check
    /// [`ContactPair::has_any_active_contact`].
    pub fn contact_pairs_with(
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

    /// All the intersection pairs involving the given collider.
    ///
    /// It is strongly recommended to use the [`NarrowPhase::intersection_pairs_with`]  method instead.
    /// This method can be used if the generation number of the collider handle isn't known.
    pub fn intersection_pairs_with_unknown_gen(
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

    /// All the intersection pairs involving the given collider, where at least one collider
    /// involved in the intersection is a sensor.
    ///
    /// The returned contact pairs identify pairs of colliders (where at least one is a sensor) with
    /// intersecting bounding-volumes. To check if any geometric overlap happened between the collider shapes, check
    /// the returned boolean.
    pub fn intersection_pairs_with(
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

    /// `(edge_id, parent1, parent2)` for every *touching* contact pair. Used to (re)build
    /// the persistent islands from scratch (bootstrap after construction or
    /// deserialization) and by their debug validation.
    pub(crate) fn touching_pairs_with_ids<'a>(
        &'a self,
        colliders: &'a ColliderSet,
    ) -> impl Iterator<
        Item = (
            u32,
            Option<crate::dynamics::RigidBodyHandle>,
            Option<crate::dynamics::RigidBodyHandle>,
        ),
    > + 'a {
        self.contact_graph
            .graph
            .edges
            .iter()
            .enumerate()
            .filter_map(move |(edge_id, edge)| {
                let pair = &edge.weight;
                if !pair.has_any_active_contact() {
                    return None;
                }
                let parent = |co: crate::geometry::ColliderHandle| {
                    colliders.get(co).and_then(|c| c.parent.map(|p| p.handle))
                };
                Some((
                    edge_id as u32,
                    parent(pair.collider1),
                    parent(pair.collider2),
                ))
            })
    }

    /// `(edge_id, other collider)` for every *touching* contact pair of `collider` — the
    /// adjacency the persistent islands' local split search walks. Edge id and touching
    /// predicate match the island contact links exactly.
    pub(crate) fn touching_edges_with(
        &self,
        collider: ColliderHandle,
    ) -> impl Iterator<Item = (u32, ColliderHandle)> + '_ {
        self.graph_indices
            .get(collider.0)
            .map(|id| id.contact_graph_index)
            .into_iter()
            .flat_map(move |id| self.contact_graph.graph.edges(id))
            .filter(|edge| edge.weight().has_any_active_contact())
            .map(move |edge| {
                let pair = edge.weight();
                let other = if pair.collider1 == collider {
                    pair.collider2
                } else {
                    pair.collider1
                };
                (edge.id().index() as u32, other)
            })
    }

    /// All the intersection pairs maintained by this narrow-phase.
    pub fn intersection_pairs(
        &self,
    ) -> impl Iterator<Item = (ColliderHandle, ColliderHandle, bool)> + '_ {
        self.intersection_graph
            .interactions_with_endpoints()
            .map(|e| (e.0, e.1, e.2.intersecting))
    }
}
