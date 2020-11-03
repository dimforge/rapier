use crate::data::graph::{Direction, EdgeIndex, Graph, NodeIndex};
use crate::geometry::ColliderHandle;

/// Index of a node of the interaction graph.
pub type ColliderGraphIndex = NodeIndex;
/// Index of a node of the interaction graph.
pub type RigidBodyGraphIndex = NodeIndex;
/// Temporary index to and edge of the interaction graph.
pub type TemporaryInteractionIndex = EdgeIndex;

/// A graph where nodes are collision objects and edges are contact or proximity algorithms.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct InteractionGraph<T> {
    pub(crate) graph: Graph<ColliderHandle, T>,
}

impl<T> InteractionGraph<T> {
    /// Creates a new empty collection of collision objects.
    pub fn new() -> Self {
        InteractionGraph {
            graph: Graph::with_capacity(10, 10),
        }
    }

    /// The underlying raw graph structure of this interaction graph.
    pub fn raw_graph(&self) -> &Graph<ColliderHandle, T> {
        &self.graph
    }

    pub(crate) fn invalid_graph_index() -> ColliderGraphIndex {
        ColliderGraphIndex::new(crate::INVALID_U32)
    }

    pub(crate) fn is_graph_index_valid(index: ColliderGraphIndex) -> bool {
        index.index() != crate::INVALID_USIZE
    }

    pub(crate) fn add_edge(
        &mut self,
        index1: ColliderGraphIndex,
        index2: ColliderGraphIndex,
        interaction: T,
    ) -> TemporaryInteractionIndex {
        self.graph.add_edge(index1, index2, interaction)
    }

    pub(crate) fn remove_edge(
        &mut self,
        index1: ColliderGraphIndex,
        index2: ColliderGraphIndex,
    ) -> Option<T> {
        let id = self.graph.find_edge(index1, index2)?;
        self.graph.remove_edge(id)
    }

    /// Removes a handle from this graph and returns a handle that must have its graph index changed to `id`.
    ///
    /// When a node is removed, another node of the graph takes it place. This means that the `ColliderGraphIndex`
    /// of the collision object returned by this method will be equal to `id`. Thus if you maintain
    /// a map between `CollisionObjectSlabHandle` and `ColliderGraphIndex`, then you should update this
    /// map to associate `id` to the handle returned by this method. For example:
    ///
    /// ```.ignore
    /// // Let `id` be the graph index of the collision object we want to remove.
    /// if let Some(other_handle) = graph.remove_node(id) {
    ///    // The graph index of `other_handle` changed to `id` due to the removal.
    ///    map.insert(other_handle, id) ;
    /// }
    /// ```
    #[must_use = "The graph index of the collision object returned by this method has been changed to `id`."]
    pub(crate) fn remove_node(&mut self, id: ColliderGraphIndex) -> Option<ColliderHandle> {
        let _ = self.graph.remove_node(id);
        self.graph.node_weight(id).cloned()
    }

    /// All the interactions pairs on this graph.
    pub fn interaction_pairs(&self) -> impl Iterator<Item = (ColliderHandle, ColliderHandle, &T)> {
        self.graph.raw_edges().iter().map(move |edge| {
            (
                self.graph[edge.source()],
                self.graph[edge.target()],
                &edge.weight,
            )
        })
    }

    /// The interaction between the two collision objects identified by their graph index.
    pub fn interaction_pair(
        &self,
        id1: ColliderGraphIndex,
        id2: ColliderGraphIndex,
    ) -> Option<(ColliderHandle, ColliderHandle, &T)> {
        self.graph.find_edge(id1, id2).and_then(|edge| {
            let endpoints = self.graph.edge_endpoints(edge)?;
            let h1 = self.graph.node_weight(endpoints.0)?;
            let h2 = self.graph.node_weight(endpoints.1)?;
            let weight = self.graph.edge_weight(edge)?;
            Some((*h1, *h2, weight))
        })
    }

    /// The interaction between the two collision objects identified by their graph index.
    pub fn interaction_pair_mut(
        &mut self,
        id1: ColliderGraphIndex,
        id2: ColliderGraphIndex,
    ) -> Option<(ColliderHandle, ColliderHandle, &mut T)> {
        let edge = self.graph.find_edge(id1, id2)?;
        let endpoints = self.graph.edge_endpoints(edge)?;
        let h1 = *self.graph.node_weight(endpoints.0)?;
        let h2 = *self.graph.node_weight(endpoints.1)?;
        let weight = self.graph.edge_weight_mut(edge)?;
        Some((h1, h2, weight))
    }

    /// All the interaction involving the collision object with graph index `id`.
    pub fn interactions_with(
        &self,
        id: ColliderGraphIndex,
    ) -> impl Iterator<Item = (ColliderHandle, ColliderHandle, &T)> {
        self.graph.edges(id).filter_map(move |e| {
            let endpoints = self.graph.edge_endpoints(e.id()).unwrap();
            Some((self.graph[endpoints.0], self.graph[endpoints.1], e.weight()))
        })
    }

    /// Gets the interaction with the given index.
    pub fn index_interaction(
        &self,
        id: TemporaryInteractionIndex,
    ) -> Option<(ColliderHandle, ColliderHandle, &T)> {
        if let (Some(e), Some(endpoints)) =
            (self.graph.edge_weight(id), self.graph.edge_endpoints(id))
        {
            Some((self.graph[endpoints.0], self.graph[endpoints.1], e))
        } else {
            None
        }
    }

    /// All the mutable references to interactions involving the collision object with graph index `id`.
    pub fn interactions_with_mut(
        &mut self,
        id: ColliderGraphIndex,
    ) -> impl Iterator<
        Item = (
            ColliderHandle,
            ColliderHandle,
            TemporaryInteractionIndex,
            &mut T,
        ),
    > {
        let incoming_edge = self.graph.first_edge(id, Direction::Incoming);
        let outgoing_edge = self.graph.first_edge(id, Direction::Outgoing);

        InteractionsWithMut {
            graph: &mut self.graph,
            incoming_edge,
            outgoing_edge,
        }
    }

    // /// All the collision object handles of collision objects interacting with the collision object with graph index `id`.
    // pub fn colliders_interacting_with<'a>(
    //     &'a self,
    //     id: ColliderGraphIndex,
    // ) -> impl Iterator<Item = ColliderHandle> + 'a {
    //     self.graph.edges(id).filter_map(move |e| {
    //         let inter = e.weight();
    //
    //         if e.source() == id {
    //             Some(self.graph[e.target()])
    //         } else {
    //             Some(self.graph[e.source()])
    //         }
    //     })
    // }

    // /// All the collision object handles of collision objects in contact with the collision object with graph index `id`.
    // pub fn colliders_in_contact_with<'a>(
    //     &'a self,
    //     id: ColliderGraphIndex,
    // ) -> impl Iterator<Item = ColliderHandle> + 'a {
    //     self.graph.edges(id).filter_map(move |e| {
    //         let inter = e.weight();
    //
    //         if inter.is_contact() && Self::is_interaction_effective(inter) {
    //             if e.source() == id {
    //                 Some(self.graph[e.target()])
    //             } else {
    //                 Some(self.graph[e.source()])
    //             }
    //         } else {
    //             None
    //         }
    //     })
    // }
    //
    // /// All the collision object handles of collision objects in proximity of with the collision object with graph index `id`.
    // /// for details.
    // pub fn colliders_in_proximity_of<'a>(
    //     &'a self,
    //     id: ColliderGraphIndex,
    // ) -> impl Iterator<Item = ColliderHandle> + 'a {
    //     self.graph.edges(id).filter_map(move |e| {
    //         if let Interaction::Proximity(_, prox) = e.weight() {
    //             if *prox == Proximity::Intersecting {
    //                 if e.source() == id {
    //                     return Some(self.graph[e.target()]);
    //                 } else {
    //                     return Some(self.graph[e.source()]);
    //                 }
    //             }
    //         }
    //
    //         None
    //     })
    // }
}

pub struct InteractionsWithMut<'a, T> {
    graph: &'a mut Graph<ColliderHandle, T>,
    incoming_edge: Option<EdgeIndex>,
    outgoing_edge: Option<EdgeIndex>,
}

impl<'a, T> Iterator for InteractionsWithMut<'a, T> {
    type Item = (
        ColliderHandle,
        ColliderHandle,
        TemporaryInteractionIndex,
        &'a mut T,
    );

    #[inline]
    fn next(
        &mut self,
    ) -> Option<(
        ColliderHandle,
        ColliderHandle,
        TemporaryInteractionIndex,
        &'a mut T,
    )> {
        if let Some(edge) = self.incoming_edge {
            self.incoming_edge = self.graph.next_edge(edge, Direction::Incoming);
            let endpoints = self.graph.edge_endpoints(edge).unwrap();
            let (co1, co2) = (self.graph[endpoints.0], self.graph[endpoints.1]);
            let interaction = &mut self.graph[edge];
            return Some((co1, co2, edge, unsafe { std::mem::transmute(interaction) }));
        }

        let edge = self.outgoing_edge?;
        self.outgoing_edge = self.graph.next_edge(edge, Direction::Outgoing);
        let endpoints = self.graph.edge_endpoints(edge).unwrap();
        let (co1, co2) = (self.graph[endpoints.0], self.graph[endpoints.1]);
        let interaction = &mut self.graph[edge];
        Some((co1, co2, edge, unsafe { std::mem::transmute(interaction) }))
    }
}
