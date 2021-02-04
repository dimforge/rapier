use super::Joint;
use crate::geometry::{InteractionGraph, RigidBodyGraphIndex, TemporaryInteractionIndex};

use crate::data::arena::Arena;
use crate::dynamics::{IslandSet, JointParams, RigidBodyHandle, RigidBodySet};

/// The unique identifier of a joint added to the joint set.
/// The unique identifier of a collider added to a collider set.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[repr(transparent)]
pub struct JointHandle(pub(crate) crate::data::arena::Index);

impl JointHandle {
    /// Converts this handle into its (index, generation) components.
    pub fn into_raw_parts(self) -> (usize, u64) {
        self.0.into_raw_parts()
    }

    /// Reconstructs an handle from its (index, generation) components.
    pub fn from_raw_parts(id: usize, generation: u64) -> Self {
        Self(crate::data::arena::Index::from_raw_parts(id, generation))
    }

    /// An always-invalid joint handle.
    pub fn invalid() -> Self {
        Self(crate::data::arena::Index::from_raw_parts(
            crate::INVALID_USIZE,
            crate::INVALID_U64,
        ))
    }
}

pub(crate) type JointIndex = usize;
pub(crate) type JointGraphEdge = crate::data::graph::Edge<Joint>;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A set of joints that can be handled by a physics `World`.
pub struct JointSet {
    joint_ids: Arena<TemporaryInteractionIndex>, // Map joint handles to edge ids on the graph.
    joint_graph: InteractionGraph<RigidBodyHandle, Joint>,
}

impl JointSet {
    /// Creates a new empty set of joints.
    pub fn new() -> Self {
        Self {
            joint_ids: Arena::new(),
            joint_graph: InteractionGraph::new(),
        }
    }

    /// The number of joints on this set.
    pub fn len(&self) -> usize {
        self.joint_graph.graph.edges.len()
    }

    /// Retrieve the joint graph where edges are joints and nodes are rigid body handles.
    pub fn joint_graph(&self) -> &InteractionGraph<RigidBodyHandle, Joint> {
        &self.joint_graph
    }

    /// Is the given joint handle valid?
    pub fn contains(&self, handle: JointHandle) -> bool {
        self.joint_ids.contains(handle.0)
    }

    /// Gets the joint with the given handle.
    pub fn get(&self, handle: JointHandle) -> Option<&Joint> {
        let id = self.joint_ids.get(handle.0)?;
        self.joint_graph.graph.edge_weight(*id)
    }

    /// Gets the joint with the given handle without a known generation.
    ///
    /// This is useful when you know you want the joint at position `i` but
    /// don't know what is its current generation number. Generation numbers are
    /// used to protect from the ABA problem because the joint position `i`
    /// are recycled between two insertion and a removal.
    ///
    /// Using this is discouraged in favor of `self.get(handle)` which does not
    /// suffer form the ABA problem.
    pub fn get_unknown_gen(&self, i: usize) -> Option<(&Joint, JointHandle)> {
        let (id, handle) = self.joint_ids.get_unknown_gen(i)?;
        Some((
            self.joint_graph.graph.edge_weight(*id)?,
            JointHandle(handle),
        ))
    }

    /// Iterates through all the joint on this set.
    pub fn iter(&self) -> impl Iterator<Item = (JointHandle, &Joint)> {
        self.joint_graph
            .graph
            .edges
            .iter()
            .map(|e| (e.weight.handle, &e.weight))
    }

    /// Iterates mutably through all the joint on this set.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (JointHandle, &mut Joint)> {
        self.joint_graph
            .graph
            .edges
            .iter_mut()
            .map(|e| (e.weight.handle, &mut e.weight))
    }

    // /// The set of joints as an array.
    // pub(crate) fn joints(&self) -> &[JointGraphEdge] {
    //     // self.joint_graph
    //     //     .graph
    //     //     .edges
    //     //     .iter_mut()
    //     //     .map(|e| &mut e.weight)
    // }

    #[cfg(not(feature = "parallel"))]
    pub(crate) fn joints_mut(&mut self) -> &mut [JointGraphEdge] {
        &mut self.joint_graph.graph.edges[..]
    }

    #[cfg(feature = "parallel")]
    pub(crate) fn joints_vec_mut(&mut self) -> &mut Vec<JointGraphEdge> {
        &mut self.joint_graph.graph.edges
    }

    /// Inserts a new joint into this set and retrieve its handle.
    pub fn insert<J>(
        &mut self,
        bodies: &mut RigidBodySet,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
        joint_params: J,
    ) -> JointHandle
    where
        J: Into<JointParams>,
    {
        let handle = self.joint_ids.insert(0.into());
        let joint = Joint {
            body1,
            body2,
            handle: JointHandle(handle),
            #[cfg(feature = "parallel")]
            constraint_index: 0,
            #[cfg(feature = "parallel")]
            position_constraint_index: 0,
            params: joint_params.into(),
        };

        let (rb1, rb2) = bodies.get2_mut_internal(joint.body1, joint.body2);
        let (rb1, rb2) = (
            rb1.expect("Attempt to attach a joint to a non-existing body."),
            rb2.expect("Attempt to attach a joint to a non-existing body."),
        );

        // NOTE: the body won't have a graph index if it does not
        // have any joint attached.
        if !InteractionGraph::<RigidBodyHandle, Joint>::is_graph_index_valid(rb1.joint_graph_index)
        {
            rb1.joint_graph_index = self.joint_graph.graph.add_node(joint.body1);
        }

        if !InteractionGraph::<RigidBodyHandle, Joint>::is_graph_index_valid(rb2.joint_graph_index)
        {
            rb2.joint_graph_index = self.joint_graph.graph.add_node(joint.body2);
        }

        let id = self
            .joint_graph
            .add_edge(rb1.joint_graph_index, rb2.joint_graph_index, joint);

        self.joint_ids[handle] = id;
        JointHandle(handle)
    }

    /// Retrieve all the joints happening between two active bodies.
    // NOTE: this is very similar to the code from NarrowPhase::select_active_interactions.
    pub(crate) fn select_active_interactions(
        &self,
        islands: &IslandSet,
        bodies: &RigidBodySet,
        out: &mut Vec<Vec<JointIndex>>,
    ) {
        for out_island in &mut out[..islands.num_active_islands()] {
            out_island.clear();
        }

        // FIXME: don't iterate through all the interactions.
        for (i, edge) in self.joint_graph.graph.edges.iter().enumerate() {
            let joint = &edge.weight;
            let rb1 = &bodies[joint.body1];
            let rb2 = &bodies[joint.body2];

            if (rb1.is_dynamic() || rb2.is_dynamic())
                && (!rb1.is_dynamic() || !rb1.is_sleeping())
                && (!rb2.is_dynamic() || !rb2.is_sleeping())
            {
                let island_index = if !rb1.is_dynamic() {
                    0 // islands.islands()[rb2.island_id].active_island_id()
                } else {
                    0 // islands.islands()[rb1.island_id].active_island_id()
                };

                out[island_index].push(i);
            }
        }
    }

    /// Removes a joint from this set.
    ///
    /// If `wake_up` is set to `true`, then the bodies attached to this joint will be
    /// automatically woken up.
    pub fn remove(
        &mut self,
        handle: JointHandle,
        bodies: &mut RigidBodySet,
        wake_up: bool,
    ) -> Option<Joint> {
        let id = self.joint_ids.remove(handle.0)?;
        let endpoints = self.joint_graph.graph.edge_endpoints(id)?;

        if wake_up {
            // Wake-up the bodies attached to this joint.
            if let Some(rb_handle) = self.joint_graph.graph.node_weight(endpoints.0) {
                bodies.wake_up(*rb_handle, true);
            }
            if let Some(rb_handle) = self.joint_graph.graph.node_weight(endpoints.1) {
                bodies.wake_up(*rb_handle, true);
            }
        }

        let removed_joint = self.joint_graph.graph.remove_edge(id);

        if let Some(edge) = self.joint_graph.graph.edge_weight(id) {
            self.joint_ids[edge.handle.0] = id;
        }

        removed_joint
    }

    pub(crate) fn remove_rigid_body(
        &mut self,
        deleted_id: RigidBodyGraphIndex,
        bodies: &mut RigidBodySet,
    ) {
        if InteractionGraph::<(), ()>::is_graph_index_valid(deleted_id) {
            // We have to delete each joint one by one in order to:
            // - Wake-up the attached bodies.
            // - Update our Handle -> graph edge mapping.
            // Delete the node.
            let to_delete: Vec<_> = self
                .joint_graph
                .interactions_with(deleted_id)
                .map(|e| (e.0, e.1, e.2.handle))
                .collect();
            for (h1, h2, to_delete_handle) in to_delete {
                let to_delete_edge_id = self.joint_ids.remove(to_delete_handle.0).unwrap();
                self.joint_graph.graph.remove_edge(to_delete_edge_id);

                // Update the id of the edge which took the place of the deleted one.
                if let Some(j) = self.joint_graph.graph.edge_weight_mut(to_delete_edge_id) {
                    self.joint_ids[j.handle.0] = to_delete_edge_id;
                }

                // Wake up the attached bodies.
                bodies.wake_up(h1, true);
                bodies.wake_up(h2, true);
            }

            if let Some(other) = self.joint_graph.remove_node(deleted_id) {
                // One rigid-body joint graph index may have been invalidated
                // so we need to update it.
                if let Some(replacement) = bodies.get_mut_internal(other) {
                    replacement.joint_graph_index = deleted_id;
                }
            }
        }
    }
}
