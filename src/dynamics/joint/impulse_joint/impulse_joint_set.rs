use parry::utils::hashset::HashSet;

use super::ImpulseJoint;
use crate::geometry::{InteractionGraph, RigidBodyGraphIndex, TemporaryInteractionIndex};

use crate::data::arena::Arena;
use crate::data::Coarena;
use crate::dynamics::{GenericJoint, IslandManager, RigidBodyHandle, RigidBodySet};

/// The unique identifier of a joint added to the joint set.
/// The unique identifier of a collider added to a collider set.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[repr(transparent)]
pub struct ImpulseJointHandle(pub crate::data::arena::Index);

impl ImpulseJointHandle {
    /// Converts this handle into its (index, generation) components.
    pub fn into_raw_parts(self) -> (u32, u32) {
        self.0.into_raw_parts()
    }

    /// Reconstructs an handle from its (index, generation) components.
    pub fn from_raw_parts(id: u32, generation: u32) -> Self {
        Self(crate::data::arena::Index::from_raw_parts(id, generation))
    }

    /// An always-invalid joint handle.
    pub fn invalid() -> Self {
        Self(crate::data::arena::Index::from_raw_parts(
            crate::INVALID_U32,
            crate::INVALID_U32,
        ))
    }
}

pub(crate) type JointIndex = usize;
pub(crate) type JointGraphEdge = crate::data::graph::Edge<ImpulseJoint>;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Default, Debug)]
/// A set of impulse_joints that can be handled by a physics `World`.
pub struct ImpulseJointSet {
    rb_graph_ids: Coarena<RigidBodyGraphIndex>,
    /// Map joint handles to edge ids on the graph.
    joint_ids: Arena<TemporaryInteractionIndex>,
    joint_graph: InteractionGraph<RigidBodyHandle, ImpulseJoint>,
    /// A set of rigid-body handles to wake-up during the next timestep.
    pub(crate) to_wake_up: HashSet<RigidBodyHandle>,
}

impl ImpulseJointSet {
    /// Creates a new empty set of impulse_joints.
    pub fn new() -> Self {
        Self {
            rb_graph_ids: Coarena::new(),
            joint_ids: Arena::new(),
            joint_graph: InteractionGraph::new(),
            to_wake_up: HashSet::default(),
        }
    }

    /// The number of impulse_joints on this set.
    pub fn len(&self) -> usize {
        self.joint_graph.graph.edges.len()
    }

    /// `true` if there are no impulse_joints in this set.
    pub fn is_empty(&self) -> bool {
        self.joint_graph.graph.edges.is_empty()
    }

    /// Retrieve the joint graph where edges are impulse_joints and nodes are rigid body handles.
    pub fn joint_graph(&self) -> &InteractionGraph<RigidBodyHandle, ImpulseJoint> {
        &self.joint_graph
    }

    /// Iterates through all the joints between two rigid-bodies.
    pub fn joints_between(
        &self,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
    ) -> impl Iterator<Item = (ImpulseJointHandle, &ImpulseJoint)> {
        self.rb_graph_ids
            .get(body1.0)
            .zip(self.rb_graph_ids.get(body2.0))
            .into_iter()
            .flat_map(move |(id1, id2)| self.joint_graph.interaction_pair(*id1, *id2).into_iter())
            .map(|inter| (inter.2.handle, inter.2))
    }

    /// Iterates through all the impulse joints attached to the given rigid-body.
    pub fn attached_joints(
        &self,
        body: RigidBodyHandle,
    ) -> impl Iterator<
        Item = (
            RigidBodyHandle,
            RigidBodyHandle,
            ImpulseJointHandle,
            &ImpulseJoint,
        ),
    > {
        self.rb_graph_ids
            .get(body.0)
            .into_iter()
            .flat_map(move |id| self.joint_graph.interactions_with(*id))
            .map(|inter| (inter.0, inter.1, inter.2.handle, inter.2))
    }

    /// Iterates through all the impulse joints attached to the given rigid-body.
    pub fn map_attached_joints_mut(
        &mut self,
        body: RigidBodyHandle,
        mut f: impl FnMut(RigidBodyHandle, RigidBodyHandle, ImpulseJointHandle, &mut ImpulseJoint),
    ) {
        self.rb_graph_ids.get(body.0).into_iter().for_each(|id| {
            for inter in self.joint_graph.interactions_with_mut(*id) {
                (f)(inter.0, inter.1, inter.3.handle, inter.3)
            }
        })
    }

    /// Iterates through all the enabled impulse joints attached to the given rigid-body.
    pub fn attached_enabled_joints(
        &self,
        body: RigidBodyHandle,
    ) -> impl Iterator<
        Item = (
            RigidBodyHandle,
            RigidBodyHandle,
            ImpulseJointHandle,
            &ImpulseJoint,
        ),
    > {
        self.attached_joints(body)
            .filter(|inter| inter.3.data.is_enabled())
    }

    /// Is the given joint handle valid?
    pub fn contains(&self, handle: ImpulseJointHandle) -> bool {
        self.joint_ids.contains(handle.0)
    }

    /// Gets the joint with the given handle.
    pub fn get(&self, handle: ImpulseJointHandle) -> Option<&ImpulseJoint> {
        let id = self.joint_ids.get(handle.0)?;
        self.joint_graph.graph.edge_weight(*id)
    }

    /// Gets a mutable reference to the joint with the given handle.
    pub fn get_mut(
        &mut self,
        handle: ImpulseJointHandle,
        wake_up_connected_bodies: bool,
    ) -> Option<&mut ImpulseJoint> {
        let id = self.joint_ids.get(handle.0)?;
        let joint = self.joint_graph.graph.edge_weight_mut(*id);
        if wake_up_connected_bodies {
            if let Some(joint) = &joint {
                self.to_wake_up.insert(joint.body1);
                self.to_wake_up.insert(joint.body2);
            }
        }
        joint
    }

    /// Gets the joint with the given handle without a known generation.
    ///
    /// This is useful when you know you want the joint at index `i` but
    /// don't know what is its current generation number. Generation numbers are
    /// used to protect from the ABA problem because the joint position `i`
    /// are recycled between two insertion and a removal.
    ///
    /// Using this is discouraged in favor of `self.get(handle)` which does not
    /// suffer form the ABA problem.
    pub fn get_unknown_gen(&self, i: u32) -> Option<(&ImpulseJoint, ImpulseJointHandle)> {
        let (id, handle) = self.joint_ids.get_unknown_gen(i)?;
        Some((
            self.joint_graph.graph.edge_weight(*id)?,
            ImpulseJointHandle(handle),
        ))
    }

    /// Gets a mutable reference to the joint with the given handle without a known generation.
    ///
    /// This is useful when you know you want the joint at position `i` but
    /// don't know what is its current generation number. Generation numbers are
    /// used to protect from the ABA problem because the joint position `i`
    /// are recycled between two insertion and a removal.
    ///
    /// Using this is discouraged in favor of `self.get_mut(handle)` which does not
    /// suffer form the ABA problem.
    pub fn get_unknown_gen_mut(
        &mut self,
        i: u32,
    ) -> Option<(&mut ImpulseJoint, ImpulseJointHandle)> {
        let (id, handle) = self.joint_ids.get_unknown_gen(i)?;
        Some((
            self.joint_graph.graph.edge_weight_mut(*id)?,
            ImpulseJointHandle(handle),
        ))
    }

    /// Iterates through all the joint on this set.
    pub fn iter(&self) -> impl Iterator<Item = (ImpulseJointHandle, &ImpulseJoint)> {
        self.joint_graph
            .graph
            .edges
            .iter()
            .map(|e| (e.weight.handle, &e.weight))
    }

    /// Iterates mutably through all the joint on this set.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ImpulseJointHandle, &mut ImpulseJoint)> {
        self.joint_graph
            .graph
            .edges
            .iter_mut()
            .map(|e| (e.weight.handle, &mut e.weight))
    }

    // /// The set of impulse_joints as an array.
    // pub(crate) fn impulse_joints(&self) -> &[JointGraphEdge] {
    //     // self.joint_graph
    //     //     .graph
    //     //     .edges
    //     //     .iter_mut()
    //     //     .map(|e| &mut e.weight)
    // }

    // #[cfg(not(feature = "parallel"))]
    #[allow(dead_code)] // That will likely be useful when we re-introduce intra-island parallelism.
    pub(crate) fn joints_mut(&mut self) -> &mut [JointGraphEdge] {
        &mut self.joint_graph.graph.edges[..]
    }

    #[cfg(feature = "parallel")]
    pub(crate) fn joints_vec_mut(&mut self) -> &mut Vec<JointGraphEdge> {
        &mut self.joint_graph.graph.edges
    }

    /// Inserts a new joint into this set and retrieve its handle.
    ///
    /// If `wake_up` is set to `true`, then the bodies attached to this joint will be
    /// automatically woken up during the next timestep.
    #[profiling::function]
    pub fn insert(
        &mut self,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
        data: impl Into<GenericJoint>,
        wake_up: bool,
    ) -> ImpulseJointHandle {
        let data = data.into();
        let handle = self.joint_ids.insert(0.into());
        let joint = ImpulseJoint {
            body1,
            body2,
            data,
            impulses: na::zero(),
            handle: ImpulseJointHandle(handle),
        };

        let default_id = InteractionGraph::<(), ()>::invalid_graph_index();
        let mut graph_index1 = *self
            .rb_graph_ids
            .ensure_element_exist(joint.body1.0, default_id);
        let mut graph_index2 = *self
            .rb_graph_ids
            .ensure_element_exist(joint.body2.0, default_id);

        // NOTE: the body won't have a graph index if it does not
        // have any joint attached.
        if !InteractionGraph::<RigidBodyHandle, ImpulseJoint>::is_graph_index_valid(graph_index1) {
            graph_index1 = self.joint_graph.graph.add_node(joint.body1);
            self.rb_graph_ids.insert(joint.body1.0, graph_index1);
        }

        if !InteractionGraph::<RigidBodyHandle, ImpulseJoint>::is_graph_index_valid(graph_index2) {
            graph_index2 = self.joint_graph.graph.add_node(joint.body2);
            self.rb_graph_ids.insert(joint.body2.0, graph_index2);
        }

        self.joint_ids[handle] = self.joint_graph.add_edge(graph_index1, graph_index2, joint);

        if wake_up {
            self.to_wake_up.insert(body1);
            self.to_wake_up.insert(body2);
        }

        ImpulseJointHandle(handle)
    }

    /// Retrieve all the enabled impulse joints happening between two active bodies.
    // NOTE: this is very similar to the code from NarrowPhase::select_active_interactions.
    pub(crate) fn select_active_interactions(
        &self,
        islands: &IslandManager,
        bodies: &RigidBodySet,
        out: &mut [Vec<JointIndex>],
    ) {
        for out_island in &mut out[..islands.num_islands()] {
            out_island.clear();
        }

        // FIXME: don't iterate through all the interactions.
        for (i, edge) in self.joint_graph.graph.edges.iter().enumerate() {
            let joint = &edge.weight;
            let rb1 = &bodies[joint.body1];
            let rb2 = &bodies[joint.body2];

            if joint.data.is_enabled()
                && (rb1.is_dynamic() || rb2.is_dynamic())
                && (!rb1.is_dynamic() || !rb1.is_sleeping())
                && (!rb2.is_dynamic() || !rb2.is_sleeping())
            {
                let island_index = if !rb1.is_dynamic() {
                    rb2.ids.active_island_id
                } else {
                    rb1.ids.active_island_id
                };

                out[island_index].push(i);
            }
        }
    }

    /// Removes a joint from this set.
    ///
    /// If `wake_up` is set to `true`, then the bodies attached to this joint will be
    /// automatically woken up.
    #[profiling::function]
    pub fn remove(&mut self, handle: ImpulseJointHandle, wake_up: bool) -> Option<ImpulseJoint> {
        let id = self.joint_ids.remove(handle.0)?;
        let endpoints = self.joint_graph.graph.edge_endpoints(id)?;

        if wake_up {
            if let Some(rb_handle) = self.joint_graph.graph.node_weight(endpoints.0) {
                self.to_wake_up.insert(*rb_handle);
            }
            if let Some(rb_handle) = self.joint_graph.graph.node_weight(endpoints.1) {
                self.to_wake_up.insert(*rb_handle);
            }
        }

        let removed_joint = self.joint_graph.graph.remove_edge(id);

        if let Some(edge) = self.joint_graph.graph.edge_weight(id) {
            self.joint_ids[edge.handle.0] = id;
        }

        removed_joint
    }

    /// Deletes all the impulse_joints attached to the given rigid-body.
    ///
    /// The provided rigid-body handle is not required to identify a rigid-body that
    /// is still contained by the `bodies` component set.
    /// Returns the (now invalid) handles of the removed impulse_joints.
    #[profiling::function]
    pub fn remove_joints_attached_to_rigid_body(
        &mut self,
        handle: RigidBodyHandle,
    ) -> Vec<ImpulseJointHandle> {
        let mut deleted = vec![];

        if let Some(deleted_id) = self
            .rb_graph_ids
            .remove(handle.0, InteractionGraph::<(), ()>::invalid_graph_index())
        {
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
                    deleted.push(to_delete_handle);
                    let to_delete_edge_id = self.joint_ids.remove(to_delete_handle.0).unwrap();
                    self.joint_graph.graph.remove_edge(to_delete_edge_id);

                    // Update the id of the edge which took the place of the deleted one.
                    if let Some(j) = self.joint_graph.graph.edge_weight_mut(to_delete_edge_id) {
                        self.joint_ids[j.handle.0] = to_delete_edge_id;
                    }

                    // Wake up the attached bodies.
                    self.to_wake_up.insert(h1);
                    self.to_wake_up.insert(h2);
                }

                if let Some(other) = self.joint_graph.remove_node(deleted_id) {
                    // One rigid-body joint graph index may have been invalidated
                    // so we need to update it.
                    self.rb_graph_ids.insert(other.0, deleted_id);
                }
            }
        }

        deleted
    }
}
