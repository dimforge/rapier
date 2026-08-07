use crate::alloc_prelude::*;
use parry::utils::hashset::HashSet;

use super::ImpulseJoint;
use crate::geometry::{InteractionGraph, RigidBodyGraphIndex, TemporaryInteractionIndex};

use crate::data::Coarena;
use crate::data::arena::Arena;
use crate::dynamics::{
    GenericJoint, ImpulseJointHandle, IslandManager, RigidBodyHandle, RigidBodySet,
};

pub(crate) type JointIndex = usize;
pub(crate) type JointGraphEdge = crate::data::graph::Edge<ImpulseJoint>;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Default, Debug)]
/// The collection that stores all joints connecting rigid bodies in your physics world.
///
/// Joints constrain how two bodies can move relative to each other. This set manages
/// all joint instances (hinges, sliders, springs, etc.) using handles for safe access.
///
/// # Common joint types
/// - [`FixedJoint`](crate::dynamics::FixedJoint): Weld two bodies together
/// - [`RevoluteJoint`](crate::dynamics::RevoluteJoint): Hinge (rotation around axis)
/// - [`PrismaticJoint`](crate::dynamics::PrismaticJoint): Slider (translation along axis)
/// - [`SpringJoint`](crate::dynamics::SpringJoint): Elastic connection
/// - [`RopeJoint`](crate::dynamics::RopeJoint): Maximum distance limit
///
/// # Example
/// ```
/// # use rapier3d::prelude::*;
/// # let mut bodies = RigidBodySet::new();
/// # let body1 = bodies.insert(RigidBodyBuilder::dynamic());
/// # let body2 = bodies.insert(RigidBodyBuilder::dynamic());
/// let mut joints = ImpulseJointSet::new();
///
/// // Create a hinge connecting two bodies
/// let joint = RevoluteJointBuilder::new(Vector::Y)
///     .local_anchor1(Vector::new(1.0, 0.0, 0.0))
///     .local_anchor2(Vector::new(-1.0, 0.0, 0.0))
///     .build();
/// let handle = joints.insert(body1, body2, joint, true);
/// ```
pub struct ImpulseJointSet {
    rb_graph_ids: Coarena<RigidBodyGraphIndex>,
    /// Map joint handles to edge ids on the graph.
    joint_ids: Arena<TemporaryInteractionIndex>,
    joint_graph: InteractionGraph<RigidBodyHandle, ImpulseJoint>,
    /// A set of rigid-body handles to wake-up during the next timestep.
    pub(crate) to_wake_up: HashSet<RigidBodyHandle>,
    /// A set of rigid-body pairs to join in the island manager during the next timestep.
    pub(crate) to_join: HashSet<(RigidBodyHandle, RigidBodyHandle)>,
    /// Persistent-island connectivity events (joint created/removed/rewired),
    /// drained at the start of the next timestep, in order.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) island_events: Vec<crate::dynamics::ImpulseJointIslandEvent>,
    /// Bumped by every mutation that can affect the solver's joint constraint assembly (joint
    /// insertion/removal, mutable joint access, user-changes to a rigid-body with attached joints).
    /// The solver reuses its joint assembly while this, the joint list, and the island epoch are unchanged.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    pub(crate) assembly_epoch: u32,
    /// The `(active_set_epoch, assembly_epoch)` the last [`Self::select_active_interactions`] ran
    /// with: while both are unchanged the selection (and the solver-body ids it stamps) is
    /// identical, so the caller's previous output is reused untouched.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    selection_epochs: Option<(u32, u32)>,
}

impl ImpulseJointSet {
    /// Creates a new empty set of impulse_joints.
    pub fn new() -> Self {
        Self {
            rb_graph_ids: Coarena::new(),
            joint_ids: Arena::new(),
            joint_graph: InteractionGraph::new(),
            to_wake_up: HashSet::default(),
            to_join: HashSet::default(),
            island_events: Vec::new(),
            assembly_epoch: 0,
            selection_epochs: None,
        }
    }

    /// Drops the memo of the last [`Self::select_active_interactions`], forcing the next
    /// call to recompute into the caller's buffer.
    pub(crate) fn invalidate_selection_memo(&mut self) {
        self.selection_epochs = None;
    }

    /// Marks the solver-facing joint assembly inputs as changed. See
    /// [`Self::assembly_epoch`].
    pub(crate) fn bump_assembly_epoch(&mut self) {
        self.assembly_epoch = self.assembly_epoch.wrapping_add(1);
        self.selection_epochs = None;
    }

    /// `true` if this body has (or recently had) impulse joints attached.
    pub(crate) fn body_may_have_joints(&self, body: crate::dynamics::RigidBodyHandle) -> bool {
        self.rb_graph_ids.get(body.0).is_some_and(|id| {
            InteractionGraph::<RigidBodyHandle, ImpulseJoint>::is_graph_index_valid(*id)
        })
    }

    /// Returns how many joints are currently in this collection.
    pub fn len(&self) -> usize {
        self.joint_graph.graph.edges.len()
    }

    /// Returns `true` if there are no joints in this collection.
    pub fn is_empty(&self) -> bool {
        self.joint_graph.graph.edges.is_empty()
    }

    /// Returns the internal graph structure (nodes=bodies, edges=joints).
    ///
    /// Advanced usage - most users should use `attached_joints()` instead.
    pub fn joint_graph(&self) -> &InteractionGraph<RigidBodyHandle, ImpulseJoint> {
        &self.joint_graph
    }

    /// Returns all joints connecting two specific bodies.
    ///
    /// Usually returns 0 or 1 joint, but multiple joints can connect the same pair.
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut bodies = RigidBodySet::new();
    /// # let mut joints = ImpulseJointSet::new();
    /// # let body1 = bodies.insert(RigidBodyBuilder::dynamic());
    /// # let body2 = bodies.insert(RigidBodyBuilder::dynamic());
    /// # let joint = RevoluteJointBuilder::new(Vector::Y);
    /// # joints.insert(body1, body2, joint, true);
    /// for (handle, joint) in joints.joints_between(body1, body2) {
    ///     println!("Found joint {:?}", handle);
    /// }
    /// ```
    pub fn joints_between(
        &self,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
    ) -> impl Iterator<Item = (ImpulseJointHandle, &ImpulseJoint)> {
        self.rb_graph_ids
            .get(body1.0)
            .zip(self.rb_graph_ids.get(body2.0))
            .into_iter()
            .flat_map(move |(id1, id2)| self.joint_graph.interactions_between(*id1, *id2))
            .map(|inter| (inter.2.handle, inter.2))
    }

    /// Returns all joints attached to a specific body.
    ///
    /// Each result is `(body1, body2, joint_handle, joint)` where one of the bodies
    /// matches the queried body.
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut bodies = RigidBodySet::new();
    /// # let mut joints = ImpulseJointSet::new();
    /// # let body_handle = bodies.insert(RigidBodyBuilder::dynamic());
    /// # let other_body = bodies.insert(RigidBodyBuilder::dynamic());
    /// # let joint = RevoluteJointBuilder::new(Vector::Y);
    /// # joints.insert(body_handle, other_body, joint, true);
    /// for (b1, b2, j_handle, joint) in joints.attached_joints(body_handle) {
    ///     println!("Body connected to {:?} via {:?}", b2, j_handle);
    /// }
    /// ```
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
        self.bump_assembly_epoch();
        self.rb_graph_ids.get(body.0).into_iter().for_each(|id| {
            for inter in self.joint_graph.interactions_with_mut(*id) {
                (f)(inter.0, inter.1, inter.3.handle, inter.3)
            }
        })
    }

    /// Returns only the enabled joints attached to a body.
    ///
    /// Same as `attached_joints()` but filters out disabled joints.
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

    /// Checks if the given joint handle is valid (joint still exists).
    pub fn contains(&self, handle: ImpulseJointHandle) -> bool {
        self.joint_ids.contains(handle.0)
    }

    /// Returns a read-only reference to the joint with the given handle.
    pub fn get(&self, handle: ImpulseJointHandle) -> Option<&ImpulseJoint> {
        let id = self.joint_ids.get(handle.0)?;
        self.joint_graph.graph.edge_weight(*id)
    }

    /// Returns a mutable reference to the joint with the given handle.
    ///
    /// # Parameters
    /// * `wake_up_connected_bodies` - If `true`, wakes up both bodies connected by this joint
    pub fn get_mut(
        &mut self,
        handle: ImpulseJointHandle,
        wake_up_connected_bodies: bool,
    ) -> Option<&mut ImpulseJoint> {
        self.bump_assembly_epoch();
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

    /// Gets a joint by index without knowing the generation (advanced/unsafe).
    ///
    /// ⚠️ **Prefer `get()` instead!** This bypasses generation checks.
    /// See [`RigidBodySet::get_unknown_gen`] for details on the ABA problem.
    pub fn get_unknown_gen(&self, i: u32) -> Option<(&ImpulseJoint, ImpulseJointHandle)> {
        let (id, handle) = self.joint_ids.get_unknown_gen(i)?;
        Some((
            self.joint_graph.graph.edge_weight(*id)?,
            ImpulseJointHandle(handle),
        ))
    }

    /// Gets a mutable joint by index without knowing the generation (advanced/unsafe).
    ///
    /// ⚠️ **Prefer `get_mut()` instead!** This bypasses generation checks.
    pub fn get_unknown_gen_mut(
        &mut self,
        i: u32,
    ) -> Option<(&mut ImpulseJoint, ImpulseJointHandle)> {
        self.bump_assembly_epoch();
        let (id, handle) = self.joint_ids.get_unknown_gen(i)?;
        Some((
            self.joint_graph.graph.edge_weight_mut(*id)?,
            ImpulseJointHandle(handle),
        ))
    }

    /// Iterates over all joints in this collection.
    ///
    /// Each iteration yields `(joint_handle, &joint)`.
    pub fn iter(&self) -> impl Iterator<Item = (ImpulseJointHandle, &ImpulseJoint)> {
        self.joint_graph
            .graph
            .edges
            .iter()
            .map(|e| (e.weight.handle, &e.weight))
    }

    /// Iterates over all joints with mutable access.
    ///
    /// Each iteration yields `(joint_handle, &mut joint)`.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ImpulseJointHandle, &mut ImpulseJoint)> {
        self.bump_assembly_epoch();
        self.joint_graph
            .graph
            .edges
            .iter_mut()
            .map(|e| (e.weight.handle, &mut e.weight))
    }

    pub(crate) fn joints_mut(&mut self) -> &mut [JointGraphEdge] {
        &mut self.joint_graph.graph.edges[..]
    }

    /// Adds a joint connecting two bodies and returns its handle.
    ///
    /// The joint constrains how the two bodies can move relative to each other.
    ///
    /// # Parameters
    /// * `body1`, `body2` - The two bodies to connect
    /// * `data` - The joint configuration (FixedJoint, RevoluteJoint, etc.)
    /// * `wake_up` - If `true`, wakes up both bodies
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut bodies = RigidBodySet::new();
    /// # let mut joints = ImpulseJointSet::new();
    /// # let body1 = bodies.insert(RigidBodyBuilder::dynamic());
    /// # let body2 = bodies.insert(RigidBodyBuilder::dynamic());
    /// let joint = RevoluteJointBuilder::new(Vector::Y)
    ///     .local_anchor1(Vector::new(1.0, 0.0, 0.0))
    ///     .local_anchor2(Vector::new(-1.0, 0.0, 0.0))
    ///     .build();
    /// let handle = joints.insert(body1, body2, joint, true);
    /// ```
    #[profiling::function]
    pub fn insert(
        &mut self,
        body1: RigidBodyHandle,
        body2: RigidBodyHandle,
        data: impl Into<GenericJoint>,
        wake_up: bool,
    ) -> ImpulseJointHandle {
        let data = data.into();
        let joint_enabled = data.is_enabled();
        self.bump_assembly_epoch();
        let handle = self.joint_ids.insert(0.into());
        let joint = ImpulseJoint {
            body1,
            body2,
            data,
            impulses: Default::default(),
            handle: ImpulseJointHandle(handle),
            solver_body_ids: [u32::MAX; 2],
            solver_color: crate::geometry::contact_pair::SOLVER_COLOR_UNCOLORED,
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

        self.to_join.insert((body1, body2));
        if joint_enabled {
            self.island_events
                .push(crate::dynamics::ImpulseJointIslandEvent::Link {
                    handle: ImpulseJointHandle(handle),
                    body1,
                    body2,
                });
        }

        ImpulseJointHandle(handle)
    }

    /// Rewires an existing joint to a new pair of bodies, preserving its
    /// [`ImpulseJointHandle`], its `GenericJoint` configuration, and its
    /// warm-started impulses.
    ///
    /// Unlike directly mutating `body1`/`body2` (which isn't possible anyway
    /// since those fields are crate-private), this also updates the
    /// interaction graph and schedules the new pair for island merging so
    /// the solver, island manager, and joint constraint builder stay in
    /// sync. Use this when retargeting a persistent joint — e.g. a mouse
    /// "pick" joint that is pre-allocated at startup and reattached to
    /// whatever body the user grabs.
    ///
    /// # Parameters
    /// * `handle` - The joint to rewire. If the handle is invalid this
    ///   returns `None`.
    /// * `new_body1`, `new_body2` - The new endpoints.
    /// * `wake_up` - If `true`, wakes up both the previous and new endpoints.
    ///
    /// Returns a mutable reference to the updated joint, or `None` if the
    /// handle was stale. When `new_body1`/`new_body2` match the joint's
    /// current endpoints, this is a no-op and returns the joint unchanged.
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut bodies = RigidBodySet::new();
    /// # let mut joints = ImpulseJointSet::new();
    /// # let body1 = bodies.insert(RigidBodyBuilder::dynamic());
    /// # let body2 = bodies.insert(RigidBodyBuilder::dynamic());
    /// # let body3 = bodies.insert(RigidBodyBuilder::dynamic());
    /// # let joint = RevoluteJointBuilder::new(Vector::Y).build();
    /// let joint_handle = joints.insert(body1, body2, joint, true);
    /// // Swap body2 for body3 without losing the handle or the joint data.
    /// joints.set_bodies(joint_handle, body1, body3, true);
    /// ```
    #[profiling::function]
    pub fn set_bodies(
        &mut self,
        handle: ImpulseJointHandle,
        new_body1: RigidBodyHandle,
        new_body2: RigidBodyHandle,
        wake_up: bool,
    ) -> Option<&mut ImpulseJoint> {
        self.bump_assembly_epoch();
        let edge_id = *self.joint_ids.get(handle.0)?;

        // Early-out when the endpoints haven't actually changed.
        let (old_body1, old_body2) = {
            let joint = self.joint_graph.graph.edge_weight(edge_id)?;
            (joint.body1, joint.body2)
        };
        if old_body1 == new_body1 && old_body2 == new_body2 {
            return self.joint_graph.graph.edge_weight_mut(edge_id);
        }

        // Detach the edge from its current endpoints. `remove_edge` uses
        // `swap_remove`, so another edge may have taken `edge_id`'s slot —
        // patch its handle→edge mapping exactly like `ImpulseJointSet::remove`.
        let mut joint = self.joint_graph.graph.remove_edge(edge_id)?;
        if let Some(swapped) = self.joint_graph.graph.edge_weight(edge_id) {
            self.joint_ids[swapped.handle.0] = edge_id;
        }

        // Ensure both new endpoints have graph nodes (same dance `insert`
        // does for first-time endpoints).
        let default_id = InteractionGraph::<(), ()>::invalid_graph_index();
        let mut graph_index1 = *self
            .rb_graph_ids
            .ensure_element_exist(new_body1.0, default_id);
        let mut graph_index2 = *self
            .rb_graph_ids
            .ensure_element_exist(new_body2.0, default_id);
        if !InteractionGraph::<RigidBodyHandle, ImpulseJoint>::is_graph_index_valid(graph_index1) {
            graph_index1 = self.joint_graph.graph.add_node(new_body1);
            self.rb_graph_ids.insert(new_body1.0, graph_index1);
        }
        if !InteractionGraph::<RigidBodyHandle, ImpulseJoint>::is_graph_index_valid(graph_index2) {
            graph_index2 = self.joint_graph.graph.add_node(new_body2);
            self.rb_graph_ids.insert(new_body2.0, graph_index2);
        }

        joint.body1 = new_body1;
        joint.body2 = new_body2;
        let new_edge_id = self.joint_graph.add_edge(graph_index1, graph_index2, joint);
        self.joint_ids[handle.0] = new_edge_id;

        if wake_up {
            self.to_wake_up.insert(old_body1);
            self.to_wake_up.insert(old_body2);
            self.to_wake_up.insert(new_body1);
            self.to_wake_up.insert(new_body2);
        }
        self.to_join.insert((new_body1, new_body2));
        self.island_events
            .push(crate::dynamics::ImpulseJointIslandEvent::Unlink { handle });
        if self
            .joint_graph
            .graph
            .edge_weight(new_edge_id)
            .is_some_and(|j| j.data.is_enabled())
        {
            self.island_events
                .push(crate::dynamics::ImpulseJointIslandEvent::Link {
                    handle,
                    body1: new_body1,
                    body2: new_body2,
                });
        }

        self.joint_graph.graph.edge_weight_mut(new_edge_id)
    }

    /// Retrieve all the enabled impulse joints happening between two active bodies.
    // NOTE: this is very similar to the code from NarrowPhase::select_active_interactions.
    pub(crate) fn select_active_interactions(
        &mut self,
        islands: &IslandManager,
        bodies: &RigidBodySet,
        out: &mut Vec<JointIndex>,
    ) {
        // The selection depends only on the active-set epoch and the assembly epoch: while both
        // are unchanged, `out` (assumed to be the previous call's output, which the physics
        // pipeline keeps around) and the stamped solver-body ids are still exact.
        let epochs = (islands.active_set_epoch, self.assembly_epoch);
        if self.selection_epochs == Some(epochs) {
            return;
        }
        self.selection_epochs = Some(epochs);

        out.clear();

        // Only iterate joints adjacent to an active body instead of the whole graph: any joint
        // selected below has at least one awake dynamic/kinematic body, so the neighborhood walk
        // is exhaustive — and much smaller when most of the scene is asleep.
        let mut candidates: Vec<u32> = Vec::new();

        // When most bodies are awake, walking the graph adjacency (pointer-chasing) and sorting
        // costs more than the linear edge scan it replaces — just visit every joint and let the
        // per-joint checks below skip inactive ones.
        let num_active = islands.active_bodies().count();
        if num_active * 2 >= bodies.len() {
            candidates.extend(0..self.joint_graph.graph.edges.len() as u32);
        } else {
            for handle in islands.active_bodies() {
                if let Some(gid) = self.rb_graph_ids.get(handle.0) {
                    for edge in self.joint_graph.graph.edges(*gid) {
                        candidates.push(edge.id().index() as u32);
                    }
                }
            }

            // Sorting + deduplicating guarantees each joint is visited exactly once, in
            // the same deterministic edge-index order as a full graph scan.
            candidates.sort_unstable();
            candidates.dedup();
        }

        for i in candidates.iter().map(|id| *id as usize) {
            let edge = &mut self.joint_graph.graph.edges[i];
            let joint = &mut edge.weight;
            let rb1 = &bodies[joint.body1];
            let rb2 = &bodies[joint.body2];

            if joint.data.is_enabled()
                && (rb1.is_dynamic_or_kinematic() || rb2.is_dynamic_or_kinematic())
                && (!rb1.is_dynamic_or_kinematic() || !rb1.is_sleeping())
                && (!rb2.is_dynamic_or_kinematic() || !rb2.is_sleeping())
            {
                // Stamp the solver-body ids while both body cache lines are hot so the solver's
                // coloring/grouping and jacobian generation don't re-read the rigid-body set. `u32::MAX`
                // marks a world-attached side (fixed, or defensively sleeping — its `active_set_id` indexes another island).
                let solver_id = |rb: &crate::dynamics::RigidBody| {
                    if rb.is_dynamic_or_kinematic() && !rb.is_sleeping() {
                        rb.ids.active_set_id
                    } else {
                        u32::MAX
                    }
                };
                joint.solver_body_ids = [solver_id(rb1), solver_id(rb2)];
                out.push(i);
            }
        }
    }

    /// Removes a joint from the world.
    ///
    /// Returns the removed joint if it existed, or `None` if the handle was invalid.
    ///
    /// # Parameters
    /// * `wake_up` - If `true`, wakes up both bodies that were connected by this joint
    ///
    /// # Example
    /// ```
    /// # use rapier3d::prelude::*;
    /// # let mut bodies = RigidBodySet::new();
    /// # let mut joints = ImpulseJointSet::new();
    /// # let body1 = bodies.insert(RigidBodyBuilder::dynamic());
    /// # let body2 = bodies.insert(RigidBodyBuilder::dynamic());
    /// # let joint = RevoluteJointBuilder::new(Vector::Y).build();
    /// # let joint_handle = joints.insert(body1, body2, joint, true);
    /// if let Some(joint) = joints.remove(joint_handle, true) {
    ///     println!("Removed joint between {:?} and {:?}", joint.body1(), joint.body2());
    /// }
    /// ```
    #[profiling::function]
    pub fn remove(&mut self, handle: ImpulseJointHandle, wake_up: bool) -> Option<ImpulseJoint> {
        self.bump_assembly_epoch();
        let id = self.joint_ids.remove(handle.0)?;
        let endpoints = self.joint_graph.graph.edge_endpoints(id)?;

        if wake_up {
            for endpoint in [endpoints.0, endpoints.1] {
                if let Some(rb_handle) = self.joint_graph.graph.node_weight(endpoint) {
                    self.to_wake_up.insert(*rb_handle);
                }
            }
        }

        let removed_joint = self.joint_graph.graph.remove_edge(id);

        if let Some(edge) = self.joint_graph.graph.edge_weight(id) {
            self.joint_ids[edge.handle.0] = id;
        }

        self.island_events
            .push(crate::dynamics::ImpulseJointIslandEvent::Unlink { handle });

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
                    self.island_events
                        .push(crate::dynamics::ImpulseJointIslandEvent::Unlink {
                            handle: to_delete_handle,
                        });
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
