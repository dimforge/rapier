use crate::data::graph::EdgeIndex;
use crate::dynamics::{
    ImpulseJointSet, MultibodyJointSet, RigidBodyHandle, RigidBodySet, RigidBodyType,
};
use crate::geometry::{
    BroadPhaseBvh, BroadPhasePairEvent, ColliderHandle, ColliderSet, ContactData, ContactManifold,
    ContactManifoldData, ContactManifoldIndex, TemporaryInteractionIndex,
};
use crate::math::Real;
use crate::pipeline::{EventHandler, PhysicsHooks};
use crate::prelude::ContactPair;
use parry::query::{DefaultQueryDispatcher, PersistentQueryDispatcher};
use parry::utils::VecMap;
use std::collections::HashSet;
use std::ops::IndexMut;
use std::sync::Arc;

#[derive(Copy, Clone, Default)]
struct GraphNode<T> {
    prev: u32,
    next: u32,
    data: T,
}

impl<T> GraphNode<T> {
    fn new(prev: u32, next: u32, data: T) -> Self {
        Self { prev, next, data }
    }

    fn single(id: u32, data: T) -> Self {
        Self {
            prev: id,
            next: id,
            data,
        }
    }
}

struct BodyMeta {
    island: u32,
    handle: RigidBodyHandle,
    // TODO: store also the rigid-body type?
}

pub struct IslandManager2 {
    // #[cfg_attr(
    //     feature = "serde-serialize",
    //     serde(skip, default = "crate::geometry::default_persistent_query_dispatcher")
    // )]
    query_dispatcher: Arc<dyn PersistentQueryDispatcher<ContactManifoldData, ContactData>>,
    // TODO PERF: use Vec instead of an VecMap?
    bodies: VecMap<GraphNode<BodyMeta>>,
    // colliders: VecMap<GraphNode<()>>,
    contacts: Vec<GraphNode<ContactPair>>,

    // Set of collision pairs from the broad-phase, for fast lookup.
    // TODO: store this in the broad-phase directly?
    bf_pairs: HashSet<[ColliderHandle; 2]>,
    islands: Vec<Island>,
}

impl Default for IslandManager2 {
    fn default() -> Self {
        Self {
            query_dispatcher: Arc::new(DefaultQueryDispatcher),
            bodies: Default::default(),
            contacts: Default::default(),
            bf_pairs: Default::default(),
            islands: Default::default(),
        }
    }
}

#[derive(Copy, Clone)]
pub struct Island {
    sleeping: bool,

    body_head: u32,
    body_len: u32,

    // TODO: not sure having direct access to the collider indices
    //       is really useful. Maybe we can just access them through
    //       the rigid-bodies?
    //       Or maybe keep this just for the colliders with no parent?
    // collider_head: u32,
    // collider_len: u32,
    contact_head: u32,
    contact_len: u32,
}

impl Island {
    pub fn empty() -> Self {
        Self {
            sleeping: false,
            body_head: u32::MAX,
            body_len: 0,
            contact_head: u32::MAX,
            contact_len: 0,
        }
    }

    pub fn single(body_id: u32) -> Self {
        Self {
            body_head: body_id,
            body_len: 1,
            ..Self::empty()
        }
    }

    pub fn body_len(&self) -> usize {
        self.body_len as usize
    }

    pub fn clear(&mut self) {
        *self = Self::empty();
    }
}

impl IslandManager2 {
    /// The query dispatcher used by this narrow-phase to select the right collision-detection
    /// algorithms depending on the shape types.
    pub fn query_dispatcher(
        &self,
    ) -> &dyn PersistentQueryDispatcher<ContactManifoldData, ContactData> {
        &*self.query_dispatcher
    }

    pub fn num_islands(&self) -> usize {
        self.islands.len()
        // self.islands
        //     .iter()
        //     .filter(|isl| isl.contact_len > 0)
        //     .count()
    }
    pub fn add_body(&mut self, body_handle: RigidBodyHandle, body_type: RigidBodyType) {
        let body_id = body_handle.index();
        let island_id = if body_type.is_dynamic() {
            self.islands.len() as u32
        } else {
            u32::MAX
        };

        if !self.bodies.contains_key(body_id as usize) {
            self.bodies.insert(
                body_id as usize,
                GraphNode::single(
                    body_id,
                    BodyMeta {
                        island: island_id,
                        handle: body_handle,
                    },
                ),
            );

            if body_type.is_dynamic() {
                self.islands.push(Island::single(body_id));
            }
        }
    }

    // pub fn add_collider(&mut self, collider_handle: ColliderHandle) {
    //     let collider_id = collider_handle.index();
    //     self.colliders
    //         .insert(collider_id as usize, GraphNode::new(collider_id, ()));
    // }

    pub fn add_collision_pair(
        &mut self,
        mut pair: [ColliderHandle; 2],
        colliders: &ColliderSet,
        bodies: &mut RigidBodySet,
    ) {
        if pair[0].into_raw_parts().0 > pair[1].into_raw_parts().0 {
            // TODO: also swap based on the collision shape type?
            pair.swap(0, 1);
        }

        if self.bf_pairs.insert(pair) {
            // That pair didn’t already exist.
            // TODO PERF: dispatch the collision pair and cache the algorithm function ptr.
            let contact = ContactPair::new(pair[0], pair[1]);

            // Merge islands if needed.
            if let (Some(b1), Some(b2)) = (colliders[pair[0]].parent(), colliders[pair[1]].parent())
            {
                // TODO PERF: deal with the fact that non-dynamic bodies don’t merge islands.
                let island1 = self.bodies[b1.index() as usize].data.island;
                let island2 = self.bodies[b2.index() as usize].data.island;

                let result_island = if island1 == u32::MAX {
                    island2
                } else if island2 == u32::MAX {
                    island1
                } else {
                    self.merge_islands(bodies, island1, island2)
                };

                // Wireup the new pair in the result island.
                let island = &mut self.islands[result_island as usize];
                let contact_id = self.contacts.len();

                if island.contact_len == 0 {
                    // Single pair.
                    self.contacts
                        .push(GraphNode::single(contact_id as u32, contact));
                } else {
                    let head_replaced = island.contact_head;
                    let tail = self.contacts[head_replaced as usize].prev;

                    self.contacts[head_replaced as usize].prev = contact_id as u32;
                    self.contacts[tail as usize].next = contact_id as u32;

                    self.contacts
                        .push(GraphNode::new(tail, head_replaced, contact));
                }

                island.contact_len += 1;
                island.contact_head = contact_id as u32;
            } else {
                // TODO: support of parentless colliders not implemented yet.
                unimplemented!()
            }
        }
    }

    // Returns the index of the island that survived.
    // TODO PERF: get rid of the RigidBodySet here (needed for updating the rigid-body’s
    //            active_set_offset).
    fn merge_islands(
        &mut self,
        bodies: &mut RigidBodySet,
        mut island1: u32,
        mut island2: u32,
    ) -> u32 {
        if island1 == island2 {
            // Already part of the same island.
            return island1;
        }

        // Merge the smaller island into the bigger one.
        if self.islands[island1 as usize].body_len > self.islands[island2 as usize].body_len {
            std::mem::swap(&mut island1, &mut island2);
        }

        let source = self.islands[island2 as usize];
        let target = &mut self.islands[island1 as usize];

        // Update island indices.
        let mut curr_body = source.body_head;
        let mut curr_active_set_offset = target.body_len as usize;
        loop {
            let node = &mut self.bodies[curr_body as usize];
            node.data.island = island1;
            curr_body = node.next;

            bodies
                .get_mut_internal(node.data.handle)
                .unwrap()
                .ids
                .active_set_offset = curr_active_set_offset;
            curr_active_set_offset += 1;

            if curr_body == source.body_head {
                break;
            }
        }

        // Concatenate linked lists.
        Self::concat_lists(&mut self.bodies, target.body_head, source.body_head);
        Self::concat_lists(&mut self.contacts, target.contact_head, source.contact_head);

        // Special-cases where the target list is empty.
        if target.body_len == 0 {
            target.body_head = source.body_head;
        }
        if target.contact_len == 0 {
            target.contact_head = source.contact_head;
        }

        target.body_len += source.body_len;
        target.contact_len += source.contact_len;

        // Self::concat_lists(
        //     &mut self.colliders,
        //     target.collider_head,
        //     source.collider_head,
        // );
        // target.collider_len += source.collider_len;

        // TODO: do we want to make a linked-list of island ids too?
        self.islands[island2 as usize].clear();

        island1
    }

    fn concat_lists<T, Nodes>(nodes: &mut Nodes, target_head_id: u32, source_head_id: u32)
    where
        Nodes: IndexMut<usize, Output = GraphNode<T>>,
    {
        if source_head_id == u32::MAX || target_head_id == u32::MAX {
            // one of the lists is empty
            return;
        }

        let target_tail_id = nodes[target_head_id as usize].prev;
        let source_tail_id = nodes[source_head_id as usize].prev;
        nodes[target_tail_id as usize].next = source_head_id;
        nodes[source_head_id as usize].prev = target_tail_id;
        nodes[target_head_id as usize].prev = source_tail_id;
        nodes[source_tail_id as usize].next = target_head_id;
    }

    pub fn update_contacts(
        &mut self,
        prediction_distance: Real,
        dt: Real,
        broad_phase: &BroadPhaseBvh,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
        impulse_joints: &ImpulseJointSet,
        multibody_joints: &MultibodyJointSet,
        hooks: &dyn PhysicsHooks,
        events: &dyn EventHandler,
    ) {
        let mut num_updated = 0;
        let mut num_removed = 0;

        for island in &mut self.islands {
            if island.contact_len == 0 || island.sleeping {
                // Island is sleeping, nothing moved, nothing to update.
                continue;
            }

            let mut curr = island.contact_head;

            for _ in 0..island.contact_len {
                let pair = &mut self.contacts[curr as usize];

                if broad_phase
                    .narrow_phase_can_forget_pair(pair.data.collider1, pair.data.collider2)
                {
                    // The broad-phase is OK with us forgetting about this collision pair.
                    // Remove it.
                    num_removed += 1;
                    island.contact_len -= 1;

                    if island.contact_len == 0 {
                        // The contact list for this island is now empty!
                        // Stop the traversal here.
                        island.contact_head = u32::MAX;
                        break;
                    } else {
                        let prev = pair.prev;
                        let next = pair.next;

                        self.contacts[prev as usize].next = next;
                        self.contacts[next as usize].prev = prev;

                        // TODO: don’t leak the empty slot we created.
                        //       Perhaps we should just garbage-collect at the end
                        //       (and at the same time cache optimize) instead of trying
                        //       to do it right here (which needs to handle the special case
                        //       were the contact we are moving is the head of the list in another
                        //       island.
                        // self.contacts.swap_remove(curr as usize);
                        //
                        // // If a pair got swapped, update its list too.
                        // if curr as usize != self.contacts.len() {
                        //     let prev = self.contacts[curr as usize].prev;
                        //     let next = self.contacts[curr as usize].next;
                        //     self.contacts[prev as usize].next = curr;
                        //     self.contacts[next as usize].prev = curr;
                        // }

                        curr = next;
                    }
                } else {
                    num_updated += 1;
                    pair.data.update(
                        prediction_distance,
                        dt,
                        &*self.query_dispatcher,
                        bodies,
                        colliders,
                        impulse_joints,
                        multibody_joints,
                        hooks,
                        events,
                    );
                    curr = pair.next;
                }
            }
        }

        println!("Updated/removed: {}/{}", num_updated, num_removed);
    }
}

/*
 * Compatibility API.
 * This is a temporary API for integrating with the rest of the pre-existing physics pipeline.
 */

pub struct IslandBodies<'a> {
    manager: &'a IslandManager2,
    body_id: u32,
    body_len: u32,
}

impl Iterator for IslandBodies<'_> {
    type Item = RigidBodyHandle;
    fn next(&mut self) -> Option<Self::Item> {
        // Switch island if needed.
        if self.body_len == 0 {
            return None;
        }

        let body = &self.manager.bodies[self.body_id as usize];
        self.body_id = self.manager.bodies[self.body_id as usize].next;
        self.body_len -= 1;
        Some(body.data.handle)
    }
}

pub struct ActiveDynamicBodies<'a> {
    manager: &'a IslandManager2,
    island_id: usize,
    body_id: u32,
    body_len: u32,
}

impl Iterator for ActiveDynamicBodies<'_> {
    type Item = RigidBodyHandle;
    fn next(&mut self) -> Option<Self::Item> {
        // Switch island if needed.
        while self.body_len == 0 && self.island_id < self.manager.islands.len() {
            let island = &self.manager.islands[self.island_id];
            if !island.sleeping {
                self.body_len = island.body_len;
                self.body_id = island.body_head;
            }
            self.island_id += 1;
        }

        if self.body_len > 0 {
            let body = &self.manager.bodies[self.body_id as usize];
            self.body_id = self.manager.bodies[self.body_id as usize].next;
            self.body_len -= 1;
            Some(body.data.handle)
        } else {
            None
        }
    }
}

impl IslandManager2 {
    pub(crate) fn island(&self, island_id: u32) -> &Island {
        &self.islands[island_id as usize]
    }

    pub(crate) fn island_bodies(
        &self,
        island_id: u32,
    ) -> impl Iterator<Item = RigidBodyHandle> + '_ {
        let island = &self.islands[island_id as usize];
        IslandBodies {
            manager: self,
            body_id: island.body_head,
            body_len: island.body_len,
        }
    }

    pub(crate) fn active_dynamic_bodies(&self) -> ActiveDynamicBodies<'_> {
        // TODO: should only iterate on dynamic bodies.
        ActiveDynamicBodies {
            manager: self,
            island_id: 0,
            body_id: u32::MAX,
            body_len: 0,
        }
    }

    pub(crate) fn active_bodies(&self) -> ActiveDynamicBodies<'_> {
        // TODO: should only iterate on dynamic & kinematic bodies.
        ActiveDynamicBodies {
            manager: self,
            island_id: 0,
            body_id: u32::MAX,
            body_len: 0,
        }
    }

    pub(crate) fn handle_user_changes(
        &mut self,
        bodies: &RigidBodySet,
        modified_bodies: &[RigidBodyHandle],
    ) {
        for handle in modified_bodies.iter() {
            let rb = &bodies[*handle];
            self.add_body(*handle, rb.body_type);
        }
    }

    pub(crate) fn register_pairs(
        &mut self,
        colliders: &ColliderSet,
        bodies: &mut RigidBodySet,
        broad_phase_events: &[BroadPhasePairEvent],
    ) {
        for event in broad_phase_events {
            match event {
                BroadPhasePairEvent::AddPair(pair) => {
                    self.add_collision_pair([pair.collider1, pair.collider2], colliders, bodies);
                }
                BroadPhasePairEvent::DeletePair(_) => {}
            }
        }
    }

    pub(crate) fn select_active_contacts<'a>(
        &'a mut self,
        out_contact_pairs: &mut Vec<TemporaryInteractionIndex>,
        // FIXME: the 'static lifetime is a gross borrow-checker workaround.
        out_manifolds: &mut Vec<&'static mut ContactManifold>,
        out: &mut [Vec<ContactManifoldIndex>],
        out_island_ids: &mut Vec<usize>,
    ) {
        out_island_ids.clear();
        for out_island in &mut out[..self.islands.len()] {
            out_island.clear();
        }

        // TODO: if we thing implicit island grouping is still a good idea,
        //       we could do it here.
        for (island_id, island) in self.islands.iter().enumerate() {
            if island.contact_len == 0 || island.sleeping {
                continue;
            }

            let mut curr = island.contact_head;
            for _ in 0..island.contact_len {
                // SAFETY: this bypasses the borrow-checker so we can push the mutable contact
                //         manifold references directly. This is safe assuming all the indices
                //         in the contacts linked-lists are unique (as they should be).
                let pair: &'static mut GraphNode<ContactPair> =
                    unsafe { std::mem::transmute(&mut self.contacts[curr as usize]) };

                if !pair.data.manifolds.is_empty() {
                    for manifold in &mut pair.data.manifolds {
                        out[island_id].push(out_manifolds.len());
                        out_manifolds.push(manifold);
                    }

                    out_contact_pairs.push(EdgeIndex::new(curr));
                }

                curr = pair.next;
            }

            out_island_ids.push(island_id);
        }
    }
}
