#[cfg(feature = "parallel")]
use rayon::prelude::*;

use crate::data::arena::Arena;
use crate::dynamics::{BodyStatus, Joint, RigidBody};
use crate::geometry::{ColliderSet, ContactPair, InteractionGraph};
use crossbeam::channel::{Receiver, Sender};
use std::ops::{Deref, DerefMut, Index, IndexMut};

/// A mutable reference to a rigid-body.
pub struct RigidBodyMut<'a> {
    rb: &'a mut RigidBody,
    was_sleeping: bool,
    handle: RigidBodyHandle,
    sender: &'a Sender<RigidBodyHandle>,
}

impl<'a> RigidBodyMut<'a> {
    fn new(
        handle: RigidBodyHandle,
        rb: &'a mut RigidBody,
        sender: &'a Sender<RigidBodyHandle>,
    ) -> Self {
        Self {
            was_sleeping: rb.is_sleeping(),
            handle,
            sender,
            rb,
        }
    }
}

impl<'a> Deref for RigidBodyMut<'a> {
    type Target = RigidBody;
    fn deref(&self) -> &RigidBody {
        &*self.rb
    }
}

impl<'a> DerefMut for RigidBodyMut<'a> {
    fn deref_mut(&mut self) -> &mut RigidBody {
        self.rb
    }
}

impl<'a> Drop for RigidBodyMut<'a> {
    fn drop(&mut self) {
        if self.was_sleeping && !self.rb.is_sleeping() {
            self.sender.send(self.handle).unwrap();
        }
    }
}

/// The unique handle of a rigid body added to a `RigidBodySet`.
pub type RigidBodyHandle = crate::data::arena::Index;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A pair of rigid body handles.
pub struct BodyPair {
    /// The first rigid body handle.
    pub body1: RigidBodyHandle,
    /// The second rigid body handle.
    pub body2: RigidBodyHandle,
}

impl BodyPair {
    pub(crate) fn new(body1: RigidBodyHandle, body2: RigidBodyHandle) -> Self {
        BodyPair { body1, body2 }
    }

    pub(crate) fn swap(self) -> Self {
        Self::new(self.body2, self.body1)
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A set of rigid bodies that can be handled by a physics pipeline.
pub struct RigidBodySet {
    // NOTE: the pub(crate) are needed by the broad phase
    // to avoid borrowing issues. It is also needed for
    // parallelism because the `Receiver` breaks the Sync impl.
    // Could we avoid this?
    pub(crate) bodies: Arena<RigidBody>,
    pub(crate) active_dynamic_set: Vec<RigidBodyHandle>,
    pub(crate) active_kinematic_set: Vec<RigidBodyHandle>,
    // Set of inactive bodies which have been modified.
    // This typically include static bodies which have been modified.
    pub(crate) modified_inactive_set: Vec<RigidBodyHandle>,
    pub(crate) active_islands: Vec<usize>,
    active_set_timestamp: u32,
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    can_sleep: Vec<RigidBodyHandle>, // Workspace.
    #[cfg_attr(feature = "serde-serialize", serde(skip))]
    stack: Vec<RigidBodyHandle>, // Workspace.
    #[cfg_attr(
        feature = "serde-serialize",
        serde(skip, default = "crossbeam::channel::unbounded")
    )]
    activation_channel: (Sender<RigidBodyHandle>, Receiver<RigidBodyHandle>),
}

impl RigidBodySet {
    /// Create a new empty set of rigid bodies.
    pub fn new() -> Self {
        RigidBodySet {
            bodies: Arena::new(),
            active_dynamic_set: Vec::new(),
            active_kinematic_set: Vec::new(),
            modified_inactive_set: Vec::new(),
            active_islands: Vec::new(),
            active_set_timestamp: 0,
            can_sleep: Vec::new(),
            stack: Vec::new(),
            activation_channel: crossbeam::channel::unbounded(),
        }
    }

    /// An always-invalid rigid-body handle.
    pub fn invalid_handle() -> RigidBodyHandle {
        RigidBodyHandle::from_raw_parts(crate::INVALID_USIZE, crate::INVALID_U64)
    }

    /// The number of rigid bodies on this set.
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    pub(crate) fn activate(&mut self, handle: RigidBodyHandle) {
        let mut rb = &mut self.bodies[handle];
        match rb.body_status {
            // XXX: this case should only concern the dynamic bodies.
            // For static bodies we should use the modified_inactive_set, or something
            // similar. Right now we do this for static bodies as well so the broad-phase
            // takes them into account the first time they are inserted.
            BodyStatus::Dynamic | BodyStatus::Static => {
                if self.active_dynamic_set.get(rb.active_set_id) != Some(&handle) {
                    rb.active_set_id = self.active_dynamic_set.len();
                    self.active_dynamic_set.push(handle);
                }
            }
            BodyStatus::Kinematic => {
                if self.active_kinematic_set.get(rb.active_set_id) != Some(&handle) {
                    rb.active_set_id = self.active_kinematic_set.len();
                    self.active_kinematic_set.push(handle);
                }
            }
        }
    }

    /// Is the given body handle valid?
    pub fn contains(&self, handle: RigidBodyHandle) -> bool {
        self.bodies.contains(handle)
    }

    /// Insert a rigid body into this set and retrieve its handle.
    pub fn insert(&mut self, rb: RigidBody) -> RigidBodyHandle {
        let handle = self.bodies.insert(rb);
        let rb = &mut self.bodies[handle];

        if !rb.is_sleeping() && rb.is_dynamic() {
            rb.active_set_id = self.active_dynamic_set.len();
            self.active_dynamic_set.push(handle);
        }

        if rb.is_kinematic() {
            rb.active_set_id = self.active_kinematic_set.len();
            self.active_kinematic_set.push(handle);
        }

        if !rb.is_dynamic() {
            self.modified_inactive_set.push(handle);
        }

        handle
    }

    pub(crate) fn num_islands(&self) -> usize {
        self.active_islands.len() - 1
    }

    pub(crate) fn remove_internal(&mut self, handle: RigidBodyHandle) -> Option<RigidBody> {
        let rb = self.bodies.remove(handle)?;
        let mut active_sets = [&mut self.active_kinematic_set, &mut self.active_dynamic_set];

        for active_set in &mut active_sets {
            if active_set.get(rb.active_set_id) == Some(&handle) {
                active_set.swap_remove(rb.active_set_id);

                if let Some(replacement) = active_set.get(rb.active_set_id) {
                    self.bodies[*replacement].active_set_id = rb.active_set_id;
                }
            }
        }

        Some(rb)
    }

    /// Forces the specified rigid-body to wake up if it is dynamic.
    pub fn wake_up(&mut self, handle: RigidBodyHandle) {
        if let Some(rb) = self.bodies.get_mut(handle) {
            // TODO: what about kinematic bodies?
            if rb.is_dynamic() {
                rb.wake_up();

                if self.active_dynamic_set.get(rb.active_set_id) != Some(&handle) {
                    rb.active_set_id = self.active_dynamic_set.len();
                    self.active_dynamic_set.push(handle);
                }
            }
        }
    }

    /// Gets the rigid-body with the given handle without a known generation.
    ///
    /// This is useful when you know you want the rigid-body at position `i` but
    /// don't know what is its current generation number. Generation numbers are
    /// used to protect from the ABA problem because the rigid-body position `i`
    /// are recycled between two insertion and a removal.
    ///
    /// Using this is discouraged in favor of `self.get(handle)` which does not
    /// suffer form the ABA problem.
    pub fn get_unknown_gen(&self, i: usize) -> Option<(&RigidBody, RigidBodyHandle)> {
        self.bodies.get_unknown_gen(i)
    }

    /// Gets a mutable reference to the rigid-body with the given handle without a known generation.
    ///
    /// This is useful when you know you want the rigid-body at position `i` but
    /// don't know what is its current generation number. Generation numbers are
    /// used to protect from the ABA problem because the rigid-body position `i`
    /// are recycled between two insertion and a removal.
    ///
    /// Using this is discouraged in favor of `self.get_mut(handle)` which does not
    /// suffer form the ABA problem.
    pub fn get_unknown_gen_mut(&mut self, i: usize) -> Option<(RigidBodyMut, RigidBodyHandle)> {
        let sender = &self.activation_channel.0;
        self.bodies
            .get_unknown_gen_mut(i)
            .map(|(rb, handle)| (RigidBodyMut::new(handle, rb, sender), handle))
    }

    /// Gets the rigid-body with the given handle.
    pub fn get(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        self.bodies.get(handle)
    }

    /// Gets a mutable reference to the rigid-body with the given handle.
    pub fn get_mut(&mut self, handle: RigidBodyHandle) -> Option<RigidBodyMut> {
        let sender = &self.activation_channel.0;
        self.bodies
            .get_mut(handle)
            .map(|rb| RigidBodyMut::new(handle, rb, sender))
    }

    pub(crate) fn get_mut_internal(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        self.bodies.get_mut(handle)
    }

    pub(crate) fn get2_mut_internal(
        &mut self,
        h1: RigidBodyHandle,
        h2: RigidBodyHandle,
    ) -> (Option<&mut RigidBody>, Option<&mut RigidBody>) {
        self.bodies.get2_mut(h1, h2)
    }

    /// Iterates through all the rigid-bodies on this set.
    pub fn iter(&self) -> impl Iterator<Item = (RigidBodyHandle, &RigidBody)> {
        self.bodies.iter()
    }

    /// Iterates mutably through all the rigid-bodies on this set.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (RigidBodyHandle, RigidBodyMut)> {
        let sender = &self.activation_channel.0;
        self.bodies
            .iter_mut()
            .map(move |(h, rb)| (h, RigidBodyMut::new(h, rb, sender)))
    }

    /// Iter through all the active dynamic rigid-bodies on this set.
    pub fn iter_active_dynamic<'a>(
        &'a self,
    ) -> impl Iterator<Item = (RigidBodyHandle, &'a RigidBody)> {
        let bodies: &'a _ = &self.bodies;
        self.active_dynamic_set
            .iter()
            .filter_map(move |h| Some((*h, bodies.get(*h)?)))
    }

    #[cfg(not(feature = "parallel"))]
    pub(crate) fn iter_active_island<'a>(
        &'a self,
        island_id: usize,
    ) -> impl Iterator<Item = (RigidBodyHandle, &'a RigidBody)> {
        let island_range = self.active_islands[island_id]..self.active_islands[island_id + 1];
        let bodies: &'a _ = &self.bodies;
        self.active_dynamic_set[island_range]
            .iter()
            .filter_map(move |h| Some((*h, bodies.get(*h)?)))
    }

    #[inline(always)]
    pub(crate) fn foreach_active_body_mut_internal(
        &mut self,
        mut f: impl FnMut(RigidBodyHandle, &mut RigidBody),
    ) {
        for handle in &self.active_dynamic_set {
            if let Some(rb) = self.bodies.get_mut(*handle) {
                f(*handle, rb)
            }
        }

        for handle in &self.active_kinematic_set {
            if let Some(rb) = self.bodies.get_mut(*handle) {
                f(*handle, rb)
            }
        }
    }

    #[inline(always)]
    pub(crate) fn foreach_active_dynamic_body_mut_internal(
        &mut self,
        mut f: impl FnMut(RigidBodyHandle, &mut RigidBody),
    ) {
        for handle in &self.active_dynamic_set {
            if let Some(rb) = self.bodies.get_mut(*handle) {
                f(*handle, rb)
            }
        }
    }

    #[inline(always)]
    pub(crate) fn foreach_active_kinematic_body_mut_internal(
        &mut self,
        mut f: impl FnMut(RigidBodyHandle, &mut RigidBody),
    ) {
        for handle in &self.active_kinematic_set {
            if let Some(rb) = self.bodies.get_mut(*handle) {
                f(*handle, rb)
            }
        }
    }

    #[inline(always)]
    #[cfg(not(feature = "parallel"))]
    pub(crate) fn foreach_active_island_body_mut_internal(
        &mut self,
        island_id: usize,
        mut f: impl FnMut(RigidBodyHandle, &mut RigidBody),
    ) {
        let island_range = self.active_islands[island_id]..self.active_islands[island_id + 1];
        for handle in &self.active_dynamic_set[island_range] {
            if let Some(rb) = self.bodies.get_mut(*handle) {
                f(*handle, rb)
            }
        }
    }

    #[cfg(feature = "parallel")]
    #[inline(always)]
    #[allow(dead_code)]
    pub(crate) fn foreach_active_island_body_mut_internal_parallel(
        &mut self,
        island_id: usize,
        f: impl Fn(RigidBodyHandle, &mut RigidBody) + Send + Sync,
    ) {
        use std::sync::atomic::Ordering;

        let island_range = self.active_islands[island_id]..self.active_islands[island_id + 1];
        let bodies = std::sync::atomic::AtomicPtr::new(&mut self.bodies as *mut _);
        self.active_dynamic_set[island_range]
            .par_iter()
            .for_each_init(
                || bodies.load(Ordering::Relaxed),
                |bodies, handle| {
                    let bodies: &mut Arena<RigidBody> = unsafe { std::mem::transmute(*bodies) };
                    if let Some(rb) = bodies.get_mut(*handle) {
                        f(*handle, rb)
                    }
                },
            );
    }

    // pub(crate) fn active_dynamic_set(&self) -> &[RigidBodyHandle] {
    //     &self.active_dynamic_set
    // }

    pub(crate) fn active_island_range(&self, island_id: usize) -> std::ops::Range<usize> {
        self.active_islands[island_id]..self.active_islands[island_id + 1]
    }

    pub(crate) fn active_island(&self, island_id: usize) -> &[RigidBodyHandle] {
        &self.active_dynamic_set[self.active_island_range(island_id)]
    }

    pub(crate) fn maintain_active_set(&mut self) {
        for handle in self.activation_channel.1.try_iter() {
            if let Some(rb) = self.bodies.get_mut(handle) {
                // Push the body to the active set if it is not
                // sleeping and if it is not already inside of the active set.
                if !rb.is_sleeping() // May happen if the body was put to sleep manually.
                    && rb.is_dynamic() // Only dynamic bodies are in the active dynamic set.
                    && self.active_dynamic_set.get(rb.active_set_id) != Some(&handle)
                {
                    rb.active_set_id = self.active_dynamic_set.len(); // This will handle the case where the activation_channel contains duplicates.
                    self.active_dynamic_set.push(handle);
                }
            }
        }
    }

    pub(crate) fn update_active_set_with_contacts(
        &mut self,
        colliders: &ColliderSet,
        contact_graph: &InteractionGraph<ContactPair>,
        joint_graph: &InteractionGraph<Joint>,
        min_island_size: usize,
    ) {
        assert!(
            min_island_size > 0,
            "The minimum island size must be at least 1."
        );

        // Update the energy of every rigid body and
        // keep only those that may not sleep.
        //        let t = instant::now();
        self.active_set_timestamp += 1;
        self.stack.clear();
        self.can_sleep.clear();

        // NOTE: the `.rev()` is here so that two successive timesteps preserve
        // the order of the bodies in the `active_dynamic_set` vec. This reversal
        // does not seem to affect performances nor stability. However it makes
        // debugging slightly nicer so we keep this rev.
        for h in self.active_dynamic_set.drain(..).rev() {
            let rb = &mut self.bodies[h];
            rb.update_energy();
            if rb.activation.energy <= rb.activation.threshold {
                // Mark them as sleeping for now. This will
                // be set to false during the graph traversal
                // if it should not be put to sleep.
                rb.activation.sleeping = true;
                self.can_sleep.push(h);
            } else {
                self.stack.push(h);
            }
        }

        //        println!("Selection: {}", instant::now() - t);

        //        let t = instant::now();
        // Propagation of awake state and awake island computation through the
        // traversal of the interaction graph.
        self.active_islands.clear();
        self.active_islands.push(0);

        // The max avoid underflow when the stack is empty.
        let mut island_marker = self.stack.len().max(1) - 1;

        while let Some(handle) = self.stack.pop() {
            let rb = &mut self.bodies[handle];

            if rb.active_set_timestamp == self.active_set_timestamp || !rb.is_dynamic() {
                // We already visited this body and its neighbors.
                // Also, we don't propagate awake state through static bodies.
                continue;
            } else if self.stack.len() < island_marker {
                if self.active_dynamic_set.len() - *self.active_islands.last().unwrap()
                    >= min_island_size
                {
                    // We are starting a new island.
                    self.active_islands.push(self.active_dynamic_set.len());
                }

                island_marker = self.stack.len();
            }

            rb.wake_up();
            rb.active_island_id = self.active_islands.len() - 1;
            rb.active_set_id = self.active_dynamic_set.len();
            rb.active_set_offset = rb.active_set_id - self.active_islands[rb.active_island_id];
            rb.active_set_timestamp = self.active_set_timestamp;
            self.active_dynamic_set.push(handle);

            // Read all the contacts and push objects touching this one.
            for collider_handle in &rb.colliders {
                let collider = &colliders[*collider_handle];

                for inter in contact_graph.interactions_with(collider.contact_graph_index) {
                    for manifold in &inter.2.manifolds {
                        if manifold.num_active_contacts() > 0 {
                            let other =
                                crate::utils::other_handle((inter.0, inter.1), *collider_handle);
                            let other_body = colliders[other].parent;
                            self.stack.push(other_body);
                            break;
                        }
                    }
                }
            }

            for inter in joint_graph.interactions_with(rb.joint_graph_index) {
                let other = crate::utils::other_handle((inter.0, inter.1), handle);
                self.stack.push(other);
            }
        }

        self.active_islands.push(self.active_dynamic_set.len());
        //        println!(
        //            "Extraction: {}, num islands: {}",
        //            instant::now() - t,
        //            self.active_islands.len() - 1
        //        );

        // Actually put to sleep bodies which have not been detected as awake.
        //        let t = instant::now();
        for h in &self.can_sleep {
            let b = &mut self.bodies[*h];
            if b.activation.sleeping {
                b.sleep();
            }
        }
        //        println!("Activation: {}", instant::now() - t);
    }
}

impl Index<RigidBodyHandle> for RigidBodySet {
    type Output = RigidBody;

    fn index(&self, index: RigidBodyHandle) -> &RigidBody {
        &self.bodies[index]
    }
}

impl IndexMut<RigidBodyHandle> for RigidBodySet {
    fn index_mut(&mut self, index: RigidBodyHandle) -> &mut RigidBody {
        &mut self.bodies[index]
    }
}
