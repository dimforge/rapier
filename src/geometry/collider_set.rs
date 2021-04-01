use crate::data::arena::Arena;
use crate::data::pubsub::PubSub;
use crate::dynamics::{RigidBodyHandle, RigidBodySet};
use crate::geometry::collider::ColliderChanges;
use crate::geometry::{Collider, SAPProxyIndex};
use parry::partitioning::IndexedData;
use std::ops::{Index, IndexMut};

/// The unique identifier of a collider added to a collider set.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[repr(transparent)]
pub struct ColliderHandle(pub(crate) crate::data::arena::Index);

impl ColliderHandle {
    /// Converts this handle into its (index, generation) components.
    pub fn into_raw_parts(self) -> (usize, u64) {
        self.0.into_raw_parts()
    }

    /// Reconstructs an handle from its (index, generation) components.
    pub fn from_raw_parts(id: usize, generation: u64) -> Self {
        Self(crate::data::arena::Index::from_raw_parts(id, generation))
    }

    /// An always-invalid collider handle.
    pub fn invalid() -> Self {
        Self(crate::data::arena::Index::from_raw_parts(
            crate::INVALID_USIZE,
            crate::INVALID_U64,
        ))
    }
}

impl IndexedData for ColliderHandle {
    fn default() -> Self {
        Self(IndexedData::default())
    }

    fn index(&self) -> usize {
        self.0.index()
    }
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub(crate) struct RemovedCollider {
    pub handle: ColliderHandle,
    pub(crate) proxy_index: SAPProxyIndex,
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// A set of colliders that can be handled by a physics `World`.
pub struct ColliderSet {
    pub(crate) removed_colliders: PubSub<RemovedCollider>,
    pub(crate) colliders: Arena<Collider>,
    pub(crate) modified_colliders: Vec<ColliderHandle>,
    pub(crate) modified_all_colliders: bool,
}

impl ColliderSet {
    /// Create a new empty set of colliders.
    pub fn new() -> Self {
        ColliderSet {
            removed_colliders: PubSub::new(),
            colliders: Arena::new(),
            modified_colliders: Vec::new(),
            modified_all_colliders: false,
        }
    }

    /// An always-invalid collider handle.
    pub fn invalid_handle() -> ColliderHandle {
        ColliderHandle::from_raw_parts(crate::INVALID_USIZE, crate::INVALID_U64)
    }

    /// Iterate through all the colliders on this set.
    pub fn iter(&self) -> impl ExactSizeIterator<Item = (ColliderHandle, &Collider)> {
        self.colliders.iter().map(|(h, c)| (ColliderHandle(h), c))
    }

    /// Iterates mutably through all the colliders on this set.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ColliderHandle, &mut Collider)> {
        self.modified_colliders.clear();
        self.modified_all_colliders = true;
        self.colliders
            .iter_mut()
            .map(|(h, b)| (ColliderHandle(h), b))
    }

    #[inline(always)]
    pub(crate) fn foreach_modified_colliders(&self, mut f: impl FnMut(ColliderHandle, &Collider)) {
        for handle in &self.modified_colliders {
            if let Some(rb) = self.colliders.get(handle.0) {
                f(*handle, rb)
            }
        }
    }

    #[inline(always)]
    pub(crate) fn foreach_modified_colliders_mut_internal(
        &mut self,
        mut f: impl FnMut(ColliderHandle, &mut Collider),
    ) {
        for handle in &self.modified_colliders {
            if let Some(rb) = self.colliders.get_mut(handle.0) {
                f(*handle, rb)
            }
        }
    }

    /// The number of colliders on this set.
    pub fn len(&self) -> usize {
        self.colliders.len()
    }

    /// `true` if there are no colliders in this set.
    pub fn is_empty(&self) -> bool {
        self.colliders.is_empty()
    }

    /// Is this collider handle valid?
    pub fn contains(&self, handle: ColliderHandle) -> bool {
        self.colliders.contains(handle.0)
    }

    pub(crate) fn contains_any_modified_collider(&self) -> bool {
        self.modified_all_colliders || !self.modified_colliders.is_empty()
    }

    pub(crate) fn clear_modified_colliders(&mut self) {
        if self.modified_all_colliders {
            for collider in self.colliders.iter_mut() {
                collider.1.changes = ColliderChanges::empty();
            }
            self.modified_colliders.clear();
            self.modified_all_colliders = false;
        } else {
            for handle in self.modified_colliders.drain(..) {
                self.colliders[handle.0].changes = ColliderChanges::empty();
            }
        }
    }

    /// Inserts a new collider to this set and retrieve its handle.
    pub fn insert(
        &mut self,
        mut coll: Collider,
        parent_handle: RigidBodyHandle,
        bodies: &mut RigidBodySet,
    ) -> ColliderHandle {
        // Make sure the internal links are reset, they may not be
        // if this rigid-body was obtained by cloning another one.
        coll.reset_internal_references();

        coll.parent = parent_handle;

        // NOTE: we use `get_mut` instead of `get_mut_internal` so that the
        // modification flag is updated properly.
        let parent = bodies
            .get_mut_internal_with_modification_tracking(parent_handle)
            .expect("Parent rigid body not found.");
        coll.position = parent.position * coll.delta;
        let handle = ColliderHandle(self.colliders.insert(coll));
        self.modified_colliders.push(handle);

        let coll = self.colliders.get(handle.0).unwrap();
        parent.add_collider(handle, &coll);
        handle
    }

    /// Remove a collider from this set and update its parent accordingly.
    ///
    /// If `wake_up` is `true`, the rigid-body the removed collider is attached to
    /// will be woken up.
    pub fn remove(
        &mut self,
        handle: ColliderHandle,
        bodies: &mut RigidBodySet,
        wake_up: bool,
    ) -> Option<Collider> {
        let collider = self.colliders.remove(handle.0)?;

        /*
         * Delete the collider from its parent body.
         */
        // NOTE: we use `get_mut` instead of `get_mut_internal` so that the
        // modification flag is updated properly.
        if let Some(parent) = bodies.get_mut_internal_with_modification_tracking(collider.parent) {
            parent.remove_collider_internal(handle, &collider);

            if wake_up {
                bodies.wake_up(collider.parent, true);
            }
        }

        /*
         * Publish removal.
         */
        let message = RemovedCollider {
            handle,
            proxy_index: collider.proxy_index,
        };

        self.removed_colliders.publish(message);

        Some(collider)
    }

    /// Gets the collider with the given handle without a known generation.
    ///
    /// This is useful when you know you want the collider at position `i` but
    /// don't know what is its current generation number. Generation numbers are
    /// used to protect from the ABA problem because the collider position `i`
    /// are recycled between two insertion and a removal.
    ///
    /// Using this is discouraged in favor of `self.get(handle)` which does not
    /// suffer form the ABA problem.
    pub fn get_unknown_gen(&self, i: usize) -> Option<(&Collider, ColliderHandle)> {
        self.colliders
            .get_unknown_gen(i)
            .map(|(c, h)| (c, ColliderHandle(h)))
    }

    /// Gets a mutable reference to the collider with the given handle without a known generation.
    ///
    /// This is useful when you know you want the collider at position `i` but
    /// don't know what is its current generation number. Generation numbers are
    /// used to protect from the ABA problem because the collider position `i`
    /// are recycled between two insertion and a removal.
    ///
    /// Using this is discouraged in favor of `self.get_mut(handle)` which does not
    /// suffer form the ABA problem.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_unknown_gen_mut(&mut self, i: usize) -> Option<(&mut Collider, ColliderHandle)> {
        let (collider, handle) = self.colliders.get_unknown_gen_mut(i)?;
        let handle = ColliderHandle(handle);
        Self::mark_as_modified(
            handle,
            collider,
            &mut self.modified_colliders,
            self.modified_all_colliders,
        );
        Some((collider, handle))
    }

    /// Get the collider with the given handle.
    pub fn get(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.colliders.get(handle.0)
    }

    fn mark_as_modified(
        handle: ColliderHandle,
        collider: &mut Collider,
        modified_colliders: &mut Vec<ColliderHandle>,
        modified_all_colliders: bool,
    ) {
        if !modified_all_colliders && !collider.changes.contains(ColliderChanges::MODIFIED) {
            collider.changes = ColliderChanges::MODIFIED;
            modified_colliders.push(handle);
        }
    }

    /// Gets a mutable reference to the collider with the given handle.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        let result = self.colliders.get_mut(handle.0)?;
        Self::mark_as_modified(
            handle,
            result,
            &mut self.modified_colliders,
            self.modified_all_colliders,
        );
        Some(result)
    }

    pub(crate) fn get_mut_internal(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        self.colliders.get_mut(handle.0)
    }

    // Just a very long name instead of `.get_mut` to make sure
    // this is really the method we wanted to use instead of `get_mut_internal`.
    pub(crate) fn get_mut_internal_with_modification_tracking(
        &mut self,
        handle: ColliderHandle,
    ) -> Option<&mut Collider> {
        let result = self.colliders.get_mut(handle.0)?;
        Self::mark_as_modified(
            handle,
            result,
            &mut self.modified_colliders,
            self.modified_all_colliders,
        );
        Some(result)
    }

    // Utility function to avoid some borrowing issue in the `maintain` method.
    fn maintain_one(bodies: &mut RigidBodySet, collider: &mut Collider) {
        if collider
            .changes
            .contains(ColliderChanges::POSITION_WRT_PARENT)
        {
            if let Some(parent) = bodies.get_mut_internal(collider.parent()) {
                let position = parent.position * collider.position_wrt_parent();
                // NOTE: the set_position method will add the ColliderChanges::POSITION flag,
                //       which is needed for the broad-phase/narrow-phase to detect the change.
                collider.set_position(position);
            }
        }
    }

    pub(crate) fn handle_user_changes(&mut self, bodies: &mut RigidBodySet) {
        if self.modified_all_colliders {
            for (_, rb) in self.colliders.iter_mut() {
                Self::maintain_one(bodies, rb)
            }
        } else {
            for handle in self.modified_colliders.iter() {
                if let Some(rb) = self.colliders.get_mut(handle.0) {
                    Self::maintain_one(bodies, rb)
                }
            }
        }
    }
}

impl Index<ColliderHandle> for ColliderSet {
    type Output = Collider;

    fn index(&self, index: ColliderHandle) -> &Collider {
        &self.colliders[index.0]
    }
}

#[cfg(not(feature = "dev-remove-slow-accessors"))]
impl IndexMut<ColliderHandle> for ColliderSet {
    fn index_mut(&mut self, handle: ColliderHandle) -> &mut Collider {
        let collider = &mut self.colliders[handle.0];
        Self::mark_as_modified(
            handle,
            collider,
            &mut self.modified_colliders,
            self.modified_all_colliders,
        );
        collider
    }
}
