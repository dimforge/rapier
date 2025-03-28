use crate::data::arena::Arena;
use crate::data::{HasModifiedFlag, ModifiedObjects};
use crate::dynamics::{IslandManager, RigidBodyHandle, RigidBodySet};
use crate::geometry::{Collider, ColliderChanges, ColliderHandle, ColliderParent};
use crate::math::Isometry;
use std::ops::{Index, IndexMut};

pub(crate) type ModifiedColliders = ModifiedObjects<ColliderHandle, Collider>;

impl HasModifiedFlag for Collider {
    #[inline]
    fn has_modified_flag(&self) -> bool {
        self.changes.contains(ColliderChanges::MODIFIED)
    }

    #[inline]
    fn set_modified_flag(&mut self) {
        self.changes |= ColliderChanges::MODIFIED;
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Default, Debug)]
/// A set of colliders that can be handled by a physics `World`.
pub struct ColliderSet {
    pub(crate) colliders: Arena<Collider>,
    pub(crate) modified_colliders: ModifiedColliders,
    pub(crate) removed_colliders: Vec<ColliderHandle>,
}

impl ColliderSet {
    /// Create a new empty set of colliders.
    pub fn new() -> Self {
        ColliderSet {
            colliders: Arena::new(),
            modified_colliders: Default::default(),
            removed_colliders: Vec::new(),
        }
    }

    /// Create a new set of colliders, with an initial capacity
    /// for the set of colliders as well as the tracking of
    /// modified colliders.
    pub fn with_capacity(capacity: usize) -> Self {
        ColliderSet {
            colliders: Arena::with_capacity(capacity),
            modified_colliders: ModifiedColliders::with_capacity(capacity),
            removed_colliders: Vec::new(),
        }
    }

    pub(crate) fn take_modified(&mut self) -> ModifiedColliders {
        std::mem::take(&mut self.modified_colliders)
    }

    pub(crate) fn take_removed(&mut self) -> Vec<ColliderHandle> {
        std::mem::take(&mut self.removed_colliders)
    }

    /// An always-invalid collider handle.
    pub fn invalid_handle() -> ColliderHandle {
        ColliderHandle::from_raw_parts(crate::INVALID_U32, crate::INVALID_U32)
    }

    /// Iterate through all the colliders on this set.
    pub fn iter(&self) -> impl ExactSizeIterator<Item = (ColliderHandle, &Collider)> {
        self.colliders.iter().map(|(h, c)| (ColliderHandle(h), c))
    }

    /// Iterate through all the enabled colliders on this set.
    pub fn iter_enabled(&self) -> impl Iterator<Item = (ColliderHandle, &Collider)> {
        self.colliders
            .iter()
            .map(|(h, c)| (ColliderHandle(h), c))
            .filter(|(_, c)| c.is_enabled())
    }

    /// Iterates mutably through all the colliders on this set.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ColliderHandle, &mut Collider)> {
        self.modified_colliders.clear();
        let modified_colliders = &mut self.modified_colliders;
        self.colliders.iter_mut().map(move |(h, co)| {
            // NOTE: we push unchecked here since we are just re-populating the
            //       `modified_colliders` set that we just cleared before iteration.
            modified_colliders.push_unchecked(ColliderHandle(h), co);
            (ColliderHandle(h), co)
        })
    }

    /// Iterates mutably through all the enabled colliders on this set.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn iter_enabled_mut(&mut self) -> impl Iterator<Item = (ColliderHandle, &mut Collider)> {
        self.iter_mut().filter(|(_, c)| c.is_enabled())
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

    /// Inserts a new collider to this set and retrieve its handle.
    pub fn insert(&mut self, coll: impl Into<Collider>) -> ColliderHandle {
        let mut coll = coll.into();
        // Make sure the internal links are reset, they may not be
        // if this rigid-body was obtained by cloning another one.
        coll.reset_internal_references();
        coll.parent = None;
        let handle = ColliderHandle(self.colliders.insert(coll));
        // NOTE: we push unchecked because this is a brand-new collider
        //       so it was initialized with the changed flag but isn’t in
        //       the set yet.
        self.modified_colliders
            .push_unchecked(handle, &mut self.colliders[handle.0]);
        handle
    }

    /// Inserts a new collider to this set, attach it to the given rigid-body, and retrieve its handle.
    pub fn insert_with_parent(
        &mut self,
        coll: impl Into<Collider>,
        parent_handle: RigidBodyHandle,
        bodies: &mut RigidBodySet,
    ) -> ColliderHandle {
        let mut coll = coll.into();
        // Make sure the internal links are reset, they may not be
        // if this collider was obtained by cloning another one.
        coll.reset_internal_references();

        if let Some(prev_parent) = &mut coll.parent {
            prev_parent.handle = parent_handle;
        } else {
            coll.parent = Some(ColliderParent {
                handle: parent_handle,
                pos_wrt_parent: coll.pos.0,
            });
        }

        // NOTE: we use `get_mut` instead of `get_mut_internal` so that the
        // modification flag is updated properly.
        let parent = bodies
            .get_mut_internal_with_modification_tracking(parent_handle)
            .expect("Parent rigid body not found.");
        let handle = ColliderHandle(self.colliders.insert(coll));
        let coll = self.colliders.get_mut(handle.0).unwrap();
        // NOTE: we push unchecked because this is a brand-new collider
        //       so it was initialized with the changed flag but isn’t in
        //       the set yet.
        self.modified_colliders.push_unchecked(handle, coll);

        parent.add_collider_internal(
            handle,
            coll.parent.as_mut().unwrap(),
            &mut coll.pos,
            &coll.shape,
            &coll.mprops,
        );
        handle
    }

    /// Sets the parent of the given collider.
    // TODO: find a way to define this as a method of Collider.
    pub fn set_parent(
        &mut self,
        handle: ColliderHandle,
        new_parent_handle: Option<RigidBodyHandle>,
        bodies: &mut RigidBodySet,
    ) {
        if let Some(collider) = self.get_mut(handle) {
            let curr_parent = collider.parent.map(|p| p.handle);
            if new_parent_handle == curr_parent {
                return; // Nothing to do, this is the same parent.
            }

            collider.changes |= ColliderChanges::PARENT;

            if let Some(parent_handle) = curr_parent {
                if let Some(rb) = bodies.get_mut(parent_handle) {
                    rb.remove_collider_internal(handle);
                }
            }

            match new_parent_handle {
                Some(new_parent_handle) => {
                    if let Some(parent) = &mut collider.parent {
                        parent.handle = new_parent_handle;
                    } else {
                        collider.parent = Some(ColliderParent {
                            handle: new_parent_handle,
                            pos_wrt_parent: Isometry::identity(),
                        })
                    };

                    if let Some(rb) = bodies.get_mut(new_parent_handle) {
                        rb.add_collider_internal(
                            handle,
                            collider.parent.as_ref().unwrap(),
                            &mut collider.pos,
                            &collider.shape,
                            &collider.mprops,
                        );
                    }
                }
                None => collider.parent = None,
            }
        }
    }

    /// Remove a collider from this set and update its parent accordingly.
    ///
    /// If `wake_up` is `true`, the rigid-body the removed collider is attached to
    /// will be woken up.
    pub fn remove(
        &mut self,
        handle: ColliderHandle,
        islands: &mut IslandManager,
        bodies: &mut RigidBodySet,
        wake_up: bool,
    ) -> Option<Collider> {
        let collider = self.colliders.remove(handle.0)?;

        /*
         * Delete the collider from its parent body.
         */
        // NOTE: we use `get_mut_internal_with_modification_tracking` instead of `get_mut_internal` so that the
        // modification flag is updated properly.
        if let Some(parent) = &collider.parent {
            if let Some(parent_rb) =
                bodies.get_mut_internal_with_modification_tracking(parent.handle)
            {
                parent_rb.remove_collider_internal(handle);

                if wake_up {
                    islands.wake_up(bodies, parent.handle, true);
                }
            }
        }

        /*
         * Publish removal.
         */
        self.removed_colliders.push(handle);

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
    pub fn get_unknown_gen(&self, i: u32) -> Option<(&Collider, ColliderHandle)> {
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
    pub fn get_unknown_gen_mut(&mut self, i: u32) -> Option<(&mut Collider, ColliderHandle)> {
        let (collider, handle) = self.colliders.get_unknown_gen_mut(i)?;
        let handle = ColliderHandle(handle);
        self.modified_colliders.push_once(handle, collider);
        Some((collider, handle))
    }

    /// Get the collider with the given handle.
    pub fn get(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.colliders.get(handle.0)
    }

    /// Gets a mutable reference to the collider with the given handle.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        let result = self.colliders.get_mut(handle.0)?;
        self.modified_colliders.push_once(handle, result);
        Some(result)
    }

    pub(crate) fn index_mut_internal(&mut self, handle: ColliderHandle) -> &mut Collider {
        &mut self.colliders[handle.0]
    }

    pub(crate) fn get_mut_internal(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        self.colliders.get_mut(handle.0)
    }

    // Just a very long name instead of `.get_mut` to make sure
    // this is really the method we wanted to use instead of `get_mut_internal`.
    #[allow(dead_code)]
    pub(crate) fn get_mut_internal_with_modification_tracking(
        &mut self,
        handle: ColliderHandle,
    ) -> Option<&mut Collider> {
        let result = self.colliders.get_mut(handle.0)?;
        self.modified_colliders.push_once(handle, result);
        Some(result)
    }
}

impl Index<crate::data::Index> for ColliderSet {
    type Output = Collider;

    fn index(&self, index: crate::data::Index) -> &Collider {
        &self.colliders[index]
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
        self.modified_colliders.push_once(handle, collider);
        collider
    }
}
