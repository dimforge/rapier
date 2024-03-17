use crate::data::arena::Arena;
use crate::data::Coarena;
use crate::dynamics::{IslandManager, RigidBodyHandle, RigidBodySet};
use crate::geometry::{Collider, ColliderChanges, ColliderHandle, ColliderParent};
use crate::math::*;
use std::ops::{Index, IndexMut};

#[cfg(feature = "bevy")]
use crate::data::EntityArena;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Default)]
/// A set of colliders that can be handled by a physics `World`.
pub struct ColliderSet {
    #[cfg(not(feature = "bevy"))]
    colliders: Arena<Collider>,
    #[cfg(feature = "bevy")]
    colliders: EntityArena<Collider>,
    modified_colliders: Vec<ColliderHandle>,
    removed_colliders: Vec<ColliderHandle>,
}

impl ColliderSet {
    /// Create a new empty set of colliders.
    pub fn new() -> Self {
        Self::default()
    }

    pub(crate) fn take_modified(&mut self) -> Vec<ColliderHandle> {
        std::mem::take(&mut self.modified_colliders)
    }

    pub(crate) fn take_removed(&mut self) -> Vec<ColliderHandle> {
        std::mem::take(&mut self.removed_colliders)
    }

    /// An always-invalid collider handle.
    pub fn invalid_handle() -> ColliderHandle {
        ColliderHandle::PLACEHOLDER
    }

    /// Iterate through all the colliders on this set.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn iter(&self) -> impl ExactSizeIterator<Item = (ColliderHandle, &Collider)> {
        self.iter_internal()
    }

    /// Iterate through all the enabled colliders on this set.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn iter_enabled(&self) -> impl Iterator<Item = (ColliderHandle, &Collider)> {
        self.iter_enabled_internal()
    }

    pub(crate) fn iter_internal(
        &self,
    ) -> impl ExactSizeIterator<Item = (ColliderHandle, &Collider)> {
        self.colliders
            .iter()
            .map(|(h, c)| (ColliderHandle::from(h), c))
    }

    pub(crate) fn iter_enabled_internal(
        &self,
    ) -> impl Iterator<Item = (ColliderHandle, &Collider)> {
        self.colliders
            .iter()
            .map(|(h, c)| (ColliderHandle::from(h), c))
            .filter(|(_, c)| c.is_enabled())
    }

    /// Iterates mutably through all the colliders on this set.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ColliderHandle, &mut Collider)> {
        self.modified_colliders.clear();
        let modified_colliders = &mut self.modified_colliders;
        self.colliders.iter_mut().map(move |(h, b)| {
            modified_colliders.push(ColliderHandle::from(h));
            (ColliderHandle::from(h), b)
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
        self.colliders.contains(handle.into())
    }

    /// Inserts a new collider to this set and retrieve its handle.
    pub fn insert(
        &mut self,
        #[cfg(feature = "bevy")] handle: ColliderHandle,
        coll: impl Into<Collider>,
    ) -> ColliderHandle {
        let mut coll = coll.into();
        // Make sure the internal links are reset, they may not be
        // if this collider was obtained by cloning another one.
        coll.reset_internal_references();
        coll.parent = None;
        #[cfg(not(feature = "bevy"))]
        let handle = ColliderHandle(self.colliders.insert(coll));
        #[cfg(feature = "bevy")]
        self.colliders.insert(handle, coll);
        self.modified_colliders.push(handle);
        handle
    }

    /// Inserts a new collider to this set, attach it to the given rigid-body, and retrieve its handle.
    pub fn insert_with_parent(
        &mut self,
        #[cfg(feature = "bevy")] handle: ColliderHandle,
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
        #[cfg(not(feature = "bevy"))]
        let handle = ColliderHandle(self.colliders.insert(coll));
        #[cfg(feature = "bevy")]
        self.colliders.insert(handle, coll);

        self.modified_colliders.push(handle);

        let coll = self.colliders.get_mut(handle.into()).unwrap();
        parent.add_collider(
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
        if let Some(collider) = self.get_mut_internal_with_modification_tracking(handle) {
            let curr_parent = collider.parent.map(|p| p.handle);
            if new_parent_handle == curr_parent {
                return; // Nothing to do, this is the same parent.
            }

            collider.changes |= ColliderChanges::PARENT;

            if let Some(parent_handle) = curr_parent {
                if let Some(rb) = bodies.get_mut_internal_with_modification_tracking(parent_handle)
                {
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

                    if let Some(rb) =
                        bodies.get_mut_internal_with_modification_tracking(new_parent_handle)
                    {
                        rb.add_collider(
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
        let collider = self.colliders.remove(handle.into())?;

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
    #[cfg(not(feature = "bevy"))]
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
    #[cfg(not(feature = "bevy"))]
    pub fn get_unknown_gen_mut(&mut self, i: u32) -> Option<(&mut Collider, ColliderHandle)> {
        let (collider, handle) = self.colliders.get_unknown_gen_mut(i)?;
        let handle = ColliderHandle(handle);
        Self::mark_as_modified(handle, collider, &mut self.modified_colliders);
        Some((collider, handle))
    }

    /// Get the collider with the given handle.
    pub fn get(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.colliders.get(handle.into())
    }

    fn mark_as_modified(
        handle: ColliderHandle,
        collider: &mut Collider,
        modified_colliders: &mut Vec<ColliderHandle>,
    ) {
        if !collider.changes.contains(ColliderChanges::MODIFIED) {
            collider.changes = ColliderChanges::MODIFIED;
            modified_colliders.push(handle);
        }
    }

    /// Gets a mutable reference to the collider with the given handle.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        let result = self.colliders.get_mut(handle.into())?;
        Self::mark_as_modified(handle, result, &mut self.modified_colliders);
        Some(result)
    }

    pub(crate) fn index_mut_internal(&mut self, handle: ColliderHandle) -> &mut Collider {
        &mut self.colliders[handle.into()]
    }

    pub(crate) fn get_mut_internal(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        self.colliders.get_mut(handle.into())
    }

    // Just a very long name instead of `.get_mut` to make sure
    // this is really the method we wanted to use instead of `get_mut_internal`.
    #[allow(dead_code)]
    pub(crate) fn get_mut_internal_with_modification_tracking(
        &mut self,
        handle: ColliderHandle,
    ) -> Option<&mut Collider> {
        let result = self.colliders.get_mut(handle.into())?;
        Self::mark_as_modified(handle, result, &mut self.modified_colliders);
        Some(result)
    }
}

#[cfg(not(feature = "bevy"))]
impl Index<crate::data::Index> for ColliderSet {
    type Output = Collider;

    fn index(&self, index: crate::data::Index) -> &Collider {
        &self.colliders[index]
    }
}

impl Index<ColliderHandle> for ColliderSet {
    type Output = Collider;

    fn index(&self, index: ColliderHandle) -> &Collider {
        &self.colliders[index.into()]
    }
}

#[cfg(not(feature = "dev-remove-slow-accessors"))]
impl IndexMut<ColliderHandle> for ColliderSet {
    fn index_mut(&mut self, handle: ColliderHandle) -> &mut Collider {
        let collider = &mut self.colliders[handle.into()];
        Self::mark_as_modified(handle, collider, &mut self.modified_colliders);
        collider
    }
}
