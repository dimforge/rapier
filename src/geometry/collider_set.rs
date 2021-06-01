use crate::data::arena::Arena;
use crate::data::{ComponentSet, ComponentSetMut, ComponentSetOption};
use crate::dynamics::{IslandManager, RigidBodyHandle, RigidBodySet};
use crate::geometry::{
    Collider, ColliderBroadPhaseData, ColliderFlags, ColliderMassProps, ColliderMaterial,
    ColliderParent, ColliderPosition, ColliderShape, ColliderType,
};
use crate::geometry::{ColliderChanges, ColliderHandle};
use std::ops::{Index, IndexMut};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// A set of colliders that can be handled by a physics `World`.
pub struct ColliderSet {
    pub(crate) colliders: Arena<Collider>,
    pub(crate) modified_colliders: Vec<ColliderHandle>,
    pub(crate) removed_colliders: Vec<ColliderHandle>,
}

macro_rules! impl_field_component_set(
    ($T: ty, $field: ident) => {
        impl ComponentSetOption<$T> for ColliderSet {
            fn get(&self, handle: crate::data::Index) -> Option<&$T> {
                self.get(ColliderHandle(handle)).map(|b| &b.$field)
            }
        }

        impl ComponentSet<$T> for ColliderSet {
            fn size_hint(&self) -> usize {
                self.len()
            }

            #[inline(always)]
            fn for_each(&self, mut f: impl FnMut(crate::data::Index, &$T)) {
                for (handle, body) in self.colliders.iter() {
                    f(handle, &body.$field)
                }
            }
        }

        impl ComponentSetMut<$T> for ColliderSet {
            fn set_internal(&mut self, handle: crate::data::Index, val: $T) {
                if let Some(rb) = self.get_mut_internal(ColliderHandle(handle)) {
                    rb.$field = val;
                }
            }

            #[inline(always)]
            fn map_mut_internal<Result>(
                &mut self,
                handle: crate::data::Index,
                f: impl FnOnce(&mut $T) -> Result,
            ) -> Option<Result> {
                self.get_mut_internal(ColliderHandle(handle)).map(|rb| f(&mut rb.$field))
            }
        }
    }
);

impl_field_component_set!(ColliderType, co_type);
impl_field_component_set!(ColliderShape, co_shape);
impl_field_component_set!(ColliderMassProps, co_mprops);
impl_field_component_set!(ColliderChanges, co_changes);
impl_field_component_set!(ColliderPosition, co_pos);
impl_field_component_set!(ColliderMaterial, co_material);
impl_field_component_set!(ColliderFlags, co_flags);
impl_field_component_set!(ColliderBroadPhaseData, co_bf_data);

impl ComponentSetOption<ColliderParent> for ColliderSet {
    #[inline(always)]
    fn get(&self, handle: crate::data::Index) -> Option<&ColliderParent> {
        self.get(ColliderHandle(handle))
            .and_then(|b| b.co_parent.as_ref())
    }
}

impl ColliderSet {
    /// Create a new empty set of colliders.
    pub fn new() -> Self {
        ColliderSet {
            colliders: Arena::new(),
            modified_colliders: Vec::new(),
            removed_colliders: Vec::new(),
        }
    }

    pub(crate) fn take_modified(&mut self) -> Vec<ColliderHandle> {
        std::mem::replace(&mut self.modified_colliders, vec![])
    }

    pub(crate) fn take_removed(&mut self) -> Vec<ColliderHandle> {
        std::mem::replace(&mut self.removed_colliders, vec![])
    }

    /// An always-invalid collider handle.
    pub fn invalid_handle() -> ColliderHandle {
        ColliderHandle::from_raw_parts(crate::INVALID_U32, crate::INVALID_U32)
    }

    /// Iterate through all the colliders on this set.
    pub fn iter(&self) -> impl ExactSizeIterator<Item = (ColliderHandle, &Collider)> {
        self.colliders.iter().map(|(h, c)| (ColliderHandle(h), c))
    }

    /// Iterates mutably through all the colliders on this set.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ColliderHandle, &mut Collider)> {
        self.modified_colliders.clear();
        let modified_colliders = &mut self.modified_colliders;
        self.colliders.iter_mut().map(move |(h, b)| {
            modified_colliders.push(ColliderHandle(h));
            (ColliderHandle(h), b)
        })
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
    pub fn insert(&mut self, mut coll: Collider) -> ColliderHandle {
        // Make sure the internal links are reset, they may not be
        // if this rigid-body was obtained by cloning another one.
        coll.reset_internal_references();
        coll.co_parent = None;
        let handle = ColliderHandle(self.colliders.insert(coll));
        self.modified_colliders.push(handle);
        handle
    }

    /// Inserts a new collider to this set, attach it to the given rigid-body, and retrieve its handle.
    pub fn insert_with_parent(
        &mut self,
        mut coll: Collider,
        parent_handle: RigidBodyHandle,
        bodies: &mut RigidBodySet,
    ) -> ColliderHandle {
        // Make sure the internal links are reset, they may not be
        // if this collider was obtained by cloning another one.
        coll.reset_internal_references();

        if let Some(prev_parent) = &mut coll.co_parent {
            prev_parent.handle = parent_handle;
        } else {
            coll.co_parent = Some(ColliderParent {
                handle: parent_handle,
                pos_wrt_parent: coll.co_pos.0,
            });
        }

        // NOTE: we use `get_mut` instead of `get_mut_internal` so that the
        // modification flag is updated properly.
        let parent = bodies
            .get_mut_internal_with_modification_tracking(parent_handle)
            .expect("Parent rigid body not found.");
        let handle = ColliderHandle(self.colliders.insert(coll));
        self.modified_colliders.push(handle);

        let coll = self.colliders.get_mut(handle.0).unwrap();
        parent.add_collider(
            handle,
            coll.co_parent.as_mut().unwrap(),
            &mut coll.co_pos,
            &coll.co_shape,
            &coll.co_mprops,
        );
        handle
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
        if let Some(co_parent) = &collider.co_parent {
            if let Some(parent) =
                bodies.get_mut_internal_with_modification_tracking(co_parent.handle)
            {
                parent.remove_collider_internal(handle, &collider);

                if wake_up {
                    islands.wake_up(bodies, co_parent.handle, true);
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
        Self::mark_as_modified(handle, collider, &mut self.modified_colliders);
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
    ) {
        if !collider.co_changes.contains(ColliderChanges::MODIFIED) {
            collider.co_changes = ColliderChanges::MODIFIED;
            modified_colliders.push(handle);
        }
    }

    /// Gets a mutable reference to the collider with the given handle.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        let result = self.colliders.get_mut(handle.0)?;
        Self::mark_as_modified(handle, result, &mut self.modified_colliders);
        Some(result)
    }

    pub(crate) fn get_mut_internal(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        self.colliders.get_mut(handle.0)
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
        Self::mark_as_modified(handle, collider, &mut self.modified_colliders);
        collider
    }
}
