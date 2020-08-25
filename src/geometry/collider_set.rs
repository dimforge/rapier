use crate::data::arena::Arena;
use crate::dynamics::{RigidBodyHandle, RigidBodySet};
use crate::geometry::Collider;
use std::ops::{Index, IndexMut};

/// The unique identifier of a collider added to a collider set.
pub type ColliderHandle = crate::data::arena::Index;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A set of colliders that can be handled by a physics `World`.
pub struct ColliderSet {
    pub(crate) colliders: Arena<Collider>,
}

impl ColliderSet {
    /// Create a new empty set of colliders.
    pub fn new() -> Self {
        ColliderSet {
            colliders: Arena::new(),
        }
    }

    /// An always-invalid collider handle.
    pub fn invalid_handle() -> ColliderHandle {
        ColliderHandle::from_raw_parts(crate::INVALID_USIZE, crate::INVALID_U64)
    }

    /// Iterate through all the colliders on this set.
    pub fn iter(&self) -> impl Iterator<Item = (ColliderHandle, &Collider)> {
        self.colliders.iter()
    }

    /// The number of colliders on this set.
    pub fn len(&self) -> usize {
        self.colliders.len()
    }

    /// Is this collider handle valid?
    pub fn contains(&self, handle: ColliderHandle) -> bool {
        self.colliders.contains(handle)
    }

    /// Inserts a new collider to this set and retrieve its handle.
    pub fn insert(
        &mut self,
        mut coll: Collider,
        parent_handle: RigidBodyHandle,
        bodies: &mut RigidBodySet,
    ) -> ColliderHandle {
        let mass_properties = coll.mass_properties();
        coll.parent = parent_handle;
        let parent = bodies
            .get_mut_internal(parent_handle)
            .expect("Parent rigid body not found.");
        coll.position = parent.position * coll.delta;
        coll.predicted_position = parent.predicted_position * coll.delta;
        let handle = self.colliders.insert(coll);
        parent.colliders.push(handle);
        parent.mass_properties += mass_properties;
        parent.update_world_mass_properties();
        bodies.activate(parent_handle);
        handle
    }

    pub(crate) fn remove_internal(&mut self, handle: ColliderHandle) -> Option<Collider> {
        self.colliders.remove(handle)
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
        self.colliders.get_unknown_gen(i)
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
    pub fn get_unknown_gen_mut(&mut self, i: usize) -> Option<(&mut Collider, ColliderHandle)> {
        self.colliders.get_unknown_gen_mut(i)
    }

    /// Get the collider with the given handle.
    pub fn get(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.colliders.get(handle)
    }

    /// Gets a mutable reference to the collider with the given handle.
    pub fn get_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        self.colliders.get_mut(handle)
    }

    pub(crate) fn get2_mut_internal(
        &mut self,
        h1: ColliderHandle,
        h2: ColliderHandle,
    ) -> (Option<&mut Collider>, Option<&mut Collider>) {
        self.colliders.get2_mut(h1, h2)
    }

    // pub fn iter_mut(&mut self) -> impl Iterator<Item = (ColliderHandle, ColliderMut)> {
    //     //        let sender = &self.activation_channel_sender;
    //     self.colliders.iter_mut().map(move |(h, rb)| {
    //         (h, ColliderMut::new(h, rb /*sender.clone()*/))
    //     })
    // }

    //    pub(crate) fn iter_mut_internal(
    //        &mut self,
    //    ) -> impl Iterator<Item = (ColliderHandle, &mut Collider)> {
    //        self.colliders.iter_mut()
    //    }
}

impl Index<ColliderHandle> for ColliderSet {
    type Output = Collider;

    fn index(&self, index: ColliderHandle) -> &Collider {
        &self.colliders[index]
    }
}

impl IndexMut<ColliderHandle> for ColliderSet {
    fn index_mut(&mut self, index: ColliderHandle) -> &mut Collider {
        &mut self.colliders[index]
    }
}
