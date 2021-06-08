use crate::data::{Arena, ComponentSet, ComponentSetMut, ComponentSetOption};
use crate::dynamics::{
    IslandManager, RigidBodyActivation, RigidBodyColliders, RigidBodyDominance, RigidBodyHandle,
    RigidBodyType,
};
use crate::dynamics::{
    JointSet, RigidBody, RigidBodyCcd, RigidBodyChanges, RigidBodyDamping, RigidBodyForces,
    RigidBodyIds, RigidBodyMassProps, RigidBodyPosition, RigidBodyVelocity,
};
use crate::geometry::ColliderSet;
use std::ops::{Index, IndexMut};

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
    /// Builds a new pair of rigid-body handles.
    pub fn new(body1: RigidBodyHandle, body2: RigidBodyHandle) -> Self {
        BodyPair { body1, body2 }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// A set of rigid bodies that can be handled by a physics pipeline.
pub struct RigidBodySet {
    // NOTE: the pub(crate) are needed by the broad phase
    // to avoid borrowing issues. It is also needed for
    // parallelism because the `Receiver` breaks the Sync impl.
    // Could we avoid this?
    pub(crate) bodies: Arena<RigidBody>,
    pub(crate) modified_bodies: Vec<RigidBodyHandle>,
}

macro_rules! impl_field_component_set(
    ($T: ty, $field: ident) => {
        impl ComponentSetOption<$T> for RigidBodySet {
            fn get(&self, handle: crate::data::Index) -> Option<&$T> {
                self.get(RigidBodyHandle(handle)).map(|b| &b.$field)
            }
        }

        impl ComponentSet<$T> for RigidBodySet {
            fn size_hint(&self) -> usize {
                self.len()
            }

            #[inline(always)]
            fn for_each(&self, mut f: impl FnMut(crate::data::Index, &$T)) {
                for (handle, body) in self.bodies.iter() {
                    f(handle, &body.$field)
                }
            }
        }

        impl ComponentSetMut<$T> for RigidBodySet {
            fn set_internal(&mut self, handle: crate::data::Index, val: $T) {
                if let Some(rb) = self.get_mut_internal(RigidBodyHandle(handle)) {
                    rb.$field = val;
                }
            }

            #[inline(always)]
            fn map_mut_internal<Result>(
                &mut self,
                handle: crate::data::Index,
                f: impl FnOnce(&mut $T) -> Result,
            ) -> Option<Result> {
                self.get_mut_internal(RigidBodyHandle(handle)).map(|rb| f(&mut rb.$field))
            }
        }
    }
);

impl_field_component_set!(RigidBodyPosition, rb_pos);
impl_field_component_set!(RigidBodyMassProps, rb_mprops);
impl_field_component_set!(RigidBodyVelocity, rb_vels);
impl_field_component_set!(RigidBodyDamping, rb_damping);
impl_field_component_set!(RigidBodyForces, rb_forces);
impl_field_component_set!(RigidBodyCcd, rb_ccd);
impl_field_component_set!(RigidBodyIds, rb_ids);
impl_field_component_set!(RigidBodyType, rb_type);
impl_field_component_set!(RigidBodyActivation, rb_activation);
impl_field_component_set!(RigidBodyColliders, rb_colliders);
impl_field_component_set!(RigidBodyDominance, rb_dominance);
impl_field_component_set!(RigidBodyChanges, changes);

impl RigidBodySet {
    /// Create a new empty set of rigid bodies.
    pub fn new() -> Self {
        RigidBodySet {
            bodies: Arena::new(),
            modified_bodies: Vec::new(),
        }
    }

    pub(crate) fn take_modified(&mut self) -> Vec<RigidBodyHandle> {
        std::mem::replace(&mut self.modified_bodies, vec![])
    }

    /// The number of rigid bodies on this set.
    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    /// `true` if there are no rigid bodies in this set.
    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }

    /// Is the given body handle valid?
    pub fn contains(&self, handle: RigidBodyHandle) -> bool {
        self.bodies.contains(handle.0)
    }

    /// Insert a rigid body into this set and retrieve its handle.
    pub fn insert(&mut self, mut rb: RigidBody) -> RigidBodyHandle {
        // Make sure the internal links are reset, they may not be
        // if this rigid-body was obtained by cloning another one.
        rb.reset_internal_references();
        rb.changes.set(RigidBodyChanges::all(), true);

        let handle = RigidBodyHandle(self.bodies.insert(rb));
        self.modified_bodies.push(handle);
        handle
    }

    /// Removes a rigid-body, and all its attached colliders and joints, from these sets.
    pub fn remove(
        &mut self,
        handle: RigidBodyHandle,
        islands: &mut IslandManager,
        colliders: &mut ColliderSet,
        joints: &mut JointSet,
    ) -> Option<RigidBody> {
        let rb = self.bodies.remove(handle.0)?;
        /*
         * Update active sets.
         */
        islands.rigid_body_removed(handle, &rb.rb_ids, self);

        /*
         * Remove colliders attached to this rigid-body.
         */
        for collider in rb.colliders() {
            colliders.remove(*collider, islands, self, false);
        }

        /*
         * Remove joints attached to this rigid-body.
         */
        joints.remove_joints_attached_to_rigid_body(handle, islands, self);

        Some(rb)
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
    pub fn get_unknown_gen(&self, i: u32) -> Option<(&RigidBody, RigidBodyHandle)> {
        self.bodies
            .get_unknown_gen(i)
            .map(|(b, h)| (b, RigidBodyHandle(h)))
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
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_unknown_gen_mut(&mut self, i: u32) -> Option<(&mut RigidBody, RigidBodyHandle)> {
        let (rb, handle) = self.bodies.get_unknown_gen_mut(i)?;
        let handle = RigidBodyHandle(handle);
        Self::mark_as_modified(handle, rb, &mut self.modified_bodies);
        Some((rb, handle))
    }

    /// Gets the rigid-body with the given handle.
    pub fn get(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        self.bodies.get(handle.0)
    }

    pub(crate) fn mark_as_modified(
        handle: RigidBodyHandle,
        rb: &mut RigidBody,
        modified_bodies: &mut Vec<RigidBodyHandle>,
    ) {
        if !rb.changes.contains(RigidBodyChanges::MODIFIED) {
            rb.changes = RigidBodyChanges::MODIFIED;
            modified_bodies.push(handle);
        }
    }

    /// Gets a mutable reference to the rigid-body with the given handle.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        let result = self.bodies.get_mut(handle.0)?;
        Self::mark_as_modified(handle, result, &mut self.modified_bodies);
        Some(result)
    }

    pub(crate) fn get_mut_internal(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        self.bodies.get_mut(handle.0)
    }

    // Just a very long name instead of `.get_mut` to make sure
    // this is really the method we wanted to use instead of `get_mut_internal`.
    pub(crate) fn get_mut_internal_with_modification_tracking(
        &mut self,
        handle: RigidBodyHandle,
    ) -> Option<&mut RigidBody> {
        let result = self.bodies.get_mut(handle.0)?;
        Self::mark_as_modified(handle, result, &mut self.modified_bodies);
        Some(result)
    }

    /// Iterates through all the rigid-bodies on this set.
    pub fn iter(&self) -> impl Iterator<Item = (RigidBodyHandle, &RigidBody)> {
        self.bodies.iter().map(|(h, b)| (RigidBodyHandle(h), b))
    }

    /// Iterates mutably through all the rigid-bodies on this set.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (RigidBodyHandle, &mut RigidBody)> {
        self.modified_bodies.clear();
        let modified_bodies = &mut self.modified_bodies;
        self.bodies.iter_mut().map(move |(h, b)| {
            modified_bodies.push(RigidBodyHandle(h));
            (RigidBodyHandle(h), b)
        })
    }
}

impl Index<RigidBodyHandle> for RigidBodySet {
    type Output = RigidBody;

    fn index(&self, index: RigidBodyHandle) -> &RigidBody {
        &self.bodies[index.0]
    }
}

impl Index<crate::data::Index> for RigidBodySet {
    type Output = RigidBody;

    fn index(&self, index: crate::data::Index) -> &RigidBody {
        &self.bodies[index]
    }
}

#[cfg(not(feature = "dev-remove-slow-accessors"))]
impl IndexMut<RigidBodyHandle> for RigidBodySet {
    fn index_mut(&mut self, handle: RigidBodyHandle) -> &mut RigidBody {
        let rb = &mut self.bodies[handle.0];
        Self::mark_as_modified(handle, rb, &mut self.modified_bodies);
        rb
    }
}
