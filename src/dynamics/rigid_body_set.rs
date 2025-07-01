use crate::data::{Arena, HasModifiedFlag, ModifiedObjects};
use crate::dynamics::{
    ImpulseJointSet, IslandManager, MultibodyJointSet, RigidBody, RigidBodyChanges, RigidBodyHandle,
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

pub(crate) type ModifiedRigidBodies = ModifiedObjects<RigidBodyHandle, RigidBody>;

impl HasModifiedFlag for RigidBody {
    #[inline]
    fn has_modified_flag(&self) -> bool {
        self.changes.contains(RigidBodyChanges::MODIFIED)
    }

    #[inline]
    fn set_modified_flag(&mut self) {
        self.changes |= RigidBodyChanges::MODIFIED;
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Default, Debug)]
/// A set of rigid bodies that can be handled by a physics pipeline.
pub struct RigidBodySet {
    // NOTE: the pub(crate) are needed by the broad phase
    // to avoid borrowing issues. It is also needed for
    // parallelism because the `Receiver` breaks the Sync impl.
    // Could we avoid this?
    pub(crate) bodies: Arena<RigidBody>,
    pub(crate) modified_bodies: ModifiedRigidBodies,
}

impl RigidBodySet {
    /// Create a new empty set of rigid bodies.
    pub fn new() -> Self {
        RigidBodySet {
            bodies: Arena::new(),
            modified_bodies: ModifiedObjects::default(),
        }
    }

    /// Create a new set of rigid bodies, with an initial capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        RigidBodySet {
            bodies: Arena::with_capacity(capacity),
            modified_bodies: ModifiedRigidBodies::with_capacity(capacity),
        }
    }

    pub(crate) fn take_modified(&mut self) -> ModifiedRigidBodies {
        std::mem::take(&mut self.modified_bodies)
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
    pub fn insert(&mut self, rb: impl Into<RigidBody>) -> RigidBodyHandle {
        let mut rb = rb.into();
        // Make sure the internal links are reset, they may not be
        // if this rigid-body was obtained by cloning another one.
        rb.reset_internal_references();
        rb.changes.set(RigidBodyChanges::all(), true);

        let handle = RigidBodyHandle(self.bodies.insert(rb));
        // Using push_unchecked because this is a brand new rigid-body with the MODIFIED
        // flags set but isn’t in the modified_bodies yet.
        self.modified_bodies
            .push_unchecked(handle, &mut self.bodies[handle.0]);
        handle
    }

    /// Removes a rigid-body, and all its attached colliders and impulse_joints, from these sets.
    #[profiling::function]
    pub fn remove(
        &mut self,
        handle: RigidBodyHandle,
        islands: &mut IslandManager,
        colliders: &mut ColliderSet,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
        remove_attached_colliders: bool,
    ) -> Option<RigidBody> {
        let rb = self.bodies.remove(handle.0)?;
        /*
         * Update active sets.
         */
        islands.rigid_body_removed(handle, &rb.ids, self);

        /*
         * Remove colliders attached to this rigid-body.
         */
        if remove_attached_colliders {
            for collider in rb.colliders() {
                colliders.remove(*collider, islands, self, false);
            }
        } else {
            // If we don’t remove the attached colliders, simply detach them.
            let colliders_to_detach = rb.colliders().to_vec();
            for co_handle in colliders_to_detach {
                colliders.set_parent(co_handle, None, self);
            }
        }

        /*
         * Remove impulse_joints attached to this rigid-body.
         */
        impulse_joints.remove_joints_attached_to_rigid_body(handle);
        multibody_joints.remove_joints_attached_to_rigid_body(handle);

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
        self.modified_bodies.push_once(handle, rb);
        Some((rb, handle))
    }

    /// Gets the rigid-body with the given handle.
    pub fn get(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        self.bodies.get(handle.0)
    }

    /// Gets a mutable reference to the rigid-body with the given handle.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        let result = self.bodies.get_mut(handle.0)?;
        self.modified_bodies.push_once(handle, result);
        Some(result)
    }

    /// Gets a mutable reference to the two rigid-bodies with the given handles.
    ///
    /// If `handle1 == handle2`, only the first returned value will be `Some`.
    #[cfg(not(feature = "dev-remove-slow-accessors"))]
    pub fn get_pair_mut(
        &mut self,
        handle1: RigidBodyHandle,
        handle2: RigidBodyHandle,
    ) -> (Option<&mut RigidBody>, Option<&mut RigidBody>) {
        if handle1 == handle2 {
            (self.get_mut(handle1), None)
        } else {
            let (mut rb1, mut rb2) = self.bodies.get2_mut(handle1.0, handle2.0);
            if let Some(rb1) = rb1.as_deref_mut() {
                self.modified_bodies.push_once(handle1, rb1);
            }
            if let Some(rb2) = rb2.as_deref_mut() {
                self.modified_bodies.push_once(handle2, rb2);
            }
            (rb1, rb2)
        }
    }

    pub(crate) fn get_mut_internal(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        self.bodies.get_mut(handle.0)
    }

    pub(crate) fn index_mut_internal(&mut self, handle: RigidBodyHandle) -> &mut RigidBody {
        &mut self.bodies[handle.0]
    }

    // Just a very long name instead of `.get_mut` to make sure
    // this is really the method we wanted to use instead of `get_mut_internal`.
    pub(crate) fn get_mut_internal_with_modification_tracking(
        &mut self,
        handle: RigidBodyHandle,
    ) -> Option<&mut RigidBody> {
        let result = self.bodies.get_mut(handle.0)?;
        self.modified_bodies.push_once(handle, result);
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
            // NOTE: using `push_unchecked` because we just cleared `modified_bodies`
            //       before iterating.
            modified_bodies.push_unchecked(RigidBodyHandle(h), b);
            (RigidBodyHandle(h), b)
        })
    }

    /// Update colliders positions after rigid-bodies moved.
    ///
    /// When a rigid-body moves, the positions of the colliders attached to it need to be updated.
    /// This update is generally automatically done at the beginning and the end of each simulation
    /// step with `PhysicsPipeline::step`. If the positions need to be updated without running a
    /// simulation step (for example when using the `QueryPipeline` alone), this method can be called
    /// manually.  
    pub fn propagate_modified_body_positions_to_colliders(&self, colliders: &mut ColliderSet) {
        for body in self.modified_bodies.iter().filter_map(|h| self.get(*h)) {
            if body.changes.contains(RigidBodyChanges::POSITION) {
                for handle in body.colliders() {
                    if let Some(collider) = colliders.get_mut(*handle) {
                        let new_pos = body.position() * collider.position_wrt_parent().unwrap();
                        collider.set_position(new_pos);
                    }
                }
            }
        }
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
        self.modified_bodies.push_once(handle, rb);
        rb
    }
}
