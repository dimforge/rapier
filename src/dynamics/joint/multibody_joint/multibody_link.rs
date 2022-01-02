use std::ops::{Deref, DerefMut};

use crate::dynamics::{MultibodyJoint, RigidBodyHandle};
use crate::math::{Isometry, Real};
use crate::prelude::RigidBodyVelocity;

pub(crate) struct KinematicState {
    pub joint: MultibodyJoint,
    pub parent_to_world: Isometry<Real>,
    // TODO: should this be removed in favor of the rigid-body position?
    pub local_to_world: Isometry<Real>,
    pub local_to_parent: Isometry<Real>,
}

impl Clone for KinematicState {
    fn clone(&self) -> Self {
        Self {
            joint: self.joint.clone(),
            parent_to_world: self.parent_to_world.clone(),
            local_to_world: self.local_to_world.clone(),
            local_to_parent: self.local_to_parent.clone(),
        }
    }
}

/// One link of a multibody.
pub struct MultibodyLink {
    pub(crate) name: String,
    // FIXME: make all those private.
    pub(crate) internal_id: usize,
    pub(crate) assembly_id: usize,
    pub(crate) is_leaf: bool,

    pub(crate) parent_internal_id: usize,
    pub(crate) rigid_body: RigidBodyHandle,

    /*
     * Change at each time step.
     */
    pub(crate) state: KinematicState,

    // FIXME: put this on a workspace buffer instead ?
    pub(crate) velocity_dot_wrt_joint: RigidBodyVelocity,
    pub(crate) velocity_wrt_joint: RigidBodyVelocity,
}

impl MultibodyLink {
    /// Creates a new multibody link.
    pub fn new(
        rigid_body: RigidBodyHandle,
        internal_id: usize,
        assembly_id: usize,
        parent_internal_id: usize,
        joint: MultibodyJoint,
        parent_to_world: Isometry<Real>,
        local_to_world: Isometry<Real>,
        local_to_parent: Isometry<Real>,
    ) -> Self {
        let is_leaf = true;
        let velocity_dot_wrt_joint = RigidBodyVelocity::zero();
        let velocity_wrt_joint = RigidBodyVelocity::zero();
        let kinematic_state = KinematicState {
            joint,
            parent_to_world,
            local_to_world,
            local_to_parent,
        };

        MultibodyLink {
            name: String::new(),
            internal_id,
            assembly_id,
            is_leaf,
            parent_internal_id,
            state: kinematic_state,
            velocity_dot_wrt_joint,
            velocity_wrt_joint,
            rigid_body,
        }
    }

    pub fn joint(&self) -> &MultibodyJoint {
        &self.state.joint
    }

    pub fn rigid_body_handle(&self) -> RigidBodyHandle {
        self.rigid_body
    }

    /// Checks if this link is the root of the multibody.
    #[inline]
    pub fn is_root(&self) -> bool {
        self.internal_id == 0
    }

    /// This link's name.
    #[inline]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Sets this link's name.
    #[inline]
    pub fn set_name(&mut self, name: String) {
        self.name = name
    }

    /// The handle of this multibody link.
    #[inline]
    pub fn link_id(&self) -> usize {
        self.internal_id
    }

    /// The handle of the parent link.
    #[inline]
    pub fn parent_id(&self) -> Option<usize> {
        if self.internal_id != 0 {
            Some(self.parent_internal_id)
        } else {
            None
        }
    }

    #[inline]
    pub fn local_to_world(&self) -> &Isometry<Real> {
        &self.state.local_to_world
    }

    #[inline]
    pub fn local_to_parent(&self) -> &Isometry<Real> {
        &self.state.local_to_parent
    }
}

// FIXME: keep this even if we already have the Index2 traits?
pub(crate) struct MultibodyLinkVec(pub Vec<MultibodyLink>);

impl MultibodyLinkVec {
    #[inline]
    pub fn get_mut_with_parent(&mut self, i: usize) -> (&mut MultibodyLink, &MultibodyLink) {
        let parent_id = self[i].parent_internal_id;

        assert!(
            parent_id != i,
            "Internal error: circular rigid body dependency."
        );
        assert!(parent_id < self.len(), "Invalid parent index.");

        unsafe {
            let rb = &mut *(self.get_unchecked_mut(i) as *mut _);
            let parent_rb = &*(self.get_unchecked(parent_id) as *const _);
            (rb, parent_rb)
        }
    }
}

impl Deref for MultibodyLinkVec {
    type Target = Vec<MultibodyLink>;

    #[inline]
    fn deref(&self) -> &Vec<MultibodyLink> {
        let MultibodyLinkVec(ref me) = *self;
        me
    }
}

impl DerefMut for MultibodyLinkVec {
    #[inline]
    fn deref_mut(&mut self) -> &mut Vec<MultibodyLink> {
        let MultibodyLinkVec(ref mut me) = *self;
        me
    }
}
