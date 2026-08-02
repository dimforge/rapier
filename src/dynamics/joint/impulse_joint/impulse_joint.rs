use crate::dynamics::{GenericJoint, ImpulseJointHandle, RigidBodyHandle};
use crate::math::SpatialVector;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq)]
/// An impulse-based joint attached to two bodies.
pub struct ImpulseJoint {
    /// Handle to the first body attached to this joint.
    ///
    /// Kept crate-private so the attached bodies can only be changed via
    /// [`ImpulseJointSet::set_bodies`], which also updates the interaction
    /// graph and island-manager bookkeeping. Read it through
    /// [`ImpulseJoint::body1`].
    pub(crate) body1: RigidBodyHandle,
    /// Handle to the second body attached to this joint. See
    /// [`ImpulseJoint::body1`] for why this is not directly writeable.
    pub(crate) body2: RigidBodyHandle,

    /// The joint’s description.
    pub data: GenericJoint,

    /// The impulses applied by this joint.
    pub impulses: SpatialVector,

    // A joint needs to know its handle to simplify its removal.
    pub(crate) handle: ImpulseJointHandle,

    /// The solver-body index (`active_set_id`) of each attached body, or
    /// `u32::MAX` for a side that is world-attached from the solver's point of
    /// view (fixed or sleeping).
    #[cfg_attr(
        feature = "serde-serialize",
        serde(skip, default = "default_solver_body_ids")
    )]
    pub(crate) solver_body_ids: [u32; 2],
    /// The solver graph color assigned to this joint.
    #[cfg_attr(feature = "serde-serialize", serde(default = "default_solver_color"))]
    pub(crate) solver_color: u8,
}

#[cfg(feature = "serde-serialize")]
fn default_solver_color() -> u8 {
    crate::geometry::contact_pair::SOLVER_COLOR_UNCOLORED
}

#[cfg(feature = "serde-serialize")]
fn default_solver_body_ids() -> [u32; 2] {
    [u32::MAX; 2]
}

impl ImpulseJoint {
    /// Handle to the first body attached to this joint.
    #[inline]
    pub fn body1(&self) -> RigidBodyHandle {
        self.body1
    }

    /// Handle to the second body attached to this joint.
    #[inline]
    pub fn body2(&self) -> RigidBodyHandle {
        self.body2
    }
}
