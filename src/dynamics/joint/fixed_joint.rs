use crate::math::{Isometry, Real, SpacialVector};

#[derive(Copy, Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A joint that prevents all relative movement between two bodies.
///
/// Given two frames of references, this joint aims to ensure these frame always coincide in world-space.
pub struct FixedJoint {
    /// The frame of reference for the first body affected by this joint, expressed in the local frame
    /// of the first body.
    pub local_anchor1: Isometry<Real>,
    /// The frame of reference for the second body affected by this joint, expressed in the local frame
    /// of the first body.
    pub local_anchor2: Isometry<Real>,
    /// The impulse applied to the first body affected by this joint.
    ///
    /// The impulse applied to the second body affected by this joint is given by `-impulse`.
    /// This combines both linear and angular impulses:
    /// - In 2D, `impulse.xy()` gives the linear impulse, and `impulse.z` the angular impulse.
    /// - In 3D, `impulse.xyz()` gives the linear impulse, and `(impulse[3], impulse[4], impulse[5])` the angular impulse.
    pub impulse: SpacialVector<Real>,
}

impl FixedJoint {
    /// Creates a new fixed joint from the frames of reference of both bodies.
    pub fn new(local_anchor1: Isometry<Real>, local_anchor2: Isometry<Real>) -> Self {
        Self {
            local_anchor1,
            local_anchor2,
            impulse: SpacialVector::zeros(),
        }
    }
}
