use crate::dynamics::RevoluteJoint;
use crate::math::{Isometry, Real, SpacialVector, SPATIAL_DIM};
use crate::na::{Rotation3, UnitQuaternion};

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A joint that prevents all relative movement between two bodies.
///
/// Given two frames of references, this joint aims to ensure these frame always coincide in world-space.
pub struct GenericJoint {
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

    pub min_position: SpacialVector<Real>,
    pub max_position: SpacialVector<Real>,
    pub target_velocity: SpacialVector<Real>,
    /// The maximum negative impulse the joint can apply on each DoF. Must be <= 0.0
    pub max_negative_impulse: SpacialVector<Real>,
    /// The maximum positive impulse the joint can apply on each DoF. Must be >= 0.0
    pub max_positive_impulse: SpacialVector<Real>,
}

impl GenericJoint {
    /// Creates a new fixed joint from the frames of reference of both bodies.
    pub fn new(local_anchor1: Isometry<Real>, local_anchor2: Isometry<Real>) -> Self {
        Self {
            local_anchor1,
            local_anchor2,
            impulse: SpacialVector::zeros(),
            min_position: SpacialVector::zeros(),
            max_position: SpacialVector::zeros(),
            target_velocity: SpacialVector::zeros(),
            max_negative_impulse: SpacialVector::repeat(-Real::MAX),
            max_positive_impulse: SpacialVector::repeat(Real::MAX),
        }
    }
}

impl From<RevoluteJoint> for GenericJoint {
    fn from(joint: RevoluteJoint) -> Self {
        let basis1 = [joint.local_axis1, joint.basis1[0], joint.basis1[1]];
        let basis2 = [joint.local_axis2, joint.basis2[0], joint.basis2[1]];
        let quat1 = UnitQuaternion::from_basis_unchecked(&basis1[..]);
        let quat2 = UnitQuaternion::from_basis_unchecked(&basis2[..]);
        let local_anchor1 = Isometry::from_parts(joint.local_anchor1.coords.into(), quat1);
        let local_anchor2 = Isometry::from_parts(joint.local_anchor2.coords.into(), quat2);

        let mut result = Self::new(local_anchor1, local_anchor2);
        result.min_position[3] = -Real::MAX;
        result.max_position[3] = Real::MAX;
        result
    }
}
