use crate::math::{Point, Vector};
use crate::utils::WBasis;
use na::{Unit, Vector5};

#[derive(Copy, Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A joint that removes all relative motion between two bodies, except for the rotations along one axis.
pub struct RevoluteJoint {
    /// Where the revolute joint is attached on the first body, expressed in the local space of the first attached body.
    pub local_anchor1: Point<f32>,
    /// Where the revolute joint is attached on the second body, expressed in the local space of the second attached body.
    pub local_anchor2: Point<f32>,
    /// The rotation axis of this revolute joint expressed in the local space of the first attached body.
    pub local_axis1: Unit<Vector<f32>>,
    /// The rotation axis of this revolute joint expressed in the local space of the second attached body.
    pub local_axis2: Unit<Vector<f32>>,
    /// The basis orthonormal to `local_axis1`, expressed in the local space of the first attached body.
    pub basis1: [Vector<f32>; 2],
    /// The basis orthonormal to `local_axis2`, expressed in the local space of the second attached body.
    pub basis2: [Vector<f32>; 2],
    /// The impulse applied by this joint on the first body.
    ///
    /// The impulse applied to the second body is given by `-impulse`.
    pub impulse: Vector5<f32>,
}

impl RevoluteJoint {
    /// Creates a new revolute joint with the given point of applications and axis, all expressed
    /// in the local-space of the affected bodies.
    pub fn new(
        local_anchor1: Point<f32>,
        local_axis1: Unit<Vector<f32>>,
        local_anchor2: Point<f32>,
        local_axis2: Unit<Vector<f32>>,
    ) -> Self {
        Self {
            local_anchor1,
            local_anchor2,
            local_axis1,
            local_axis2,
            basis1: local_axis1.orthonormal_basis(),
            basis2: local_axis2.orthonormal_basis(),
            impulse: na::zero(),
        }
    }
}
