use crate::math::{Point, Real, Vector};

#[derive(Copy, Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A joint that removes all relative linear motion between a pair of points on two bodies.
pub struct BallJoint {
    /// Where the ball joint is attached on the first body, expressed in the first body local frame.
    pub local_anchor1: Point<Real>,
    /// Where the ball joint is attached on the first body, expressed in the first body local frame.
    pub local_anchor2: Point<Real>,
    /// The impulse applied by this joint on the first body.
    ///
    /// The impulse applied to the second body is given by `-impulse`.
    pub impulse: Vector<Real>,
}

impl BallJoint {
    /// Creates a new Ball joint from two anchors given on the local spaces of the respective bodies.
    pub fn new(local_anchor1: Point<Real>, local_anchor2: Point<Real>) -> Self {
        Self::with_impulse(local_anchor1, local_anchor2, Vector::zeros())
    }

    pub(crate) fn with_impulse(
        local_anchor1: Point<Real>,
        local_anchor2: Point<Real>,
        impulse: Vector<Real>,
    ) -> Self {
        Self {
            local_anchor1,
            local_anchor2,
            impulse,
        }
    }
}
