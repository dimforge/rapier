#[cfg(feature = "dim3")]
use crate::dynamics::RevoluteJoint;
use crate::dynamics::{BallJoint, FixedJoint, JointHandle, PrismaticJoint, RigidBodyHandle};

#[derive(Copy, Clone)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// An enum grouping all possible types of joints.
pub enum JointParams {
    /// A Ball joint that removes all relative linear degrees of freedom between the affected bodies.
    BallJoint(BallJoint),
    /// A fixed joint that removes all relative degrees of freedom between the affected bodies.
    FixedJoint(FixedJoint),
    /// A prismatic joint that removes all degrees of degrees of freedom between the affected
    /// bodies except for the translation along one axis.
    PrismaticJoint(PrismaticJoint),
    #[cfg(feature = "dim3")]
    /// A revolute joint that removes all degrees of degrees of freedom between the affected
    /// bodies except for the translation along one axis.
    RevoluteJoint(RevoluteJoint),
}

impl JointParams {
    /// An integer identifier for each type of joint.
    pub fn type_id(&self) -> usize {
        match self {
            JointParams::BallJoint(_) => 0,
            JointParams::FixedJoint(_) => 1,
            JointParams::PrismaticJoint(_) => 2,
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(_) => 3,
        }
    }

    /// Gets a reference to the underlying ball joint, if `self` is one.
    pub fn as_ball_joint(&self) -> Option<&BallJoint> {
        if let JointParams::BallJoint(j) = self {
            Some(j)
        } else {
            None
        }
    }

    /// Gets a reference to the underlying fixed joint, if `self` is one.
    pub fn as_fixed_joint(&self) -> Option<&FixedJoint> {
        if let JointParams::FixedJoint(j) = self {
            Some(j)
        } else {
            None
        }
    }

    /// Gets a reference to the underlying prismatic joint, if `self` is one.
    pub fn as_prismatic_joint(&self) -> Option<&PrismaticJoint> {
        if let JointParams::PrismaticJoint(j) = self {
            Some(j)
        } else {
            None
        }
    }

    /// Gets a reference to the underlying revolute joint, if `self` is one.
    #[cfg(feature = "dim3")]
    pub fn as_revolute_joint(&self) -> Option<&RevoluteJoint> {
        if let JointParams::RevoluteJoint(j) = self {
            Some(j)
        } else {
            None
        }
    }
}

impl From<BallJoint> for JointParams {
    fn from(j: BallJoint) -> Self {
        JointParams::BallJoint(j)
    }
}

impl From<FixedJoint> for JointParams {
    fn from(j: FixedJoint) -> Self {
        JointParams::FixedJoint(j)
    }
}

#[cfg(feature = "dim3")]
impl From<RevoluteJoint> for JointParams {
    fn from(j: RevoluteJoint) -> Self {
        JointParams::RevoluteJoint(j)
    }
}

impl From<PrismaticJoint> for JointParams {
    fn from(j: PrismaticJoint) -> Self {
        JointParams::PrismaticJoint(j)
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// A joint attached to two bodies.
pub struct Joint {
    /// Handle to the first body attached to this joint.
    pub body1: RigidBodyHandle,
    /// Handle to the second body attached to this joint.
    pub body2: RigidBodyHandle,
    // A joint needs to know its handle to simplify its removal.
    pub(crate) handle: JointHandle,
    #[cfg(feature = "parallel")]
    pub(crate) constraint_index: usize,
    #[cfg(feature = "parallel")]
    pub(crate) position_constraint_index: usize,
    /// The joint geometric parameters and impulse.
    pub params: JointParams,
}
