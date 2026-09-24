use crate::math::{AngVector, Real, Vect};
use bevy::prelude::*;
use rapier::dynamics::{
    ImpulseJoint as RapierImpulseJoint, ImpulseJointHandle, MultibodyJointHandle,
};

pub use rapier::dynamics::{JointAxesMask, JointAxis, MotorModel, SpringCoefficients};

use super::{
    FixedJoint, GenericJoint, MultibodyJointState, PrismaticJoint, RevoluteJoint, RopeJoint,
    SpringJoint,
};

#[cfg(doc)]
use super::{
    KinematicMultibodyJoint, MultibodyJointArmature, MultibodyJointCouplings,
    MultibodyJointDamping, MultibodyJointFriction, MultibodyJointSprings,
};

#[cfg(feature = "dim2")]
use super::PinSlotJoint;
#[cfg(feature = "dim3")]
use super::SphericalJoint;

/// Wrapper enum over a specific joint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TypedJoint {
    /// See [`FixedJoint`]
    FixedJoint(FixedJoint),
    /// See [`GenericJoint`]
    GenericJoint(GenericJoint),
    /// See [`PrismaticJoint`]
    PrismaticJoint(PrismaticJoint),
    /// See [`RevoluteJoint`]
    RevoluteJoint(RevoluteJoint),
    /// See [`RopeJoint`]
    RopeJoint(RopeJoint),
    /// See [`SphericalJoint`]
    #[cfg(feature = "dim3")]
    SphericalJoint(SphericalJoint),
    /// See [`SpringJoint`]
    SpringJoint(SpringJoint),
    /// See [`PinSlotJoint`]
    #[cfg(feature = "dim2")]
    PinSlotJoint(PinSlotJoint),
}

impl AsMut<GenericJoint> for TypedJoint {
    fn as_mut(&mut self) -> &mut GenericJoint {
        match self {
            TypedJoint::FixedJoint(ref mut j) => &mut j.data,
            TypedJoint::GenericJoint(ref mut j) => j,
            TypedJoint::PrismaticJoint(ref mut j) => &mut j.data,
            TypedJoint::RevoluteJoint(ref mut j) => &mut j.data,
            TypedJoint::RopeJoint(ref mut j) => &mut j.data,
            #[cfg(feature = "dim3")]
            TypedJoint::SphericalJoint(ref mut j) => &mut j.data,
            TypedJoint::SpringJoint(ref mut j) => &mut j.data,
            #[cfg(feature = "dim2")]
            TypedJoint::PinSlotJoint(ref mut j) => &mut j.data,
        }
    }
}

impl AsRef<GenericJoint> for TypedJoint {
    fn as_ref(&self) -> &GenericJoint {
        match self {
            TypedJoint::FixedJoint(j) => &j.data,
            TypedJoint::GenericJoint(j) => j,
            TypedJoint::PrismaticJoint(j) => &j.data,
            TypedJoint::RevoluteJoint(j) => &j.data,
            TypedJoint::RopeJoint(j) => &j.data,
            #[cfg(feature = "dim3")]
            TypedJoint::SphericalJoint(j) => &j.data,
            TypedJoint::SpringJoint(j) => &j.data,
            #[cfg(feature = "dim2")]
            TypedJoint::PinSlotJoint(j) => &j.data,
        }
    }
}

/// The handle of an impulse joint added to the physics scene.
#[derive(Copy, Clone, Debug, Component)]
pub struct RapierImpulseJointHandle(pub ImpulseJointHandle);

/// The handle of a multibody joint added to the physics scene.
#[derive(Copy, Clone, Debug, Component)]
pub struct RapierMultibodyJointHandle(pub MultibodyJointHandle);

/// An impulse-based joint attached to two entities.
///
/// The first end-point of the joint is the rigid-body attached to
/// `ImpulseJoint::parent`. The second endpoint of the joint is the
/// rigid-body attached to the entity (or the parent of the entity)
/// containing this `ImpulseJoint` component.
///
/// To attach multiple impulse joints to the same rigid-body, multiple
/// joints can be added in the children of the entity containing that
/// rigid-body (this is similar to the technique used to attach multiple
/// colliders to the same rigid-body).
///
/// Modifying `parent` re-attaches the joint to the rigid-body of the new parent entity. Add the
/// [`ImpulseJointDisabled`] component to disable the joint, and read the impulses it applied
/// during the last simulation step from the [`ImpulseJointImpulses`] component.
///
/// Note that the `user_data` of the underlying Rapier joint is reserved: it stores the bits of
/// the entity containing this component.
#[derive(Copy, Clone, Debug, PartialEq, Component)]
#[require(ImpulseJointImpulses)]
pub struct ImpulseJoint {
    /// The entity containing the rigid-body used as the first endpoint of this joint.
    pub parent: Entity,
    /// The joint’s description.
    pub data: TypedJoint,
}

impl ImpulseJoint {
    /// Initializes an impulse-based joint from its first endpoint and the joint description.
    pub fn new(parent: Entity, data: impl Into<TypedJoint>) -> Self {
        Self {
            parent,
            data: data.into(),
        }
    }
}

/// Marker component disabling the [`ImpulseJoint`] of its entity.
///
/// A disabled joint stays attached to its rigid-bodies but is ignored by the constraints solver.
/// Removing this component enables the joint again. Multibody joints cannot be disabled.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct ImpulseJointDisabled;

/// The number of degrees of freedom of a joint (3 in 2D, 6 in 3D).
#[cfg(feature = "dim2")]
pub const JOINT_DOFS: usize = 3;
/// The number of degrees of freedom of a joint (3 in 2D, 6 in 3D).
#[cfg(feature = "dim3")]
pub const JOINT_DOFS: usize = 6;

/// The impulses applied by an [`ImpulseJoint`] during the last simulation step.
///
/// This component is automatically added alongside [`ImpulseJoint`] and updated after each
/// simulation step. Comparing these impulses with a threshold can be used to implement
/// breakable joints (by removing the [`ImpulseJoint`] once the threshold is exceeded).
///
/// All impulses are expressed along the axes of the joint frame attached to the first
/// rigid-body. Divide them by the (sub)step length to get the corresponding forces.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct ImpulseJointImpulses {
    /// The impulse applied along the locked translational degrees of freedom.
    pub linear: Vect,
    /// The angular impulse applied along the locked rotational degrees of freedom.
    pub angular: AngVector,
    /// The impulse applied by the limits along each degree of freedom (see [`JointAxis`]).
    pub limits: [Real; JOINT_DOFS],
    /// The impulse applied by the motors along each degree of freedom (see [`JointAxis`]).
    pub motors: [Real; JOINT_DOFS],
}

impl ImpulseJointImpulses {
    /// Reads the impulses applied by the given Rapier impulse joint.
    pub fn from_rapier(joint: &RapierImpulseJoint) -> Self {
        let impulses = joint.impulses;
        #[cfg(feature = "dim2")]
        let (linear, angular) = (Vect::new(impulses[0], impulses[1]), impulses[2]);
        #[cfg(feature = "dim3")]
        let (linear, angular) = (
            Vect::new(impulses[0], impulses[1], impulses[2]),
            AngVector::new(impulses[3], impulses[4], impulses[5]),
        );
        Self {
            linear,
            angular,
            limits: std::array::from_fn(|i| joint.data.limits[i].impulse),
            motors: std::array::from_fn(|i| joint.data.motors[i].impulse),
        }
    }
}

/// An joint based on generalized coordinates, attached to two entities.
///
/// The first end-point of the joint is the rigid-body attached to
/// `MultibodyJoint::parent`. The second endpoint of the joint is the
/// rigid-body attached to the entity containing this `MultibodyJoint` component.
///
/// Note that a set of multibody joints cannot form closed loops (for example a necklace).
/// If a closed loop is detected, the last joint that closes the loop is ignored, and an
/// error is printed to `stderr` (using `log::error!`).
///
/// Modifying `parent` re-attaches the joint to the rigid-body of the new parent entity. The
/// joint can be further configured with the [`KinematicMultibodyJoint`],
/// [`MultibodyJointDamping`], [`MultibodyJointFriction`], [`MultibodyJointArmature`],
/// [`MultibodyJointSprings`] and [`MultibodyJointCouplings`] components, and its coordinates are
/// read back into the [`MultibodyJointState`] component after each simulation step.
///
/// Note that the `user_data` of the underlying Rapier joint is reserved: it stores the bits of
/// the entity containing this component.
#[derive(Copy, Clone, Debug, PartialEq, Component)]
#[require(MultibodyJointState)]
pub struct MultibodyJoint {
    /// The entity containing the rigid-body used as the first endpoint of this joint.
    pub parent: Entity,
    /// The joint’s description.
    pub data: TypedJoint,
}

impl MultibodyJoint {
    /// Initializes an joint based on reduced coordinates from its first endpoint and
    /// the joint description.
    pub fn new(parent: Entity, data: impl Into<TypedJoint>) -> Self {
        Self {
            parent,
            data: data.into(),
        }
    }
}
