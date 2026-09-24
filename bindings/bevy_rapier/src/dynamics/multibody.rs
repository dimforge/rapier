use crate::math::{Real, Rot};
use crate::reflect::JointAxisWrapper;
use bevy::prelude::*;
use rapier::dynamics::{JointAxesMask, JointAxis, MultibodyJoint as RapierMultibodyJoint};
use rapier::math::SPATIAL_DIM;

pub use rapier::dynamics::InverseKinematicsOption;

use super::JOINT_DOFS;

#[cfg(doc)]
use super::MultibodyJoint;
#[cfg(doc)]
use crate::plugin::context::RapierContextJoints;

/// Iterates through the free degrees of freedom of a joint with the given locked axes, as
/// `(dof, axis)` pairs, where `dof` is the index of the DoF in the joint’s slice of the
/// multibody’s generalized coordinates, and `axis` its [`JointAxis`] index.
pub(crate) fn free_joint_dofs(locked_axes: JointAxesMask) -> impl Iterator<Item = (usize, usize)> {
    let locked = locked_axes.bits();
    (0..SPATIAL_DIM)
        .filter(move |axis| locked & (1 << axis) == 0)
        .enumerate()
}

/// Marker component making the [`MultibodyJoint`] of its entity kinematic.
///
/// The velocities of a kinematic multibody joint are never modified by the physics engine: its
/// degrees of freedom only move according to the generalized velocities set by the user (see
/// [`RapierContextJoints::multibody_joint_velocity_mut`]), ignoring gravity, contacts and the
/// other joints of the multibody. This can be added or removed at any time.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct KinematicMultibodyJoint;

/// The per-DoF viscous damping of a [`MultibodyJoint`], indexed by [`JointAxis`].
///
/// The damping applies a generalized force opposed to the joint velocity along each free
/// degree of freedom. Values along locked axes are ignored. Removing this component restores the
/// default damping (see [`MultibodyJointDamping::default`]).
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct MultibodyJointDamping(pub [Real; JOINT_DOFS]);

impl Default for MultibodyJointDamping {
    /// The damping Rapier assigns to new multibody joints: `0.1` on angular axes, zero on
    /// linear axes.
    fn default() -> Self {
        let lin_dofs = JOINT_DOFS - rapier::math::ANG_DIM;
        Self(std::array::from_fn(
            |i| if i < lin_dofs { 0.0 } else { 0.1 },
        ))
    }
}

impl MultibodyJointDamping {
    /// The same damping along every axis.
    pub fn splat(damping: Real) -> Self {
        Self([damping; JOINT_DOFS])
    }

    /// Sets the damping along the given axis.
    pub fn with(mut self, axis: JointAxis, damping: Real) -> Self {
        self.0[axis as usize] = damping;
        self
    }
}

/// The per-DoF dry friction of a [`MultibodyJoint`], indexed by [`JointAxis`].
///
/// Each value is the largest generalized force (a force for linear axes, a torque for angular
/// axes) friction may apply to stop the motion of the corresponding degree of freedom. Values
/// along locked axes are ignored. Removing this component removes the friction.
#[derive(Copy, Clone, Default, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct MultibodyJointFriction(pub [Real; JOINT_DOFS]);

impl MultibodyJointFriction {
    /// The same friction along every axis.
    pub fn splat(friction: Real) -> Self {
        Self([friction; JOINT_DOFS])
    }

    /// Sets the friction along the given axis.
    pub fn with(mut self, axis: JointAxis, friction: Real) -> Self {
        self.0[axis as usize] = friction;
        self
    }
}

/// The per-DoF armature (reflected rotor inertia) of a [`MultibodyJoint`], indexed by
/// [`JointAxis`].
///
/// This inertia is added to the multibody’s mass matrix along each free degree of freedom, which
/// typically models the inertia of an actuator and improves stability. Values along locked axes
/// are ignored. Removing this component removes the armature.
#[derive(Copy, Clone, Default, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct MultibodyJointArmature(pub [Real; JOINT_DOFS]);

impl MultibodyJointArmature {
    /// The same armature along every axis.
    pub fn splat(armature: Real) -> Self {
        Self([armature; JOINT_DOFS])
    }

    /// Sets the armature along the given axis.
    pub fn with(mut self, axis: JointAxis, armature: Real) -> Self {
        self.0[axis as usize] = armature;
        self
    }
}

/// Passive springs acting on the degrees of freedom of a [`MultibodyJoint`], indexed by
/// [`JointAxis`].
///
/// Along each free axis, the spring applies the generalized force
/// `-stiffness * (coord - rest)`, integrated implicitly (so stiff springs remain stable), where
/// `coord` is the joint coordinate reported by [`MultibodyJointState::coords`]. Axes with a zero
/// stiffness have no spring. Removing this component removes all the springs.
#[derive(Copy, Clone, Default, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct MultibodyJointSprings {
    /// The spring stiffness along each axis.
    pub stiffness: [Real; JOINT_DOFS],
    /// The rest coordinate of the spring along each axis.
    pub rest: [Real; JOINT_DOFS],
}

impl MultibodyJointSprings {
    /// Sets the spring along the given axis.
    pub fn with(mut self, axis: JointAxis, stiffness: Real, rest: Real) -> Self {
        self.stiffness[axis as usize] = stiffness;
        self.rest[axis as usize] = rest;
        self
    }
}

/// A linear coupling between a degree of freedom of this [`MultibodyJoint`] and a degree of
/// freedom of another joint of the same multibody.
///
/// The coupling enforces `coord(axis) = coeff * source_coord(source_axis) + offset`, similar to
/// a "mimic" joint. See [`MultibodyJointCouplings`].
#[derive(Copy, Clone, Debug, PartialEq, Reflect)]
#[reflect(PartialEq)]
pub struct MultibodyJointCoupling {
    /// The coupled axis of the joint containing this coupling.
    #[reflect(remote = JointAxisWrapper)]
    pub axis: JointAxis,
    /// The entity with the [`MultibodyJoint`] (or the root rigid-body) driving this coupling.
    pub source: Entity,
    /// The axis of `source` driving this coupling.
    #[reflect(remote = JointAxisWrapper)]
    pub source_axis: JointAxis,
    /// The linear coupling coefficient.
    pub coeff: Real,
    /// The constant offset of the coupling.
    pub offset: Real,
}

impl MultibodyJointCoupling {
    /// Couples `axis` of this joint to `source_axis` of `source` with
    /// `coord(axis) = coeff * source_coord(source_axis) + offset`.
    pub fn new(
        axis: JointAxis,
        source: Entity,
        source_axis: JointAxis,
        coeff: Real,
        offset: Real,
    ) -> Self {
        Self {
            axis,
            source,
            source_axis,
            coeff,
            offset,
        }
    }
}

/// The couplings between the degrees of freedom of this [`MultibodyJoint`] and other joints of
/// the same multibody.
///
/// A coupling is only active while both joints belong to the same multibody and both coupled
/// axes are free; it is ignored otherwise. Removing a coupling (or this component) removes it
/// from the multibody without affecting the joint state.
#[derive(Clone, Default, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct MultibodyJointCouplings(pub Vec<MultibodyJointCoupling>);

/// Marker component disabling the contacts between the links of the multibody containing the
/// rigid-body of its entity.
///
/// This can be added to any rigid-body of the multibody (typically its root), and overrides the
/// per-joint [`GenericJoint::contacts_enabled`](crate::dynamics::GenericJoint::contacts_enabled)
/// flags. Removing it enables self-contacts again (unless another link of the multibody still has
/// this component).
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct MultibodySelfContactsDisabled;

/// The state of a [`MultibodyJoint`], written back after each simulation step.
///
/// This component is automatically added alongside [`MultibodyJoint`]. Modifying it has no
/// effect: use the helpers of [`RapierContextJoints`] to control the joint coordinates and
/// velocities.
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct MultibodyJointState {
    /// The generalized coordinates of the joint, indexed by [`JointAxis`], zero along locked
    /// axes.
    ///
    /// These are translations along linear axes and angles along angular axes. With more than
    /// one free angular axis (3D only), the angular coordinates are integrated angular
    /// velocities: use [`Self::rotation`] instead.
    pub coords: [Real; JOINT_DOFS],
    /// The generalized velocities of the joint, indexed by [`JointAxis`], zero along locked axes.
    pub velocities: [Real; JOINT_DOFS],
    /// The rotation of the joint’s second frame relative to its first frame.
    pub rotation: Rot,
}

impl Default for MultibodyJointState {
    fn default() -> Self {
        Self {
            coords: [0.0; JOINT_DOFS],
            velocities: [0.0; JOINT_DOFS],
            #[cfg(feature = "dim2")]
            rotation: 0.0,
            #[cfg(feature = "dim3")]
            rotation: Rot::IDENTITY,
        }
    }
}

impl MultibodyJointState {
    /// Reads the state of a Rapier multibody joint, given its generalized velocities (the
    /// joint’s slice of the multibody’s generalized velocities).
    pub fn from_rapier(joint: &RapierMultibodyJoint, velocities: &[Real]) -> Self {
        let coords = joint.coords();
        let mut result = Self {
            coords: std::array::from_fn(|i| coords[i]),
            #[cfg(feature = "dim2")]
            rotation: joint.joint_rot().angle(),
            #[cfg(feature = "dim3")]
            rotation: joint.joint_rot(),
            ..Default::default()
        };
        for (dof, axis) in free_joint_dofs(joint.data.locked_axes) {
            if let Some(vel) = velocities.get(dof) {
                result.velocities[axis] = *vel;
            }
        }
        result
    }
}
