use crate::dynamics::{GenericJoint, GenericJointBuilder};
use crate::math::{Real, Rot, Vect};
use rapier::dynamics::JointAxesMask;

use super::{SpringCoefficients, TypedJoint};

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
/// A fixed joint, locks all relative motion between two bodies.
pub struct FixedJoint {
    /// The underlying joint data.
    pub data: GenericJoint,
}

impl Default for FixedJoint {
    fn default() -> Self {
        FixedJoint::new()
    }
}

impl FixedJoint {
    /// Creates a new fixed joint.
    #[must_use]
    pub fn new() -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::LOCKED_FIXED_AXES).build();
        Self { data }
    }

    /// Are contacts between the attached rigid-bodies enabled?
    pub fn contacts_enabled(&self) -> bool {
        self.data.contacts_enabled()
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    pub fn set_contacts_enabled(&mut self, enabled: bool) -> &mut Self {
        self.data.set_contacts_enabled(enabled);
        self
    }

    /// The spring coefficients controlling the softness of this joint’s locked degrees of freedom.
    #[must_use]
    pub fn softness(&self) -> SpringCoefficients<Real> {
        self.data.softness()
    }

    /// Sets the spring coefficients controlling the softness of this joint’s locked degrees of freedom.
    pub fn set_softness(&mut self, softness: SpringCoefficients<Real>) -> &mut Self {
        self.data.set_softness(softness);
        self
    }

    /// The joint’s basis, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_basis1(&self) -> Rot {
        self.data.local_basis1()
    }

    /// Sets the joint’s basis, expressed in the first rigid-body’s local-space.
    pub fn set_local_basis1(&mut self, local_basis: Rot) -> &mut Self {
        self.data.set_local_basis1(local_basis);
        self
    }

    /// The joint’s basis, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_basis2(&self) -> Rot {
        self.data.local_basis2()
    }

    /// Sets joint’s basis, expressed in the second rigid-body’s local-space.
    pub fn set_local_basis2(&mut self, local_basis: Rot) -> &mut Self {
        self.data.set_local_basis2(local_basis);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(&self) -> Vect {
        self.data.local_anchor1()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    pub fn set_local_anchor1(&mut self, anchor1: Vect) -> &mut Self {
        self.data.set_local_anchor1(anchor1);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(&self) -> Vect {
        self.data.local_anchor2()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    pub fn set_local_anchor2(&mut self, anchor2: Vect) -> &mut Self {
        self.data.set_local_anchor2(anchor2);
        self
    }
}

impl From<FixedJoint> for GenericJoint {
    fn from(joint: FixedJoint) -> GenericJoint {
        joint.data
    }
}

/// Create fixed joints using the builder pattern.
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct FixedJointBuilder(FixedJoint);

impl FixedJointBuilder {
    /// Creates a new builder for fixed joints.
    pub fn new() -> Self {
        Self(FixedJoint::new())
    }

    /// Sets the joint’s basis, expressed in the first rigid-body’s local-space.
    #[must_use]
    pub fn local_basis1(mut self, local_basis: Rot) -> Self {
        self.0.set_local_basis1(local_basis);
        self
    }

    /// Sets joint’s basis, expressed in the second rigid-body’s local-space.
    #[must_use]
    pub fn local_basis2(mut self, local_basis: Rot) -> Self {
        self.0.set_local_basis2(local_basis);
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Vect) -> Self {
        self.0.set_local_anchor1(anchor1);
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Vect) -> Self {
        self.0.set_local_anchor2(anchor2);
        self
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    #[must_use]
    pub fn contacts_enabled(mut self, enabled: bool) -> Self {
        self.0.set_contacts_enabled(enabled);
        self
    }

    /// Sets the spring coefficients controlling the softness of this joint’s locked degrees of freedom.
    #[must_use]
    pub fn softness(mut self, softness: SpringCoefficients<Real>) -> Self {
        self.0.set_softness(softness);
        self
    }

    /// Build the fixed joint.
    #[must_use]
    pub fn build(self) -> FixedJoint {
        self.0
    }
}

impl From<FixedJointBuilder> for TypedJoint {
    fn from(joint: FixedJointBuilder) -> TypedJoint {
        joint.0.into()
    }
}

impl From<FixedJoint> for TypedJoint {
    fn from(joint: FixedJoint) -> TypedJoint {
        TypedJoint::FixedJoint(joint)
    }
}
