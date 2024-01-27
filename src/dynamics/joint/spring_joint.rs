use crate::dynamics::joint::{GenericJoint, GenericJointBuilder, JointAxesMask};
use crate::dynamics::{JointAxis, MotorModel};
use crate::math::{Point, Real};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
/// A spring-damper joint, applies a force proportional to the distance between two objects.
///
/// The spring is integrated implicitly, implying that even an undamped spring will be subject to some
/// amount of numerical damping (so it will eventually come to a rest). More solver iterations, or smaller
/// timesteps, will lower the effect of numerical damping, providing a more realistic result.
pub struct SpringJoint {
    /// The underlying joint data.
    pub data: GenericJoint,
}

impl SpringJoint {
    /// Creates a new spring joint limiting the max distance between two bodies.
    ///
    /// The `max_dist` must be strictly greater than 0.0.
    pub fn new(rest_length: Real, stiffness: Real, damping: Real) -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::empty())
            .coupled_axes(JointAxesMask::LIN_AXES)
            .motor_position(JointAxis::X, rest_length, stiffness, damping)
            .motor_model(JointAxis::X, MotorModel::ForceBased)
            .build();
        Self { data }
    }

    /// The underlying generic joint.
    pub fn data(&self) -> &GenericJoint {
        &self.data
    }

    /// Are contacts between the attached rigid-bodies enabled?
    pub fn contacts_enabled(&self) -> bool {
        self.data.contacts_enabled
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    pub fn set_contacts_enabled(&mut self, enabled: bool) -> &mut Self {
        self.data.set_contacts_enabled(enabled);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(&self) -> Point<Real> {
        self.data.local_anchor1()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    pub fn set_local_anchor1(&mut self, anchor1: Point<Real>) -> &mut Self {
        self.data.set_local_anchor1(anchor1);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(&self) -> Point<Real> {
        self.data.local_anchor2()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    pub fn set_local_anchor2(&mut self, anchor2: Point<Real>) -> &mut Self {
        self.data.set_local_anchor2(anchor2);
        self
    }

    /// Set the spring model used by this joint to reach the desired target velocity and position.
    ///
    /// Setting this to `MotorModel::ForceBased` (which is the default value for this joint) makes the spring constants
    /// (stiffness and damping) parameter understood as in the regular spring-mass-damper system. With
    /// `MotorModel::AccelerationBased`, the spring constants will be automatically scaled by the attached masses,
    /// making the spring more mass-independent.
    pub fn set_spring_model(&mut self, model: MotorModel) -> &mut Self {
        self.data.set_motor_model(JointAxis::X, model);
        self
    }

    // /// The maximum distance allowed between the attached objects.
    // #[must_use]
    // pub fn rest_length(&self) -> Option<Real> {
    //     self.data.limits(JointAxis::X).map(|l| l.max)
    // }
    //
    // /// Sets the maximum allowed distance between the attached objects.
    // ///
    // /// The `max_dist` must be strictly greater than 0.0.
    // pub fn set_rest_length(&mut self, max_dist: Real) -> &mut Self {
    //     self.data.set_limits(JointAxis::X, [0.0, max_dist]);
    //     self
    // }
}

impl From<SpringJoint> for GenericJoint {
    fn from(val: SpringJoint) -> GenericJoint {
        val.data
    }
}

/// A [SpringJoint] joint using the builder pattern.
///
/// This builds a spring-damper joint which applies a force proportional to the distance between two objects.
/// See the documentation of [SpringJoint] for more information on its behavior.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpringJointBuilder(pub SpringJoint);

impl SpringJointBuilder {
    /// Creates a new builder for spring joints.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    pub fn new(rest_length: Real, stiffness: Real, damping: Real) -> Self {
        Self(SpringJoint::new(rest_length, stiffness, damping))
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    #[must_use]
    pub fn contacts_enabled(mut self, enabled: bool) -> Self {
        self.0.set_contacts_enabled(enabled);
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Point<Real>) -> Self {
        self.0.set_local_anchor1(anchor1);
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Point<Real>) -> Self {
        self.0.set_local_anchor2(anchor2);
        self
    }

    /// Set the spring used by this joint to reach the desired target velocity and position.
    ///
    /// Setting this to `MotorModel::ForceBased` (which is the default value for this joint) makes the spring constants
    /// (stiffness and damping) parameter understood as in the regular spring-mass-damper system. With
    /// `MotorModel::AccelerationBased`, the spring constants will be automatically scaled by the attached masses,
    /// making the spring more mass-independent.
    #[must_use]
    pub fn spring_model(mut self, model: MotorModel) -> Self {
        self.0.set_spring_model(model);
        self
    }

    // /// Sets the maximum allowed distance between the attached bodies.
    // ///
    // /// The `max_dist` must be strictly greater than 0.0.
    // #[must_use]
    // pub fn max_distance(mut self, max_dist: Real) -> Self {
    //     self.0.set_max_distance(max_dist);
    //     self
    // }

    /// Builds the spring joint.
    #[must_use]
    pub fn build(self) -> SpringJoint {
        self.0
    }
}

impl From<SpringJointBuilder> for GenericJoint {
    fn from(val: SpringJointBuilder) -> GenericJoint {
        val.0.into()
    }
}
