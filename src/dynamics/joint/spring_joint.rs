use crate::dynamics::joint::{GenericJoint, GenericJointBuilder, JointAxesMask};
use crate::dynamics::{JointAxis, MotorModel};
use crate::math::{Real, Vector};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
/// A spring joint that pulls/pushes two bodies toward a target distance (like a spring or shock absorber).
///
/// Springs apply force based on:
/// - **Stiffness**: How strong the spring force is (higher = stiffer spring)
/// - **Damping**: Resistance to motion (higher = less bouncy, settles faster)
/// - **Rest length**: The distance the spring "wants" to be
///
/// Use for:
/// - Suspension systems (vehicles, furniture)
/// - Bouncy connections
/// - Soft constraints between objects
/// - Procedural animation (following targets softly)
///
/// **Technical note**: Springs use implicit integration which adds some numerical damping.
/// This means even zero-damping springs will eventually settle (more iterations = less damping).
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
            .motor_position(JointAxis::LinX, rest_length, stiffness, damping)
            .motor_model(JointAxis::LinX, MotorModel::ForceBased)
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
    pub fn local_anchor1(&self) -> Vector {
        self.data.local_anchor1()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    pub fn set_local_anchor1(&mut self, anchor1: Vector) -> &mut Self {
        self.data.set_local_anchor1(anchor1);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(&self) -> Vector {
        self.data.local_anchor2()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    pub fn set_local_anchor2(&mut self, anchor2: Vector) -> &mut Self {
        self.data.set_local_anchor2(anchor2);
        self
    }

    /// Sets whether spring constants are mass-dependent or mass-independent.
    ///
    /// - `MotorModel::ForceBased` (default): Stiffness/damping are absolute values.
    ///   Heavier objects will respond differently to the same spring constants.
    /// - `MotorModel::AccelerationBased`: Spring constants auto-scale with mass.
    ///   Objects of different masses behave more similarly.
    ///
    /// Most users can ignore this and use the default.
    pub fn set_spring_model(&mut self, model: MotorModel) -> &mut Self {
        self.data.set_motor_model(JointAxis::LinX, model);
        self
    }

    /// The distance the spring tries to reach between the two anchors.
    #[must_use]
    pub fn rest_length(&self) -> Real {
        self.data
            .motor(JointAxis::LinX)
            .map(|m| m.target_pos)
            .unwrap_or(0.0)
    }

    /// Sets the distance the spring tries to reach between the two anchors.
    ///
    /// The stiffness, damping and spring model are left unchanged.
    pub fn set_rest_length(&mut self, rest_length: Real) -> &mut Self {
        let (target_vel, stiffness, damping) = self
            .data
            .motor(JointAxis::LinX)
            .map(|m| (m.target_vel, m.stiffness, m.damping))
            .unwrap_or((0.0, 0.0, 0.0));
        self.data
            .set_motor(JointAxis::LinX, rest_length, target_vel, stiffness, damping);
        self
    }
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
    pub fn local_anchor1(mut self, anchor1: Vector) -> Self {
        self.0.set_local_anchor1(anchor1);
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Vector) -> Self {
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

    /// Sets the distance the spring tries to reach between the two anchors.
    #[must_use]
    pub fn rest_length(mut self, rest_length: Real) -> Self {
        self.0.set_rest_length(rest_length);
        self
    }

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

#[cfg(test)]
mod test {
    use super::*;
    use crate::dynamics::MotorModel;

    #[test]
    fn set_rest_length_keeps_spring_constants() {
        let mut joint = SpringJoint::new(1.0, 20.0, 3.0);
        joint.set_spring_model(MotorModel::AccelerationBased);
        assert_eq!(joint.rest_length(), 1.0);
        joint.set_rest_length(2.5);
        assert_eq!(joint.rest_length(), 2.5);
        let motor = joint.data.motor(JointAxis::LinX).unwrap();
        assert_eq!(motor.stiffness, 20.0);
        assert_eq!(motor.damping, 3.0);
        assert_eq!(motor.model, MotorModel::AccelerationBased);
    }

    #[test]
    fn builder_rest_length() {
        let joint = SpringJointBuilder::new(1.0, 20.0, 3.0)
            .rest_length(4.0)
            .build();
        assert_eq!(joint.rest_length(), 4.0);
    }
}
