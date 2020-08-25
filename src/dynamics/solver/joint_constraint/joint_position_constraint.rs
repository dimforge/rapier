use super::{
    BallPositionConstraint, BallPositionGroundConstraint, FixedPositionConstraint,
    FixedPositionGroundConstraint, PrismaticPositionConstraint, PrismaticPositionGroundConstraint,
};
#[cfg(feature = "dim3")]
use super::{RevolutePositionConstraint, RevolutePositionGroundConstraint};
#[cfg(feature = "simd-is-enabled")]
use super::{WBallPositionConstraint, WBallPositionGroundConstraint};
use crate::dynamics::{IntegrationParameters, Joint, JointParams, RigidBodySet};
use crate::math::Isometry;
#[cfg(feature = "simd-is-enabled")]
use crate::math::SIMD_WIDTH;

pub(crate) enum AnyJointPositionConstraint {
    BallJoint(BallPositionConstraint),
    BallGroundConstraint(BallPositionGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WBallJoint(WBallPositionConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WBallGroundConstraint(WBallPositionGroundConstraint),
    FixedJoint(FixedPositionConstraint),
    FixedGroundConstraint(FixedPositionGroundConstraint),
    PrismaticJoint(PrismaticPositionConstraint),
    PrismaticGroundConstraint(PrismaticPositionGroundConstraint),
    #[cfg(feature = "dim3")]
    RevoluteJoint(RevolutePositionConstraint),
    #[cfg(feature = "dim3")]
    RevoluteGroundConstraint(RevolutePositionGroundConstraint),
    #[allow(dead_code)] // The Empty variant is only used with parallel code.
    Empty,
}

impl AnyJointPositionConstraint {
    #[cfg(feature = "parallel")]
    pub fn num_active_constraints(joint: &Joint, grouped: bool) -> usize {
        #[cfg(feature = "simd-is-enabled")]
        if !grouped {
            1
        } else {
            match &joint.params {
                JointParams::BallJoint(_) => 1,
                _ => SIMD_WIDTH, // For joints that don't support SIMD position constraints yet.
            }
        }

        #[cfg(not(feature = "simd-is-enabled"))]
        {
            1
        }
    }

    pub fn from_joint(joint: &Joint, bodies: &RigidBodySet) -> Self {
        let rb1 = &bodies[joint.body1];
        let rb2 = &bodies[joint.body2];

        match &joint.params {
            JointParams::BallJoint(p) => AnyJointPositionConstraint::BallJoint(
                BallPositionConstraint::from_params(rb1, rb2, p),
            ),
            JointParams::FixedJoint(p) => AnyJointPositionConstraint::FixedJoint(
                FixedPositionConstraint::from_params(rb1, rb2, p),
            ),
            JointParams::PrismaticJoint(p) => AnyJointPositionConstraint::PrismaticJoint(
                PrismaticPositionConstraint::from_params(rb1, rb2, p),
            ),
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(p) => AnyJointPositionConstraint::RevoluteJoint(
                RevolutePositionConstraint::from_params(rb1, rb2, p),
            ),
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    pub fn from_wide_joint(joints: [&Joint; SIMD_WIDTH], bodies: &RigidBodySet) -> Option<Self> {
        let rbs1 = array![|ii| &bodies[joints[ii].body1]; SIMD_WIDTH];
        let rbs2 = array![|ii| &bodies[joints[ii].body2]; SIMD_WIDTH];

        match &joints[0].params {
            JointParams::BallJoint(_) => {
                let joints = array![|ii| joints[ii].params.as_ball_joint().unwrap(); SIMD_WIDTH];
                Some(AnyJointPositionConstraint::WBallJoint(
                    WBallPositionConstraint::from_params(rbs1, rbs2, joints),
                ))
            }
            JointParams::FixedJoint(_) => None,
            JointParams::PrismaticJoint(_) => None,
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(_) => None,
        }
    }

    pub fn from_joint_ground(joint: &Joint, bodies: &RigidBodySet) -> Self {
        let mut rb1 = &bodies[joint.body1];
        let mut rb2 = &bodies[joint.body2];
        let flipped = !rb2.is_dynamic();

        if flipped {
            std::mem::swap(&mut rb1, &mut rb2);
        }

        match &joint.params {
            JointParams::BallJoint(p) => AnyJointPositionConstraint::BallGroundConstraint(
                BallPositionGroundConstraint::from_params(rb1, rb2, p, flipped),
            ),
            JointParams::FixedJoint(p) => AnyJointPositionConstraint::FixedGroundConstraint(
                FixedPositionGroundConstraint::from_params(rb1, rb2, p, flipped),
            ),
            JointParams::PrismaticJoint(p) => {
                AnyJointPositionConstraint::PrismaticGroundConstraint(
                    PrismaticPositionGroundConstraint::from_params(rb1, rb2, p, flipped),
                )
            }
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(p) => AnyJointPositionConstraint::RevoluteGroundConstraint(
                RevolutePositionGroundConstraint::from_params(rb1, rb2, p, flipped),
            ),
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    pub fn from_wide_joint_ground(
        joints: [&Joint; SIMD_WIDTH],
        bodies: &RigidBodySet,
    ) -> Option<Self> {
        let mut rbs1 = array![|ii| &bodies[joints[ii].body1]; SIMD_WIDTH];
        let mut rbs2 = array![|ii| &bodies[joints[ii].body2]; SIMD_WIDTH];
        let mut flipped = [false; SIMD_WIDTH];

        for ii in 0..SIMD_WIDTH {
            if !rbs2[ii].is_dynamic() {
                std::mem::swap(&mut rbs1[ii], &mut rbs2[ii]);
                flipped[ii] = true;
            }
        }

        match &joints[0].params {
            JointParams::BallJoint(_) => {
                let joints = array![|ii| joints[ii].params.as_ball_joint().unwrap(); SIMD_WIDTH];
                Some(AnyJointPositionConstraint::WBallGroundConstraint(
                    WBallPositionGroundConstraint::from_params(rbs1, rbs2, joints, flipped),
                ))
            }
            JointParams::FixedJoint(_) => None,
            JointParams::PrismaticJoint(_) => None,
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(_) => None,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<f32>]) {
        match self {
            AnyJointPositionConstraint::BallJoint(c) => c.solve(params, positions),
            AnyJointPositionConstraint::BallGroundConstraint(c) => c.solve(params, positions),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointPositionConstraint::WBallJoint(c) => c.solve(params, positions),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointPositionConstraint::WBallGroundConstraint(c) => c.solve(params, positions),
            AnyJointPositionConstraint::FixedJoint(c) => c.solve(params, positions),
            AnyJointPositionConstraint::FixedGroundConstraint(c) => c.solve(params, positions),
            AnyJointPositionConstraint::PrismaticJoint(c) => c.solve(params, positions),
            AnyJointPositionConstraint::PrismaticGroundConstraint(c) => c.solve(params, positions),
            #[cfg(feature = "dim3")]
            AnyJointPositionConstraint::RevoluteJoint(c) => c.solve(params, positions),
            #[cfg(feature = "dim3")]
            AnyJointPositionConstraint::RevoluteGroundConstraint(c) => c.solve(params, positions),
            AnyJointPositionConstraint::Empty => unreachable!(),
        }
    }
}
