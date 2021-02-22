use super::{
    BallVelocityConstraint, BallVelocityGroundConstraint, FixedVelocityConstraint,
    FixedVelocityGroundConstraint, PrismaticVelocityConstraint, PrismaticVelocityGroundConstraint,
};
#[cfg(feature = "dim3")]
use super::{RevoluteVelocityConstraint, RevoluteVelocityGroundConstraint};
#[cfg(feature = "simd-is-enabled")]
use super::{
    WBallVelocityConstraint, WBallVelocityGroundConstraint, WFixedVelocityConstraint,
    WFixedVelocityGroundConstraint, WPrismaticVelocityConstraint,
    WPrismaticVelocityGroundConstraint,
};
#[cfg(feature = "dim3")]
#[cfg(feature = "simd-is-enabled")]
use super::{WRevoluteVelocityConstraint, WRevoluteVelocityGroundConstraint};
// use crate::dynamics::solver::joint_constraint::generic_velocity_constraint::{
//     GenericVelocityConstraint, GenericVelocityGroundConstraint,
// };
use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    IntegrationParameters, Joint, JointGraphEdge, JointIndex, JointParams, RigidBodySet,
};
use crate::math::Real;
#[cfg(feature = "simd-is-enabled")]
use crate::math::SIMD_WIDTH;

pub(crate) enum AnyJointVelocityConstraint {
    BallConstraint(BallVelocityConstraint),
    BallGroundConstraint(BallVelocityGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WBallConstraint(WBallVelocityConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WBallGroundConstraint(WBallVelocityGroundConstraint),
    FixedConstraint(FixedVelocityConstraint),
    FixedGroundConstraint(FixedVelocityGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WFixedConstraint(WFixedVelocityConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WFixedGroundConstraint(WFixedVelocityGroundConstraint),
    // GenericConstraint(GenericVelocityConstraint),
    // GenericGroundConstraint(GenericVelocityGroundConstraint),
    // #[cfg(feature = "simd-is-enabled")]
    // WGenericConstraint(WGenericVelocityConstraint),
    // #[cfg(feature = "simd-is-enabled")]
    // WGenericGroundConstraint(WGenericVelocityGroundConstraint),
    PrismaticConstraint(PrismaticVelocityConstraint),
    PrismaticGroundConstraint(PrismaticVelocityGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WPrismaticConstraint(WPrismaticVelocityConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WPrismaticGroundConstraint(WPrismaticVelocityGroundConstraint),
    #[cfg(feature = "dim3")]
    RevoluteConstraint(RevoluteVelocityConstraint),
    #[cfg(feature = "dim3")]
    RevoluteGroundConstraint(RevoluteVelocityGroundConstraint),
    #[cfg(feature = "dim3")]
    #[cfg(feature = "simd-is-enabled")]
    WRevoluteConstraint(WRevoluteVelocityConstraint),
    #[cfg(feature = "dim3")]
    #[cfg(feature = "simd-is-enabled")]
    WRevoluteGroundConstraint(WRevoluteVelocityGroundConstraint),
    #[allow(dead_code)] // The Empty variant is only used with parallel code.
    Empty,
}

impl AnyJointVelocityConstraint {
    #[cfg(feature = "parallel")]
    pub fn num_active_constraints(_: &Joint) -> usize {
        1
    }

    pub fn from_joint(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        joint: &Joint,
        bodies: &RigidBodySet,
    ) -> Self {
        let rb1 = &bodies[joint.body1];
        let rb2 = &bodies[joint.body2];

        match &joint.params {
            JointParams::BallJoint(p) => AnyJointVelocityConstraint::BallConstraint(
                BallVelocityConstraint::from_params(params, joint_id, rb1, rb2, p),
            ),
            JointParams::FixedJoint(p) => AnyJointVelocityConstraint::FixedConstraint(
                FixedVelocityConstraint::from_params(params, joint_id, rb1, rb2, p),
            ),
            JointParams::PrismaticJoint(p) => AnyJointVelocityConstraint::PrismaticConstraint(
                PrismaticVelocityConstraint::from_params(params, joint_id, rb1, rb2, p),
            ),
            // JointParams::GenericJoint(p) => AnyJointVelocityConstraint::GenericConstraint(
            //     GenericVelocityConstraint::from_params(params, joint_id, rb1, rb2, p),
            // ),
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(p) => AnyJointVelocityConstraint::RevoluteConstraint(
                RevoluteVelocityConstraint::from_params(params, joint_id, rb1, rb2, p),
            ),
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    pub fn from_wide_joint(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        joints: [&Joint; SIMD_WIDTH],
        bodies: &RigidBodySet,
    ) -> Self {
        let rbs1 = array![|ii| &bodies[joints[ii].body1]; SIMD_WIDTH];
        let rbs2 = array![|ii| &bodies[joints[ii].body2]; SIMD_WIDTH];

        match &joints[0].params {
            JointParams::BallJoint(_) => {
                let joints = array![|ii| joints[ii].params.as_ball_joint().unwrap(); SIMD_WIDTH];
                AnyJointVelocityConstraint::WBallConstraint(WBallVelocityConstraint::from_params(
                    params, joint_id, rbs1, rbs2, joints,
                ))
            }
            JointParams::FixedJoint(_) => {
                let joints = array![|ii| joints[ii].params.as_fixed_joint().unwrap(); SIMD_WIDTH];
                AnyJointVelocityConstraint::WFixedConstraint(WFixedVelocityConstraint::from_params(
                    params, joint_id, rbs1, rbs2, joints,
                ))
            }
            // JointParams::GenericJoint(_) => {
            //     let joints = array![|ii| joints[ii].params.as_generic_joint().unwrap(); SIMD_WIDTH];
            //     AnyJointVelocityConstraint::WGenericConstraint(
            //         WGenericVelocityConstraint::from_params(params, joint_id, rbs1, rbs2, joints),
            //     )
            // }
            JointParams::PrismaticJoint(_) => {
                let joints =
                    array![|ii| joints[ii].params.as_prismatic_joint().unwrap(); SIMD_WIDTH];
                AnyJointVelocityConstraint::WPrismaticConstraint(
                    WPrismaticVelocityConstraint::from_params(params, joint_id, rbs1, rbs2, joints),
                )
            }
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(_) => {
                let joints =
                    array![|ii| joints[ii].params.as_revolute_joint().unwrap(); SIMD_WIDTH];
                AnyJointVelocityConstraint::WRevoluteConstraint(
                    WRevoluteVelocityConstraint::from_params(params, joint_id, rbs1, rbs2, joints),
                )
            }
        }
    }

    pub fn from_joint_ground(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        joint: &Joint,
        bodies: &RigidBodySet,
    ) -> Self {
        let mut rb1 = &bodies[joint.body1];
        let mut rb2 = &bodies[joint.body2];
        let flipped = !rb2.is_dynamic();

        if flipped {
            std::mem::swap(&mut rb1, &mut rb2);
        }

        match &joint.params {
            JointParams::BallJoint(p) => AnyJointVelocityConstraint::BallGroundConstraint(
                BallVelocityGroundConstraint::from_params(params, joint_id, rb1, rb2, p, flipped),
            ),
            JointParams::FixedJoint(p) => AnyJointVelocityConstraint::FixedGroundConstraint(
                FixedVelocityGroundConstraint::from_params(params, joint_id, rb1, rb2, p, flipped),
            ),
            // JointParams::GenericJoint(p) => AnyJointVelocityConstraint::GenericGroundConstraint(
            //     GenericVelocityGroundConstraint::from_params(
            //         params, joint_id, rb1, rb2, p, flipped,
            //     ),
            // ),
            JointParams::PrismaticJoint(p) => {
                AnyJointVelocityConstraint::PrismaticGroundConstraint(
                    PrismaticVelocityGroundConstraint::from_params(
                        params, joint_id, rb1, rb2, p, flipped,
                    ),
                )
            }
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(p) => RevoluteVelocityGroundConstraint::from_params(
                params, joint_id, rb1, rb2, p, flipped,
            ),
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    pub fn from_wide_joint_ground(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        joints: [&Joint; SIMD_WIDTH],
        bodies: &RigidBodySet,
    ) -> Self {
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
                AnyJointVelocityConstraint::WBallGroundConstraint(
                    WBallVelocityGroundConstraint::from_params(
                        params, joint_id, rbs1, rbs2, joints, flipped,
                    ),
                )
            }
            JointParams::FixedJoint(_) => {
                let joints = array![|ii| joints[ii].params.as_fixed_joint().unwrap(); SIMD_WIDTH];
                AnyJointVelocityConstraint::WFixedGroundConstraint(
                    WFixedVelocityGroundConstraint::from_params(
                        params, joint_id, rbs1, rbs2, joints, flipped,
                    ),
                )
            }
            // JointParams::GenericJoint(_) => {
            //     let joints = array![|ii| joints[ii].params.as_generic_joint().unwrap(); SIMD_WIDTH];
            //     AnyJointVelocityConstraint::WGenericGroundConstraint(
            //         WGenericVelocityGroundConstraint::from_params(
            //             params, joint_id, rbs1, rbs2, joints, flipped,
            //         ),
            //     )
            // }
            JointParams::PrismaticJoint(_) => {
                let joints =
                    array![|ii| joints[ii].params.as_prismatic_joint().unwrap(); SIMD_WIDTH];
                AnyJointVelocityConstraint::WPrismaticGroundConstraint(
                    WPrismaticVelocityGroundConstraint::from_params(
                        params, joint_id, rbs1, rbs2, joints, flipped,
                    ),
                )
            }
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(_) => {
                let joints =
                    array![|ii| joints[ii].params.as_revolute_joint().unwrap(); SIMD_WIDTH];
                AnyJointVelocityConstraint::WRevoluteGroundConstraint(
                    WRevoluteVelocityGroundConstraint::from_params(
                        params, joint_id, rbs1, rbs2, joints, flipped,
                    ),
                )
            }
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<Real>]) {
        match self {
            AnyJointVelocityConstraint::BallConstraint(c) => c.warmstart(mj_lambdas),
            AnyJointVelocityConstraint::BallGroundConstraint(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WBallConstraint(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WBallGroundConstraint(c) => c.warmstart(mj_lambdas),
            AnyJointVelocityConstraint::FixedConstraint(c) => c.warmstart(mj_lambdas),
            AnyJointVelocityConstraint::FixedGroundConstraint(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WFixedConstraint(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WFixedGroundConstraint(c) => c.warmstart(mj_lambdas),
            // AnyJointVelocityConstraint::GenericConstraint(c) => c.warmstart(mj_lambdas),
            // AnyJointVelocityConstraint::GenericGroundConstraint(c) => c.warmstart(mj_lambdas),
            // #[cfg(feature = "simd-is-enabled")]
            // AnyJointVelocityConstraint::WGenericConstraint(c) => c.warmstart(mj_lambdas),
            // #[cfg(feature = "simd-is-enabled")]
            // AnyJointVelocityConstraint::WGenericGroundConstraint(c) => c.warmstart(mj_lambdas),
            AnyJointVelocityConstraint::PrismaticConstraint(c) => c.warmstart(mj_lambdas),
            AnyJointVelocityConstraint::PrismaticGroundConstraint(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WPrismaticConstraint(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WPrismaticGroundConstraint(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "dim3")]
            AnyJointVelocityConstraint::RevoluteConstraint(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "dim3")]
            AnyJointVelocityConstraint::RevoluteGroundConstraint(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "dim3")]
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WRevoluteConstraint(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "dim3")]
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WRevoluteGroundConstraint(c) => c.warmstart(mj_lambdas),
            AnyJointVelocityConstraint::Empty => unreachable!(),
        }
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        match self {
            AnyJointVelocityConstraint::BallConstraint(c) => c.solve(mj_lambdas),
            AnyJointVelocityConstraint::BallGroundConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WBallConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WBallGroundConstraint(c) => c.solve(mj_lambdas),
            AnyJointVelocityConstraint::FixedConstraint(c) => c.solve(mj_lambdas),
            AnyJointVelocityConstraint::FixedGroundConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WFixedConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WFixedGroundConstraint(c) => c.solve(mj_lambdas),
            // AnyJointVelocityConstraint::GenericConstraint(c) => c.solve(mj_lambdas),
            // AnyJointVelocityConstraint::GenericGroundConstraint(c) => c.solve(mj_lambdas),
            // #[cfg(feature = "simd-is-enabled")]
            // AnyJointVelocityConstraint::WGenericConstraint(c) => c.solve(mj_lambdas),
            // #[cfg(feature = "simd-is-enabled")]
            // AnyJointVelocityConstraint::WGenericGroundConstraint(c) => c.solve(mj_lambdas),
            AnyJointVelocityConstraint::PrismaticConstraint(c) => c.solve(mj_lambdas),
            AnyJointVelocityConstraint::PrismaticGroundConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WPrismaticConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WPrismaticGroundConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "dim3")]
            AnyJointVelocityConstraint::RevoluteConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "dim3")]
            AnyJointVelocityConstraint::RevoluteGroundConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "dim3")]
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WRevoluteConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "dim3")]
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WRevoluteGroundConstraint(c) => c.solve(mj_lambdas),
            AnyJointVelocityConstraint::Empty => unreachable!(),
        }
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        match self {
            AnyJointVelocityConstraint::BallConstraint(c) => c.writeback_impulses(joints_all),

            AnyJointVelocityConstraint::BallGroundConstraint(c) => c.writeback_impulses(joints_all),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WBallConstraint(c) => c.writeback_impulses(joints_all),

            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WBallGroundConstraint(c) => {
                c.writeback_impulses(joints_all)
            }
            AnyJointVelocityConstraint::FixedConstraint(c) => c.writeback_impulses(joints_all),
            AnyJointVelocityConstraint::FixedGroundConstraint(c) => {
                c.writeback_impulses(joints_all)
            }
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WFixedConstraint(c) => c.writeback_impulses(joints_all),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WFixedGroundConstraint(c) => {
                c.writeback_impulses(joints_all)
            }
            // AnyJointVelocityConstraint::GenericConstraint(c) => c.writeback_impulses(joints_all),
            // AnyJointVelocityConstraint::GenericGroundConstraint(c) => {
            //     c.writeback_impulses(joints_all)
            // }
            // #[cfg(feature = "simd-is-enabled")]
            // AnyJointVelocityConstraint::WGenericConstraint(c) => c.writeback_impulses(joints_all),
            // #[cfg(feature = "simd-is-enabled")]
            // AnyJointVelocityConstraint::WGenericGroundConstraint(c) => {
            //     c.writeback_impulses(joints_all)
            // }
            AnyJointVelocityConstraint::PrismaticConstraint(c) => c.writeback_impulses(joints_all),
            AnyJointVelocityConstraint::PrismaticGroundConstraint(c) => {
                c.writeback_impulses(joints_all)
            }
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WPrismaticConstraint(c) => c.writeback_impulses(joints_all),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WPrismaticGroundConstraint(c) => {
                c.writeback_impulses(joints_all)
            }
            #[cfg(feature = "dim3")]
            AnyJointVelocityConstraint::RevoluteConstraint(c) => c.writeback_impulses(joints_all),
            #[cfg(feature = "dim3")]
            AnyJointVelocityConstraint::RevoluteGroundConstraint(c) => {
                c.writeback_impulses(joints_all)
            }
            #[cfg(feature = "dim3")]
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WRevoluteConstraint(c) => c.writeback_impulses(joints_all),
            #[cfg(feature = "dim3")]
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WRevoluteGroundConstraint(c) => {
                c.writeback_impulses(joints_all)
            }
            AnyJointVelocityConstraint::Empty => unreachable!(),
        }
    }
}
