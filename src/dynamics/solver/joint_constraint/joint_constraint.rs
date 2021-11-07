use super::{
    BallVelocityConstraint, BallVelocityGroundConstraint, FixedVelocityConstraint,
    FixedVelocityGroundConstraint, PrismaticVelocityConstraint, PrismaticVelocityGroundConstraint,
    SpringVelocityConstraint, SpringVelocityGroundConstraint,
};
#[cfg(feature = "dim3")]
use super::{RevoluteVelocityConstraint, RevoluteVelocityGroundConstraint};
#[cfg(feature = "simd-is-enabled")]
use super::{
    WBallVelocityConstraint, WBallVelocityGroundConstraint, WFixedVelocityConstraint,
    WFixedVelocityGroundConstraint, WPrismaticVelocityConstraint,
    WPrismaticVelocityGroundConstraint, WSpringVelocityConstraint, WSpringVelocityGroundConstraint,
};
#[cfg(feature = "dim3")]
#[cfg(feature = "simd-is-enabled")]
use super::{WRevoluteVelocityConstraint, WRevoluteVelocityGroundConstraint};
// use crate::dynamics::solver::joint_constraint::generic_velocity_constraint::{
//     GenericVelocityConstraint, GenericVelocityGroundConstraint,
// };
use crate::data::{BundleSet, ComponentSet};
use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    IntegrationParameters, Joint, JointGraphEdge, JointIndex, JointParams, RigidBodyIds,
    RigidBodyMassProps, RigidBodyPosition, RigidBodyType, RigidBodyVelocity,
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
    SpringConstraint(SpringVelocityConstraint),
    SpringGroundConstraint(SpringVelocityGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WSpringConstraint(WSpringVelocityConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WSpringGroundConstraint(WSpringVelocityGroundConstraint),
    #[allow(dead_code)] // The Empty variant is only used with parallel code.
    Empty,
}

impl AnyJointVelocityConstraint {
    #[cfg(feature = "parallel")]
    pub fn num_active_constraints(_: &Joint) -> usize {
        1
    }

    pub fn from_joint<Bodies>(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        joint: &Joint,
        bodies: &Bodies,
    ) -> Self
    where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>,
    {
        let rb1 = (
            bodies.index(joint.body1.0),
            bodies.index(joint.body1.0),
            bodies.index(joint.body1.0),
            bodies.index(joint.body1.0),
        );
        let rb2 = (
            bodies.index(joint.body2.0),
            bodies.index(joint.body2.0),
            bodies.index(joint.body2.0),
            bodies.index(joint.body2.0),
        );

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
            JointParams::SpringJoint(p) => AnyJointVelocityConstraint::SpringConstraint(
                SpringVelocityConstraint::from_params(params, joint_id, rb1, rb2, p),
            ),
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    pub fn from_wide_joint<Bodies>(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        joints: [&Joint; SIMD_WIDTH],
        bodies: &Bodies,
    ) -> Self
    where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>,
    {
        let rbs1 = (
            gather![|ii| bodies.index(joints[ii].body1.0)],
            gather![|ii| bodies.index(joints[ii].body1.0)],
            gather![|ii| bodies.index(joints[ii].body1.0)],
            gather![|ii| bodies.index(joints[ii].body1.0)],
        );
        let rbs2 = (
            gather![|ii| bodies.index(joints[ii].body2.0)],
            gather![|ii| bodies.index(joints[ii].body2.0)],
            gather![|ii| bodies.index(joints[ii].body2.0)],
            gather![|ii| bodies.index(joints[ii].body2.0)],
        );

        match &joints[0].params {
            JointParams::BallJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_ball_joint().unwrap()];
                AnyJointVelocityConstraint::WBallConstraint(WBallVelocityConstraint::from_params(
                    params, joint_id, rbs1, rbs2, joints,
                ))
            }
            JointParams::FixedJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_fixed_joint().unwrap()];
                AnyJointVelocityConstraint::WFixedConstraint(WFixedVelocityConstraint::from_params(
                    params, joint_id, rbs1, rbs2, joints,
                ))
            }
            // JointParams::GenericJoint(_) => {
            //     let joints = gather![|ii| joints[ii].params.as_generic_joint().unwrap()];
            //     AnyJointVelocityConstraint::WGenericConstraint(
            //         WGenericVelocityConstraint::from_params(params, joint_id, rbs1, rbs2, joints),
            //     )
            // }
            JointParams::PrismaticJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_prismatic_joint().unwrap()];
                AnyJointVelocityConstraint::WPrismaticConstraint(
                    WPrismaticVelocityConstraint::from_params(params, joint_id, rbs1, rbs2, joints),
                )
            }
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_revolute_joint().unwrap()];
                AnyJointVelocityConstraint::WRevoluteConstraint(
                    WRevoluteVelocityConstraint::from_params(params, joint_id, rbs1, rbs2, joints),
                )
            }
            JointParams::SpringJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_spring_joint().unwrap()];
                AnyJointVelocityConstraint::WSpringConstraint(
                    WSpringVelocityConstraint::from_params(params, joint_id, rbs1, rbs2, joints),
                )
            }
        }
    }

    pub fn from_joint_ground<Bodies>(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        joint: &Joint,
        bodies: &Bodies,
    ) -> Self
    where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyType>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>,
    {
        let mut handle1 = joint.body1;
        let mut handle2 = joint.body2;
        let status2: &RigidBodyType = bodies.index(handle2.0);
        let flipped = !status2.is_dynamic();

        if flipped {
            std::mem::swap(&mut handle1, &mut handle2);
        }

        let rb1 = bodies.index_bundle(handle1.0);
        let rb2 = bodies.index_bundle(handle2.0);

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
            JointParams::SpringJoint(p) => AnyJointVelocityConstraint::SpringGroundConstraint(
                SpringVelocityGroundConstraint::from_params(params, joint_id, rb1, rb2, p, flipped),
            ),
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    pub fn from_wide_joint_ground<Bodies>(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        joints: [&Joint; SIMD_WIDTH],
        bodies: &Bodies,
    ) -> Self
    where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyType>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>,
    {
        let mut handles1 = gather![|ii| joints[ii].body1];
        let mut handles2 = gather![|ii| joints[ii].body2];
        let status2: [&RigidBodyType; SIMD_WIDTH] = gather![|ii| bodies.index(handles2[ii].0)];
        let mut flipped = [false; SIMD_WIDTH];

        for ii in 0..SIMD_WIDTH {
            if !status2[ii].is_dynamic() {
                std::mem::swap(&mut handles1[ii], &mut handles2[ii]);
                flipped[ii] = true;
            }
        }

        let rbs1 = (
            gather![|ii| bodies.index(handles1[ii].0)],
            gather![|ii| bodies.index(handles1[ii].0)],
            gather![|ii| bodies.index(handles1[ii].0)],
        );
        let rbs2 = (
            gather![|ii| bodies.index(handles2[ii].0)],
            gather![|ii| bodies.index(handles2[ii].0)],
            gather![|ii| bodies.index(handles2[ii].0)],
            gather![|ii| bodies.index(handles2[ii].0)],
        );

        match &joints[0].params {
            JointParams::BallJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_ball_joint().unwrap()];
                AnyJointVelocityConstraint::WBallGroundConstraint(
                    WBallVelocityGroundConstraint::from_params(
                        params, joint_id, rbs1, rbs2, joints, flipped,
                    ),
                )
            }
            JointParams::FixedJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_fixed_joint().unwrap()];
                AnyJointVelocityConstraint::WFixedGroundConstraint(
                    WFixedVelocityGroundConstraint::from_params(
                        params, joint_id, rbs1, rbs2, joints, flipped,
                    ),
                )
            }
            // JointParams::GenericJoint(_) => {
            //     let joints = gather![|ii| joints[ii].params.as_generic_joint().unwrap()];
            //     AnyJointVelocityConstraint::WGenericGroundConstraint(
            //         WGenericVelocityGroundConstraint::from_params(
            //             params, joint_id, rbs1, rbs2, joints, flipped,
            //         ),
            //     )
            // }
            JointParams::PrismaticJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_prismatic_joint().unwrap()];
                AnyJointVelocityConstraint::WPrismaticGroundConstraint(
                    WPrismaticVelocityGroundConstraint::from_params(
                        params, joint_id, rbs1, rbs2, joints, flipped,
                    ),
                )
            }
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_revolute_joint().unwrap()];
                AnyJointVelocityConstraint::WRevoluteGroundConstraint(
                    WRevoluteVelocityGroundConstraint::from_params(
                        params, joint_id, rbs1, rbs2, joints, flipped,
                    ),
                )
            }
            JointParams::SpringJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_spring_joint().unwrap()];
                AnyJointVelocityConstraint::WSpringGroundConstraint(
                    WSpringVelocityGroundConstraint::from_params(
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
            AnyJointVelocityConstraint::SpringConstraint(c) => c.warmstart(mj_lambdas),
            AnyJointVelocityConstraint::SpringGroundConstraint(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WSpringConstraint(c) => c.warmstart(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WSpringGroundConstraint(c) => c.warmstart(mj_lambdas),
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
            AnyJointVelocityConstraint::SpringConstraint(c) => c.solve(mj_lambdas),
            AnyJointVelocityConstraint::SpringGroundConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WSpringConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WSpringGroundConstraint(c) => c.solve(mj_lambdas),
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
            AnyJointVelocityConstraint::SpringConstraint(c) => c.writeback_impulses(joints_all),
            AnyJointVelocityConstraint::SpringGroundConstraint(c) => {
                c.writeback_impulses(joints_all)
            }
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WSpringConstraint(c) => c.writeback_impulses(joints_all),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::WSpringGroundConstraint(c) => {
                c.writeback_impulses(joints_all)
            }
            AnyJointVelocityConstraint::Empty => unreachable!(),
        }
    }
}
