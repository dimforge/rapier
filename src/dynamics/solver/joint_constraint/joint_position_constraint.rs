use super::{
    BallPositionConstraint, BallPositionGroundConstraint, FixedPositionConstraint,
    FixedPositionGroundConstraint, PrismaticPositionConstraint, PrismaticPositionGroundConstraint,
};
#[cfg(feature = "dim3")]
use super::{RevolutePositionConstraint, RevolutePositionGroundConstraint};
#[cfg(all(feature = "dim3", feature = "simd-is-enabled"))]
use super::{WRevolutePositionConstraint, WRevolutePositionGroundConstraint};

#[cfg(feature = "simd-is-enabled")]
use super::{
    WBallPositionConstraint, WBallPositionGroundConstraint, WFixedPositionConstraint,
    WFixedPositionGroundConstraint, WPrismaticPositionConstraint,
    WPrismaticPositionGroundConstraint,
};
use crate::data::{BundleSet, ComponentSet};
use crate::dynamics::{
    IntegrationParameters, Joint, JointParams, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition,
    RigidBodyType,
};
#[cfg(feature = "simd-is-enabled")]
use crate::math::SIMD_WIDTH;
use crate::math::{Isometry, Real};

pub(crate) enum AnyJointPositionConstraint {
    BallJoint(BallPositionConstraint),
    BallGroundConstraint(BallPositionGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WBallJoint(WBallPositionConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WBallGroundConstraint(WBallPositionGroundConstraint),
    FixedJoint(FixedPositionConstraint),
    FixedGroundConstraint(FixedPositionGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WFixedJoint(WFixedPositionConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WFixedGroundConstraint(WFixedPositionGroundConstraint),
    // GenericJoint(GenericPositionConstraint),
    // GenericGroundConstraint(GenericPositionGroundConstraint),
    // #[cfg(feature = "simd-is-enabled")]
    // WGenericJoint(WGenericPositionConstraint),
    // #[cfg(feature = "simd-is-enabled")]
    // WGenericGroundConstraint(WGenericPositionGroundConstraint),
    PrismaticJoint(PrismaticPositionConstraint),
    PrismaticGroundConstraint(PrismaticPositionGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WPrismaticJoint(WPrismaticPositionConstraint),
    #[cfg(feature = "simd-is-enabled")]
    WPrismaticGroundConstraint(WPrismaticPositionGroundConstraint),
    #[cfg(feature = "dim3")]
    RevoluteJoint(RevolutePositionConstraint),
    #[cfg(feature = "dim3")]
    RevoluteGroundConstraint(RevolutePositionGroundConstraint),
    #[cfg(all(feature = "dim3", feature = "simd-is-enabled"))]
    WRevoluteJoint(WRevolutePositionConstraint),
    #[cfg(all(feature = "dim3", feature = "simd-is-enabled"))]
    WRevoluteGroundConstraint(WRevolutePositionGroundConstraint),
    #[allow(dead_code)] // The Empty variant is only used with parallel code.
    Empty,
}

impl AnyJointPositionConstraint {
    pub fn from_joint<Bodies>(joint: &Joint, bodies: &Bodies) -> Self
    where
        Bodies: ComponentSet<RigidBodyMassProps> + ComponentSet<RigidBodyIds>,
    {
        let rb1 = bodies.index_bundle(joint.body1.0);
        let rb2 = bodies.index_bundle(joint.body2.0);

        match &joint.params {
            JointParams::BallJoint(p) => AnyJointPositionConstraint::BallJoint(
                BallPositionConstraint::from_params(rb1, rb2, p),
            ),
            JointParams::FixedJoint(p) => AnyJointPositionConstraint::FixedJoint(
                FixedPositionConstraint::from_params(rb1, rb2, p),
            ),
            // JointParams::GenericJoint(p) => AnyJointPositionConstraint::GenericJoint(
            //     GenericPositionConstraint::from_params(rb1, rb2, p),
            // ),
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
    pub fn from_wide_joint<Bodies>(joints: [&Joint; SIMD_WIDTH], bodies: &Bodies) -> Self
    where
        Bodies: ComponentSet<RigidBodyMassProps> + ComponentSet<RigidBodyIds>,
    {
        let rbs1 = (
            gather![|ii| bodies.index(joints[ii].body1.0)],
            gather![|ii| bodies.index(joints[ii].body1.0)],
        );
        let rbs2 = (
            gather![|ii| bodies.index(joints[ii].body2.0)],
            gather![|ii| bodies.index(joints[ii].body2.0)],
        );

        match &joints[0].params {
            JointParams::BallJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_ball_joint().unwrap()];
                AnyJointPositionConstraint::WBallJoint(WBallPositionConstraint::from_params(
                    rbs1, rbs2, joints,
                ))
            }
            JointParams::FixedJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_fixed_joint().unwrap()];
                AnyJointPositionConstraint::WFixedJoint(WFixedPositionConstraint::from_params(
                    rbs1, rbs2, joints,
                ))
            }
            // JointParams::GenericJoint(_) => {
            //     let joints = gather![|ii| joints[ii].params.as_generic_joint().unwrap()];
            //     AnyJointPositionConstraint::WGenericJoint(WGenericPositionConstraint::from_params(
            //         rbs1, rbs2, joints,
            //     ))
            // }
            JointParams::PrismaticJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_prismatic_joint().unwrap()];
                AnyJointPositionConstraint::WPrismaticJoint(
                    WPrismaticPositionConstraint::from_params(rbs1, rbs2, joints),
                )
            }
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_revolute_joint().unwrap()];
                AnyJointPositionConstraint::WRevoluteJoint(
                    WRevolutePositionConstraint::from_params(rbs1, rbs2, joints),
                )
            }
        }
    }

    pub fn from_joint_ground<Bodies>(joint: &Joint, bodies: &Bodies) -> Self
    where
        Bodies: ComponentSet<RigidBodyType>
            + ComponentSet<RigidBodyPosition>
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

        let rb1 = bodies.index(handle1.0);
        let rb2 = (bodies.index(handle2.0), bodies.index(handle2.0));

        match &joint.params {
            JointParams::BallJoint(p) => AnyJointPositionConstraint::BallGroundConstraint(
                BallPositionGroundConstraint::from_params(rb1, rb2, p, flipped),
            ),
            JointParams::FixedJoint(p) => AnyJointPositionConstraint::FixedGroundConstraint(
                FixedPositionGroundConstraint::from_params(rb1, rb2, p, flipped),
            ),
            // JointParams::GenericJoint(p) => AnyJointPositionConstraint::GenericGroundConstraint(
            //     GenericPositionGroundConstraint::from_params(rb1, rb2, p, flipped),
            // ),
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
    pub fn from_wide_joint_ground<Bodies>(joints: [&Joint; SIMD_WIDTH], bodies: &Bodies) -> Self
    where
        Bodies: ComponentSet<RigidBodyType>
            + ComponentSet<RigidBodyPosition>
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

        let rbs1 = gather![|ii| bodies.index(handles1[ii].0)];
        let rbs2 = (
            gather![|ii| bodies.index(handles2[ii].0)],
            gather![|ii| bodies.index(handles2[ii].0)],
        );

        match &joints[0].params {
            JointParams::BallJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_ball_joint().unwrap()];
                AnyJointPositionConstraint::WBallGroundConstraint(
                    WBallPositionGroundConstraint::from_params(rbs1, rbs2, joints, flipped),
                )
            }
            JointParams::FixedJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_fixed_joint().unwrap()];
                AnyJointPositionConstraint::WFixedGroundConstraint(
                    WFixedPositionGroundConstraint::from_params(rbs1, rbs2, joints, flipped),
                )
            }
            // JointParams::GenericJoint(_) => {
            //     let joints = gather![|ii| joints[ii].params.as_generic_joint().unwrap()];
            //     AnyJointPositionConstraint::WGenericGroundConstraint(
            //         WGenericPositionGroundConstraint::from_params(rbs1, rbs2, joints, flipped),
            //     )
            // }
            JointParams::PrismaticJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_prismatic_joint().unwrap()];
                AnyJointPositionConstraint::WPrismaticGroundConstraint(
                    WPrismaticPositionGroundConstraint::from_params(rbs1, rbs2, joints, flipped),
                )
            }
            #[cfg(feature = "dim3")]
            JointParams::RevoluteJoint(_) => {
                let joints = gather![|ii| joints[ii].params.as_revolute_joint().unwrap()];
                AnyJointPositionConstraint::WRevoluteGroundConstraint(
                    WRevolutePositionGroundConstraint::from_params(rbs1, rbs2, joints, flipped),
                )
            }
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        match self {
            AnyJointPositionConstraint::BallJoint(c) => c.solve(params, positions),
            AnyJointPositionConstraint::BallGroundConstraint(c) => c.solve(params, positions),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointPositionConstraint::WBallJoint(c) => c.solve(params, positions),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointPositionConstraint::WBallGroundConstraint(c) => c.solve(params, positions),
            AnyJointPositionConstraint::FixedJoint(c) => c.solve(params, positions),
            AnyJointPositionConstraint::FixedGroundConstraint(c) => c.solve(params, positions),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointPositionConstraint::WFixedJoint(c) => c.solve(params, positions),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointPositionConstraint::WFixedGroundConstraint(c) => c.solve(params, positions),
            // AnyJointPositionConstraint::GenericJoint(c) => c.solve(params, positions),
            // AnyJointPositionConstraint::GenericGroundConstraint(c) => c.solve(params, positions),
            // #[cfg(feature = "simd-is-enabled")]
            // AnyJointPositionConstraint::WGenericJoint(c) => c.solve(params, positions),
            // #[cfg(feature = "simd-is-enabled")]
            // AnyJointPositionConstraint::WGenericGroundConstraint(c) => c.solve(params, positions),
            AnyJointPositionConstraint::PrismaticJoint(c) => c.solve(params, positions),
            AnyJointPositionConstraint::PrismaticGroundConstraint(c) => c.solve(params, positions),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointPositionConstraint::WPrismaticJoint(c) => c.solve(params, positions),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointPositionConstraint::WPrismaticGroundConstraint(c) => c.solve(params, positions),
            #[cfg(feature = "dim3")]
            AnyJointPositionConstraint::RevoluteJoint(c) => c.solve(params, positions),
            #[cfg(feature = "dim3")]
            AnyJointPositionConstraint::RevoluteGroundConstraint(c) => c.solve(params, positions),
            #[cfg(all(feature = "dim3", feature = "simd-is-enabled"))]
            AnyJointPositionConstraint::WRevoluteJoint(c) => c.solve(params, positions),
            #[cfg(all(feature = "dim3", feature = "simd-is-enabled"))]
            AnyJointPositionConstraint::WRevoluteGroundConstraint(c) => c.solve(params, positions),
            AnyJointPositionConstraint::Empty => unreachable!(),
        }
    }
}
