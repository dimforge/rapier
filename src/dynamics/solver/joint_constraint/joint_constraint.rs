use crate::dynamics::solver::joint_constraint::joint_generic_velocity_constraint::{
    JointGenericVelocityConstraint, JointGenericVelocityGroundConstraint,
};
use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::{
    JointVelocityConstraint, JointVelocityGroundConstraint, SolverBody,
};
use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    ImpulseJoint, IntegrationParameters, JointGraphEdge, JointIndex, RigidBodySet,
};
use crate::math::{Real, SPATIAL_DIM};
use crate::prelude::MultibodyJointSet;
use na::DVector;

#[cfg(feature = "simd-is-enabled")]
use crate::math::{Isometry, SimdReal, SIMD_WIDTH};

#[cfg(feature = "parallel")]
use crate::dynamics::JointAxesMask;

pub enum AnyJointVelocityConstraint {
    JointConstraint(JointVelocityConstraint<Real, 1>),
    JointGroundConstraint(JointVelocityGroundConstraint<Real, 1>),
    JointGenericConstraint(JointGenericVelocityConstraint),
    JointGenericGroundConstraint(JointGenericVelocityGroundConstraint),
    #[cfg(feature = "simd-is-enabled")]
    JointConstraintSimd(JointVelocityConstraint<SimdReal, SIMD_WIDTH>),
    #[cfg(feature = "simd-is-enabled")]
    JointGroundConstraintSimd(JointVelocityGroundConstraint<SimdReal, SIMD_WIDTH>),
    Empty,
}

impl AnyJointVelocityConstraint {
    #[cfg(feature = "parallel")]
    pub fn num_active_constraints_and_jacobian_lines(joint: &ImpulseJoint) -> (usize, usize) {
        let joint = &joint.data;
        let locked_axes = joint.locked_axes.bits();
        let motor_axes = joint.motor_axes.bits() & !locked_axes;
        let limit_axes = joint.limit_axes.bits() & !locked_axes;
        let coupled_axes = joint.coupled_axes.bits();

        let num_constraints = (motor_axes & !coupled_axes).count_ones() as usize
            + ((motor_axes & coupled_axes) & JointAxesMask::ANG_AXES.bits() != 0) as usize
            + ((motor_axes & coupled_axes) & JointAxesMask::LIN_AXES.bits() != 0) as usize
            + locked_axes.count_ones() as usize
            + (limit_axes & !coupled_axes).count_ones() as usize
            + ((limit_axes & coupled_axes) & JointAxesMask::ANG_AXES.bits() != 0) as usize
            + ((limit_axes & coupled_axes) & JointAxesMask::LIN_AXES.bits() != 0) as usize;
        (num_constraints, num_constraints)
    }

    pub fn from_joint(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        joint: &ImpulseJoint,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        j_id: &mut usize,
        jacobians: &mut DVector<Real>,
        out: &mut Vec<Self>,
        insert_at: Option<usize>,
    ) {
        let local_frame1 = joint.data.local_frame1;
        let local_frame2 = joint.data.local_frame2;
        let rb1 = &bodies[joint.body1];
        let rb2 = &bodies[joint.body2];
        let frame1 = rb1.pos.position * local_frame1;
        let frame2 = rb2.pos.position * local_frame2;

        let body1 = SolverBody {
            linvel: rb1.vels.linvel,
            angvel: rb1.vels.angvel,
            im: rb1.mprops.effective_inv_mass,
            sqrt_ii: rb1.mprops.effective_world_inv_inertia_sqrt,
            world_com: rb1.mprops.world_com,
            mj_lambda: [rb1.ids.active_set_offset],
        };
        let body2 = SolverBody {
            linvel: rb2.vels.linvel,
            angvel: rb2.vels.angvel,
            im: rb2.mprops.effective_inv_mass,
            sqrt_ii: rb2.mprops.effective_world_inv_inertia_sqrt,
            world_com: rb2.mprops.world_com,
            mj_lambda: [rb2.ids.active_set_offset],
        };

        let mb1 = multibodies
            .rigid_body_link(joint.body1)
            .map(|link| (&multibodies[link.multibody], link.id));
        let mb2 = multibodies
            .rigid_body_link(joint.body2)
            .map(|link| (&multibodies[link.multibody], link.id));

        if mb1.is_some() || mb2.is_some() {
            let multibodies_ndof = mb1.map(|m| m.0.ndofs()).unwrap_or(SPATIAL_DIM)
                + mb2.map(|m| m.0.ndofs()).unwrap_or(SPATIAL_DIM);

            if multibodies_ndof == 0 {
                // Both multibodies are fixed, don’t generate any constraint.
                return;
            }

            // For each solver contact we generate up to SPATIAL_DIM constraints, and each
            // constraints appends the multibodies jacobian and weighted jacobians.
            // Also note that for impulse_joints, the rigid-bodies will also add their jacobians
            // to the generic DVector.
            // TODO: is this count correct when we take both motors and limits into account?
            let required_jacobian_len = *j_id + multibodies_ndof * 2 * SPATIAL_DIM;

            if jacobians.nrows() < required_jacobian_len && !cfg!(feature = "parallel") {
                jacobians.resize_vertically_mut(required_jacobian_len, 0.0);
            }

            // TODO: find a way to avoid the temporary buffer.
            let mut out_tmp = [JointGenericVelocityConstraint::invalid(); 12];
            let out_tmp_len = JointGenericVelocityConstraint::lock_axes(
                params,
                joint_id,
                &body1,
                &body2,
                mb1,
                mb2,
                &frame1,
                &frame2,
                &joint.data,
                jacobians,
                j_id,
                &mut out_tmp,
            );

            if let Some(at) = insert_at {
                for (i, c) in out_tmp.into_iter().take(out_tmp_len).enumerate() {
                    out[at + i] = AnyJointVelocityConstraint::JointGenericConstraint(c);
                }
            } else {
                for c in out_tmp.into_iter().take(out_tmp_len) {
                    out.push(AnyJointVelocityConstraint::JointGenericConstraint(c));
                }
            }
        } else {
            // TODO: find a way to avoid the temporary buffer.
            let mut out_tmp = [JointVelocityConstraint::invalid(); 12];
            let out_tmp_len = JointVelocityConstraint::<Real, 1>::lock_axes(
                params,
                joint_id,
                &body1,
                &body2,
                &frame1,
                &frame2,
                &joint.data,
                &mut out_tmp,
            );

            if let Some(at) = insert_at {
                for (i, c) in out_tmp.into_iter().take(out_tmp_len).enumerate() {
                    out[at + i] = AnyJointVelocityConstraint::JointConstraint(c);
                }
            } else {
                for c in out_tmp.into_iter().take(out_tmp_len) {
                    out.push(AnyJointVelocityConstraint::JointConstraint(c));
                }
            }
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    pub fn from_wide_joint(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        impulse_joints: [&ImpulseJoint; SIMD_WIDTH],
        bodies: &RigidBodySet,
        out: &mut Vec<Self>,
        insert_at: Option<usize>,
    ) {
        use crate::dynamics::{
            RigidBodyIds, RigidBodyMassProps, RigidBodyPosition, RigidBodyVelocity,
        };

        let rbs1: (
            [&RigidBodyPosition; SIMD_WIDTH],
            [&RigidBodyVelocity; SIMD_WIDTH],
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ) = (
            gather![|ii| &bodies[impulse_joints[ii].body1].pos],
            gather![|ii| &bodies[impulse_joints[ii].body1].vels],
            gather![|ii| &bodies[impulse_joints[ii].body1].mprops],
            gather![|ii| &bodies[impulse_joints[ii].body1].ids],
        );
        let rbs2: (
            [&RigidBodyPosition; SIMD_WIDTH],
            [&RigidBodyVelocity; SIMD_WIDTH],
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ) = (
            gather![|ii| &bodies[impulse_joints[ii].body2].pos],
            gather![|ii| &bodies[impulse_joints[ii].body2].vels],
            gather![|ii| &bodies[impulse_joints[ii].body2].mprops],
            gather![|ii| &bodies[impulse_joints[ii].body2].ids],
        );

        let (rb_pos1, rb_vel1, rb_mprops1, rb_ids1) = rbs1;
        let (rb_pos2, rb_vel2, rb_mprops2, rb_ids2) = rbs2;
        let pos1: Isometry<SimdReal> = gather![|ii| rb_pos1[ii].position].into();
        let pos2: Isometry<SimdReal> = gather![|ii| rb_pos2[ii].position].into();

        let local_frame1: Isometry<SimdReal> =
            gather![|ii| impulse_joints[ii].data.local_frame1].into();
        let local_frame2: Isometry<SimdReal> =
            gather![|ii| impulse_joints[ii].data.local_frame2].into();

        let frame1 = pos1 * local_frame1;
        let frame2 = pos2 * local_frame2;

        let body1: SolverBody<SimdReal, SIMD_WIDTH> = SolverBody {
            linvel: gather![|ii| rb_vel1[ii].linvel].into(),
            angvel: gather![|ii| rb_vel1[ii].angvel].into(),
            im: gather![|ii| rb_mprops1[ii].effective_inv_mass].into(),
            sqrt_ii: gather![|ii| rb_mprops1[ii].effective_world_inv_inertia_sqrt].into(),
            world_com: gather![|ii| rb_mprops1[ii].world_com].into(),
            mj_lambda: gather![|ii| rb_ids1[ii].active_set_offset],
        };
        let body2: SolverBody<SimdReal, SIMD_WIDTH> = SolverBody {
            linvel: gather![|ii| rb_vel2[ii].linvel].into(),
            angvel: gather![|ii| rb_vel2[ii].angvel].into(),
            im: gather![|ii| rb_mprops2[ii].effective_inv_mass].into(),
            sqrt_ii: gather![|ii| rb_mprops2[ii].effective_world_inv_inertia_sqrt].into(),
            world_com: gather![|ii| rb_mprops2[ii].world_com].into(),
            mj_lambda: gather![|ii| rb_ids2[ii].active_set_offset],
        };

        // TODO: find a way to avoid the temporary buffer.
        let mut out_tmp = [JointVelocityConstraint::invalid(); 12];
        let out_tmp_len = JointVelocityConstraint::<SimdReal, SIMD_WIDTH>::lock_axes(
            params,
            joint_id,
            &body1,
            &body2,
            &frame1,
            &frame2,
            impulse_joints[0].data.locked_axes.bits(),
            &mut out_tmp,
        );

        if let Some(at) = insert_at {
            for (i, c) in out_tmp.into_iter().take(out_tmp_len).enumerate() {
                out[at + i] = AnyJointVelocityConstraint::JointConstraintSimd(c);
            }
        } else {
            for c in out_tmp.into_iter().take(out_tmp_len) {
                out.push(AnyJointVelocityConstraint::JointConstraintSimd(c));
            }
        }
    }

    pub fn from_joint_ground(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        joint: &ImpulseJoint,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        j_id: &mut usize,
        jacobians: &mut DVector<Real>,
        out: &mut Vec<Self>,
        insert_at: Option<usize>,
    ) {
        let mut handle1 = joint.body1;
        let mut handle2 = joint.body2;
        let flipped = !bodies[handle2].is_dynamic();

        let (local_frame1, local_frame2) = if flipped {
            std::mem::swap(&mut handle1, &mut handle2);
            (joint.data.local_frame2, joint.data.local_frame1)
        } else {
            (joint.data.local_frame1, joint.data.local_frame2)
        };

        let rb1 = &bodies[handle1];
        let rb2 = &bodies[handle2];

        let frame1 = rb1.pos.position * local_frame1;
        let frame2 = rb2.pos.position * local_frame2;

        let body1 = SolverBody {
            linvel: rb1.vels.linvel,
            angvel: rb1.vels.angvel,
            im: rb1.mprops.effective_inv_mass,
            sqrt_ii: rb1.mprops.effective_world_inv_inertia_sqrt,
            world_com: rb1.mprops.world_com,
            mj_lambda: [crate::INVALID_USIZE],
        };
        let body2 = SolverBody {
            linvel: rb2.vels.linvel,
            angvel: rb2.vels.angvel,
            im: rb2.mprops.effective_inv_mass,
            sqrt_ii: rb2.mprops.effective_world_inv_inertia_sqrt,
            world_com: rb2.mprops.world_com,
            mj_lambda: [rb2.ids.active_set_offset],
        };

        if let Some(mb2) = multibodies
            .rigid_body_link(handle2)
            .map(|link| (&multibodies[link.multibody], link.id))
        {
            let multibodies_ndof = mb2.0.ndofs();

            if multibodies_ndof == 0 {
                // The multibody is fixed, don’t generate any constraint.
                return;
            }

            // For each solver contact we generate up to SPATIAL_DIM constraints, and each
            // constraints appends the multibodies jacobian and weighted jacobians.
            // Also note that for impulse_joints, the rigid-bodies will also add their jacobians
            // to the generic DVector.
            // TODO: is this count correct when we take both motors and limits into account?
            let required_jacobian_len = *j_id + multibodies_ndof * 2 * SPATIAL_DIM;

            if jacobians.nrows() < required_jacobian_len && !cfg!(feature = "parallel") {
                jacobians.resize_vertically_mut(required_jacobian_len, 0.0);
            }

            // TODO: find a way to avoid the temporary buffer.
            let mut out_tmp = [JointGenericVelocityGroundConstraint::invalid(); 12];
            let out_tmp_len = JointGenericVelocityGroundConstraint::lock_axes(
                params,
                joint_id,
                &body1,
                &body2,
                mb2,
                &frame1,
                &frame2,
                &joint.data,
                jacobians,
                j_id,
                &mut out_tmp,
            );

            if let Some(at) = insert_at {
                for (i, c) in out_tmp.into_iter().take(out_tmp_len).enumerate() {
                    out[at + i] = AnyJointVelocityConstraint::JointGenericGroundConstraint(c);
                }
            } else {
                for c in out_tmp.into_iter().take(out_tmp_len) {
                    out.push(AnyJointVelocityConstraint::JointGenericGroundConstraint(c));
                }
            }
        } else {
            // TODO: find a way to avoid the temporary buffer.
            let mut out_tmp = [JointVelocityGroundConstraint::invalid(); 12];
            let out_tmp_len = JointVelocityGroundConstraint::<Real, 1>::lock_axes(
                params,
                joint_id,
                &body1,
                &body2,
                &frame1,
                &frame2,
                &joint.data,
                &mut out_tmp,
            );

            if let Some(at) = insert_at {
                for (i, c) in out_tmp.into_iter().take(out_tmp_len).enumerate() {
                    out[at + i] = AnyJointVelocityConstraint::JointGroundConstraint(c);
                }
            } else {
                for c in out_tmp.into_iter().take(out_tmp_len) {
                    out.push(AnyJointVelocityConstraint::JointGroundConstraint(c));
                }
            }
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    pub fn from_wide_joint_ground(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        impulse_joints: [&ImpulseJoint; SIMD_WIDTH],
        bodies: &RigidBodySet,
        out: &mut Vec<Self>,
        insert_at: Option<usize>,
    ) {
        use crate::dynamics::{
            RigidBodyIds, RigidBodyMassProps, RigidBodyPosition, RigidBodyType, RigidBodyVelocity,
        };

        let mut handles1 = gather![|ii| impulse_joints[ii].body1];
        let mut handles2 = gather![|ii| impulse_joints[ii].body2];
        let status2: [&RigidBodyType; SIMD_WIDTH] = gather![|ii| &bodies[handles2[ii]].body_type];
        let mut flipped = [false; SIMD_WIDTH];

        for ii in 0..SIMD_WIDTH {
            if !status2[ii].is_dynamic() {
                std::mem::swap(&mut handles1[ii], &mut handles2[ii]);
                flipped[ii] = true;
            }
        }

        let local_frame1: Isometry<SimdReal> = gather![|ii| if flipped[ii] {
            impulse_joints[ii].data.local_frame2
        } else {
            impulse_joints[ii].data.local_frame1
        }]
        .into();
        let local_frame2: Isometry<SimdReal> = gather![|ii| if flipped[ii] {
            impulse_joints[ii].data.local_frame1
        } else {
            impulse_joints[ii].data.local_frame2
        }]
        .into();

        let rbs1: (
            [&RigidBodyPosition; SIMD_WIDTH],
            [&RigidBodyVelocity; SIMD_WIDTH],
            [&RigidBodyMassProps; SIMD_WIDTH],
        ) = (
            gather![|ii| &bodies[handles1[ii]].pos],
            gather![|ii| &bodies[handles1[ii]].vels],
            gather![|ii| &bodies[handles1[ii]].mprops],
        );
        let rbs2: (
            [&RigidBodyPosition; SIMD_WIDTH],
            [&RigidBodyVelocity; SIMD_WIDTH],
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ) = (
            gather![|ii| &bodies[handles2[ii]].pos],
            gather![|ii| &bodies[handles2[ii]].vels],
            gather![|ii| &bodies[handles2[ii]].mprops],
            gather![|ii| &bodies[handles2[ii]].ids],
        );

        let (rb_pos1, rb_vel1, rb_mprops1) = rbs1;
        let (rb_pos2, rb_vel2, rb_mprops2, rb_ids2) = rbs2;
        let pos1: Isometry<SimdReal> = gather![|ii| rb_pos1[ii].position].into();
        let pos2: Isometry<SimdReal> = gather![|ii| rb_pos2[ii].position].into();

        let frame1 = pos1 * local_frame1;
        let frame2 = pos2 * local_frame2;

        let body1: SolverBody<SimdReal, SIMD_WIDTH> = SolverBody {
            linvel: gather![|ii| rb_vel1[ii].linvel].into(),
            angvel: gather![|ii| rb_vel1[ii].angvel].into(),
            im: gather![|ii| rb_mprops1[ii].effective_inv_mass].into(),
            sqrt_ii: gather![|ii| rb_mprops1[ii].effective_world_inv_inertia_sqrt].into(),
            world_com: gather![|ii| rb_mprops1[ii].world_com].into(),
            mj_lambda: [crate::INVALID_USIZE; SIMD_WIDTH],
        };
        let body2: SolverBody<SimdReal, SIMD_WIDTH> = SolverBody {
            linvel: gather![|ii| rb_vel2[ii].linvel].into(),
            angvel: gather![|ii| rb_vel2[ii].angvel].into(),
            im: gather![|ii| rb_mprops2[ii].effective_inv_mass].into(),
            sqrt_ii: gather![|ii| rb_mprops2[ii].effective_world_inv_inertia_sqrt].into(),
            world_com: gather![|ii| rb_mprops2[ii].world_com].into(),
            mj_lambda: gather![|ii| rb_ids2[ii].active_set_offset],
        };

        // TODO: find a way to avoid the temporary buffer.
        let mut out_tmp = [JointVelocityGroundConstraint::invalid(); 12];
        let out_tmp_len = JointVelocityGroundConstraint::<SimdReal, SIMD_WIDTH>::lock_axes(
            params,
            joint_id,
            &body1,
            &body2,
            &frame1,
            &frame2,
            impulse_joints[0].data.locked_axes.bits(),
            &mut out_tmp,
        );

        if let Some(at) = insert_at {
            for (i, c) in out_tmp.into_iter().take(out_tmp_len).enumerate() {
                out[at + i] = AnyJointVelocityConstraint::JointGroundConstraintSimd(c);
            }
        } else {
            for c in out_tmp.into_iter().take(out_tmp_len) {
                out.push(AnyJointVelocityConstraint::JointGroundConstraintSimd(c));
            }
        }
    }

    pub fn remove_bias_from_rhs(&mut self) {
        match self {
            AnyJointVelocityConstraint::JointConstraint(c) => c.remove_bias_from_rhs(),
            AnyJointVelocityConstraint::JointGroundConstraint(c) => c.remove_bias_from_rhs(),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::JointConstraintSimd(c) => c.remove_bias_from_rhs(),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::JointGroundConstraintSimd(c) => c.remove_bias_from_rhs(),
            AnyJointVelocityConstraint::JointGenericConstraint(c) => c.remove_bias_from_rhs(),
            AnyJointVelocityConstraint::JointGenericGroundConstraint(c) => c.remove_bias_from_rhs(),
            AnyJointVelocityConstraint::Empty => unreachable!(),
        }
    }

    pub fn solve(
        &mut self,
        jacobians: &DVector<Real>,
        mj_lambdas: &mut [DeltaVel<Real>],
        generic_mj_lambdas: &mut DVector<Real>,
    ) {
        match self {
            AnyJointVelocityConstraint::JointConstraint(c) => c.solve(mj_lambdas),
            AnyJointVelocityConstraint::JointGroundConstraint(c) => c.solve(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::JointConstraintSimd(c) => c.solve(mj_lambdas),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::JointGroundConstraintSimd(c) => c.solve(mj_lambdas),
            AnyJointVelocityConstraint::JointGenericConstraint(c) => {
                c.solve(jacobians, mj_lambdas, generic_mj_lambdas)
            }
            AnyJointVelocityConstraint::JointGenericGroundConstraint(c) => {
                c.solve(jacobians, mj_lambdas, generic_mj_lambdas)
            }
            AnyJointVelocityConstraint::Empty => unreachable!(),
        }
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        match self {
            AnyJointVelocityConstraint::JointConstraint(c) => c.writeback_impulses(joints_all),
            AnyJointVelocityConstraint::JointGroundConstraint(c) => {
                c.writeback_impulses(joints_all)
            }
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::JointConstraintSimd(c) => c.writeback_impulses(joints_all),
            #[cfg(feature = "simd-is-enabled")]
            AnyJointVelocityConstraint::JointGroundConstraintSimd(c) => {
                c.writeback_impulses(joints_all)
            }
            AnyJointVelocityConstraint::JointGenericConstraint(c) => {
                c.writeback_impulses(joints_all)
            }
            AnyJointVelocityConstraint::JointGenericGroundConstraint(c) => {
                c.writeback_impulses(joints_all)
            }
            AnyJointVelocityConstraint::Empty => unreachable!(),
        }
    }
}
