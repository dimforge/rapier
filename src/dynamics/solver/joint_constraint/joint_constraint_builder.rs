use crate::dynamics::solver::ConstraintsCounts;
use crate::dynamics::solver::MotorParameters;
use crate::dynamics::solver::joint_constraint::JointSolverBody;
use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::{
    JointConstraint, WritebackId,
};
use crate::dynamics::solver::solver_body::SolverBodies;
use crate::dynamics::{GenericJoint, ImpulseJoint, IntegrationParameters, JointIndex};
use crate::math::{ANG_DIM, AngVector, DIM, Isometry, Matrix, Point, Real, Rotation, Vector};
use crate::prelude::RigidBodySet;
use crate::utils;
use crate::utils::{IndexMut2, SimdCrossMatrix, SimdDot, SimdRealCopy};
#[cfg(feature = "dim3")]
use crate::utils::{SimdBasis, SimdQuat};
use na::SMatrix;

#[cfg(feature = "simd-is-enabled")]
use crate::math::{SIMD_WIDTH, SimdReal};

pub struct JointConstraintBuilder {
    body1: u32,
    body2: u32,
    joint_id: JointIndex,
    joint: GenericJoint,
    constraint_id: usize,
}

impl JointConstraintBuilder {
    pub fn generate(
        joint: &ImpulseJoint,
        bodies: &RigidBodySet,
        joint_id: JointIndex,
        out_builder: &mut Self,
        out_constraint_id: &mut usize,
    ) {
        let rb1 = &bodies[joint.body1];
        let rb2 = &bodies[joint.body2];
        let solver_body1 = rb1.effective_active_set_offset();
        let solver_body2 = rb2.effective_active_set_offset();

        *out_builder = Self {
            body1: solver_body1,
            body2: solver_body2,
            joint_id,
            joint: joint.data,
            constraint_id: *out_constraint_id,
        };
        // Since solver body poses are given in center-of-mass space,
        // we need to transform the anchors to that space.
        out_builder.joint.transform_to_solver_body_space(rb1, rb2);

        let count = ConstraintsCounts::from_joint(joint);
        *out_constraint_id += count.num_constraints;
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        bodies: &SolverBodies,
        out: &mut [JointConstraint<Real, 1>],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?

        let rb1 = bodies.get_pose(self.body1);
        let rb2 = bodies.get_pose(self.body2);
        let frame1 = rb1.pose * self.joint.local_frame1;
        let frame2 = rb2.pose * self.joint.local_frame2;
        let world_com1 = Point::from(rb1.pose.translation.vector);
        let world_com2 = Point::from(rb2.pose.translation.vector);

        let joint_body1 = JointSolverBody {
            im: rb1.im,
            ii: rb1.ii,
            world_com: world_com1,
            solver_vel: [self.body1],
        };
        let joint_body2 = JointSolverBody {
            im: rb2.im,
            ii: rb2.ii,
            world_com: world_com2,
            solver_vel: [self.body2],
        };

        JointConstraint::<Real, 1>::update(
            params,
            self.joint_id,
            &joint_body1,
            &joint_body2,
            &frame1,
            &frame2,
            &self.joint,
            &mut out[self.constraint_id..],
        );
    }
}

#[cfg(feature = "simd-is-enabled")]
pub struct JointConstraintBuilderSimd {
    body1: [u32; SIMD_WIDTH],
    body2: [u32; SIMD_WIDTH],
    joint_id: [JointIndex; SIMD_WIDTH],
    local_frame1: Isometry<SimdReal>,
    local_frame2: Isometry<SimdReal>,
    locked_axes: u8,
    constraint_id: usize,
}

#[cfg(feature = "simd-is-enabled")]
impl JointConstraintBuilderSimd {
    pub fn generate(
        joint: [&ImpulseJoint; SIMD_WIDTH],
        bodies: &RigidBodySet,
        joint_id: [JointIndex; SIMD_WIDTH],
        out_builder: &mut Self,
        out_constraint_id: &mut usize,
    ) {
        let rb1 = array![|ii| &bodies[joint[ii].body1]];
        let rb2 = array![|ii| &bodies[joint[ii].body2]];

        let body1 = array![|ii| if rb1[ii].is_dynamic_or_kinematic() {
            rb1[ii].ids.active_set_offset
        } else {
            u32::MAX
        }];
        let body2 = array![|ii| if rb2[ii].is_dynamic_or_kinematic() {
            rb2[ii].ids.active_set_offset
        } else {
            u32::MAX
        }];

        let local_frame1 = array![|ii| if body1[ii] != u32::MAX {
            joint[ii].data.local_frame1
        } else {
            rb1[ii].pos.position * joint[ii].data.local_frame1
        }]
        .into();
        let local_frame2 = array![|ii| if body2[ii] != u32::MAX {
            joint[ii].data.local_frame2
        } else {
            rb2[ii].pos.position * joint[ii].data.local_frame2
        }]
        .into();

        *out_builder = Self {
            body1,
            body2,
            joint_id,
            local_frame1,
            local_frame2,
            locked_axes: joint[0].data.locked_axes.bits(),
            constraint_id: *out_constraint_id,
        };

        let count = ConstraintsCounts::from_joint(joint[0]);
        *out_constraint_id += count.num_constraints;
    }

    pub fn update(
        &mut self,
        params: &IntegrationParameters,
        bodies: &SolverBodies,
        out: &mut [JointConstraint<SimdReal, SIMD_WIDTH>],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?

        let rb1 = bodies.gather_poses(self.body1);
        let rb2 = bodies.gather_poses(self.body2);
        let frame1 = rb1.pose * self.local_frame1;
        let frame2 = rb2.pose * self.local_frame2;

        let joint_body1 = JointSolverBody {
            im: rb1.im,
            ii: rb1.ii,
            world_com: rb1.pose.translation.vector.into(),
            solver_vel: self.body1,
        };
        let joint_body2 = JointSolverBody {
            im: rb2.im,
            ii: rb2.ii,
            world_com: rb2.pose.translation.vector.into(),
            solver_vel: self.body2,
        };

        JointConstraint::<SimdReal, SIMD_WIDTH>::update(
            params,
            self.joint_id,
            &joint_body1,
            &joint_body2,
            &frame1,
            &frame2,
            self.locked_axes,
            &mut out[self.constraint_id..],
        );
    }
}

#[derive(Debug, Copy, Clone)]
pub struct JointConstraintHelper<N: SimdRealCopy> {
    pub basis: Matrix<N>,
    #[cfg(feature = "dim3")]
    pub basis2: Matrix<N>, // TODO: used for angular coupling. Can we avoid storing this?
    pub cmat1_basis: SMatrix<N, ANG_DIM, DIM>,
    pub cmat2_basis: SMatrix<N, ANG_DIM, DIM>,
    #[cfg(feature = "dim3")]
    pub ang_basis: SMatrix<N, ANG_DIM, ANG_DIM>,
    pub lin_err: Vector<N>,
    pub ang_err: Rotation<N>,
}

impl<N: SimdRealCopy> JointConstraintHelper<N> {
    pub fn new(
        frame1: &Isometry<N>,
        frame2: &Isometry<N>,
        world_com1: &Point<N>,
        world_com2: &Point<N>,
        locked_lin_axes: u8,
    ) -> Self {
        let mut frame1 = *frame1;
        let basis = frame1.rotation.to_rotation_matrix().into_inner();
        let lin_err = frame2.translation.vector - frame1.translation.vector;

        // Adjust the point of application of the force for the first body,
        // by snapping free axes to the second frame’s center (to account for
        // the allowed relative movement).
        {
            let mut new_center1 = frame2.translation.vector; // First, assume all dofs are free.

            // Then snap the locked ones.
            for i in 0..DIM {
                if locked_lin_axes & (1 << i) != 0 {
                    let axis = basis.column(i);
                    new_center1 -= axis * lin_err.dot(&axis);
                }
            }
            frame1.translation.vector = new_center1;
        }

        let r1 = frame1.translation.vector - world_com1.coords;
        let r2 = frame2.translation.vector - world_com2.coords;

        let cmat1 = r1.gcross_matrix();
        let cmat2 = r2.gcross_matrix();

        #[cfg(feature = "dim3")]
        let mut ang_basis = frame1.rotation.diff_conj1_2(&frame2.rotation).transpose();
        #[allow(unused_mut)] // The mut is needed for 3D
        let mut ang_err = frame1.rotation.inverse() * frame2.rotation;

        #[cfg(feature = "dim3")]
        {
            let sgn = N::one().simd_copysign(frame1.rotation.dot(&frame2.rotation));
            ang_basis *= sgn;
            *ang_err.as_mut_unchecked() *= sgn;
        }

        Self {
            basis,
            #[cfg(feature = "dim3")]
            basis2: frame2.rotation.to_rotation_matrix().into_inner(),
            cmat1_basis: cmat1 * basis,
            cmat2_basis: cmat2 * basis,
            #[cfg(feature = "dim3")]
            ang_basis,
            lin_err,
            ang_err,
        }
    }

    pub fn limit_linear<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<N, LANES>,
        body2: &JointSolverBody<N, LANES>,
        limited_axis: usize,
        limits: [N; 2],
        writeback_id: WritebackId,
    ) -> JointConstraint<N, LANES> {
        let zero = N::zero();
        let mut constraint =
            self.lock_linear(params, joint_id, body1, body2, limited_axis, writeback_id);

        let dist = self.lin_err.dot(&constraint.lin_jac);
        let min_enabled = dist.simd_le(limits[0]);
        let max_enabled = limits[1].simd_le(dist);

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias =
            ((dist - limits[1]).simd_max(zero) - (limits[0] - dist).simd_max(zero)) * erp_inv_dt;
        constraint.rhs = constraint.rhs_wo_bias + rhs_bias;
        constraint.cfm_coeff = cfm_coeff;
        constraint.impulse_bounds = [
            N::splat(-Real::INFINITY).select(min_enabled, zero),
            N::splat(Real::INFINITY).select(max_enabled, zero),
        ];

        constraint
    }

    pub fn limit_linear_coupled<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<N, LANES>,
        body2: &JointSolverBody<N, LANES>,
        coupled_axes: u8,
        limits: [N; 2],
        writeback_id: WritebackId,
    ) -> JointConstraint<N, LANES> {
        let zero = N::zero();
        let mut lin_jac = Vector::zeros();
        let mut ang_jac1: AngVector<N> = na::zero();
        let mut ang_jac2: AngVector<N> = na::zero();

        for i in 0..DIM {
            if coupled_axes & (1 << i) != 0 {
                let coeff = self.basis.column(i).dot(&self.lin_err);
                lin_jac += self.basis.column(i) * coeff;
                #[cfg(feature = "dim2")]
                {
                    ang_jac1 += self.cmat1_basis[i] * coeff;
                    ang_jac2 += self.cmat2_basis[i] * coeff;
                }
                #[cfg(feature = "dim3")]
                {
                    ang_jac1 += self.cmat1_basis.column(i) * coeff;
                    ang_jac2 += self.cmat2_basis.column(i) * coeff;
                }
            }
        }

        // FIXME: handle min limit too.

        let dist = lin_jac.norm();
        let inv_dist = crate::utils::simd_inv(dist);
        lin_jac *= inv_dist;
        ang_jac1 *= inv_dist;
        ang_jac2 *= inv_dist;

        let rhs_wo_bias = (dist - limits[1]).simd_min(zero) * N::splat(params.inv_dt());

        let ii_ang_jac1 = body1.ii * ang_jac1;
        let ii_ang_jac2 = body2.ii * ang_jac2;

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = (dist - limits[1]).simd_max(zero) * erp_inv_dt;
        let rhs = rhs_wo_bias + rhs_bias;
        let impulse_bounds = [N::zero(), N::splat(Real::INFINITY)];

        JointConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds,
            lin_jac,
            ang_jac1,
            ang_jac2,
            ii_ang_jac1,
            ii_ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn motor_linear<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<N, LANES>,
        body2: &JointSolverBody<N, LANES>,
        motor_axis: usize,
        motor_params: &MotorParameters<N>,
        limits: Option<[N; 2]>,
        writeback_id: WritebackId,
    ) -> JointConstraint<N, LANES> {
        let inv_dt = N::splat(params.inv_dt());
        let mut constraint =
            self.lock_linear(params, joint_id, body1, body2, motor_axis, writeback_id);

        let mut rhs_wo_bias = N::zero();
        if motor_params.erp_inv_dt != N::zero() {
            let dist = self.lin_err.dot(&constraint.lin_jac);
            rhs_wo_bias += (dist - motor_params.target_pos) * motor_params.erp_inv_dt;
        }

        let mut target_vel = motor_params.target_vel;
        if let Some(limits) = limits {
            let dist = self.lin_err.dot(&constraint.lin_jac);
            target_vel =
                target_vel.simd_clamp((limits[0] - dist) * inv_dt, (limits[1] - dist) * inv_dt);
        };

        rhs_wo_bias += -target_vel;

        constraint.cfm_coeff = motor_params.cfm_coeff;
        constraint.cfm_gain = motor_params.cfm_gain;
        constraint.impulse_bounds = [-motor_params.max_impulse, motor_params.max_impulse];
        constraint.rhs = rhs_wo_bias;
        constraint.rhs_wo_bias = rhs_wo_bias;
        constraint
    }

    pub fn motor_linear_coupled<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<N, LANES>,
        body2: &JointSolverBody<N, LANES>,
        coupled_axes: u8,
        motor_params: &MotorParameters<N>,
        limits: Option<[N; 2]>,
        writeback_id: WritebackId,
    ) -> JointConstraint<N, LANES> {
        let inv_dt = N::splat(params.inv_dt());

        let mut lin_jac = Vector::zeros();
        let mut ang_jac1: AngVector<N> = na::zero();
        let mut ang_jac2: AngVector<N> = na::zero();

        for i in 0..DIM {
            if coupled_axes & (1 << i) != 0 {
                let coeff = self.basis.column(i).dot(&self.lin_err);
                lin_jac += self.basis.column(i) * coeff;
                #[cfg(feature = "dim2")]
                {
                    ang_jac1 += self.cmat1_basis[i] * coeff;
                    ang_jac2 += self.cmat2_basis[i] * coeff;
                }
                #[cfg(feature = "dim3")]
                {
                    ang_jac1 += self.cmat1_basis.column(i) * coeff;
                    ang_jac2 += self.cmat2_basis.column(i) * coeff;
                }
            }
        }

        let dist = lin_jac.norm();
        let inv_dist = crate::utils::simd_inv(dist);
        lin_jac *= inv_dist;
        ang_jac1 *= inv_dist;
        ang_jac2 *= inv_dist;

        let mut rhs_wo_bias = N::zero();
        if motor_params.erp_inv_dt != N::zero() {
            rhs_wo_bias += (dist - motor_params.target_pos) * motor_params.erp_inv_dt;
        }

        let mut target_vel = motor_params.target_vel;
        if let Some(limits) = limits {
            target_vel =
                target_vel.simd_clamp((limits[0] - dist) * inv_dt, (limits[1] - dist) * inv_dt);
        };

        rhs_wo_bias += -target_vel;

        let ii_ang_jac1 = body1.ii * ang_jac1;
        let ii_ang_jac2 = body2.ii * ang_jac2;

        JointConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac,
            ang_jac1,
            ang_jac2,
            ii_ang_jac1,
            ii_ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff: motor_params.cfm_coeff,
            cfm_gain: motor_params.cfm_gain,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn lock_linear<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<N, LANES>,
        body2: &JointSolverBody<N, LANES>,
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointConstraint<N, LANES> {
        let lin_jac = self.basis.column(locked_axis).into_owned();
        #[cfg(feature = "dim2")]
        let ang_jac1 = self.cmat1_basis[locked_axis];
        #[cfg(feature = "dim2")]
        let ang_jac2 = self.cmat2_basis[locked_axis];
        #[cfg(feature = "dim3")]
        let ang_jac1 = self.cmat1_basis.column(locked_axis).into_owned();
        #[cfg(feature = "dim3")]
        let ang_jac2 = self.cmat2_basis.column(locked_axis).into_owned();

        let rhs_wo_bias = N::zero();
        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = lin_jac.dot(&self.lin_err) * erp_inv_dt;

        let ii_ang_jac1 = body1.ii * ang_jac1;
        let ii_ang_jac2 = body2.ii * ang_jac2;

        JointConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-N::splat(Real::MAX), N::splat(Real::MAX)],
            lin_jac,
            ang_jac1,
            ang_jac2,
            ii_ang_jac1,
            ii_ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn limit_angular<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<N, LANES>,
        body2: &JointSolverBody<N, LANES>,
        _limited_axis: usize,
        limits: [N; 2],
        writeback_id: WritebackId,
    ) -> JointConstraint<N, LANES> {
        let zero = N::zero();
        let half = N::splat(0.5);
        let s_limits = [(limits[0] * half).simd_sin(), (limits[1] * half).simd_sin()];
        #[cfg(feature = "dim2")]
        let s_ang = (self.ang_err.angle() * half).simd_sin();
        #[cfg(feature = "dim3")]
        let s_ang = self.ang_err.imag()[_limited_axis];
        let min_enabled = s_ang.simd_le(s_limits[0]);
        let max_enabled = s_limits[1].simd_le(s_ang);

        let impulse_bounds = [
            N::splat(-Real::INFINITY).select(min_enabled, zero),
            N::splat(Real::INFINITY).select(max_enabled, zero),
        ];

        #[cfg(feature = "dim2")]
        let ang_jac = N::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(_limited_axis).into_owned();
        let rhs_wo_bias = N::zero();
        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = ((s_ang - s_limits[1]).simd_max(zero)
            - (s_limits[0] - s_ang).simd_max(zero))
            * erp_inv_dt;

        let ii_ang_jac1 = body1.ii * ang_jac;
        let ii_ang_jac2 = body2.ii * ang_jac;

        JointConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds,
            lin_jac: na::zero(),
            ang_jac1: ang_jac,
            ang_jac2: ang_jac,
            ii_ang_jac1,
            ii_ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn motor_angular<const LANES: usize>(
        &self,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<N, LANES>,
        body2: &JointSolverBody<N, LANES>,
        _motor_axis: usize,
        motor_params: &MotorParameters<N>,
        writeback_id: WritebackId,
    ) -> JointConstraint<N, LANES> {
        #[cfg(feature = "dim2")]
        let ang_jac = N::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.basis.column(_motor_axis).into_owned();

        let mut rhs_wo_bias = N::zero();
        if motor_params.erp_inv_dt != N::zero() {
            let ang_dist;

            #[cfg(feature = "dim2")]
            {
                ang_dist = self.ang_err.angle();
            }

            #[cfg(feature = "dim3")]
            {
                // Clamp the component from -1.0 to 1.0 to account for slight imprecision
                let clamped_err = self.ang_err.imag()[_motor_axis].simd_clamp(-N::one(), N::one());
                ang_dist = clamped_err.simd_asin() * N::splat(2.0);
            }

            let target_ang = motor_params.target_pos;
            rhs_wo_bias += utils::smallest_abs_diff_between_angles(ang_dist, target_ang)
                * motor_params.erp_inv_dt;
        }

        rhs_wo_bias += -motor_params.target_vel;

        let ii_ang_jac1 = body1.ii * ang_jac;
        let ii_ang_jac2 = body2.ii * ang_jac;

        JointConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac: na::zero(),
            ang_jac1: ang_jac,
            ang_jac2: ang_jac,
            ii_ang_jac1,
            ii_ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff: motor_params.cfm_coeff,
            cfm_gain: motor_params.cfm_gain,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn lock_angular<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<N, LANES>,
        body2: &JointSolverBody<N, LANES>,
        _locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointConstraint<N, LANES> {
        #[cfg(feature = "dim2")]
        let ang_jac = N::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(_locked_axis).into_owned();

        let rhs_wo_bias = N::zero();
        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        #[cfg(feature = "dim2")]
        let rhs_bias = self.ang_err.im * erp_inv_dt;
        #[cfg(feature = "dim3")]
        let rhs_bias = self.ang_err.imag()[_locked_axis] * erp_inv_dt;

        let ii_ang_jac1 = body1.ii * ang_jac;
        let ii_ang_jac2 = body2.ii * ang_jac;

        JointConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-N::splat(Real::MAX), N::splat(Real::MAX)],
            lin_jac: na::zero(),
            ang_jac1: ang_jac,
            ang_jac2: ang_jac,
            ii_ang_jac1,
            ii_ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    /// Orthogonalize the constraints and set their inv_lhs field.
    pub fn finalize_constraints<const LANES: usize>(constraints: &mut [JointConstraint<N, LANES>]) {
        let len = constraints.len();

        if len == 0 {
            return;
        }

        let imsum = constraints[0].im1 + constraints[0].im2;

        // Use the modified Gram-Schmidt orthogonalization.
        for j in 0..len {
            let c_j = &mut constraints[j];
            let dot_jj = c_j.lin_jac.dot(&imsum.component_mul(&c_j.lin_jac))
                + c_j.ii_ang_jac1.gdot(c_j.ang_jac1)
                + c_j.ii_ang_jac2.gdot(c_j.ang_jac2);
            let cfm_gain = dot_jj * c_j.cfm_coeff + c_j.cfm_gain;
            let inv_dot_jj = crate::utils::simd_inv(dot_jj);
            c_j.inv_lhs = crate::utils::simd_inv(dot_jj + cfm_gain); // Don’t forget to update the inv_lhs.
            c_j.cfm_gain = cfm_gain;

            if c_j.impulse_bounds != [-N::splat(Real::MAX), N::splat(Real::MAX)] {
                // Don't remove constraints with limited forces from the others
                // because they may not deliver the necessary forces to fulfill
                // the removed parts of other constraints.
                continue;
            }

            for i in (j + 1)..len {
                let (c_i, c_j) = constraints.index_mut_const(i, j);

                let dot_ij = c_i.lin_jac.dot(&imsum.component_mul(&c_j.lin_jac))
                    + c_i.ii_ang_jac1.gdot(c_j.ang_jac1)
                    + c_i.ii_ang_jac2.gdot(c_j.ang_jac2);
                let coeff = dot_ij * inv_dot_jj;

                c_i.lin_jac -= c_j.lin_jac * coeff;
                c_i.ang_jac1 -= c_j.ang_jac1 * coeff;
                c_i.ang_jac2 -= c_j.ang_jac2 * coeff;
                c_i.ii_ang_jac1 -= c_j.ii_ang_jac1 * coeff;
                c_i.ii_ang_jac2 -= c_j.ii_ang_jac2 * coeff;
                c_i.rhs_wo_bias -= c_j.rhs_wo_bias * coeff;
                c_i.rhs -= c_j.rhs * coeff;
            }
        }
    }
}

impl JointConstraintHelper<Real> {
    #[cfg(feature = "dim3")]
    pub fn limit_angular_coupled(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; 1],
        body1: &JointSolverBody<Real, 1>,
        body2: &JointSolverBody<Real, 1>,
        coupled_axes: u8,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointConstraint<Real, 1> {
        // NOTE: right now, this only supports exactly 2 coupled axes.
        let ang_coupled_axes = coupled_axes >> DIM;
        assert_eq!(ang_coupled_axes.count_ones(), 2);
        let not_coupled_index = ang_coupled_axes.trailing_ones() as usize;
        let axis1 = self.basis.column(not_coupled_index).into_owned();
        let axis2 = self.basis2.column(not_coupled_index).into_owned();

        let rot = Rotation::rotation_between(&axis1, &axis2).unwrap_or_else(Rotation::identity);
        let (ang_jac, angle) = rot
            .axis_angle()
            .map(|(axis, angle)| (axis.into_inner(), angle))
            .unwrap_or_else(|| (axis1.orthonormal_basis()[0], 0.0));

        let min_enabled = angle <= limits[0];
        let max_enabled = limits[1] <= angle;

        let impulse_bounds = [
            if min_enabled { -Real::INFINITY } else { 0.0 },
            if max_enabled { Real::INFINITY } else { 0.0 },
        ];

        let rhs_wo_bias = 0.0;

        let erp_inv_dt = params.joint_erp_inv_dt();
        let cfm_coeff = params.joint_cfm_coeff();
        let rhs_bias = ((angle - limits[1]).max(0.0) - (limits[0] - angle).max(0.0)) * erp_inv_dt;

        let ii_ang_jac1 = body1.ii * ang_jac;
        let ii_ang_jac2 = body2.ii * ang_jac;

        JointConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: 0.0,
            impulse_bounds,
            lin_jac: na::zero(),
            ang_jac1: ang_jac,
            ang_jac2: ang_jac,
            ii_ang_jac1,
            ii_ang_jac2,
            inv_lhs: 0.0, // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: 0.0,
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }
}
