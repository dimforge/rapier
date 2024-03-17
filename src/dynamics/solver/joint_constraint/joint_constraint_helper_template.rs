use na::SMatrix;
use simba::simd::{SimdComplexField, SimdPartialOrd, SimdRealField, SimdValue};

use super::math::*;
use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::{
    JointFixedSolverBody, JointOneBodyConstraint, JointTwoBodyConstraint, WritebackId,
};
use crate::dynamics::solver::joint_constraint::JointSolverBody;
use crate::dynamics::solver::MotorParameters;
use crate::dynamics::{IntegrationParameters, JointIndex};
use crate::num::{One, Zero};
use crate::utils;
use crate::utils::{IndexMut2, SimdCrossMatrix, SimdDot, SimdQuat, SimdRealCopy, SimdVec};

#[derive(Debug, Copy, Clone)]
pub struct JointConstraintHelperTemplate {
    pub basis: Matrix,
    pub basis2: Matrix,
    // TODO: used for angular coupling. Can we avoid storing this?
    pub cmat1_basis: <Vector as SimdCrossMatrix>::CrossMat,
    pub cmat2_basis: <Vector as SimdCrossMatrix>::CrossMat,
    #[cfg(feature = "dim3")]
    pub ang_basis: Matrix,
    pub lin_err: Vector,
    pub ang_err: Rotation,
}

impl JointConstraintHelperTemplate {
    pub fn new(
        frame1: &Isometry,
        frame2: &Isometry,
        world_com1: &Point,
        world_com2: &Point,
        locked_lin_axes: u8,
    ) -> Self {
        let mut frame1 = *frame1;
        let basis = frame1.rotation().to_rotation_matrix().into_inner();
        let lin_err = frame2.translation.as_vector() - frame1.translation.as_vector();

        // Adjust the point of application of the force for the first body,
        // by snapping free axes to the second frame’s center (to account for
        // the allowed relative movement).
        {
            let mut new_center1 = frame2.translation.into_vector(); // First, assume all dofs are free.

            // Then snap the locked ones.
            for i in 0..DIM {
                if locked_lin_axes & (1 << i) != 0 {
                    let axis = basis.column(i).into_owned();
                    new_center1 -= axis * lin_err.gdot(axis);
                }
            }
            *frame1.translation.as_vector_mut() = new_center1;
        }

        let r1 = frame1.translation.as_vector() - world_com1.as_vector();
        let r2 = frame2.translation.as_vector() - world_com2.as_vector();

        let cmat1 = r1.gcross_matrix();
        let cmat2 = r2.gcross_matrix();

        #[cfg(feature = "dim3")]
        let mut ang_basis = frame1
            .rotation()
            .diff_conj1_2(frame2.rotation())
            .transpose();

        #[allow(unused_mut)] // The mut is needed for 3D
        #[cfg(feature = "linalg-nalgebra")]
        let mut ang_err = frame1.rotation().inverse() * *frame2.rotation();
        #[cfg(all(feature = "dim2", feature = "linalg-glam"))]
        // Transpose will be way faster than inverse.
        let mut ang_err = frame1.rotation().transpose() * *frame2.rotation();
        #[cfg(all(feature = "dim3", feature = "linalg-glam"))]
        let mut ang_err = frame1.rotation().inverse() * *frame2.rotation();

        #[cfg(feature = "dim3")]
        {
            let sgn =
                Real::one().simd_copysign(frame1.rotation().dot(frame2.rotation().into_inner()));
            ang_basis *= sgn;
            *ang_err.as_mut_unchecked() = ang_err.into_inner() * sgn;
        }

        Self {
            basis,
            basis2: frame2.rotation().to_rotation_matrix().into_inner(),
            #[cfg(any(feature = "dim3", feature = "linalg-nalgebra"))]
            cmat1_basis: cmat1 * basis,
            #[cfg(any(feature = "dim3", feature = "linalg-nalgebra"))]
            cmat2_basis: cmat2 * basis,
            #[cfg(all(feature = "dim2", feature = "linalg-glam"))]
            cmat1_basis: basis.transpose() * cmat1,
            #[cfg(all(feature = "dim2", feature = "linalg-glam"))]
            cmat2_basis: basis.transpose() * cmat2,
            #[cfg(feature = "dim3")]
            ang_basis,
            lin_err,
            ang_err,
        }
    }

    pub fn limit_linear(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<Real, LANES>,
        body2: &JointSolverBody<Real, LANES>,
        limited_axis: usize,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointTwoBodyConstraint<Real, LANES> {
        let zero = Real::zero();
        let mut constraint =
            self.lock_linear(params, joint_id, body1, body2, limited_axis, writeback_id);

        let dist = self.lin_err.gdot(constraint.lin_jac);
        let min_enabled = dist.simd_le(limits[0]);
        let max_enabled = limits[1].simd_le(dist);

        let erp_inv_dt = Real::splat(params.joint_erp_inv_dt());
        let cfm_coeff = Real::splat(params.joint_cfm_coeff());
        let rhs_bias =
            ((dist - limits[1]).simd_max(zero) - (limits[0] - dist).simd_max(zero)) * erp_inv_dt;
        constraint.rhs = constraint.rhs_wo_bias + rhs_bias;
        constraint.cfm_coeff = cfm_coeff;
        constraint.impulse_bounds = [
            Real::splat(-RealConst::INFINITY).select(min_enabled, zero),
            Real::splat(RealConst::INFINITY).select(max_enabled, zero),
        ];

        constraint
    }

    pub fn limit_linear_coupled(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<Real, LANES>,
        body2: &JointSolverBody<Real, LANES>,
        coupled_axes: u8,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointTwoBodyConstraint<Real, LANES> {
        let zero = Real::zero();
        let mut lin_jac = Vector::zeros();
        let mut ang_jac1: AngVector = Default::default();
        let mut ang_jac2: AngVector = Default::default();

        for i in 0..DIM {
            if coupled_axes & (1 << i) != 0 {
                let coeff = self.basis.column(i).dot(self.lin_err);
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

        let rhs_wo_bias = (dist - limits[1]).simd_min(zero) * Real::splat(params.inv_dt());

        ang_jac1 = body1.sqrt_ii * ang_jac1;
        ang_jac2 = body2.sqrt_ii * ang_jac2;

        let erp_inv_dt = Real::splat(params.joint_erp_inv_dt());
        let cfm_coeff = Real::splat(params.joint_cfm_coeff());
        let rhs_bias = (dist - limits[1]).simd_max(zero) * erp_inv_dt;
        let rhs = rhs_wo_bias + rhs_bias;
        let impulse_bounds = [Real::zero(), Real::splat(RealConst::INFINITY)];

        JointTwoBodyConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: Real::zero(),
            impulse_bounds,
            lin_jac,
            ang_jac1,
            ang_jac2,
            inv_lhs: Real::zero(), // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: Real::zero(),
            rhs,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn motor_linear(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<Real, LANES>,
        body2: &JointSolverBody<Real, LANES>,
        motor_axis: usize,
        motor_params: &MotorParameters<Real>,
        limits: Option<[Real; 2]>,
        writeback_id: WritebackId,
    ) -> JointTwoBodyConstraint<Real, LANES> {
        let inv_dt = Real::splat(params.inv_dt());
        let mut constraint =
            self.lock_linear(params, joint_id, body1, body2, motor_axis, writeback_id);

        let mut rhs_wo_bias = Real::zero();
        if motor_params.erp_inv_dt != Real::zero() {
            let dist = self.lin_err.gdot(constraint.lin_jac);
            rhs_wo_bias += (dist - motor_params.target_pos) * motor_params.erp_inv_dt;
        }

        let mut target_vel = motor_params.target_vel;
        if let Some(limits) = limits {
            let dist = self.lin_err.gdot(constraint.lin_jac);
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

    pub fn lock_linear(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<Real, LANES>,
        body2: &JointSolverBody<Real, LANES>,
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointTwoBodyConstraint<Real, LANES> {
        let lin_jac = self.basis.column(locked_axis).into_owned();
        #[cfg(feature = "dim2")]
        let mut ang_jac1 = self.cmat1_basis[locked_axis];
        #[cfg(feature = "dim2")]
        let mut ang_jac2 = self.cmat2_basis[locked_axis];
        #[cfg(feature = "dim3")]
        let mut ang_jac1 = self.cmat1_basis.column(locked_axis).into_owned();
        #[cfg(feature = "dim3")]
        let mut ang_jac2 = self.cmat2_basis.column(locked_axis).into_owned();

        let rhs_wo_bias = Real::zero();
        let erp_inv_dt = Real::splat(params.joint_erp_inv_dt());
        let cfm_coeff = Real::splat(params.joint_cfm_coeff());
        let rhs_bias = lin_jac.gdot(self.lin_err) * erp_inv_dt;

        ang_jac1 = body1.sqrt_ii * ang_jac1;
        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointTwoBodyConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: Real::zero(),
            impulse_bounds: [-Real::splat(RealConst::MAX), Real::splat(RealConst::MAX)],
            lin_jac,
            ang_jac1,
            ang_jac2,
            inv_lhs: Real::zero(), // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: Real::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn limit_angular(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<Real, LANES>,
        body2: &JointSolverBody<Real, LANES>,
        _limited_axis: usize,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointTwoBodyConstraint<Real, LANES> {
        let zero = Real::zero();
        let half = Real::splat(0.5);
        let s_limits = [(limits[0] * half).simd_sin(), (limits[1] * half).simd_sin()];
        #[cfg(feature = "dim2")]
        let s_ang = (self.ang_err.angle() * half).simd_sin();
        #[cfg(feature = "dim3")]
        let s_ang = self.ang_err.imag()[_limited_axis];
        let min_enabled = s_ang.simd_le(s_limits[0]);
        let max_enabled = s_limits[1].simd_le(s_ang);

        let impulse_bounds = [
            Real::splat(-RealConst::INFINITY).select(min_enabled, zero),
            Real::splat(RealConst::INFINITY).select(max_enabled, zero),
        ];

        #[cfg(feature = "dim2")]
        let ang_jac = Real::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(_limited_axis).into_owned();
        let rhs_wo_bias = Real::zero();
        let erp_inv_dt = Real::splat(params.joint_erp_inv_dt());
        let cfm_coeff = Real::splat(params.joint_cfm_coeff());
        let rhs_bias = ((s_ang - s_limits[1]).simd_max(zero)
            - (s_limits[0] - s_ang).simd_max(zero))
            * erp_inv_dt;

        let ang_jac1 = body1.sqrt_ii * ang_jac;
        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointTwoBodyConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: Real::zero(),
            impulse_bounds,
            lin_jac: Default::default(),
            ang_jac1,
            ang_jac2,
            inv_lhs: Real::zero(), // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: Real::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn motor_angular(
        &self,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<Real, LANES>,
        body2: &JointSolverBody<Real, LANES>,
        _motor_axis: usize,
        motor_params: &MotorParameters<Real>,
        writeback_id: WritebackId,
    ) -> JointTwoBodyConstraint<Real, LANES> {
        #[cfg(feature = "dim2")]
        let ang_jac = Real::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.basis.column(_motor_axis).into_owned();

        let mut rhs_wo_bias = Real::zero();
        if motor_params.erp_inv_dt != Real::zero() {
            #[cfg(feature = "dim2")]
            let ang_dist = self.ang_err.angle();
            #[cfg(feature = "dim3")]
            let ang_dist = self.ang_err.imag()[_motor_axis].simd_asin() * Real::splat(2.0);
            let target_ang = motor_params.target_pos;
            rhs_wo_bias += utils::smallest_abs_diff_between_angles(ang_dist, target_ang)
                * motor_params.erp_inv_dt;
        }

        rhs_wo_bias += -motor_params.target_vel;

        let ang_jac1 = body1.sqrt_ii * ang_jac;
        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointTwoBodyConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: Real::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac: Default::default(),
            ang_jac1,
            ang_jac2,
            inv_lhs: Real::zero(), // Will be set during ortogonalization.
            cfm_coeff: motor_params.cfm_coeff,
            cfm_gain: motor_params.cfm_gain,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn lock_angular(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<Real, LANES>,
        body2: &JointSolverBody<Real, LANES>,
        _locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointTwoBodyConstraint<Real, LANES> {
        #[cfg(feature = "dim2")]
        let ang_jac = Real::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(_locked_axis).into_owned();

        let rhs_wo_bias = Real::zero();
        let erp_inv_dt = Real::splat(params.joint_erp_inv_dt());
        let cfm_coeff = Real::splat(params.joint_cfm_coeff());
        #[cfg(feature = "dim2")]
        let rhs_bias = self.ang_err.imag() * erp_inv_dt;
        #[cfg(feature = "dim3")]
        let rhs_bias = self.ang_err.imag()[_locked_axis] * erp_inv_dt;

        let ang_jac1 = body1.sqrt_ii * ang_jac;
        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointTwoBodyConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: Real::zero(),
            impulse_bounds: [-Real::splat(RealConst::MAX), Real::splat(RealConst::MAX)],
            lin_jac: Default::default(),
            ang_jac1,
            ang_jac2,
            inv_lhs: Real::zero(), // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: Real::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    /// Orthogonalize the constraints and set their inv_lhs field.
    pub fn finalize_constraints(constraints: &mut [JointTwoBodyConstraint<Real, LANES>]) {
        let len = constraints.len();

        if len == 0 {
            return;
        }

        let imsum = constraints[0].im1 + constraints[0].im2;

        // Use the modified Gram-Schmidt orthogonalization.
        for j in 0..len {
            let c_j = &mut constraints[j];
            let dot_jj = c_j.lin_jac.gdot(imsum.component_mul(&c_j.lin_jac))
                + c_j.ang_jac1.gdot(c_j.ang_jac1)
                + c_j.ang_jac2.gdot(c_j.ang_jac2);
            let cfm_gain = dot_jj * c_j.cfm_coeff + c_j.cfm_gain;
            let inv_dot_jj = crate::utils::simd_inv(dot_jj);
            c_j.inv_lhs = crate::utils::simd_inv(dot_jj + cfm_gain); // Don’t forget to update the inv_lhs.
            c_j.cfm_gain = cfm_gain;

            if c_j.impulse_bounds != [-Real::splat(RealConst::MAX), Real::splat(RealConst::MAX)] {
                // Don't remove constraints with limited forces from the others
                // because they may not deliver the necessary forces to fulfill
                // the removed parts of other constraints.
                continue;
            }

            for i in (j + 1)..len {
                let (c_i, c_j) = constraints.index_mut_const(i, j);

                let dot_ij = c_i.lin_jac.gdot(imsum.component_mul(&c_j.lin_jac))
                    + c_i.ang_jac1.gdot(c_j.ang_jac1)
                    + c_i.ang_jac2.gdot(c_j.ang_jac2);
                let coeff = dot_ij * inv_dot_jj;

                c_i.lin_jac -= c_j.lin_jac * coeff;
                c_i.ang_jac1 -= c_j.ang_jac1 * coeff;
                c_i.ang_jac2 -= c_j.ang_jac2 * coeff;
                c_i.rhs_wo_bias -= c_j.rhs_wo_bias * coeff;
                c_i.rhs -= c_j.rhs * coeff;
            }
        }
    }

    pub fn limit_linear_one_body(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<Real>,
        body2: &JointSolverBody<Real, LANES>,
        limited_axis: usize,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<Real, LANES> {
        let zero = Real::zero();
        let lin_jac = self.basis.column(limited_axis).into_owned();
        let dist = self.lin_err.gdot(lin_jac);

        let min_enabled = dist.simd_le(limits[0]);
        let max_enabled = limits[1].simd_le(dist);

        let impulse_bounds = [
            Real::splat(-RealConst::INFINITY).select(min_enabled, zero),
            Real::splat(RealConst::INFINITY).select(max_enabled, zero),
        ];

        let ang_jac1 = self.cmat1_basis.column(limited_axis).into_owned();
        #[cfg(feature = "dim2")]
        let mut ang_jac2 = self.cmat2_basis[limited_axis];
        #[cfg(feature = "dim3")]
        let mut ang_jac2 = self.cmat2_basis.column(limited_axis).into_owned();

        let rhs_wo_bias = -lin_jac.gdot(body1.linvel) - ang_jac1.gdot(body1.angvel);
        let erp_inv_dt = Real::splat(params.joint_erp_inv_dt());
        let cfm_coeff = Real::splat(params.joint_cfm_coeff());
        let rhs_bias =
            ((dist - limits[1]).simd_max(zero) - (limits[0] - dist).simd_max(zero)) * erp_inv_dt;

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: zero,
            impulse_bounds,
            lin_jac,
            ang_jac2,
            inv_lhs: zero, // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: Real::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn limit_linear_coupled_one_body(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<Real>,
        body2: &JointSolverBody<Real, LANES>,
        coupled_axes: u8,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<Real, LANES> {
        let zero = Real::zero();
        let mut lin_jac = Vector::zeros();
        let mut ang_jac1: AngVector = Default::default();
        let mut ang_jac2: AngVector = Default::default();

        for i in 0..DIM {
            if coupled_axes & (1 << i) != 0 {
                let coeff = self.basis.column(i).dot(self.lin_err);
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

        // FIXME: handle min limit too.
        let proj_vel1 = -lin_jac.gdot(body1.linvel) - ang_jac1.gdot(body1.angvel);
        let rhs_wo_bias =
            proj_vel1 + (dist - limits[1]).simd_min(zero) * Real::splat(params.inv_dt());

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        let erp_inv_dt = Real::splat(params.joint_erp_inv_dt());
        let cfm_coeff = Real::splat(params.joint_cfm_coeff());
        let rhs_bias = (dist - limits[1]).simd_max(zero) * erp_inv_dt;
        let rhs = rhs_wo_bias + rhs_bias;
        let impulse_bounds = [Real::zero(), Real::splat(RealConst::INFINITY)];

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: Real::zero(),
            impulse_bounds,
            lin_jac,
            ang_jac2,
            inv_lhs: Real::zero(), // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: Real::zero(),
            rhs,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn motor_linear_one_body(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<Real>,
        body2: &JointSolverBody<Real, LANES>,
        motor_axis: usize,
        motor_params: &MotorParameters<Real>,
        limits: Option<[Real; 2]>,
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<Real, LANES> {
        let inv_dt = Real::splat(params.inv_dt());

        let lin_jac = self.basis.column(motor_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(motor_axis).into_owned();
        #[cfg(feature = "dim2")]
        let mut ang_jac2 = self.cmat2_basis[motor_axis];
        #[cfg(feature = "dim3")]
        let mut ang_jac2 = self.cmat2_basis.column(motor_axis).into_owned();

        let mut rhs_wo_bias = Real::zero();
        if motor_params.erp_inv_dt != Real::zero() {
            let dist = self.lin_err.gdot(lin_jac);
            rhs_wo_bias += (dist - motor_params.target_pos) * motor_params.erp_inv_dt;
        }

        let mut target_vel = motor_params.target_vel;
        if let Some(limits) = limits {
            let dist = self.lin_err.gdot(lin_jac);
            target_vel =
                target_vel.simd_clamp((limits[0] - dist) * inv_dt, (limits[1] - dist) * inv_dt);
        };

        let proj_vel1 = -lin_jac.gdot(body1.linvel) - ang_jac1.gdot(body1.angvel);
        rhs_wo_bias += proj_vel1 - target_vel;

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: Real::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac,
            ang_jac2,
            inv_lhs: Real::zero(), // Will be set during ortogonalization.
            cfm_coeff: motor_params.cfm_coeff,
            cfm_gain: motor_params.cfm_gain,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn motor_linear_coupled_one_body(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<Real>,
        body2: &JointSolverBody<Real, LANES>,
        coupled_axes: u8,
        motor_params: &MotorParameters<Real>,
        limits: Option<[Real; 2]>,
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<Real, LANES> {
        let inv_dt = Real::splat(params.inv_dt());

        let mut lin_jac = Vector::zeros();
        let mut ang_jac1: AngVector = Default::default();
        let mut ang_jac2: AngVector = Default::default();

        for i in 0..DIM {
            if coupled_axes & (1 << i) != 0 {
                let coeff = self.basis.column(i).dot(self.lin_err);
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

        let mut rhs_wo_bias = Real::zero();
        if motor_params.erp_inv_dt != Real::zero() {
            rhs_wo_bias += (dist - motor_params.target_pos) * motor_params.erp_inv_dt;
        }

        let mut target_vel = motor_params.target_vel;
        if let Some(limits) = limits {
            target_vel =
                target_vel.simd_clamp((limits[0] - dist) * inv_dt, (limits[1] - dist) * inv_dt);
        };

        let proj_vel1 = -lin_jac.gdot(body1.linvel) - ang_jac1.gdot(body1.angvel);
        rhs_wo_bias += proj_vel1 - target_vel;

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: Real::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac,
            ang_jac2,
            inv_lhs: Real::zero(), // Will be set during ortogonalization.
            cfm_coeff: motor_params.cfm_coeff,
            cfm_gain: motor_params.cfm_gain,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn lock_linear_one_body(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<Real>,
        body2: &JointSolverBody<Real, LANES>,
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<Real, LANES> {
        let lin_jac = self.basis.column(locked_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(locked_axis).into_owned();
        #[cfg(feature = "dim2")]
        let mut ang_jac2 = self.cmat2_basis[locked_axis];
        #[cfg(feature = "dim3")]
        let mut ang_jac2 = self.cmat2_basis.column(locked_axis).into_owned();

        let rhs_wo_bias = -lin_jac.gdot(body1.linvel) - ang_jac1.gdot(body1.angvel);

        let erp_inv_dt = Real::splat(params.joint_erp_inv_dt());
        let cfm_coeff = Real::splat(params.joint_cfm_coeff());
        let rhs_bias = lin_jac.gdot(self.lin_err) * erp_inv_dt;

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: Real::zero(),
            impulse_bounds: [-Real::splat(RealConst::MAX), Real::splat(RealConst::MAX)],
            lin_jac,
            ang_jac2,
            inv_lhs: Real::zero(), // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: Real::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn motor_angular_one_body(
        &self,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<Real>,
        body2: &JointSolverBody<Real, LANES>,
        _motor_axis: usize,
        motor_params: &MotorParameters<Real>,
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<Real, LANES> {
        #[cfg(feature = "dim2")]
        let ang_jac = Real::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.basis.column(_motor_axis).into_owned();

        let mut rhs_wo_bias = Real::zero();
        if motor_params.erp_inv_dt != Real::zero() {
            #[cfg(feature = "dim2")]
            let ang_dist = self.ang_err.angle();
            #[cfg(feature = "dim3")]
            let ang_dist = self.ang_err.imag()[_motor_axis].simd_asin() * Real::splat(2.0);
            let target_ang = motor_params.target_pos;
            rhs_wo_bias += utils::smallest_abs_diff_between_angles(ang_dist, target_ang)
                * motor_params.erp_inv_dt;
        }

        let proj_vel1 = -ang_jac.gdot(body1.angvel);
        rhs_wo_bias += proj_vel1 - motor_params.target_vel;

        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: Real::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac: Default::default(),
            ang_jac2,
            inv_lhs: Real::zero(), // Will be set during ortogonalization.
            cfm_coeff: motor_params.cfm_coeff,
            cfm_gain: motor_params.cfm_gain,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn limit_angular_one_body(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<Real>,
        body2: &JointSolverBody<Real, LANES>,
        _limited_axis: usize,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<Real, LANES> {
        let zero = Real::zero();
        let half = Real::splat(0.5);
        let s_limits = [(limits[0] * half).simd_sin(), (limits[1] * half).simd_sin()];
        #[cfg(feature = "dim2")]
        let s_ang = (self.ang_err.angle() * half).simd_sin();
        #[cfg(feature = "dim3")]
        let s_ang = self.ang_err.imag()[_limited_axis];
        let min_enabled = s_ang.simd_le(s_limits[0]);
        let max_enabled = s_limits[1].simd_le(s_ang);

        let impulse_bounds = [
            Real::splat(-RealConst::INFINITY).select(min_enabled, zero),
            Real::splat(RealConst::INFINITY).select(max_enabled, zero),
        ];

        #[cfg(feature = "dim2")]
        let ang_jac = Real::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(_limited_axis).into_owned();
        let rhs_wo_bias = -ang_jac.gdot(body1.angvel);

        let erp_inv_dt = Real::splat(params.joint_erp_inv_dt());
        let cfm_coeff = Real::splat(params.joint_cfm_coeff());
        let rhs_bias = ((s_ang - s_limits[1]).simd_max(zero)
            - (s_limits[0] - s_ang).simd_max(zero))
            * erp_inv_dt;

        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: zero,
            impulse_bounds,
            lin_jac: Default::default(),
            ang_jac2,
            inv_lhs: zero, // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: Default::default(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn lock_angular_one_body(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<Real>,
        body2: &JointSolverBody<Real, LANES>,
        _locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<Real, LANES> {
        #[cfg(feature = "dim2")]
        let ang_jac = Real::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(_locked_axis).into_owned();

        let rhs_wo_bias = -ang_jac.gdot(body1.angvel);

        let erp_inv_dt = Real::splat(params.joint_erp_inv_dt());
        let cfm_coeff = Real::splat(params.joint_cfm_coeff());
        #[cfg(feature = "dim2")]
        let rhs_bias = self.ang_err.imag() * erp_inv_dt;
        #[cfg(feature = "dim3")]
        let rhs_bias = self.ang_err.imag()[_locked_axis] * erp_inv_dt;

        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: Real::zero(),
            impulse_bounds: [-Real::splat(RealConst::MAX), Real::splat(RealConst::MAX)],
            lin_jac: Default::default(),
            ang_jac2,
            inv_lhs: Real::zero(), // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: Real::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    /// Orthogonalize the constraints and set their inv_lhs field.
    pub fn finalize_one_body_constraints(constraints: &mut [JointOneBodyConstraint<Real, LANES>]) {
        let len = constraints.len();

        if len == 0 {
            return;
        }

        let imsum = constraints[0].im2;

        // Use the modified Gram-Schmidt orthogonalization.
        for j in 0..len {
            let c_j = &mut constraints[j];
            let dot_jj = c_j.lin_jac.gdot(imsum.component_mul_simd(&c_j.lin_jac))
                + c_j.ang_jac2.gdot(c_j.ang_jac2);
            let cfm_gain = dot_jj * c_j.cfm_coeff + c_j.cfm_gain;
            let inv_dot_jj = crate::utils::simd_inv(dot_jj);
            c_j.inv_lhs = crate::utils::simd_inv(dot_jj + cfm_gain); // Don’t forget to update the inv_lhs.
            c_j.cfm_gain = cfm_gain;

            if c_j.impulse_bounds != [-Real::splat(RealConst::MAX), Real::splat(RealConst::MAX)] {
                // Don't remove constraints with limited forces from the others
                // because they may not deliver the necessary forces to fulfill
                // the removed parts of other constraints.
                continue;
            }

            for i in j + 1..len {
                let (c_i, c_j) = constraints.index_mut_const(i, j);

                let dot_ij = c_i.lin_jac.gdot(imsum.component_mul_simd(&c_j.lin_jac))
                    + c_i.ang_jac2.gdot(c_j.ang_jac2);
                let coeff = dot_ij * inv_dot_jj;

                c_i.lin_jac -= c_j.lin_jac * coeff;
                c_i.ang_jac2 -= c_j.ang_jac2 * coeff;
                c_i.rhs_wo_bias -= c_j.rhs_wo_bias * coeff;
                c_i.rhs -= c_j.rhs * coeff;
            }
        }
    }
}
