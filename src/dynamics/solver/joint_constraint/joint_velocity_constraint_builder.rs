use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::{
    JointVelocityConstraint, JointVelocityGroundConstraint, WritebackId,
};
use crate::dynamics::solver::joint_constraint::SolverBody;
use crate::dynamics::solver::MotorParameters;
use crate::dynamics::{IntegrationParameters, JointIndex, JointLimits};
use crate::math::{AngVector, Isometry, Matrix, Point, Real, Rotation, Vector, ANG_DIM, DIM};
use crate::utils::{IndexMut2, WCrossMatrix, WDot, WQuat, WReal};
use na::SMatrix;

#[cfg(feature = "dim3")]
use crate::utils::WBasis;

#[derive(Debug, Copy, Clone)]
pub struct JointVelocityConstraintBuilder<N: WReal> {
    pub basis: Matrix<N>,
    pub basis2: Matrix<N>, // TODO: used for angular coupling. Can we avoid storing this?
    pub cmat1_basis: SMatrix<N, ANG_DIM, DIM>,
    pub cmat2_basis: SMatrix<N, ANG_DIM, DIM>,
    pub ang_basis: SMatrix<N, ANG_DIM, ANG_DIM>,
    pub lin_err: Vector<N>,
    pub ang_err: Rotation<N>,
}

impl<N: WReal> JointVelocityConstraintBuilder<N> {
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

        #[allow(unused_mut)] // The mut is needed for 3D
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
            basis2: frame2.rotation.to_rotation_matrix().into_inner(),
            cmat1_basis: cmat1 * basis,
            cmat2_basis: cmat2 * basis,
            ang_basis,
            lin_err,
            ang_err,
        }
    }

    pub fn limit_linear<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        limited_axis: usize,
        limits: [N; 2],
        writeback_id: WritebackId,
    ) -> JointVelocityConstraint<N, LANES> {
        let zero = N::zero();
        let mut constraint =
            self.lock_linear(params, joint_id, body1, body2, limited_axis, writeback_id);

        let dist = self.lin_err.dot(&constraint.lin_jac);
        let min_enabled = dist.simd_lt(limits[0]);
        let max_enabled = limits[1].simd_lt(dist);

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
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        limited_coupled_axes: u8,
        limits: &[JointLimits<N>],
        writeback_id: WritebackId,
    ) -> JointVelocityConstraint<N, LANES> {
        let zero = N::zero();
        let mut lin_jac = Vector::zeros();
        let mut ang_jac1: AngVector<N> = na::zero();
        let mut ang_jac2: AngVector<N> = na::zero();
        let mut limit = N::zero();

        for i in 0..DIM {
            if limited_coupled_axes & (1 << i) != 0 {
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
                limit += limits[i].max * limits[i].max;
            }
        }

        limit = limit.simd_sqrt();
        let dist = lin_jac.norm();
        let inv_dist = crate::utils::simd_inv(dist);
        lin_jac *= inv_dist;
        ang_jac1 *= inv_dist;
        ang_jac2 *= inv_dist;

        let dvel = lin_jac.dot(&(body2.linvel - body1.linvel))
            + (ang_jac2.gdot(body2.angvel) - ang_jac1.gdot(body1.angvel));
        let rhs_wo_bias = dvel + (dist - limit).simd_min(zero) * N::splat(params.inv_dt());

        ang_jac1 = body1.sqrt_ii * ang_jac1;
        ang_jac2 = body2.sqrt_ii * ang_jac2;

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = (dist - limit).simd_max(zero) * erp_inv_dt;
        let rhs = rhs_wo_bias + rhs_bias;
        let impulse_bounds = [N::zero(), N::splat(Real::INFINITY)];

        JointVelocityConstraint {
            joint_id,
            mj_lambda1: body1.mj_lambda,
            mj_lambda2: body2.mj_lambda,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds,
            lin_jac,
            ang_jac1,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during ortogonalization.
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
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        motor_axis: usize,
        motor_params: &MotorParameters<N>,
        limits: Option<[N; 2]>,
        writeback_id: WritebackId,
    ) -> JointVelocityConstraint<N, LANES> {
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

        let dvel = constraint.lin_jac.dot(&(body2.linvel - body1.linvel))
            + (constraint.ang_jac2.gdot(body2.angvel) - constraint.ang_jac1.gdot(body1.angvel));
        rhs_wo_bias += dvel - target_vel;

        constraint.cfm_coeff = motor_params.cfm_coeff;
        constraint.cfm_gain = motor_params.cfm_gain;
        constraint.impulse_bounds = [-motor_params.max_impulse, motor_params.max_impulse];
        constraint.rhs = rhs_wo_bias;
        constraint.rhs_wo_bias = rhs_wo_bias;
        constraint
    }

    pub fn lock_linear<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointVelocityConstraint<N, LANES> {
        let lin_jac = self.basis.column(locked_axis).into_owned();
        #[cfg(feature = "dim2")]
        let mut ang_jac1 = self.cmat1_basis[locked_axis];
        #[cfg(feature = "dim2")]
        let mut ang_jac2 = self.cmat2_basis[locked_axis];
        #[cfg(feature = "dim3")]
        let mut ang_jac1 = self.cmat1_basis.column(locked_axis).into_owned();
        #[cfg(feature = "dim3")]
        let mut ang_jac2 = self.cmat2_basis.column(locked_axis).into_owned();

        let dvel = lin_jac.dot(&(body2.linvel - body1.linvel))
            + (ang_jac2.gdot(body2.angvel) - ang_jac1.gdot(body1.angvel));
        let rhs_wo_bias = dvel;

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = lin_jac.dot(&self.lin_err) * erp_inv_dt;

        ang_jac1 = body1.sqrt_ii * ang_jac1;
        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointVelocityConstraint {
            joint_id,
            mj_lambda1: body1.mj_lambda,
            mj_lambda2: body2.mj_lambda,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-N::splat(Real::MAX), N::splat(Real::MAX)],
            lin_jac,
            ang_jac1,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during ortogonalization.
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
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        limited_axis: usize,
        limits: [N; 2],
        writeback_id: WritebackId,
    ) -> JointVelocityConstraint<N, LANES> {
        let zero = N::zero();
        let half = N::splat(0.5);
        let s_limits = [(limits[0] * half).simd_sin(), (limits[1] * half).simd_sin()];
        #[cfg(feature = "dim2")]
        let s_ang = self.ang_err.im;
        #[cfg(feature = "dim3")]
        let s_ang = self.ang_err.imag()[limited_axis];
        let min_enabled = s_ang.simd_lt(s_limits[0]);
        let max_enabled = s_limits[1].simd_lt(s_ang);

        let impulse_bounds = [
            N::splat(-Real::INFINITY).select(min_enabled, zero),
            N::splat(Real::INFINITY).select(max_enabled, zero),
        ];

        #[cfg(feature = "dim2")]
        let ang_jac = self.ang_basis[limited_axis];
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(limited_axis).into_owned();
        let dvel = ang_jac.gdot(body2.angvel) - ang_jac.gdot(body1.angvel);
        let rhs_wo_bias = dvel;

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = ((s_ang - s_limits[1]).simd_max(zero)
            - (s_limits[0] - s_ang).simd_max(zero))
            * erp_inv_dt;

        let ang_jac1 = body1.sqrt_ii * ang_jac;
        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointVelocityConstraint {
            joint_id,
            mj_lambda1: body1.mj_lambda,
            mj_lambda2: body2.mj_lambda,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds,
            lin_jac: na::zero(),
            ang_jac1,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during ortogonalization.
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
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        _motor_axis: usize,
        motor_params: &MotorParameters<N>,
        writeback_id: WritebackId,
    ) -> JointVelocityConstraint<N, LANES> {
        // let mut ang_jac = self.ang_basis.column(_motor_axis).into_owned();
        #[cfg(feature = "dim2")]
        let ang_jac = N::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.basis.column(_motor_axis).into_owned();

        let mut rhs_wo_bias = N::zero();
        if motor_params.erp_inv_dt != N::zero() {
            #[cfg(feature = "dim2")]
            {
                let s_ang_dist = self.ang_err.angle();
                let s_target_ang = motor_params.target_pos;
                rhs_wo_bias += ((s_ang_dist - s_target_ang) % N::simd_two_pi()) / N::simd_two_pi()
                    * motor_params.erp_inv_dt;
            }
            #[cfg(feature = "dim3")]
            {
                let s_ang_dist = self.ang_err.imag()[_motor_axis];
                let s_target_ang = motor_params.target_pos.simd_sin();
                rhs_wo_bias += (s_ang_dist - s_target_ang) * motor_params.erp_inv_dt;
            }
        }

        let dvel = ang_jac.gdot(body2.angvel) - ang_jac.gdot(body1.angvel);
        rhs_wo_bias += dvel - motor_params.target_vel;

        let ang_jac1 = body1.sqrt_ii * ang_jac;
        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointVelocityConstraint {
            joint_id,
            mj_lambda1: body1.mj_lambda,
            mj_lambda2: body2.mj_lambda,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac: na::zero(),
            ang_jac1,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during ortogonalization.
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
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointVelocityConstraint<N, LANES> {
        #[cfg(feature = "dim2")]
        let ang_jac = self.ang_basis[locked_axis];
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(locked_axis).into_owned();

        let dvel = ang_jac.gdot(body2.angvel) - ang_jac.gdot(body1.angvel);
        let rhs_wo_bias = dvel;

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        #[cfg(feature = "dim2")]
        let rhs_bias = self.ang_err.im * erp_inv_dt;
        #[cfg(feature = "dim3")]
        let rhs_bias = self.ang_err.imag()[locked_axis] * erp_inv_dt;

        let ang_jac1 = body1.sqrt_ii * ang_jac;
        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointVelocityConstraint {
            joint_id,
            mj_lambda1: body1.mj_lambda,
            mj_lambda2: body2.mj_lambda,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-N::splat(Real::MAX), N::splat(Real::MAX)],
            lin_jac: na::zero(),
            ang_jac1,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    /// Orthogonalize the constraints and set their inv_lhs field.
    pub fn finalize_constraints<const LANES: usize>(
        constraints: &mut [JointVelocityConstraint<N, LANES>],
    ) {
        let len = constraints.len();

        if len == 0 {
            return;
        }

        let imsum = constraints[0].im1 + constraints[0].im2;

        // Use the modified Gram-Schmidt orthogonalization.
        for j in 0..len {
            let c_j = &mut constraints[j];
            let dot_jj = c_j.lin_jac.dot(&imsum.component_mul(&c_j.lin_jac))
                + c_j.ang_jac1.gdot(c_j.ang_jac1)
                + c_j.ang_jac2.gdot(c_j.ang_jac2);
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

    pub fn limit_linear_ground<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        limited_axis: usize,
        limits: [N; 2],
        writeback_id: WritebackId,
    ) -> JointVelocityGroundConstraint<N, LANES> {
        let zero = N::zero();
        let lin_jac = self.basis.column(limited_axis).into_owned();
        let dist = self.lin_err.dot(&lin_jac);

        let min_enabled = dist.simd_lt(limits[0]);
        let max_enabled = limits[1].simd_lt(dist);

        let impulse_bounds = [
            N::splat(-Real::INFINITY).select(min_enabled, zero),
            N::splat(Real::INFINITY).select(max_enabled, zero),
        ];

        let ang_jac1 = self.cmat1_basis.column(limited_axis).into_owned();
        #[cfg(feature = "dim2")]
        let mut ang_jac2 = self.cmat2_basis[limited_axis];
        #[cfg(feature = "dim3")]
        let mut ang_jac2 = self.cmat2_basis.column(limited_axis).into_owned();

        let dvel = lin_jac.dot(&(body2.linvel - body1.linvel))
            + (ang_jac2.gdot(body2.angvel) - ang_jac1.gdot(body1.angvel));
        let rhs_wo_bias = dvel;

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias =
            ((dist - limits[1]).simd_max(zero) - (limits[0] - dist).simd_max(zero)) * erp_inv_dt;

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointVelocityGroundConstraint {
            joint_id,
            mj_lambda2: body2.mj_lambda,
            im2: body2.im,
            impulse: zero,
            impulse_bounds,
            lin_jac,
            ang_jac2,
            inv_lhs: zero, // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn limit_linear_coupled_ground<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        limited_coupled_axes: u8,
        limits: &[JointLimits<N>],
        writeback_id: WritebackId,
    ) -> JointVelocityGroundConstraint<N, LANES> {
        let zero = N::zero();
        let mut lin_jac = Vector::zeros();
        let mut ang_jac1: AngVector<N> = na::zero();
        let mut ang_jac2: AngVector<N> = na::zero();
        let mut limit = N::zero();

        for i in 0..DIM {
            if limited_coupled_axes & (1 << i) != 0 {
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
                limit += limits[i].max * limits[i].max;
            }
        }

        limit = limit.simd_sqrt();
        let dist = lin_jac.norm();
        let inv_dist = crate::utils::simd_inv(dist);
        lin_jac *= inv_dist;
        ang_jac1 *= inv_dist;
        ang_jac2 *= inv_dist;

        let dvel = lin_jac.dot(&(body2.linvel - body1.linvel))
            + (ang_jac2.gdot(body2.angvel) - ang_jac1.gdot(body1.angvel));
        let rhs_wo_bias = dvel + (dist - limit).simd_min(zero) * N::splat(params.inv_dt());

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = (dist - limit).simd_max(zero) * erp_inv_dt;
        let rhs = rhs_wo_bias + rhs_bias;
        let impulse_bounds = [N::zero(), N::splat(Real::INFINITY)];

        JointVelocityGroundConstraint {
            joint_id,
            mj_lambda2: body2.mj_lambda,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds,
            lin_jac,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn motor_linear_ground<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        motor_axis: usize,
        motor_params: &MotorParameters<N>,
        limits: Option<[N; 2]>,
        writeback_id: WritebackId,
    ) -> JointVelocityGroundConstraint<N, LANES> {
        let inv_dt = N::splat(params.inv_dt());

        let lin_jac = self.basis.column(motor_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(motor_axis).into_owned();
        #[cfg(feature = "dim2")]
        let mut ang_jac2 = self.cmat2_basis[motor_axis];
        #[cfg(feature = "dim3")]
        let mut ang_jac2 = self.cmat2_basis.column(motor_axis).into_owned();

        let mut rhs_wo_bias = N::zero();
        if motor_params.erp_inv_dt != N::zero() {
            let dist = self.lin_err.dot(&lin_jac);
            rhs_wo_bias += (dist - motor_params.target_pos) * motor_params.erp_inv_dt;
        }

        let mut target_vel = motor_params.target_vel;
        if let Some(limits) = limits {
            let dist = self.lin_err.dot(&lin_jac);
            target_vel =
                target_vel.simd_clamp((limits[0] - dist) * inv_dt, (limits[1] - dist) * inv_dt);
        };

        let dvel = lin_jac.dot(&(body2.linvel - body1.linvel))
            + (ang_jac2.gdot(body2.angvel) - ang_jac1.gdot(body1.angvel));
        rhs_wo_bias += dvel - target_vel;

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointVelocityGroundConstraint {
            joint_id,
            mj_lambda2: body2.mj_lambda,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during ortogonalization.
            cfm_coeff: motor_params.cfm_coeff,
            cfm_gain: motor_params.cfm_gain,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    // pub fn motor_linear_coupled_ground<const LANES: usize>(
    //     &self,
    //     _joint_id: [JointIndex; LANES],
    //     _body1: &SolverBody<N, LANES>,
    //     _body2: &SolverBody<N, LANES>,
    //     _motor_coupled_axes: u8,
    //     _motors: &[MotorParameters<N>],
    //     _limited_coupled_axes: u8,
    //     _limits: &[JointLimits<N>],
    //     _writeback_id: WritebackId,
    // ) -> JointVelocityGroundConstraint<N, LANES> {
    //     let zero = N::zero();
    //     let mut lin_jac = Vector::zeros();
    //     let mut ang_jac1: AngVector<N> = na::zero();
    //     let mut ang_jac2: AngVector<N> = na::zero();
    //     let mut limit = N::zero();

    //     for i in 0..DIM {
    //         if limited_coupled_axes & (1 << i) != 0 {
    //             let coeff = self.basis.column(i).dot(&self.lin_err);
    //             lin_jac += self.basis.column(i) * coeff;
    //             #[cfg(feature = "dim2")]
    //             {
    //                 ang_jac1 += self.cmat1_basis[i] * coeff;
    //                 ang_jac2 += self.cmat2_basis[i] * coeff;
    //             }
    //             #[cfg(feature = "dim3")]
    //             {
    //                 ang_jac1 += self.cmat1_basis.column(i) * coeff;
    //                 ang_jac2 += self.cmat2_basis.column(i) * coeff;
    //             }
    //             limit += limits[i].max * limits[i].max;
    //         }
    //     }

    //     limit = limit.simd_sqrt();
    //     let dist = lin_jac.norm();
    //     let inv_dist = crate::utils::simd_inv(dist);
    //     lin_jac *= inv_dist;
    //     ang_jac1 *= inv_dist;
    //     ang_jac2 *= inv_dist;

    //     let dvel = lin_jac.dot(&(body2.linvel - body1.linvel))
    //         + (ang_jac2.gdot(body2.angvel) - ang_jac1.gdot(body1.angvel));
    //     let rhs_wo_bias = dvel + (dist - limit).simd_min(zero) * N::splat(params.inv_dt());

    //     ang_jac2 = body2.sqrt_ii * ang_jac2;

    //     let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
    //     let cfm_coeff = N::splat(params.joint_cfm_coeff());
    //     let rhs_bias = (dist - limit).simd_max(zero) * erp_inv_dt;
    //     let rhs = rhs_wo_bias + rhs_bias;
    //     let impulse_bounds = [N::zero(), N::splat(Real::INFINITY)];

    //     JointVelocityGroundConstraint {
    //         joint_id,
    //         mj_lambda2: body2.mj_lambda,
    //         im2: body2.im,
    //         impulse: N::zero(),
    //         impulse_bounds,
    //         lin_jac,
    //         ang_jac2,
    //         inv_lhs: N::zero(), // Will be set during ortogonalization.
    //         cfm_coeff,
    //         cfm_gain: N::zero(),
    //         rhs,
    //         rhs_wo_bias,
    //         writeback_id,
    //     }
    // }

    pub fn lock_linear_ground<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointVelocityGroundConstraint<N, LANES> {
        let lin_jac = self.basis.column(locked_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(locked_axis).into_owned();
        #[cfg(feature = "dim2")]
        let mut ang_jac2 = self.cmat2_basis[locked_axis];
        #[cfg(feature = "dim3")]
        let mut ang_jac2 = self.cmat2_basis.column(locked_axis).into_owned();

        let dvel = lin_jac.dot(&(body2.linvel - body1.linvel))
            + (ang_jac2.gdot(body2.angvel) - ang_jac1.gdot(body1.angvel));
        let rhs_wo_bias = dvel;

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = lin_jac.dot(&self.lin_err) * erp_inv_dt;

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointVelocityGroundConstraint {
            joint_id,
            mj_lambda2: body2.mj_lambda,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-N::splat(Real::MAX), N::splat(Real::MAX)],
            lin_jac,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn motor_angular_ground<const LANES: usize>(
        &self,
        joint_id: [JointIndex; LANES],
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        _motor_axis: usize,
        motor_params: &MotorParameters<N>,
        writeback_id: WritebackId,
    ) -> JointVelocityGroundConstraint<N, LANES> {
        // let mut ang_jac = self.ang_basis.column(_motor_axis).into_owned();
        #[cfg(feature = "dim2")]
        let ang_jac = N::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.basis.column(_motor_axis).into_owned();

        let mut rhs_wo_bias = N::zero();
        if motor_params.erp_inv_dt != N::zero() {
            #[cfg(feature = "dim2")]
            {
                let s_ang_dist = self.ang_err.angle();
                let s_target_ang = motor_params.target_pos;
                rhs_wo_bias += ((s_ang_dist - s_target_ang) % N::simd_two_pi()) / N::simd_two_pi()
                    * motor_params.erp_inv_dt;
            }
            #[cfg(feature = "dim3")]
            {
                let s_ang_dist = self.ang_err.imag()[_motor_axis];
                let s_target_ang = motor_params.target_pos.simd_sin();
                rhs_wo_bias += (s_ang_dist - s_target_ang) * motor_params.erp_inv_dt;
            }
        }

        let dvel = ang_jac.gdot(body2.angvel) - ang_jac.gdot(body1.angvel);
        rhs_wo_bias += dvel - motor_params.target_vel;

        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointVelocityGroundConstraint {
            joint_id,
            mj_lambda2: body2.mj_lambda,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac: na::zero(),
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during ortogonalization.
            cfm_coeff: motor_params.cfm_coeff,
            cfm_gain: motor_params.cfm_gain,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn limit_angular_ground<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        limited_axis: usize,
        limits: [N; 2],
        writeback_id: WritebackId,
    ) -> JointVelocityGroundConstraint<N, LANES> {
        let zero = N::zero();
        let half = N::splat(0.5);
        let s_limits = [(limits[0] * half).simd_sin(), (limits[1] * half).simd_sin()];
        #[cfg(feature = "dim2")]
        let s_ang = self.ang_err.im;
        #[cfg(feature = "dim3")]
        let s_ang = self.ang_err.imag()[limited_axis];
        let min_enabled = s_ang.simd_lt(s_limits[0]);
        let max_enabled = s_limits[1].simd_lt(s_ang);

        let impulse_bounds = [
            N::splat(-Real::INFINITY).select(min_enabled, zero),
            N::splat(Real::INFINITY).select(max_enabled, zero),
        ];

        #[cfg(feature = "dim2")]
        let ang_jac = self.ang_basis[limited_axis];
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(limited_axis).into_owned();
        let dvel = ang_jac.gdot(body2.angvel) - ang_jac.gdot(body1.angvel);
        let rhs_wo_bias = dvel;

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = ((s_ang - s_limits[1]).simd_max(zero)
            - (s_limits[0] - s_ang).simd_max(zero))
            * erp_inv_dt;

        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointVelocityGroundConstraint {
            joint_id,
            mj_lambda2: body2.mj_lambda,
            im2: body2.im,
            impulse: zero,
            impulse_bounds,
            lin_jac: na::zero(),
            ang_jac2,
            inv_lhs: zero, // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn lock_angular_ground<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &SolverBody<N, LANES>,
        body2: &SolverBody<N, LANES>,
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointVelocityGroundConstraint<N, LANES> {
        #[cfg(feature = "dim2")]
        let ang_jac = self.ang_basis[locked_axis];
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(locked_axis).into_owned();
        let dvel = ang_jac.gdot(body2.angvel) - ang_jac.gdot(body1.angvel);
        let rhs_wo_bias = dvel;

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        #[cfg(feature = "dim2")]
        let rhs_bias = self.ang_err.im * erp_inv_dt;
        #[cfg(feature = "dim3")]
        let rhs_bias = self.ang_err.imag()[locked_axis] * erp_inv_dt;

        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointVelocityGroundConstraint {
            joint_id,
            mj_lambda2: body2.mj_lambda,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-N::splat(Real::MAX), N::splat(Real::MAX)],
            lin_jac: na::zero(),
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    /// Orthogonalize the constraints and set their inv_lhs field.
    pub fn finalize_ground_constraints<const LANES: usize>(
        constraints: &mut [JointVelocityGroundConstraint<N, LANES>],
    ) {
        let len = constraints.len();

        if len == 0 {
            return;
        }

        let imsum = constraints[0].im2;

        // Use the modified Gram-Schmidt orthogonalization.
        for j in 0..len {
            let c_j = &mut constraints[j];
            let dot_jj = c_j.lin_jac.dot(&imsum.component_mul(&c_j.lin_jac))
                + c_j.ang_jac2.gdot(c_j.ang_jac2);
            let cfm_gain = dot_jj * c_j.cfm_coeff + c_j.cfm_gain;
            let inv_dot_jj = crate::utils::simd_inv(dot_jj + cfm_gain);
            c_j.inv_lhs = inv_dot_jj; // Don’t forget to update the inv_lhs.
            c_j.cfm_gain = cfm_gain;

            if c_j.impulse_bounds != [-N::splat(Real::MAX), N::splat(Real::MAX)]
                || c_j.cfm_gain != N::zero()
            {
                // Don't remove constraints with limited forces from the others
                // because they may not deliver the necessary forces to fulfill
                // the removed parts of other constraints.
                continue;
            }

            for i in j + 1..len {
                let (c_i, c_j) = constraints.index_mut_const(i, j);

                let dot_ij = c_i.lin_jac.dot(&imsum.component_mul(&c_j.lin_jac))
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

impl JointVelocityConstraintBuilder<Real> {
    // TODO: this method is almost identical to the ground version, except for the
    //       return type. Could they share their implementation somehow?
    #[cfg(feature = "dim3")]
    pub fn limit_angular_coupled(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; 1],
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
        limited_coupled_axes: u8,
        limits: &[JointLimits<Real>],
        writeback_id: WritebackId,
    ) -> JointVelocityConstraint<Real, 1> {
        // NOTE: right now, this only supports exactly 2 coupled axes.
        let ang_coupled_axes = limited_coupled_axes >> DIM;
        assert_eq!(ang_coupled_axes.count_ones(), 2);
        let not_coupled_index = ang_coupled_axes.trailing_ones() as usize;
        let axis1 = self.basis.column(not_coupled_index).into_owned();
        let axis2 = self.basis2.column(not_coupled_index).into_owned();

        let rot = Rotation::rotation_between(&axis1, &axis2).unwrap_or_else(Rotation::identity);
        let (ang_jac, angle) = rot
            .axis_angle()
            .map(|(axis, angle)| (axis.into_inner(), angle))
            .unwrap_or_else(|| (axis1.orthonormal_basis()[0], 0.0));
        let mut ang_limits = [0.0, 0.0];

        for k in 0..3 {
            if (ang_coupled_axes & (1 << k)) != 0 {
                let limit = &limits[DIM + k];
                ang_limits[0] += limit.min * limit.min;
                ang_limits[1] += limit.max * limit.max;
            }
        }

        ang_limits[0] = ang_limits[0].sqrt();
        ang_limits[1] = ang_limits[1].sqrt();

        let min_enabled = angle <= ang_limits[0];
        let max_enabled = ang_limits[1] <= angle;

        let impulse_bounds = [
            if min_enabled { -Real::INFINITY } else { 0.0 },
            if max_enabled { Real::INFINITY } else { 0.0 },
        ];

        let dvel = ang_jac.gdot(body2.angvel) - ang_jac.gdot(body1.angvel);
        let rhs_wo_bias = dvel;

        let erp_inv_dt = params.joint_erp_inv_dt();
        let cfm_coeff = params.joint_cfm_coeff();
        let rhs_bias =
            ((angle - ang_limits[1]).max(0.0) - (ang_limits[0] - angle).max(0.0)) * erp_inv_dt;

        let ang_jac1 = body1.sqrt_ii * ang_jac;
        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointVelocityConstraint {
            joint_id,
            mj_lambda1: body1.mj_lambda,
            mj_lambda2: body2.mj_lambda,
            im1: body1.im,
            im2: body2.im,
            impulse: 0.0,
            impulse_bounds,
            lin_jac: na::zero(),
            ang_jac1,
            ang_jac2,
            inv_lhs: 0.0, // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: 0.0,
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    #[cfg(feature = "dim3")]
    pub fn limit_angular_coupled_ground(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; 1],
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
        limited_coupled_axes: u8,
        limits: &[JointLimits<Real>],
        writeback_id: WritebackId,
    ) -> JointVelocityGroundConstraint<Real, 1> {
        // NOTE: right now, this only supports exactly 2 coupled axes.
        let ang_coupled_axes = limited_coupled_axes >> DIM;
        assert_eq!(ang_coupled_axes.count_ones(), 2);
        let not_coupled_index = ang_coupled_axes.trailing_ones() as usize;
        let axis1 = self.basis.column(not_coupled_index).into_owned();
        let axis2 = self.basis2.column(not_coupled_index).into_owned();

        let rot = Rotation::rotation_between(&axis1, &axis2).unwrap_or_else(Rotation::identity);
        let (ang_jac, angle) = rot
            .axis_angle()
            .map(|(axis, angle)| (axis.into_inner(), angle))
            .unwrap_or_else(|| (axis1.orthonormal_basis()[0], 0.0));
        let mut ang_limits = [0.0, 0.0];

        for k in 0..3 {
            if (ang_coupled_axes & (1 << k)) != 0 {
                let limit = &limits[DIM + k];
                ang_limits[0] += limit.min * limit.min;
                ang_limits[1] += limit.max * limit.max;
            }
        }

        ang_limits[0] = ang_limits[0].sqrt();
        ang_limits[1] = ang_limits[1].sqrt();

        let min_enabled = angle <= ang_limits[0];
        let max_enabled = ang_limits[1] <= angle;

        let impulse_bounds = [
            if min_enabled { -Real::INFINITY } else { 0.0 },
            if max_enabled { Real::INFINITY } else { 0.0 },
        ];

        let dvel = ang_jac.gdot(body2.angvel) - ang_jac.gdot(body1.angvel);
        let rhs_wo_bias = dvel;

        let erp_inv_dt = params.joint_erp_inv_dt();
        let cfm_coeff = params.joint_cfm_coeff();
        let rhs_bias =
            ((angle - ang_limits[1]).max(0.0) - (ang_limits[0] - angle).max(0.0)) * erp_inv_dt;

        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointVelocityGroundConstraint {
            joint_id,
            mj_lambda2: body2.mj_lambda,
            im2: body2.im,
            impulse: 0.0,
            impulse_bounds,
            lin_jac: na::zero(),
            ang_jac2,
            inv_lhs: 0.0, // Will be set during ortogonalization.
            cfm_coeff,
            cfm_gain: 0.0,
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }
}
