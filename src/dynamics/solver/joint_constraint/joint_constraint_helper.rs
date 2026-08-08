//! Per-row construction helpers shared by the joint constraint builders:
//! jacobian bases, lock/limit/motor row assembly, and the Gram-Schmidt
//! orthogonalization of a joint's rows.

use crate::dynamics::solver::MotorParameters;
use crate::dynamics::solver::joint_constraint::JointSolverBody;
use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::{
    JointConstraint, WritebackId,
};
use crate::dynamics::{IntegrationParameters, JointIndex};
use crate::math::{DIM, Real};
use crate::utils;
#[cfg(feature = "dim3")]
use crate::utils::OrthonormalBasis;
use crate::utils::{
    AngularInertiaOps, ComponentMul, CrossProductMatrix, DotProduct, IndexMut2, MatrixColumn,
    PoseOps, RotationOps, ScalarType, SimdLength,
};

#[cfg(not(feature = "std"))]
use simba::scalar::ComplexField as _;

use crate::num::FloatConst;
#[cfg(feature = "dim2")]
use crate::num::One;

#[cfg(feature = "dim3")]
use parry::math::Rot3;

/// The parameters of one angular limit row, expressed relative to the MIDDLE of the
/// allowed range: the row measures the wrapped, re-centered joint angle and compares it
/// against the symmetric bound `±half_range`.
#[derive(Debug, Copy, Clone)]
pub struct AngularLimitParams<N> {
    /// The center of the allowed range, as `[cos, sin]` — of *half* the center angle in 3D,
    /// where the relative rotation is a quaternion.
    pub center: [N; 2],
    /// Half the allowed range (radians): the symmetric bound the re-centered angle is
    /// tested against. Larger than π when the joint is effectively free, which no wrapped
    /// angle can trigger.
    pub half_range: N,
}

impl AngularLimitParams<Real> {
    /// The row parameters of an angular limit allowing `[min, max]` (radians).
    pub fn new(min: Real, max: Real) -> Self {
        let half_range = (max - min) * 0.5;

        // A range of a full turn or more is indistinguishable from "no limit" for an angle
        // read off a relative rotation, so the row is disabled instead. This is also where the
        // huge range of an unset limit (`JointLimits::default`) lands, and where NaN bounds
        // are caught before they can poison the row.
        if half_range >= Real::PI() || half_range.is_nan() {
            return Self {
                center: [1.0, 0.0],
                half_range: 10.0, // Value greater than π means it’s unconstrained.
            };
        }

        let center = (min + max) * 0.5;
        #[cfg(feature = "dim2")]
        let (sin, cos) = center.sin_cos();
        #[cfg(feature = "dim3")]
        let (sin, cos) = (center * 0.5).sin_cos();

        Self {
            center: [cos, sin],
            // Negative for an empty range (`min > max`), which makes both rows active and
            // pulls the joint to the center — the only sensible reading of such a range.
            half_range,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct JointConstraintHelper<N: ScalarType> {
    pub basis: N::Matrix,
    #[cfg(feature = "dim3")]
    pub basis2: N::Matrix, // TODO: used for angular coupling. Can we avoid storing this?
    #[cfg(feature = "dim3")]
    pub cmat1_basis: N::Matrix,
    #[cfg(feature = "dim3")]
    pub cmat2_basis: N::Matrix,
    #[cfg(feature = "dim3")]
    pub ang_basis: N::Matrix,
    #[cfg(feature = "dim2")]
    pub cmat1_basis: [N::AngVector; 2],
    #[cfg(feature = "dim2")]
    pub cmat2_basis: [N::AngVector; 2],
    pub lin_err: N::Vector,
    pub ang_err: N::Rotation,
}

impl<N: ScalarType> JointConstraintHelper<N> {
    pub fn new(
        frame1: &N::Pose,
        frame2: &N::Pose,
        world_com1: &N::Vector,
        world_com2: &N::Vector,
        locked_lin_axes: u8,
    ) -> Self {
        let mut frame1 = *frame1;
        let basis = frame1.rotation().to_mat();
        let lin_err = frame2.translation() - frame1.translation();

        // Adjust the point of application of the force for the first body,
        // by snapping free axes to the second frame's center (to account for
        // the allowed relative movement).
        {
            let mut new_center1 = frame2.translation(); // First, assume all dofs are free.

            // Then snap the locked ones.
            for i in 0..DIM {
                if locked_lin_axes & (1 << i) != 0 {
                    let axis = basis.column(i);
                    new_center1 -= axis * lin_err.gdot(axis);
                }
            }
            frame1.set_translation(new_center1);
        }

        let r1 = frame1.translation() - *world_com1;
        let r2 = frame2.translation() - *world_com2;

        let cmat1 = r1.gcross_matrix();
        let cmat2 = r2.gcross_matrix();

        #[cfg(feature = "dim3")]
        let mut ang_basis = frame1.rotation().diff_conj1_2_tr(&frame2.rotation());
        #[allow(unused_mut)] // The mut is needed for 3D
        let mut ang_err = frame1.rotation().inverse() * frame2.rotation();

        #[cfg(feature = "dim3")]
        {
            let sgn = N::one().simd_copysign(frame1.rotation().dot(&frame2.rotation()));
            ang_basis *= sgn;
            ang_err.mul_assign_unchecked(sgn);
        }

        #[cfg(feature = "dim2")]
        return Self {
            basis,
            cmat1_basis: [
                cmat1.gdot(basis.column(0)).into(),
                cmat1.gdot(basis.column(1)).into(),
            ],
            cmat2_basis: [
                cmat2.gdot(basis.column(0)).into(),
                cmat2.gdot(basis.column(1)).into(),
            ],
            lin_err,
            ang_err,
        };
        #[cfg(feature = "dim3")]
        return Self {
            basis,
            basis2: frame2.rotation().to_mat(),
            cmat1_basis: cmat1 * basis,
            cmat2_basis: cmat2 * basis,
            ang_basis,
            lin_err,
            ang_err,
        };
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
        erp_inv_dt: N,
        cfm_coeff: N,
    ) -> JointConstraint<N, LANES> {
        let zero = N::zero();
        let mut constraint = self.lock_linear(
            params,
            joint_id,
            body1,
            body2,
            limited_axis,
            writeback_id,
            erp_inv_dt,
            cfm_coeff,
        );

        let dist = self.lin_err.gdot(constraint.lin_jac);
        let min_enabled = dist.simd_le(limits[0]);
        let max_enabled = limits[1].simd_le(dist);

        // Like the contact solver, cap the bias so a deep limit violation recovers over a
        // few steps instead of catapulting the bodies (the erp gain is ~1/dt).
        let max_bias = N::splat(params.max_corrective_velocity());
        let rhs_bias = (((dist - limits[1]).simd_max(zero) - (limits[0] - dist).simd_max(zero))
            * erp_inv_dt)
            .simd_clamp(-max_bias, max_bias);
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
        erp_inv_dt: N,
        cfm_coeff: N,
    ) -> JointConstraint<N, LANES> {
        let zero = N::zero();
        let mut lin_jac: N::Vector = Default::default();
        let mut ang_jac1: N::AngVector = Default::default();
        let mut ang_jac2: N::AngVector = Default::default();

        for i in 0..DIM {
            if coupled_axes & (1 << i) != 0 {
                let coeff = self.basis.column(i).gdot(self.lin_err);
                lin_jac += self.basis.column(i) * coeff;
                #[cfg(feature = "dim2")]
                {
                    ang_jac1 += self.cmat1_basis[i] * coeff;
                    ang_jac2 += self.cmat2_basis[i] * coeff;
                }
                #[cfg(feature = "dim3")]
                {
                    ang_jac1 += self.cmat1_basis.column(i).into() * coeff;
                    ang_jac2 += self.cmat2_basis.column(i).into() * coeff;
                }
            }
        }

        // FIXME: handle min limit too.

        let dist = lin_jac.simd_length();
        let inv_dist = crate::utils::simd_inv(dist);
        lin_jac *= inv_dist;
        ang_jac1 *= inv_dist;
        ang_jac2 *= inv_dist;

        let rhs_wo_bias = (dist - limits[1]).simd_min(zero) * N::splat(params.inv_dt());

        let ii_ang_jac1 = body1.ii.transform_vector(ang_jac1);
        let ii_ang_jac2 = body2.ii.transform_vector(ang_jac2);

        let max_bias = N::splat(params.max_corrective_velocity());
        let rhs_bias =
            ((dist - limits[1]).simd_max(zero) * erp_inv_dt).simd_clamp(-max_bias, max_bias);
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
        let mut constraint = self.lock_linear(
            params,
            joint_id,
            body1,
            body2,
            motor_axis,
            writeback_id,
            // Set regularization factors to zero.
            // The motor impl. will overwrite them after.
            N::zero(),
            N::zero(),
        );

        let mut rhs_wo_bias = N::zero();
        if motor_params.erp_inv_dt != N::zero() {
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

        let mut lin_jac: N::Vector = Default::default();
        let mut ang_jac1: N::AngVector = Default::default();
        let mut ang_jac2: N::AngVector = Default::default();

        for i in 0..DIM {
            if coupled_axes & (1 << i) != 0 {
                let coeff = self.basis.column(i).gdot(self.lin_err);
                lin_jac += self.basis.column(i) * coeff;
                #[cfg(feature = "dim2")]
                {
                    ang_jac1 += self.cmat1_basis[i] * coeff;
                    ang_jac2 += self.cmat2_basis[i] * coeff;
                }
                #[cfg(feature = "dim3")]
                {
                    ang_jac1 += self.cmat1_basis.column(i).into() * coeff;
                    ang_jac2 += self.cmat2_basis.column(i).into() * coeff;
                }
            }
        }

        let dist = lin_jac.simd_length();
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

        let ii_ang_jac1 = body1.ii.transform_vector(ang_jac1);
        let ii_ang_jac2 = body2.ii.transform_vector(ang_jac2);

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
        _params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<N, LANES>,
        body2: &JointSolverBody<N, LANES>,
        locked_axis: usize,
        writeback_id: WritebackId,
        erp_inv_dt: N,
        cfm_coeff: N,
    ) -> JointConstraint<N, LANES> {
        let lin_jac = self.basis.column(locked_axis);
        #[cfg(feature = "dim2")]
        let ang_jac1 = self.cmat1_basis[locked_axis];
        #[cfg(feature = "dim2")]
        let ang_jac2 = self.cmat2_basis[locked_axis];
        #[cfg(feature = "dim3")]
        let ang_jac1 = self.cmat1_basis.column(locked_axis).into();
        #[cfg(feature = "dim3")]
        let ang_jac2 = self.cmat2_basis.column(locked_axis).into();

        let rhs_wo_bias = N::zero();
        let rhs_bias = lin_jac.gdot(self.lin_err) * erp_inv_dt;

        let ii_ang_jac1 = body1.ii.transform_vector(ang_jac1);
        let ii_ang_jac2 = body2.ii.transform_vector(ang_jac2);

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

    /// The relative rotation's angle around `_limited_axis`, measured from `limit`'s center
    /// and wrapped to (-π, π] — the quantity an angular limit row compares against
    /// [`AngularLimitParams::half_range`].
    ///
    /// Measuring the angle itself (rather than a sine of it) keeps the row's gradient with
    /// respect to the joint angle equal to one everywhere on the circle, so the plain joint
    /// axis is an exact jacobian for the row: a sine-space measure has a vanishing gradient
    /// at the antipode of the range's center, where a limit row would degenerate.
    pub fn recentered_angle(&self, _limited_axis: usize, limit: &AngularLimitParams<N>) -> N {
        let [c_cos, c_sin] = limit.center;

        // Rotate the angular error by minus the range's center, so the row measures the angle
        // from there instead of from the joint's rest frame (see `AngularLimitParams`).
        #[cfg(feature = "dim2")]
        {
            // `ang_err` is a unit complex here: (cos θ, sin θ), so re-centering is a plain
            // complex product, and `atan2` wraps the result to (-π, π] by itself.
            let re = c_cos * self.ang_err.real() + c_sin * self.ang_err.imag();
            let im = c_cos * self.ang_err.imag() - c_sin * self.ang_err.real();
            im.simd_atan2(re)
        }
        #[cfg(feature = "dim3")]
        {
            // Only the limited axis' imaginary part and the real part of
            // `conj(center) * ang_err` are needed; the two other imaginary components only
            // mix with each other.
            let x = self.ang_err.imag()[_limited_axis];
            let w = self.ang_err.real();
            let sin_half = c_cos * x - c_sin * w;
            let cos_half = c_cos * w + c_sin * x;
            // The re-centered HALF angle, in (-π, π]. Doubling it must wrap back to
            // (-π, π] as a full angle, which is a ±π shift of the half angle whenever it
            // leaves (-π/2, π/2].
            let half = sin_half.simd_atan2(cos_half);
            let half_pi = N::splat(Real::FRAC_PI_2());
            let shift = N::splat(Real::PI()).simd_copysign(half);
            let wrapped_half = (half - shift).select(half.simd_abs().simd_gt(half_pi), half);
            wrapped_half * N::splat(2.0)
        }
    }

    /// The limit is measured as the wrapped joint angle relative to the middle of the
    /// allowed range (see [`AngularLimitParams`]).
    pub fn limit_angular<const LANES: usize>(
        &self,
        _params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<N, LANES>,
        body2: &JointSolverBody<N, LANES>,
        _limited_axis: usize,
        limit: AngularLimitParams<N>,
        writeback_id: WritebackId,
        erp_inv_dt: N,
        cfm_coeff: N,
    ) -> JointConstraint<N, LANES> {
        let zero = N::zero();
        let ang = self.recentered_angle(_limited_axis, &limit);
        let ang_limits = [-limit.half_range, limit.half_range];
        let min_enabled = ang.simd_le(ang_limits[0]);
        let max_enabled = ang_limits[1].simd_le(ang);

        let impulse_bounds = [
            N::splat(-Real::INFINITY).select(min_enabled, zero),
            N::splat(Real::INFINITY).select(max_enabled, zero),
        ];

        // The angle measure's gradient is the plain joint axis (like the angular motor row).
        #[cfg(feature = "dim2")]
        let ang_jac = N::AngVector::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.basis.column(_limited_axis).into();
        let rhs_wo_bias = N::zero();
        // Like the contact solver, cap the bias so a deep limit violation recovers over a
        // few steps instead of catapulting the bodies (the erp gain is ~1/dt and the wrapped
        // angular error can approach π).
        let max_bias = N::splat(_params.max_corrective_velocity());
        let rhs_bias = (((ang - ang_limits[1]).simd_max(zero)
            - (ang_limits[0] - ang).simd_max(zero))
            * erp_inv_dt)
            .simd_clamp(-max_bias, max_bias);

        let ii_ang_jac1 = body1.ii.transform_vector(ang_jac);
        let ii_ang_jac2 = body2.ii.transform_vector(ang_jac);

        JointConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds,
            lin_jac: Default::default(),
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
        let ang_jac = N::AngVector::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.basis.column(_motor_axis).into();

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

        let ii_ang_jac1 = body1.ii.transform_vector(ang_jac);
        let ii_ang_jac2 = body2.ii.transform_vector(ang_jac);

        JointConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac: Default::default(),
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
        _params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointSolverBody<N, LANES>,
        body2: &JointSolverBody<N, LANES>,
        _locked_axis: usize,
        writeback_id: WritebackId,
        erp_inv_dt: N,
        cfm_coeff: N,
    ) -> JointConstraint<N, LANES> {
        #[cfg(feature = "dim2")]
        let ang_jac = N::AngVector::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(_locked_axis).into();

        let rhs_wo_bias = N::zero();
        #[cfg(feature = "dim2")]
        let rhs_bias = self.ang_err.imag() * erp_inv_dt;
        #[cfg(feature = "dim3")]
        let rhs_bias = self.ang_err.imag()[_locked_axis] * erp_inv_dt;

        let ii_ang_jac1 = body1.ii.transform_vector(ang_jac);
        let ii_ang_jac2 = body2.ii.transform_vector(ang_jac);

        JointConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-N::splat(Real::MAX), N::splat(Real::MAX)],
            lin_jac: Default::default(),
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
            let dot_jj = c_j.lin_jac.gdot(imsum.component_mul(&c_j.lin_jac))
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

                let dot_ij = c_i.lin_jac.gdot(imsum.component_mul(&c_j.lin_jac))
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
        _params: &IntegrationParameters,
        joint_id: [JointIndex; 1],
        body1: &JointSolverBody<Real, 1>,
        body2: &JointSolverBody<Real, 1>,
        coupled_axes: u8,
        limits: [Real; 2],
        writeback_id: WritebackId,
        erp_inv_dt: Real,
        cfm_coeff: Real,
    ) -> JointConstraint<Real, 1> {
        // NOTE: right now, this only supports exactly 2 coupled axes.
        let ang_coupled_axes = coupled_axes >> DIM;
        assert_eq!(ang_coupled_axes.count_ones(), 2);
        let not_coupled_index = ang_coupled_axes.trailing_ones() as usize;
        let axis1 = self.basis.column(not_coupled_index);
        let axis2 = self.basis2.column(not_coupled_index);

        let rot = Rot3::from_rotation_arc(axis1, axis2);
        let (mut ang_jac, angle) = rot.to_axis_angle();

        if angle == 0.0 {
            ang_jac = axis1.orthonormal_basis()[0];
        }

        let min_enabled = angle <= limits[0];
        let max_enabled = limits[1] <= angle;

        let impulse_bounds = [
            if min_enabled { -Real::INFINITY } else { 0.0 },
            if max_enabled { Real::INFINITY } else { 0.0 },
        ];

        let rhs_wo_bias = 0.0;

        // See `limit_angular`: the bias is capped so deep violations don't catapult.
        let max_bias = _params.max_corrective_velocity();
        let rhs_bias = (((angle - limits[1]).max(0.0) - (limits[0] - angle).max(0.0)) * erp_inv_dt)
            .clamp(-max_bias, max_bias);

        let ii_ang_jac1 = body1.ii.transform_vector(ang_jac);
        let ii_ang_jac2 = body2.ii.transform_vector(ang_jac);

        JointConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: 0.0,
            impulse_bounds,
            lin_jac: Default::default(),
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

#[cfg(all(test, feature = "dim3"))]
mod test {
    use super::*;
    use crate::math::{Pose, Real, Rotation, Vector};

    /// The limit row measures the wrapped re-centered joint angle with the plain joint
    /// axis as its jacobian; for that pair to be consistent, the measure's slope with
    /// respect to the joint angle must be exactly one everywhere on the circle (away from
    /// the wrap discontinuity at the antipode of the range's center). A sine-space
    /// measure fails this: its gradient vanishes at the antipode, degenerating the row
    /// (issue #499 follow-up).
    #[test]
    fn recentered_angle_slope_is_one_everywhere() {
        let helper_at = |theta: Real| {
            JointConstraintHelper::<Real>::new(
                &Pose::IDENTITY,
                &Pose::from_rotation(Rotation::from_axis_angle(Vector::X, theta)),
                &Vector::ZERO,
                &Vector::ZERO,
                0,
            )
        };
        for center_deg in [-180, -135, -90, 0, 45, 135, 180] {
            let limit = AngularLimitParams::<Real>::new(
                (center_deg as Real - 45.0).to_radians(),
                (center_deg as Real + 45.0).to_radians(),
            );
            // At the center of the range the measure is zero; at the limits, ±half_range.
            let at_center =
                helper_at((center_deg as Real).to_radians()).recentered_angle(0, &limit);
            assert!(at_center.abs() < 1.0e-5, "center {center_deg}: {at_center}");
            let at_max =
                helper_at((center_deg as Real + 45.0).to_radians()).recentered_angle(0, &limit);
            assert!(
                (at_max - (45.0 as Real).to_radians()).abs() < 1.0e-4,
                "center {center_deg}: at_max {at_max}"
            );

            for theta_deg in (-350..=350).step_by(7) {
                let theta = (theta_deg as Real).to_radians();
                let eps = 1.0e-3;
                let a0 = helper_at(theta - eps).recentered_angle(0, &limit);
                let a1 = helper_at(theta + eps).recentered_angle(0, &limit);
                // Skip the wrap discontinuity itself.
                if (a1 - a0).abs() > 1.0 {
                    continue;
                }
                let slope = (a1 - a0) / (2.0 * eps);
                assert!(
                    (slope - 1.0).abs() < 1.0e-2,
                    "center {center_deg} theta {theta_deg}: slope {slope}"
                );
            }
        }
    }
}
