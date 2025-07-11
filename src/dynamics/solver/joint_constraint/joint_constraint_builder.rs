use crate::dynamics::solver::ConstraintsCounts;
use crate::dynamics::solver::MotorParameters;
use crate::dynamics::solver::joint_constraint::JointSolverBody;
use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::{
    JointFixedSolverBody, JointOneBodyConstraint, JointTwoBodyConstraint, WritebackId,
};
use crate::dynamics::solver::solver_body::SolverBody;
use crate::dynamics::{GenericJoint, ImpulseJoint, IntegrationParameters, JointIndex};
use crate::math::{ANG_DIM, AngVector, DIM, Isometry, Matrix, Point, Real, Rotation, Vector};
use crate::prelude::RigidBodySet;
use crate::utils;
use crate::utils::{IndexMut2, SimdCrossMatrix, SimdDot, SimdRealCopy};
use na::SMatrix;

#[cfg(feature = "dim3")]
use crate::utils::{SimdBasis, SimdQuat};

#[cfg(feature = "simd-is-enabled")]
use crate::math::{SIMD_WIDTH, SimdReal};

pub struct JointTwoBodyConstraintBuilder {
    body1: usize,
    body2: usize,
    joint_id: JointIndex,
    joint: GenericJoint,
    constraint_id: usize,
}

impl JointTwoBodyConstraintBuilder {
    pub fn generate(
        joint: &ImpulseJoint,
        bodies: &RigidBodySet,
        joint_id: JointIndex,
        out_builder: &mut Self,
        out_constraint_id: &mut usize,
    ) {
        let rb1 = &bodies[joint.body1];
        let rb2 = &bodies[joint.body2];
        *out_builder = Self {
            body1: rb1.ids.active_set_offset,
            body2: rb2.ids.active_set_offset,
            joint_id,
            joint: joint.data,
            constraint_id: *out_constraint_id,
        };

        let count = ConstraintsCounts::from_joint(joint);
        *out_constraint_id += count.num_constraints;
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        bodies: &[SolverBody],
        out: &mut [JointTwoBodyConstraint<Real, 1>],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?

        let rb1 = &bodies[self.body1];
        let rb2 = &bodies[self.body2];
        let frame1 = rb1.position * self.joint.local_frame1;
        let frame2 = rb2.position * self.joint.local_frame2;

        let joint_body1 = JointSolverBody {
            im: rb1.im,
            sqrt_ii: rb1.sqrt_ii,
            world_com: rb1.world_com,
            solver_vel: [self.body1],
        };
        let joint_body2 = JointSolverBody {
            im: rb2.im,
            sqrt_ii: rb2.sqrt_ii,
            world_com: rb2.world_com,
            solver_vel: [self.body2],
        };

        JointTwoBodyConstraint::<Real, 1>::lock_axes(
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
pub struct JointTwoBodyConstraintBuilderSimd {
    body1: [usize; SIMD_WIDTH],
    body2: [usize; SIMD_WIDTH],
    joint_body1: JointSolverBody<SimdReal, SIMD_WIDTH>,
    joint_body2: JointSolverBody<SimdReal, SIMD_WIDTH>,
    joint_id: [JointIndex; SIMD_WIDTH],
    local_frame1: Isometry<SimdReal>,
    local_frame2: Isometry<SimdReal>,
    locked_axes: u8,
    constraint_id: usize,
}

#[cfg(feature = "simd-is-enabled")]
impl JointTwoBodyConstraintBuilderSimd {
    pub fn generate(
        joint: [&ImpulseJoint; SIMD_WIDTH],
        bodies: &RigidBodySet,
        joint_id: [JointIndex; SIMD_WIDTH],
        out_builder: &mut Self,
        out_constraint_id: &mut usize,
    ) {
        let rb1 = gather![|ii| &bodies[joint[ii].body1]];
        let rb2 = gather![|ii| &bodies[joint[ii].body2]];

        let body1 = gather![|ii| rb1[ii].ids.active_set_offset];
        let body2 = gather![|ii| rb2[ii].ids.active_set_offset];

        let joint_body1 = JointSolverBody {
            im: gather![|ii| rb1[ii].mprops.effective_inv_mass].into(),
            sqrt_ii: gather![|ii| rb1[ii].mprops.effective_world_inv_inertia_sqrt].into(),
            world_com: Point::origin(),
            solver_vel: body1,
        };
        let joint_body2 = JointSolverBody {
            im: gather![|ii| rb2[ii].mprops.effective_inv_mass].into(),
            sqrt_ii: gather![|ii| rb2[ii].mprops.effective_world_inv_inertia_sqrt].into(),
            world_com: Point::origin(),
            solver_vel: body2,
        };

        *out_builder = Self {
            body1,
            body2,
            joint_body1,
            joint_body2,
            joint_id,
            local_frame1: gather![|ii| joint[ii].data.local_frame1].into(),
            local_frame2: gather![|ii| joint[ii].data.local_frame2].into(),
            locked_axes: joint[0].data.locked_axes.bits(),
            constraint_id: *out_constraint_id,
        };

        let count = ConstraintsCounts::from_joint(joint[0]);
        *out_constraint_id += count.num_constraints;
    }

    pub fn update(
        &mut self,
        params: &IntegrationParameters,
        bodies: &[SolverBody],
        out: &mut [JointTwoBodyConstraint<SimdReal, SIMD_WIDTH>],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?

        let rb1 = gather![|ii| &bodies[self.body1[ii]]];
        let rb2 = gather![|ii| &bodies[self.body2[ii]]];
        let frame1 = Isometry::from(gather![|ii| rb1[ii].position]) * self.local_frame1;
        let frame2 = Isometry::from(gather![|ii| rb2[ii].position]) * self.local_frame2;
        self.joint_body1.world_com = gather![|ii| rb1[ii].world_com].into();
        self.joint_body2.world_com = gather![|ii| rb2[ii].world_com].into();

        JointTwoBodyConstraint::<SimdReal, SIMD_WIDTH>::lock_axes(
            params,
            self.joint_id,
            &self.joint_body1,
            &self.joint_body2,
            &frame1,
            &frame2,
            self.locked_axes,
            &mut out[self.constraint_id..],
        );
    }
}

pub struct JointOneBodyConstraintBuilder {
    body1: JointFixedSolverBody<Real>,
    frame1: Isometry<Real>,
    body2: usize,
    joint_id: JointIndex,
    joint: GenericJoint,
    constraint_id: usize,
}

impl JointOneBodyConstraintBuilder {
    pub fn generate(
        joint: &ImpulseJoint,
        bodies: &RigidBodySet,
        joint_id: JointIndex,
        out_builder: &mut Self,
        out_constraint_id: &mut usize,
    ) {
        let mut joint_data = joint.data;
        let mut handle1 = joint.body1;
        let mut handle2 = joint.body2;
        let flipped = !bodies[handle2].is_dynamic();

        if flipped {
            std::mem::swap(&mut handle1, &mut handle2);
            joint_data.flip();
        };

        let rb1 = &bodies[handle1];
        let rb2 = &bodies[handle2];

        let frame1 = rb1.pos.position * joint_data.local_frame1;
        let joint_body1 = JointFixedSolverBody {
            linvel: rb1.vels.linvel,
            angvel: rb1.vels.angvel,
            world_com: rb1.mprops.world_com,
        };

        *out_builder = Self {
            body1: joint_body1,
            frame1,
            body2: rb2.ids.active_set_offset,
            joint_id,
            joint: joint_data,
            constraint_id: *out_constraint_id,
        };

        let count = ConstraintsCounts::from_joint(joint);
        *out_constraint_id += count.num_constraints;
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        bodies: &[SolverBody],
        out: &mut [JointOneBodyConstraint<Real, 1>],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?

        let rb2 = &bodies[self.body2];
        let frame2 = rb2.position * self.joint.local_frame2;

        let joint_body2 = JointSolverBody {
            im: rb2.im,
            sqrt_ii: rb2.sqrt_ii,
            world_com: rb2.world_com,
            solver_vel: [self.body2],
        };

        JointOneBodyConstraint::<Real, 1>::lock_axes(
            params,
            self.joint_id,
            &self.body1,
            &joint_body2,
            &self.frame1,
            &frame2,
            &self.joint,
            &mut out[self.constraint_id..],
        );
    }
}

#[cfg(feature = "simd-is-enabled")]
pub struct JointOneBodyConstraintBuilderSimd {
    body1: JointFixedSolverBody<SimdReal>,
    frame1: Isometry<SimdReal>,
    body2: [usize; SIMD_WIDTH],
    joint_id: [JointIndex; SIMD_WIDTH],
    local_frame2: Isometry<SimdReal>,
    locked_axes: u8,
    constraint_id: usize,
}

#[cfg(feature = "simd-is-enabled")]
impl JointOneBodyConstraintBuilderSimd {
    pub fn generate(
        joint: [&ImpulseJoint; SIMD_WIDTH],
        bodies: &RigidBodySet,
        joint_id: [JointIndex; SIMD_WIDTH],
        out_builder: &mut Self,
        out_constraint_id: &mut usize,
    ) {
        let mut rb1 = gather![|ii| &bodies[joint[ii].body1]];
        let mut rb2 = gather![|ii| &bodies[joint[ii].body2]];
        let mut local_frame1 = gather![|ii| joint[ii].data.local_frame1];
        let mut local_frame2 = gather![|ii| joint[ii].data.local_frame2];

        for ii in 0..SIMD_WIDTH {
            if !rb2[ii].is_dynamic() {
                std::mem::swap(&mut rb1[ii], &mut rb2[ii]);
                std::mem::swap(&mut local_frame1[ii], &mut local_frame2[ii]);
            }
        }

        let poss1 = Isometry::from(gather![|ii| rb1[ii].pos.position]);

        let joint_body1 = JointFixedSolverBody {
            linvel: gather![|ii| rb1[ii].vels.linvel].into(),
            angvel: gather![|ii| rb1[ii].vels.angvel].into(),
            world_com: gather![|ii| rb1[ii].mprops.world_com].into(),
        };

        *out_builder = Self {
            body1: joint_body1,
            body2: gather![|ii| rb2[ii].ids.active_set_offset],
            joint_id,
            frame1: poss1 * Isometry::from(local_frame1),
            local_frame2: local_frame2.into(),
            locked_axes: joint[0].data.locked_axes.bits(),
            constraint_id: *out_constraint_id,
        };

        let count = ConstraintsCounts::from_joint(joint[0]);
        *out_constraint_id += count.num_constraints;
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        bodies: &[SolverBody],
        out: &mut [JointOneBodyConstraint<SimdReal, SIMD_WIDTH>],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?

        let rb2 = gather![|ii| &bodies[self.body2[ii]]];
        let frame2 = Isometry::from(gather![|ii| rb2[ii].position]) * self.local_frame2;

        let joint_body2 = JointSolverBody {
            im: gather![|ii| rb2[ii].im].into(),
            sqrt_ii: gather![|ii| rb2[ii].sqrt_ii].into(),
            world_com: gather![|ii| rb2[ii].world_com].into(),
            solver_vel: self.body2,
        };

        JointOneBodyConstraint::<SimdReal, SIMD_WIDTH>::lock_axes(
            params,
            self.joint_id,
            &self.body1,
            &joint_body2,
            &self.frame1,
            &frame2,
            self.locked_axes,
            &mut out[self.constraint_id..],
        );
    }
}

#[derive(Debug, Copy, Clone)]
pub struct JointTwoBodyConstraintHelper<N: SimdRealCopy> {
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

impl<N: SimdRealCopy> JointTwoBodyConstraintHelper<N> {
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
    ) -> JointTwoBodyConstraint<N, LANES> {
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
    ) -> JointTwoBodyConstraint<N, LANES> {
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

        ang_jac1 = body1.sqrt_ii * ang_jac1;
        ang_jac2 = body2.sqrt_ii * ang_jac2;

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = (dist - limits[1]).simd_max(zero) * erp_inv_dt;
        let rhs = rhs_wo_bias + rhs_bias;
        let impulse_bounds = [N::zero(), N::splat(Real::INFINITY)];

        JointTwoBodyConstraint {
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
    ) -> JointTwoBodyConstraint<N, LANES> {
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
    ) -> JointTwoBodyConstraint<N, LANES> {
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

        ang_jac1 = body1.sqrt_ii * ang_jac1;
        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointTwoBodyConstraint {
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
    ) -> JointTwoBodyConstraint<N, LANES> {
        let lin_jac = self.basis.column(locked_axis).into_owned();
        #[cfg(feature = "dim2")]
        let mut ang_jac1 = self.cmat1_basis[locked_axis];
        #[cfg(feature = "dim2")]
        let mut ang_jac2 = self.cmat2_basis[locked_axis];
        #[cfg(feature = "dim3")]
        let mut ang_jac1 = self.cmat1_basis.column(locked_axis).into_owned();
        #[cfg(feature = "dim3")]
        let mut ang_jac2 = self.cmat2_basis.column(locked_axis).into_owned();

        let rhs_wo_bias = N::zero();
        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = lin_jac.dot(&self.lin_err) * erp_inv_dt;

        ang_jac1 = body1.sqrt_ii * ang_jac1;
        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointTwoBodyConstraint {
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
    ) -> JointTwoBodyConstraint<N, LANES> {
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

        let ang_jac1 = body1.sqrt_ii * ang_jac;
        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointTwoBodyConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds,
            lin_jac: na::zero(),
            ang_jac1,
            ang_jac2,
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
    ) -> JointTwoBodyConstraint<N, LANES> {
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

        let ang_jac1 = body1.sqrt_ii * ang_jac;
        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointTwoBodyConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac: na::zero(),
            ang_jac1,
            ang_jac2,
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
    ) -> JointTwoBodyConstraint<N, LANES> {
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

        let ang_jac1 = body1.sqrt_ii * ang_jac;
        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointTwoBodyConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-N::splat(Real::MAX), N::splat(Real::MAX)],
            lin_jac: na::zero(),
            ang_jac1,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    /// Orthogonalize the constraints and set their inv_lhs field.
    pub fn finalize_constraints<const LANES: usize>(
        constraints: &mut [JointTwoBodyConstraint<N, LANES>],
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

    pub fn limit_linear_one_body<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<N>,
        body2: &JointSolverBody<N, LANES>,
        limited_axis: usize,
        limits: [N; 2],
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<N, LANES> {
        let zero = N::zero();
        let lin_jac = self.basis.column(limited_axis).into_owned();
        let dist = self.lin_err.dot(&lin_jac);

        let min_enabled = dist.simd_le(limits[0]);
        let max_enabled = limits[1].simd_le(dist);

        let impulse_bounds = [
            N::splat(-Real::INFINITY).select(min_enabled, zero),
            N::splat(Real::INFINITY).select(max_enabled, zero),
        ];

        let ang_jac1 = self.cmat1_basis.column(limited_axis).into_owned();
        #[cfg(feature = "dim2")]
        let mut ang_jac2 = self.cmat2_basis[limited_axis];
        #[cfg(feature = "dim3")]
        let mut ang_jac2 = self.cmat2_basis.column(limited_axis).into_owned();

        let rhs_wo_bias = -lin_jac.dot(&body1.linvel) - ang_jac1.gdot(body1.angvel);
        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
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
            inv_lhs: zero, // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn limit_linear_coupled_one_body<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<N>,
        body2: &JointSolverBody<N, LANES>,
        coupled_axes: u8,
        limits: [N; 2],
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<N, LANES> {
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

        let dist = lin_jac.norm();
        let inv_dist = crate::utils::simd_inv(dist);
        lin_jac *= inv_dist;
        ang_jac1 *= inv_dist;
        ang_jac2 *= inv_dist;

        // FIXME: handle min limit too.
        let proj_vel1 = -lin_jac.dot(&body1.linvel) - ang_jac1.gdot(body1.angvel);
        let rhs_wo_bias = proj_vel1 + (dist - limits[1]).simd_min(zero) * N::splat(params.inv_dt());

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = (dist - limits[1]).simd_max(zero) * erp_inv_dt;
        let rhs = rhs_wo_bias + rhs_bias;
        let impulse_bounds = [N::zero(), N::splat(Real::INFINITY)];

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds,
            lin_jac,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn motor_linear_one_body<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<N>,
        body2: &JointSolverBody<N, LANES>,
        motor_axis: usize,
        motor_params: &MotorParameters<N>,
        limits: Option<[N; 2]>,
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<N, LANES> {
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

        let proj_vel1 = -lin_jac.dot(&body1.linvel) - ang_jac1.gdot(body1.angvel);
        rhs_wo_bias += proj_vel1 - target_vel;

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff: motor_params.cfm_coeff,
            cfm_gain: motor_params.cfm_gain,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn motor_linear_coupled_one_body<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<N>,
        body2: &JointSolverBody<N, LANES>,
        coupled_axes: u8,
        motor_params: &MotorParameters<N>,
        limits: Option<[N; 2]>,
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<N, LANES> {
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

        let proj_vel1 = -lin_jac.dot(&body1.linvel) - ang_jac1.gdot(body1.angvel);
        rhs_wo_bias += proj_vel1 - target_vel;

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff: motor_params.cfm_coeff,
            cfm_gain: motor_params.cfm_gain,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn lock_linear_one_body<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<N>,
        body2: &JointSolverBody<N, LANES>,
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<N, LANES> {
        let lin_jac = self.basis.column(locked_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(locked_axis).into_owned();
        #[cfg(feature = "dim2")]
        let mut ang_jac2 = self.cmat2_basis[locked_axis];
        #[cfg(feature = "dim3")]
        let mut ang_jac2 = self.cmat2_basis.column(locked_axis).into_owned();

        let rhs_wo_bias = -lin_jac.dot(&body1.linvel) - ang_jac1.gdot(body1.angvel);

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        let rhs_bias = lin_jac.dot(&self.lin_err) * erp_inv_dt;

        ang_jac2 = body2.sqrt_ii * ang_jac2;

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-N::splat(Real::MAX), N::splat(Real::MAX)],
            lin_jac,
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn motor_angular_one_body<const LANES: usize>(
        &self,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<N>,
        body2: &JointSolverBody<N, LANES>,
        _motor_axis: usize,
        motor_params: &MotorParameters<N>,
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<N, LANES> {
        #[cfg(feature = "dim2")]
        let ang_jac = N::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.basis.column(_motor_axis).into_owned();

        let mut rhs_wo_bias = N::zero();
        if motor_params.erp_inv_dt != N::zero() {
            #[cfg(feature = "dim2")]
            let ang_dist = self.ang_err.angle();
            #[cfg(feature = "dim3")]
            let ang_dist = self.ang_err.imag()[_motor_axis].simd_asin() * N::splat(2.0);
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
            impulse: N::zero(),
            impulse_bounds: [-motor_params.max_impulse, motor_params.max_impulse],
            lin_jac: na::zero(),
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff: motor_params.cfm_coeff,
            cfm_gain: motor_params.cfm_gain,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn limit_angular_one_body<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<N>,
        body2: &JointSolverBody<N, LANES>,
        _limited_axis: usize,
        limits: [N; 2],
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<N, LANES> {
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
        let rhs_wo_bias = -ang_jac.gdot(body1.angvel);

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
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
            lin_jac: na::zero(),
            ang_jac2,
            inv_lhs: zero, // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    pub fn lock_angular_one_body<const LANES: usize>(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; LANES],
        body1: &JointFixedSolverBody<N>,
        body2: &JointSolverBody<N, LANES>,
        _locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<N, LANES> {
        #[cfg(feature = "dim2")]
        let ang_jac = N::one();
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(_locked_axis).into_owned();

        let rhs_wo_bias = -ang_jac.gdot(body1.angvel);

        let erp_inv_dt = N::splat(params.joint_erp_inv_dt());
        let cfm_coeff = N::splat(params.joint_cfm_coeff());
        #[cfg(feature = "dim2")]
        let rhs_bias = self.ang_err.im * erp_inv_dt;
        #[cfg(feature = "dim3")]
        let rhs_bias = self.ang_err.imag()[_locked_axis] * erp_inv_dt;

        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: N::zero(),
            impulse_bounds: [-N::splat(Real::MAX), N::splat(Real::MAX)],
            lin_jac: na::zero(),
            ang_jac2,
            inv_lhs: N::zero(), // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: N::zero(),
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    /// Orthogonalize the constraints and set their inv_lhs field.
    pub fn finalize_one_body_constraints<const LANES: usize>(
        constraints: &mut [JointOneBodyConstraint<N, LANES>],
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
            let inv_dot_jj = crate::utils::simd_inv(dot_jj);
            c_j.inv_lhs = crate::utils::simd_inv(dot_jj + cfm_gain); // Don’t forget to update the inv_lhs.
            c_j.cfm_gain = cfm_gain;

            if c_j.impulse_bounds != [-N::splat(Real::MAX), N::splat(Real::MAX)] {
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

impl JointTwoBodyConstraintHelper<Real> {
    // TODO: this method is almost identical to the one_body version, except for the
    //       return type. Could they share their implementation somehow?
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
    ) -> JointTwoBodyConstraint<Real, 1> {
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

        let ang_jac1 = body1.sqrt_ii * ang_jac;
        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointTwoBodyConstraint {
            joint_id,
            solver_vel1: body1.solver_vel,
            solver_vel2: body2.solver_vel,
            im1: body1.im,
            im2: body2.im,
            impulse: 0.0,
            impulse_bounds,
            lin_jac: na::zero(),
            ang_jac1,
            ang_jac2,
            inv_lhs: 0.0, // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: 0.0,
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }

    #[cfg(feature = "dim3")]
    pub fn limit_angular_coupled_one_body(
        &self,
        params: &IntegrationParameters,
        joint_id: [JointIndex; 1],
        body1: &JointFixedSolverBody<Real>,
        body2: &JointSolverBody<Real, 1>,
        coupled_axes: u8,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointOneBodyConstraint<Real, 1> {
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

        let rhs_wo_bias = -ang_jac.gdot(body1.angvel);

        let erp_inv_dt = params.joint_erp_inv_dt();
        let cfm_coeff = params.joint_cfm_coeff();
        let rhs_bias = ((angle - limits[1]).max(0.0) - (limits[0] - angle).max(0.0)) * erp_inv_dt;

        let ang_jac2 = body2.sqrt_ii * ang_jac;

        JointOneBodyConstraint {
            joint_id,
            solver_vel2: body2.solver_vel,
            im2: body2.im,
            impulse: 0.0,
            impulse_bounds,
            lin_jac: na::zero(),
            ang_jac2,
            inv_lhs: 0.0, // Will be set during orthogonalization.
            cfm_coeff,
            cfm_gain: 0.0,
            rhs: rhs_wo_bias + rhs_bias,
            rhs_wo_bias,
            writeback_id,
        }
    }
}
