use crate::dynamics::solver::SolverVel;
use crate::dynamics::solver::joint_constraint::JointTwoBodyConstraintHelper;
use crate::dynamics::{
    GenericJoint, IntegrationParameters, JointAxesMask, JointGraphEdge, JointIndex,
};
use crate::math::{AngVector, AngularInertia, DIM, Isometry, Point, Real, SPATIAL_DIM, Vector};
use crate::num::Zero;
use crate::utils::{SimdDot, SimdRealCopy};

#[cfg(feature = "simd-is-enabled")]
use {
    crate::math::{SIMD_WIDTH, SimdReal},
    na::SimdValue,
};

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct MotorParameters<N: SimdRealCopy> {
    pub erp_inv_dt: N,
    pub cfm_coeff: N,
    pub cfm_gain: N,
    pub target_pos: N,
    pub target_vel: N,
    pub max_impulse: N,
}

impl<N: SimdRealCopy> Default for MotorParameters<N> {
    fn default() -> Self {
        Self {
            erp_inv_dt: N::zero(),
            cfm_coeff: N::zero(),
            cfm_gain: N::zero(),
            target_pos: N::zero(),
            target_vel: N::zero(),
            max_impulse: N::zero(),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum WritebackId {
    Dof(usize),
    Limit(usize),
    Motor(usize),
}

// TODO: right now we only use this for impulse_joints.
// However, it may actually be a good idea to use this everywhere in
// the solver, to avoid fetching data from the rigid-body set
// every time.
#[derive(Copy, Clone)]
pub struct JointSolverBody<N: SimdRealCopy, const LANES: usize> {
    pub im: Vector<N>,
    pub sqrt_ii: AngularInertia<N>,
    pub world_com: Point<N>,
    pub solver_vel: [usize; LANES],
}

impl<N: SimdRealCopy, const LANES: usize> JointSolverBody<N, LANES> {
    pub fn invalid() -> Self {
        Self {
            im: Vector::zeros(),
            sqrt_ii: AngularInertia::zero(),
            world_com: Point::origin(),
            solver_vel: [usize::MAX; LANES],
        }
    }
}

#[derive(Copy, Clone)]
pub struct JointFixedSolverBody<N: SimdRealCopy> {
    pub linvel: Vector<N>,
    pub angvel: AngVector<N>,
    // TODO: is this really needed?
    pub world_com: Point<N>,
}

impl<N: SimdRealCopy> JointFixedSolverBody<N> {
    pub fn invalid() -> Self {
        Self {
            linvel: Vector::zeros(),
            angvel: AngVector::zero(),
            world_com: Point::origin(),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct JointTwoBodyConstraint<N: SimdRealCopy, const LANES: usize> {
    pub solver_vel1: [usize; LANES],
    pub solver_vel2: [usize; LANES],

    pub joint_id: [JointIndex; LANES],

    pub impulse: N,
    pub impulse_bounds: [N; 2],
    pub lin_jac: Vector<N>,
    pub ang_jac1: AngVector<N>,
    pub ang_jac2: AngVector<N>,

    pub inv_lhs: N,
    pub rhs: N,
    pub rhs_wo_bias: N,
    pub cfm_gain: N,
    pub cfm_coeff: N,

    pub im1: Vector<N>,
    pub im2: Vector<N>,

    pub writeback_id: WritebackId,
}

impl<N: SimdRealCopy, const LANES: usize> JointTwoBodyConstraint<N, LANES> {
    #[profiling::function]
    pub fn solve_generic(
        &mut self,
        solver_vel1: &mut SolverVel<N>,
        solver_vel2: &mut SolverVel<N>,
    ) {
        let dlinvel = self.lin_jac.dot(&(solver_vel2.linear - solver_vel1.linear));
        let dangvel =
            self.ang_jac2.gdot(solver_vel2.angular) - self.ang_jac1.gdot(solver_vel1.angular);

        let rhs = dlinvel + dangvel + self.rhs;
        let total_impulse = (self.impulse + self.inv_lhs * (rhs - self.cfm_gain * self.impulse))
            .simd_clamp(self.impulse_bounds[0], self.impulse_bounds[1]);
        let delta_impulse = total_impulse - self.impulse;
        self.impulse = total_impulse;

        let lin_impulse = self.lin_jac * delta_impulse;
        let ang_impulse1 = self.ang_jac1 * delta_impulse;
        let ang_impulse2 = self.ang_jac2 * delta_impulse;

        solver_vel1.linear += lin_impulse.component_mul(&self.im1);
        solver_vel1.angular += ang_impulse1;
        solver_vel2.linear -= lin_impulse.component_mul(&self.im2);
        solver_vel2.angular -= ang_impulse2;
    }

    pub fn remove_bias_from_rhs(&mut self) {
        self.rhs = self.rhs_wo_bias;
    }
}

impl JointTwoBodyConstraint<Real, 1> {
    pub fn lock_axes(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        body1: &JointSolverBody<Real, 1>,
        body2: &JointSolverBody<Real, 1>,
        frame1: &Isometry<Real>,
        frame2: &Isometry<Real>,
        joint: &GenericJoint,
        out: &mut [Self],
    ) -> usize {
        let mut len = 0;
        let locked_axes = joint.locked_axes.bits();
        let motor_axes = joint.motor_axes.bits() & !locked_axes;
        let limit_axes = joint.limit_axes.bits() & !locked_axes;
        let coupled_axes = joint.coupled_axes.bits();

        // The has_lin/ang_coupling test is needed to avoid shl overflow later.
        let has_lin_coupling = (coupled_axes & JointAxesMask::LIN_AXES.bits()) != 0;
        let first_coupled_lin_axis_id =
            (coupled_axes & JointAxesMask::LIN_AXES.bits()).trailing_zeros() as usize;

        #[cfg(feature = "dim3")]
        let has_ang_coupling = (coupled_axes & JointAxesMask::ANG_AXES.bits()) != 0;
        #[cfg(feature = "dim3")]
        let first_coupled_ang_axis_id =
            (coupled_axes & JointAxesMask::ANG_AXES.bits()).trailing_zeros() as usize;

        let builder = JointTwoBodyConstraintHelper::new(
            frame1,
            frame2,
            &body1.world_com,
            &body2.world_com,
            locked_axes,
        );

        let start = len;
        for i in DIM..SPATIAL_DIM {
            if (motor_axes & !coupled_axes) & (1 << i) != 0 {
                out[len] = builder.motor_angular(
                    [joint_id],
                    body1,
                    body2,
                    i - DIM,
                    &joint.motors[i].motor_params(params.dt),
                    WritebackId::Motor(i),
                );
                len += 1;
            }
        }
        for i in 0..DIM {
            if (motor_axes & !coupled_axes) & (1 << i) != 0 {
                let limits = if limit_axes & (1 << i) != 0 {
                    Some([joint.limits[i].min, joint.limits[i].max])
                } else {
                    None
                };

                out[len] = builder.motor_linear(
                    params,
                    [joint_id],
                    body1,
                    body2,
                    i,
                    &joint.motors[i].motor_params(params.dt),
                    limits,
                    WritebackId::Motor(i),
                );
                len += 1;
            }
        }

        if (motor_axes & coupled_axes) & JointAxesMask::ANG_AXES.bits() != 0 {
            // TODO: coupled angular motor constraint.
        }

        if (motor_axes & coupled_axes) & JointAxesMask::LIN_AXES.bits() != 0 {
            let limits = if (limit_axes & (1 << first_coupled_lin_axis_id)) != 0 {
                Some([
                    joint.limits[first_coupled_lin_axis_id].min,
                    joint.limits[first_coupled_lin_axis_id].max,
                ])
            } else {
                None
            };

            out[len] = builder.motor_linear_coupled(
                params,
                [joint_id],
                body1,
                body2,
                coupled_axes,
                &joint.motors[first_coupled_lin_axis_id].motor_params(params.dt),
                limits,
                WritebackId::Motor(first_coupled_lin_axis_id),
            );
            len += 1;
        }

        JointTwoBodyConstraintHelper::finalize_constraints(&mut out[start..len]);

        let start = len;
        for i in DIM..SPATIAL_DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] = builder.lock_angular(
                    params,
                    [joint_id],
                    body1,
                    body2,
                    i - DIM,
                    WritebackId::Dof(i),
                );
                len += 1;
            }
        }
        for i in 0..DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] =
                    builder.lock_linear(params, [joint_id], body1, body2, i, WritebackId::Dof(i));
                len += 1;
            }
        }

        for i in DIM..SPATIAL_DIM {
            if (limit_axes & !coupled_axes) & (1 << i) != 0 {
                out[len] = builder.limit_angular(
                    params,
                    [joint_id],
                    body1,
                    body2,
                    i - DIM,
                    [joint.limits[i].min, joint.limits[i].max],
                    WritebackId::Limit(i),
                );
                len += 1;
            }
        }
        for i in 0..DIM {
            if (limit_axes & !coupled_axes) & (1 << i) != 0 {
                out[len] = builder.limit_linear(
                    params,
                    [joint_id],
                    body1,
                    body2,
                    i,
                    [joint.limits[i].min, joint.limits[i].max],
                    WritebackId::Limit(i),
                );
                len += 1;
            }
        }

        #[cfg(feature = "dim3")]
        if has_ang_coupling && (limit_axes & (1 << first_coupled_ang_axis_id)) != 0 {
            out[len] = builder.limit_angular_coupled(
                params,
                [joint_id],
                body1,
                body2,
                coupled_axes,
                [
                    joint.limits[first_coupled_ang_axis_id].min,
                    joint.limits[first_coupled_ang_axis_id].max,
                ],
                WritebackId::Limit(first_coupled_ang_axis_id),
            );
            len += 1;
        }

        if has_lin_coupling && (limit_axes & (1 << first_coupled_lin_axis_id)) != 0 {
            out[len] = builder.limit_linear_coupled(
                params,
                [joint_id],
                body1,
                body2,
                coupled_axes,
                [
                    joint.limits[first_coupled_lin_axis_id].min,
                    joint.limits[first_coupled_lin_axis_id].max,
                ],
                WritebackId::Limit(first_coupled_lin_axis_id),
            );
            len += 1;
        }
        JointTwoBodyConstraintHelper::finalize_constraints(&mut out[start..len]);

        len
    }

    pub fn solve(&mut self, solver_vels: &mut [SolverVel<Real>]) {
        let mut solver_vel1 = solver_vels[self.solver_vel1[0]];
        let mut solver_vel2 = solver_vels[self.solver_vel2[0]];

        self.solve_generic(&mut solver_vel1, &mut solver_vel2);

        solver_vels[self.solver_vel1[0]] = solver_vel1;
        solver_vels[self.solver_vel2[0]] = solver_vel2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id[0]].weight;
        match self.writeback_id {
            WritebackId::Dof(i) => joint.impulses[i] = self.impulse,
            WritebackId::Limit(i) => joint.data.limits[i].impulse = self.impulse,
            WritebackId::Motor(i) => joint.data.motors[i].impulse = self.impulse,
        }
    }
}

#[cfg(feature = "simd-is-enabled")]
impl JointTwoBodyConstraint<SimdReal, SIMD_WIDTH> {
    pub fn lock_axes(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        body1: &JointSolverBody<SimdReal, SIMD_WIDTH>,
        body2: &JointSolverBody<SimdReal, SIMD_WIDTH>,
        frame1: &Isometry<SimdReal>,
        frame2: &Isometry<SimdReal>,
        locked_axes: u8,
        out: &mut [Self],
    ) -> usize {
        let builder = JointTwoBodyConstraintHelper::new(
            frame1,
            frame2,
            &body1.world_com,
            &body2.world_com,
            locked_axes,
        );

        let mut len = 0;
        for i in 0..DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] =
                    builder.lock_linear(params, joint_id, body1, body2, i, WritebackId::Dof(i));
                len += 1;
            }
        }

        for i in DIM..SPATIAL_DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] = builder.lock_angular(
                    params,
                    joint_id,
                    body1,
                    body2,
                    i - DIM,
                    WritebackId::Dof(i),
                );
                len += 1;
            }
        }

        JointTwoBodyConstraintHelper::finalize_constraints(&mut out[..len]);
        len
    }

    pub fn solve(&mut self, solver_vels: &mut [SolverVel<Real>]) {
        let mut solver_vel1 = SolverVel {
            linear: Vector::from(gather![|ii| solver_vels[self.solver_vel1[ii]].linear]),
            angular: AngVector::from(gather![|ii| solver_vels[self.solver_vel1[ii]].angular]),
        };
        let mut solver_vel2 = SolverVel {
            linear: Vector::from(gather![|ii| solver_vels[self.solver_vel2[ii]].linear]),
            angular: AngVector::from(gather![|ii| solver_vels[self.solver_vel2[ii]].angular]),
        };

        self.solve_generic(&mut solver_vel1, &mut solver_vel2);

        for ii in 0..SIMD_WIDTH {
            solver_vels[self.solver_vel1[ii]].linear = solver_vel1.linear.extract(ii);
            solver_vels[self.solver_vel1[ii]].angular = solver_vel1.angular.extract(ii);
            solver_vels[self.solver_vel2[ii]].linear = solver_vel2.linear.extract(ii);
            solver_vels[self.solver_vel2[ii]].angular = solver_vel2.angular.extract(ii);
        }
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let impulses: [_; SIMD_WIDTH] = self.impulse.into();

        // TODO: should we move the iteration on ii deeper in the mested match?
        for ii in 0..SIMD_WIDTH {
            let joint = &mut joints_all[self.joint_id[ii]].weight;
            match self.writeback_id {
                WritebackId::Dof(i) => joint.impulses[i] = impulses[ii],
                WritebackId::Limit(i) => joint.data.limits[i].impulse = impulses[ii],
                WritebackId::Motor(i) => joint.data.motors[i].impulse = impulses[ii],
            }
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct JointOneBodyConstraint<N: SimdRealCopy, const LANES: usize> {
    pub solver_vel2: [usize; LANES],

    pub joint_id: [JointIndex; LANES],

    pub impulse: N,
    pub impulse_bounds: [N; 2],
    pub lin_jac: Vector<N>,
    pub ang_jac2: AngVector<N>,

    pub inv_lhs: N,
    pub cfm_coeff: N,
    pub cfm_gain: N,
    pub rhs: N,
    pub rhs_wo_bias: N,

    pub im2: Vector<N>,

    pub writeback_id: WritebackId,
}

impl<N: SimdRealCopy, const LANES: usize> JointOneBodyConstraint<N, LANES> {
    pub fn solve_generic(&mut self, solver_vel2: &mut SolverVel<N>) {
        let dlinvel = solver_vel2.linear;
        let dangvel = solver_vel2.angular;

        let dvel = self.lin_jac.dot(&dlinvel) + self.ang_jac2.gdot(dangvel) + self.rhs;
        let total_impulse = (self.impulse + self.inv_lhs * (dvel - self.cfm_gain * self.impulse))
            .simd_clamp(self.impulse_bounds[0], self.impulse_bounds[1]);
        let delta_impulse = total_impulse - self.impulse;
        self.impulse = total_impulse;

        let lin_impulse = self.lin_jac * delta_impulse;
        let ang_impulse = self.ang_jac2 * delta_impulse;

        solver_vel2.linear -= lin_impulse.component_mul(&self.im2);
        solver_vel2.angular -= ang_impulse;
    }

    pub fn remove_bias_from_rhs(&mut self) {
        self.rhs = self.rhs_wo_bias;
    }
}

impl JointOneBodyConstraint<Real, 1> {
    pub fn lock_axes(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        body1: &JointFixedSolverBody<Real>,
        body2: &JointSolverBody<Real, 1>,
        frame1: &Isometry<Real>,
        frame2: &Isometry<Real>,
        joint: &GenericJoint,
        out: &mut [Self],
    ) -> usize {
        let mut len = 0;
        let locked_axes = joint.locked_axes.bits();
        let motor_axes = joint.motor_axes.bits() & !locked_axes;
        let limit_axes = joint.limit_axes.bits() & !locked_axes;
        let coupled_axes = joint.coupled_axes.bits();

        // The has_lin/ang_coupling test is needed to avoid shl overflow later.
        let has_lin_coupling = (coupled_axes & JointAxesMask::LIN_AXES.bits()) != 0;
        let first_coupled_lin_axis_id =
            (coupled_axes & JointAxesMask::LIN_AXES.bits()).trailing_zeros() as usize;

        #[cfg(feature = "dim3")]
        let has_ang_coupling = (coupled_axes & JointAxesMask::ANG_AXES.bits()) != 0;
        #[cfg(feature = "dim3")]
        let first_coupled_ang_axis_id =
            (coupled_axes & JointAxesMask::ANG_AXES.bits()).trailing_zeros() as usize;

        let builder = JointTwoBodyConstraintHelper::new(
            frame1,
            frame2,
            &body1.world_com,
            &body2.world_com,
            locked_axes,
        );

        let start = len;
        for i in DIM..SPATIAL_DIM {
            if (motor_axes & !coupled_axes) & (1 << i) != 0 {
                out[len] = builder.motor_angular_one_body(
                    [joint_id],
                    body1,
                    body2,
                    i - DIM,
                    &joint.motors[i].motor_params(params.dt),
                    WritebackId::Motor(i),
                );
                len += 1;
            }
        }
        for i in 0..DIM {
            if (motor_axes & !coupled_axes) & (1 << i) != 0 {
                let limits = if limit_axes & (1 << i) != 0 {
                    Some([joint.limits[i].min, joint.limits[i].max])
                } else {
                    None
                };

                out[len] = builder.motor_linear_one_body(
                    params,
                    [joint_id],
                    body1,
                    body2,
                    i,
                    &joint.motors[i].motor_params(params.dt),
                    limits,
                    WritebackId::Motor(i),
                );
                len += 1;
            }
        }

        #[cfg(feature = "dim3")]
        if has_ang_coupling && (motor_axes & (1 << first_coupled_ang_axis_id)) != 0 {
            // TODO: coupled angular motor constraint.
        }

        if has_lin_coupling && (motor_axes & (1 << first_coupled_lin_axis_id)) != 0 {
            let limits = if (limit_axes & (1 << first_coupled_lin_axis_id)) != 0 {
                Some([
                    joint.limits[first_coupled_lin_axis_id].min,
                    joint.limits[first_coupled_lin_axis_id].max,
                ])
            } else {
                None
            };

            out[len] = builder.motor_linear_coupled_one_body(
                params,
                [joint_id],
                body1,
                body2,
                coupled_axes,
                &joint.motors[first_coupled_lin_axis_id].motor_params(params.dt),
                limits,
                WritebackId::Motor(first_coupled_lin_axis_id),
            );
            len += 1;
        }

        JointTwoBodyConstraintHelper::finalize_one_body_constraints(&mut out[start..len]);

        let start = len;
        for i in DIM..SPATIAL_DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] = builder.lock_angular_one_body(
                    params,
                    [joint_id],
                    body1,
                    body2,
                    i - DIM,
                    WritebackId::Dof(i),
                );
                len += 1;
            }
        }
        for i in 0..DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] = builder.lock_linear_one_body(
                    params,
                    [joint_id],
                    body1,
                    body2,
                    i,
                    WritebackId::Dof(i),
                );
                len += 1;
            }
        }

        for i in DIM..SPATIAL_DIM {
            if (limit_axes & !coupled_axes) & (1 << i) != 0 {
                out[len] = builder.limit_angular_one_body(
                    params,
                    [joint_id],
                    body1,
                    body2,
                    i - DIM,
                    [joint.limits[i].min, joint.limits[i].max],
                    WritebackId::Limit(i),
                );
                len += 1;
            }
        }
        for i in 0..DIM {
            if (limit_axes & !coupled_axes) & (1 << i) != 0 {
                out[len] = builder.limit_linear_one_body(
                    params,
                    [joint_id],
                    body1,
                    body2,
                    i,
                    [joint.limits[i].min, joint.limits[i].max],
                    WritebackId::Limit(i),
                );
                len += 1;
            }
        }

        #[cfg(feature = "dim3")]
        if has_ang_coupling && (limit_axes & (1 << first_coupled_ang_axis_id)) != 0 {
            out[len] = builder.limit_angular_coupled_one_body(
                params,
                [joint_id],
                body1,
                body2,
                coupled_axes,
                [
                    joint.limits[first_coupled_ang_axis_id].min,
                    joint.limits[first_coupled_ang_axis_id].max,
                ],
                WritebackId::Limit(first_coupled_ang_axis_id),
            );
            len += 1;
        }

        if has_lin_coupling && (limit_axes & (1 << first_coupled_lin_axis_id)) != 0 {
            out[len] = builder.limit_linear_coupled_one_body(
                params,
                [joint_id],
                body1,
                body2,
                coupled_axes,
                [
                    joint.limits[first_coupled_lin_axis_id].min,
                    joint.limits[first_coupled_lin_axis_id].max,
                ],
                WritebackId::Limit(first_coupled_lin_axis_id),
            );
            len += 1;
        }
        JointTwoBodyConstraintHelper::finalize_one_body_constraints(&mut out[start..len]);

        len
    }

    pub fn solve(&mut self, solver_vels: &mut [SolverVel<Real>]) {
        let mut solver_vel2 = solver_vels[self.solver_vel2[0]];
        self.solve_generic(&mut solver_vel2);
        solver_vels[self.solver_vel2[0]] = solver_vel2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id[0]].weight;
        match self.writeback_id {
            WritebackId::Dof(i) => joint.impulses[i] = self.impulse,
            WritebackId::Limit(i) => joint.data.limits[i].impulse = self.impulse,
            WritebackId::Motor(i) => joint.data.motors[i].impulse = self.impulse,
        }
    }
}

#[cfg(feature = "simd-is-enabled")]
impl JointOneBodyConstraint<SimdReal, SIMD_WIDTH> {
    pub fn lock_axes(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        body1: &JointFixedSolverBody<SimdReal>,
        body2: &JointSolverBody<SimdReal, SIMD_WIDTH>,
        frame1: &Isometry<SimdReal>,
        frame2: &Isometry<SimdReal>,
        locked_axes: u8,
        out: &mut [Self],
    ) -> usize {
        let mut len = 0;
        let builder = JointTwoBodyConstraintHelper::new(
            frame1,
            frame2,
            &body1.world_com,
            &body2.world_com,
            locked_axes,
        );

        for i in 0..DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] = builder.lock_linear_one_body(
                    params,
                    joint_id,
                    body1,
                    body2,
                    i,
                    WritebackId::Dof(i),
                );
                len += 1;
            }
        }
        for i in DIM..SPATIAL_DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] = builder.lock_angular_one_body(
                    params,
                    joint_id,
                    body1,
                    body2,
                    i - DIM,
                    WritebackId::Dof(i),
                );
                len += 1;
            }
        }

        JointTwoBodyConstraintHelper::finalize_one_body_constraints(&mut out[..len]);
        len
    }

    pub fn solve(&mut self, solver_vels: &mut [SolverVel<Real>]) {
        let mut solver_vel2 = SolverVel {
            linear: Vector::from(gather![|ii| solver_vels[self.solver_vel2[ii]].linear]),
            angular: AngVector::from(gather![|ii| solver_vels[self.solver_vel2[ii]].angular]),
        };

        self.solve_generic(&mut solver_vel2);

        for ii in 0..SIMD_WIDTH {
            solver_vels[self.solver_vel2[ii]].linear = solver_vel2.linear.extract(ii);
            solver_vels[self.solver_vel2[ii]].angular = solver_vel2.angular.extract(ii);
        }
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let impulses: [_; SIMD_WIDTH] = self.impulse.into();

        // TODO: should we move the iteration on ii deeper in the mested match?
        for ii in 0..SIMD_WIDTH {
            let joint = &mut joints_all[self.joint_id[ii]].weight;
            match self.writeback_id {
                WritebackId::Dof(i) => joint.impulses[i] = impulses[ii],
                WritebackId::Limit(i) => joint.data.limits[i].impulse = impulses[ii],
                WritebackId::Motor(i) => joint.data.motors[i].impulse = impulses[ii],
            }
        }
    }
}
