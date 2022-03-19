use crate::dynamics::solver::joint_constraint::JointVelocityConstraintBuilder;
use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    GenericJoint, IntegrationParameters, JointAxesMask, JointGraphEdge, JointIndex,
};
use crate::math::{AngVector, AngularInertia, Isometry, Point, Real, Vector, DIM, SPATIAL_DIM};
use crate::utils::{WDot, WReal};

#[cfg(feature = "simd-is-enabled")]
use {
    crate::math::{SimdReal, SIMD_WIDTH},
    na::SimdValue,
};

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct MotorParameters<N: WReal> {
    pub erp_inv_dt: N,
    pub cfm_coeff: N,
    pub cfm_gain: N,
    pub target_pos: N,
    pub target_vel: N,
    pub max_impulse: N,
}

impl<N: WReal> Default for MotorParameters<N> {
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
pub struct SolverBody<N: WReal, const LANES: usize> {
    pub linvel: Vector<N>,
    pub angvel: AngVector<N>,
    pub im: Vector<N>,
    pub sqrt_ii: AngularInertia<N>,
    pub world_com: Point<N>,
    pub mj_lambda: [usize; LANES],
}

#[derive(Debug, Copy, Clone)]
pub struct JointVelocityConstraint<N: WReal, const LANES: usize> {
    pub mj_lambda1: [usize; LANES],
    pub mj_lambda2: [usize; LANES],

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

impl<N: WReal, const LANES: usize> JointVelocityConstraint<N, LANES> {
    pub fn invalid() -> Self {
        Self {
            mj_lambda1: [crate::INVALID_USIZE; LANES],
            mj_lambda2: [crate::INVALID_USIZE; LANES],
            joint_id: [crate::INVALID_USIZE; LANES],
            impulse: N::zero(),
            impulse_bounds: [N::zero(), N::zero()],
            lin_jac: Vector::zeros(),
            ang_jac1: na::zero(),
            ang_jac2: na::zero(),
            inv_lhs: N::zero(),
            cfm_gain: N::zero(),
            cfm_coeff: N::zero(),
            rhs: N::zero(),
            rhs_wo_bias: N::zero(),
            im1: na::zero(),
            im2: na::zero(),
            writeback_id: WritebackId::Dof(0),
        }
    }

    pub fn solve_generic(&mut self, mj_lambda1: &mut DeltaVel<N>, mj_lambda2: &mut DeltaVel<N>) {
        let dlinvel = self.lin_jac.dot(&(mj_lambda2.linear - mj_lambda1.linear));
        let dangvel =
            self.ang_jac2.gdot(mj_lambda2.angular) - self.ang_jac1.gdot(mj_lambda1.angular);

        let rhs = dlinvel + dangvel + self.rhs;
        let total_impulse = (self.impulse + self.inv_lhs * (rhs - self.cfm_gain * self.impulse))
            .simd_clamp(self.impulse_bounds[0], self.impulse_bounds[1]);
        let delta_impulse = total_impulse - self.impulse;
        self.impulse = total_impulse;

        let lin_impulse = self.lin_jac * delta_impulse;
        let ang_impulse1 = self.ang_jac1 * delta_impulse;
        let ang_impulse2 = self.ang_jac2 * delta_impulse;

        mj_lambda1.linear += lin_impulse.component_mul(&self.im1);
        mj_lambda1.angular += ang_impulse1;
        mj_lambda2.linear -= lin_impulse.component_mul(&self.im2);
        mj_lambda2.angular -= ang_impulse2;
    }

    pub fn remove_bias_from_rhs(&mut self) {
        self.rhs = self.rhs_wo_bias;
    }
}

impl JointVelocityConstraint<Real, 1> {
    pub fn lock_axes(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
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

        let builder = JointVelocityConstraintBuilder::new(
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
            // TODO: coupled linear motor constraint.
            // out[len] = builder.motor_linear_coupled(
            //     params,
            //     [joint_id],
            //     body1,
            //     body2,
            //     limit_axes & coupled_axes,
            //     &joint.limits,
            //     WritebackId::Limit(0), // TODO: writeback
            // );
            // len += 1;
        }

        JointVelocityConstraintBuilder::finalize_constraints(&mut out[start..len]);

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
        if (limit_axes & coupled_axes) & JointAxesMask::ANG_AXES.bits() != 0 {
            out[len] = builder.limit_angular_coupled(
                params,
                [joint_id],
                body1,
                body2,
                limit_axes & coupled_axes,
                &joint.limits,
                WritebackId::Limit(0), // TODO: writeback
            );
            len += 1;
        }

        if (limit_axes & coupled_axes) & JointAxesMask::LIN_AXES.bits() != 0 {
            out[len] = builder.limit_linear_coupled(
                params,
                [joint_id],
                body1,
                body2,
                limit_axes & coupled_axes,
                &joint.limits,
                WritebackId::Limit(0), // TODO: writeback
            );
            len += 1;
        }
        JointVelocityConstraintBuilder::finalize_constraints(&mut out[start..len]);

        len
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1[0] as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2[0] as usize];

        self.solve_generic(&mut mj_lambda1, &mut mj_lambda2);

        mj_lambdas[self.mj_lambda1[0] as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2[0] as usize] = mj_lambda2;
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
impl JointVelocityConstraint<SimdReal, SIMD_WIDTH> {
    pub fn lock_axes(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        body1: &SolverBody<SimdReal, SIMD_WIDTH>,
        body2: &SolverBody<SimdReal, SIMD_WIDTH>,
        frame1: &Isometry<SimdReal>,
        frame2: &Isometry<SimdReal>,
        locked_axes: u8,
        out: &mut [Self],
    ) -> usize {
        let builder = JointVelocityConstraintBuilder::new(
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

        JointVelocityConstraintBuilder::finalize_constraints(&mut out[..len]);
        len
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda1 = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda1[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda1[ii] as usize].angular
            ]),
        };
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular
            ]),
        };

        self.solve_generic(&mut mj_lambda1, &mut mj_lambda2);

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda1[ii] as usize].linear = mj_lambda1.linear.extract(ii);
            mj_lambdas[self.mj_lambda1[ii] as usize].angular = mj_lambda1.angular.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
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
pub struct JointVelocityGroundConstraint<N: WReal, const LANES: usize> {
    pub mj_lambda2: [usize; LANES],

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

impl<N: WReal, const LANES: usize> JointVelocityGroundConstraint<N, LANES> {
    pub fn invalid() -> Self {
        Self {
            mj_lambda2: [crate::INVALID_USIZE; LANES],
            joint_id: [crate::INVALID_USIZE; LANES],
            impulse: N::zero(),
            impulse_bounds: [N::zero(), N::zero()],
            lin_jac: Vector::zeros(),
            ang_jac2: na::zero(),
            inv_lhs: N::zero(),
            cfm_coeff: N::zero(),
            cfm_gain: N::zero(),
            rhs: N::zero(),
            rhs_wo_bias: N::zero(),
            im2: na::zero(),
            writeback_id: WritebackId::Dof(0),
        }
    }

    pub fn solve_generic(&mut self, mj_lambda2: &mut DeltaVel<N>) {
        let dlinvel = mj_lambda2.linear;
        let dangvel = mj_lambda2.angular;

        let dvel = self.lin_jac.dot(&dlinvel) + self.ang_jac2.gdot(dangvel) + self.rhs;
        let total_impulse = (self.impulse + self.inv_lhs * (dvel - self.cfm_gain * self.impulse))
            .simd_clamp(self.impulse_bounds[0], self.impulse_bounds[1]);
        let delta_impulse = total_impulse - self.impulse;
        self.impulse = total_impulse;

        let lin_impulse = self.lin_jac * delta_impulse;
        let ang_impulse = self.ang_jac2 * delta_impulse;

        mj_lambda2.linear -= lin_impulse.component_mul(&self.im2);
        mj_lambda2.angular -= ang_impulse;
    }

    pub fn remove_bias_from_rhs(&mut self) {
        self.rhs = self.rhs_wo_bias;
    }
}

impl JointVelocityGroundConstraint<Real, 1> {
    pub fn lock_axes(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
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

        let builder = JointVelocityConstraintBuilder::new(
            frame1,
            frame2,
            &body1.world_com,
            &body2.world_com,
            locked_axes,
        );

        let start = len;
        for i in DIM..SPATIAL_DIM {
            if (motor_axes & !coupled_axes) & (1 << i) != 0 {
                out[len] = builder.motor_angular_ground(
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

                out[len] = builder.motor_linear_ground(
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
            /*
            // TODO: coupled linear motor constraint.
            out[len] = builder.motor_linear_coupled_ground(
                params,
                [joint_id],
                body1,
                body2,
                motor_axes & coupled_axes,
                &joint.motors,
                limit_axes & coupled_axes,
                &joint.limits,
                WritebackId::Limit(0), // TODO: writeback
            );
            len += 1;
            */
            todo!()
        }

        JointVelocityConstraintBuilder::finalize_ground_constraints(&mut out[start..len]);

        let start = len;
        for i in DIM..SPATIAL_DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] = builder.lock_angular_ground(
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
                out[len] = builder.lock_linear_ground(
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
                out[len] = builder.limit_angular_ground(
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
                out[len] = builder.limit_linear_ground(
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
        if (limit_axes & coupled_axes) & JointAxesMask::ANG_AXES.bits() != 0 {
            out[len] = builder.limit_angular_coupled_ground(
                params,
                [joint_id],
                body1,
                body2,
                limit_axes & coupled_axes,
                &joint.limits,
                WritebackId::Limit(0), // TODO: writeback
            );
            len += 1;
        }

        if (limit_axes & coupled_axes) & JointAxesMask::LIN_AXES.bits() != 0 {
            out[len] = builder.limit_linear_coupled_ground(
                params,
                [joint_id],
                body1,
                body2,
                limit_axes & coupled_axes,
                &joint.limits,
                WritebackId::Limit(0), // TODO: writeback
            );
            len += 1;
        }
        JointVelocityConstraintBuilder::finalize_ground_constraints(&mut out[start..len]);

        len
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2[0] as usize];
        self.solve_generic(&mut mj_lambda2);
        mj_lambdas[self.mj_lambda2[0] as usize] = mj_lambda2;
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
impl JointVelocityGroundConstraint<SimdReal, SIMD_WIDTH> {
    pub fn lock_axes(
        params: &IntegrationParameters,
        joint_id: [JointIndex; SIMD_WIDTH],
        body1: &SolverBody<SimdReal, SIMD_WIDTH>,
        body2: &SolverBody<SimdReal, SIMD_WIDTH>,
        frame1: &Isometry<SimdReal>,
        frame2: &Isometry<SimdReal>,
        locked_axes: u8,
        out: &mut [Self],
    ) -> usize {
        let mut len = 0;
        let builder = JointVelocityConstraintBuilder::new(
            frame1,
            frame2,
            &body1.world_com,
            &body2.world_com,
            locked_axes,
        );

        for i in 0..DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] = builder.lock_linear_ground(
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
                out[len] = builder.lock_angular_ground(
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

        JointVelocityConstraintBuilder::finalize_ground_constraints(&mut out[..len]);
        len
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<Real>]) {
        let mut mj_lambda2 = DeltaVel {
            linear: Vector::from(gather![|ii| mj_lambdas[self.mj_lambda2[ii] as usize].linear]),
            angular: AngVector::from(gather![
                |ii| mj_lambdas[self.mj_lambda2[ii] as usize].angular
            ]),
        };

        self.solve_generic(&mut mj_lambda2);

        for ii in 0..SIMD_WIDTH {
            mj_lambdas[self.mj_lambda2[ii] as usize].linear = mj_lambda2.linear.extract(ii);
            mj_lambdas[self.mj_lambda2[ii] as usize].angular = mj_lambda2.angular.extract(ii);
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
