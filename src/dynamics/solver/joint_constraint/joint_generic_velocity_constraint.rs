use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::WritebackId;
use crate::dynamics::solver::joint_constraint::{JointVelocityConstraintBuilder, SolverBody};
use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{GenericJoint, IntegrationParameters, JointGraphEdge, JointIndex, Multibody};
use crate::math::{Isometry, Real, DIM};
use crate::prelude::SPATIAL_DIM;
use na::{DVector, DVectorView, DVectorViewMut};

#[derive(Debug, Copy, Clone)]
pub struct JointGenericVelocityConstraint {
    pub is_rigid_body1: bool,
    pub is_rigid_body2: bool,
    pub mj_lambda1: usize,
    pub mj_lambda2: usize,

    pub ndofs1: usize,
    pub j_id1: usize,
    pub ndofs2: usize,
    pub j_id2: usize,

    pub joint_id: JointIndex,

    pub impulse: Real,
    pub impulse_bounds: [Real; 2],
    pub inv_lhs: Real,
    pub rhs: Real,
    pub rhs_wo_bias: Real,
    pub cfm_coeff: Real,
    pub cfm_gain: Real,

    pub writeback_id: WritebackId,
}

impl Default for JointGenericVelocityConstraint {
    fn default() -> Self {
        JointGenericVelocityConstraint::invalid()
    }
}

impl JointGenericVelocityConstraint {
    pub fn invalid() -> Self {
        Self {
            is_rigid_body1: false,
            is_rigid_body2: false,
            mj_lambda1: 0,
            mj_lambda2: 0,
            ndofs1: 0,
            j_id1: 0,
            ndofs2: 0,
            j_id2: 0,
            joint_id: 0,
            impulse: 0.0,
            impulse_bounds: [-Real::MAX, Real::MAX],
            inv_lhs: 0.0,
            rhs: 0.0,
            rhs_wo_bias: 0.0,
            cfm_coeff: 0.0,
            cfm_gain: 0.0,
            writeback_id: WritebackId::Dof(0),
        }
    }

    pub fn lock_axes(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        frame1: &Isometry<Real>,
        frame2: &Isometry<Real>,
        joint: &GenericJoint,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        out: &mut [Self],
    ) -> usize {
        let mut len = 0;
        let locked_axes = joint.locked_axes.bits();
        let motor_axes = joint.motor_axes.bits();
        let limit_axes = joint.limit_axes.bits();

        let builder = JointVelocityConstraintBuilder::new(
            frame1,
            frame2,
            &body1.world_com,
            &body2.world_com,
            locked_axes,
        );

        let start = len;
        for i in DIM..SPATIAL_DIM {
            if (motor_axes >> DIM) & (1 << i) != 0 {
                out[len] = builder.motor_angular_generic(
                    params,
                    jacobians,
                    j_id,
                    joint_id,
                    body1,
                    body2,
                    mb1,
                    mb2,
                    i - DIM,
                    &joint.motors[i].motor_params(params.dt),
                    WritebackId::Motor(i),
                );
                len += 1;
            }
        }
        for i in 0..DIM {
            if motor_axes & (1 << i) != 0 {
                out[len] = builder.motor_linear_generic(
                    params,
                    jacobians,
                    j_id,
                    joint_id,
                    body1,
                    body2,
                    mb1,
                    mb2,
                    // locked_ang_axes,
                    i,
                    &joint.motors[i].motor_params(params.dt),
                    WritebackId::Motor(i),
                );
                len += 1;
            }
        }
        JointVelocityConstraintBuilder::finalize_generic_constraints(
            jacobians,
            &mut out[start..len],
        );

        let start = len;
        for i in DIM..SPATIAL_DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] = builder.lock_angular_generic(
                    params,
                    jacobians,
                    j_id,
                    joint_id,
                    body1,
                    body2,
                    mb1,
                    mb2,
                    i - DIM,
                    WritebackId::Dof(i),
                );
                len += 1;
            }
        }
        for i in 0..DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] = builder.lock_linear_generic(
                    params,
                    jacobians,
                    j_id,
                    joint_id,
                    body1,
                    body2,
                    mb1,
                    mb2,
                    i,
                    WritebackId::Dof(i),
                );
                len += 1;
            }
        }

        for i in DIM..SPATIAL_DIM {
            if limit_axes & (1 << i) != 0 {
                out[len] = builder.limit_angular_generic(
                    params,
                    jacobians,
                    j_id,
                    joint_id,
                    body1,
                    body2,
                    mb1,
                    mb2,
                    i - DIM,
                    [joint.limits[i].min, joint.limits[i].max],
                    WritebackId::Limit(i),
                );
                len += 1;
            }
        }
        for i in 0..DIM {
            if limit_axes & (1 << i) != 0 {
                out[len] = builder.limit_linear_generic(
                    params,
                    jacobians,
                    j_id,
                    joint_id,
                    body1,
                    body2,
                    mb1,
                    mb2,
                    i,
                    [joint.limits[i].min, joint.limits[i].max],
                    WritebackId::Limit(i),
                );
                len += 1;
            }
        }

        JointVelocityConstraintBuilder::finalize_generic_constraints(
            jacobians,
            &mut out[start..len],
        );
        len
    }

    fn wj_id1(&self) -> usize {
        self.j_id1 + self.ndofs1
    }

    fn wj_id2(&self) -> usize {
        self.j_id2 + self.ndofs2
    }

    fn mj_lambda1<'a>(
        &self,
        mj_lambdas: &'a [DeltaVel<Real>],
        generic_mj_lambdas: &'a DVector<Real>,
    ) -> DVectorView<'a, Real> {
        if self.is_rigid_body1 {
            mj_lambdas[self.mj_lambda1].as_vector_slice()
        } else {
            generic_mj_lambdas.rows(self.mj_lambda1, self.ndofs1)
        }
    }

    fn mj_lambda1_mut<'a>(
        &self,
        mj_lambdas: &'a mut [DeltaVel<Real>],
        generic_mj_lambdas: &'a mut DVector<Real>,
    ) -> DVectorViewMut<'a, Real> {
        if self.is_rigid_body1 {
            mj_lambdas[self.mj_lambda1].as_vector_slice_mut()
        } else {
            generic_mj_lambdas.rows_mut(self.mj_lambda1, self.ndofs1)
        }
    }

    fn mj_lambda2<'a>(
        &self,
        mj_lambdas: &'a [DeltaVel<Real>],
        generic_mj_lambdas: &'a DVector<Real>,
    ) -> DVectorView<'a, Real> {
        if self.is_rigid_body2 {
            mj_lambdas[self.mj_lambda2].as_vector_slice()
        } else {
            generic_mj_lambdas.rows(self.mj_lambda2, self.ndofs2)
        }
    }

    fn mj_lambda2_mut<'a>(
        &self,
        mj_lambdas: &'a mut [DeltaVel<Real>],
        generic_mj_lambdas: &'a mut DVector<Real>,
    ) -> DVectorViewMut<'a, Real> {
        if self.is_rigid_body2 {
            mj_lambdas[self.mj_lambda2].as_vector_slice_mut()
        } else {
            generic_mj_lambdas.rows_mut(self.mj_lambda2, self.ndofs2)
        }
    }

    pub fn solve(
        &mut self,
        jacobians: &DVector<Real>,
        mj_lambdas: &mut [DeltaVel<Real>],
        generic_mj_lambdas: &mut DVector<Real>,
    ) {
        let jacobians = jacobians.as_slice();

        let mj_lambda1 = self.mj_lambda1(mj_lambdas, generic_mj_lambdas);
        let j1 = DVectorView::from_slice(&jacobians[self.j_id1..], self.ndofs1);
        let vel1 = j1.dot(&mj_lambda1);

        let mj_lambda2 = self.mj_lambda2(mj_lambdas, generic_mj_lambdas);
        let j2 = DVectorView::from_slice(&jacobians[self.j_id2..], self.ndofs2);
        let vel2 = j2.dot(&mj_lambda2);

        let dvel = self.rhs + (vel2 - vel1);
        let total_impulse = na::clamp(
            self.impulse + self.inv_lhs * (dvel - self.cfm_gain * self.impulse),
            self.impulse_bounds[0],
            self.impulse_bounds[1],
        );
        let delta_impulse = total_impulse - self.impulse;
        self.impulse = total_impulse;

        let mut mj_lambda1 = self.mj_lambda1_mut(mj_lambdas, generic_mj_lambdas);
        let wj1 = DVectorView::from_slice(&jacobians[self.wj_id1()..], self.ndofs1);
        mj_lambda1.axpy(delta_impulse, &wj1, 1.0);

        let mut mj_lambda2 = self.mj_lambda2_mut(mj_lambdas, generic_mj_lambdas);
        let wj2 = DVectorView::from_slice(&jacobians[self.wj_id2()..], self.ndofs2);
        mj_lambda2.axpy(-delta_impulse, &wj2, 1.0);
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        match self.writeback_id {
            WritebackId::Dof(i) => joint.impulses[i] = self.impulse,
            WritebackId::Limit(i) => joint.data.limits[i].impulse = self.impulse,
            WritebackId::Motor(i) => joint.data.motors[i].impulse = self.impulse,
        }
    }

    pub fn remove_bias_from_rhs(&mut self) {
        self.rhs = self.rhs_wo_bias;
    }
}

#[derive(Debug, Copy, Clone)]
pub struct JointGenericVelocityGroundConstraint {
    pub mj_lambda2: usize,
    pub ndofs2: usize,
    pub j_id2: usize,

    pub joint_id: JointIndex,

    pub impulse: Real,
    pub impulse_bounds: [Real; 2],
    pub inv_lhs: Real,
    pub rhs: Real,
    pub rhs_wo_bias: Real,
    pub cfm_coeff: Real,
    pub cfm_gain: Real,

    pub writeback_id: WritebackId,
}

impl Default for JointGenericVelocityGroundConstraint {
    fn default() -> Self {
        JointGenericVelocityGroundConstraint::invalid()
    }
}

impl JointGenericVelocityGroundConstraint {
    pub fn invalid() -> Self {
        Self {
            mj_lambda2: crate::INVALID_USIZE,
            ndofs2: 0,
            j_id2: crate::INVALID_USIZE,
            joint_id: 0,
            impulse: 0.0,
            impulse_bounds: [-Real::MAX, Real::MAX],
            inv_lhs: 0.0,
            rhs: 0.0,
            rhs_wo_bias: 0.0,
            cfm_coeff: 0.0,
            cfm_gain: 0.0,
            writeback_id: WritebackId::Dof(0),
        }
    }

    pub fn lock_axes(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
        mb2: (&Multibody, usize),
        frame1: &Isometry<Real>,
        frame2: &Isometry<Real>,
        joint: &GenericJoint,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        out: &mut [Self],
    ) -> usize {
        let mut len = 0;
        let locked_axes = joint.locked_axes.bits();
        let motor_axes = joint.motor_axes.bits();
        let limit_axes = joint.limit_axes.bits();

        let builder = JointVelocityConstraintBuilder::new(
            frame1,
            frame2,
            &body1.world_com,
            &body2.world_com,
            locked_axes,
        );

        let start = len;
        for i in DIM..SPATIAL_DIM {
            if (motor_axes >> DIM) & (1 << i) != 0 {
                out[len] = builder.motor_angular_generic_ground(
                    params,
                    jacobians,
                    j_id,
                    joint_id,
                    body1,
                    body2,
                    mb2,
                    i - DIM,
                    &joint.motors[i].motor_params(params.dt),
                    WritebackId::Motor(i),
                );
                len += 1;
            }
        }

        for i in 0..DIM {
            if motor_axes & (1 << i) != 0 {
                out[len] = builder.motor_linear_generic_ground(
                    params,
                    jacobians,
                    j_id,
                    joint_id,
                    body1,
                    body2,
                    mb2,
                    // locked_ang_axes,
                    i,
                    &joint.motors[i].motor_params(params.dt),
                    WritebackId::Motor(i),
                );
                len += 1;
            }
        }

        JointVelocityConstraintBuilder::finalize_generic_constraints_ground(
            jacobians,
            &mut out[start..len],
        );

        let start = len;
        for i in DIM..SPATIAL_DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] = builder.lock_angular_generic_ground(
                    params,
                    jacobians,
                    j_id,
                    joint_id,
                    body1,
                    mb2,
                    i - DIM,
                    WritebackId::Dof(i),
                );
                len += 1;
            }
        }
        for i in 0..DIM {
            if locked_axes & (1 << i) != 0 {
                out[len] = builder.lock_linear_generic_ground(
                    params,
                    jacobians,
                    j_id,
                    joint_id,
                    body1,
                    mb2,
                    i,
                    WritebackId::Dof(i),
                );
                len += 1;
            }
        }

        for i in DIM..SPATIAL_DIM {
            if limit_axes & (1 << i) != 0 {
                out[len] = builder.limit_angular_generic_ground(
                    params,
                    jacobians,
                    j_id,
                    joint_id,
                    body1,
                    mb2,
                    i - DIM,
                    [joint.limits[i].min, joint.limits[i].max],
                    WritebackId::Limit(i),
                );
                len += 1;
            }
        }
        for i in 0..DIM {
            if limit_axes & (1 << i) != 0 {
                out[len] = builder.limit_linear_generic_ground(
                    params,
                    jacobians,
                    j_id,
                    joint_id,
                    body1,
                    mb2,
                    i,
                    [joint.limits[i].min, joint.limits[i].max],
                    WritebackId::Limit(i),
                );
                len += 1;
            }
        }

        JointVelocityConstraintBuilder::finalize_generic_constraints_ground(
            jacobians,
            &mut out[start..len],
        );
        len
    }

    fn wj_id2(&self) -> usize {
        self.j_id2 + self.ndofs2
    }

    fn mj_lambda2<'a>(
        &self,
        _mj_lambdas: &'a [DeltaVel<Real>],
        generic_mj_lambdas: &'a DVector<Real>,
    ) -> DVectorView<'a, Real> {
        generic_mj_lambdas.rows(self.mj_lambda2, self.ndofs2)
    }

    fn mj_lambda2_mut<'a>(
        &self,
        _mj_lambdas: &'a mut [DeltaVel<Real>],
        generic_mj_lambdas: &'a mut DVector<Real>,
    ) -> DVectorViewMut<'a, Real> {
        generic_mj_lambdas.rows_mut(self.mj_lambda2, self.ndofs2)
    }

    pub fn solve(
        &mut self,
        jacobians: &DVector<Real>,
        mj_lambdas: &mut [DeltaVel<Real>],
        generic_mj_lambdas: &mut DVector<Real>,
    ) {
        let jacobians = jacobians.as_slice();

        let mj_lambda2 = self.mj_lambda2(mj_lambdas, generic_mj_lambdas);
        let j2 = DVectorView::from_slice(&jacobians[self.j_id2..], self.ndofs2);
        let vel2 = j2.dot(&mj_lambda2);

        let dvel = self.rhs + vel2;
        let total_impulse = na::clamp(
            self.impulse + self.inv_lhs * (dvel - self.cfm_gain * self.impulse),
            self.impulse_bounds[0],
            self.impulse_bounds[1],
        );
        let delta_impulse = total_impulse - self.impulse;
        self.impulse = total_impulse;

        let mut mj_lambda2 = self.mj_lambda2_mut(mj_lambdas, generic_mj_lambdas);
        let wj2 = DVectorView::from_slice(&jacobians[self.wj_id2()..], self.ndofs2);
        mj_lambda2.axpy(-delta_impulse, &wj2, 1.0);
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        // FIXME: impulse writeback isn’t supported yet for internal multibody_joint constraints.
        if self.joint_id != usize::MAX {
            let joint = &mut joints_all[self.joint_id].weight;
            match self.writeback_id {
                WritebackId::Dof(i) => joint.impulses[i] = self.impulse,
                WritebackId::Limit(i) => joint.data.limits[i].impulse = self.impulse,
                WritebackId::Motor(i) => joint.data.motors[i].impulse = self.impulse,
            }
        }
    }

    pub fn remove_bias_from_rhs(&mut self) {
        self.rhs = self.rhs_wo_bias;
    }
}
