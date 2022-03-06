use crate::dynamics::solver::joint_constraint::joint_generic_velocity_constraint::{
    JointGenericVelocityConstraint, JointGenericVelocityGroundConstraint,
};
use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::WritebackId;
use crate::dynamics::solver::joint_constraint::{JointVelocityConstraintBuilder, SolverBody};
use crate::dynamics::solver::MotorParameters;
use crate::dynamics::{IntegrationParameters, JointIndex, Multibody};
use crate::math::{Real, Vector, ANG_DIM, DIM, SPATIAL_DIM};
use crate::utils::IndexMut2;
use crate::utils::WDot;
use na::{DVector, SVector};

#[cfg(feature = "dim3")]
use crate::utils::WAngularInertia;

impl SolverBody<Real, 1> {
    pub fn fill_jacobians(
        &self,
        unit_force: Vector<Real>,
        unit_torque: SVector<Real, ANG_DIM>,
        j_id: &mut usize,
        jacobians: &mut DVector<Real>,
    ) -> Real {
        let wj_id = *j_id + SPATIAL_DIM;
        jacobians
            .fixed_rows_mut::<DIM>(*j_id)
            .copy_from(&unit_force);
        jacobians
            .fixed_rows_mut::<ANG_DIM>(*j_id + DIM)
            .copy_from(&unit_torque);

        {
            let mut out_invm_j = jacobians.fixed_rows_mut::<SPATIAL_DIM>(wj_id);
            out_invm_j
                .fixed_rows_mut::<DIM>(0)
                .copy_from(&self.im.component_mul(&unit_force));

            #[cfg(feature = "dim2")]
            {
                out_invm_j[DIM] *= self.sqrt_ii;
            }
            #[cfg(feature = "dim3")]
            {
                out_invm_j.fixed_rows_mut::<ANG_DIM>(DIM).gemv(
                    1.0,
                    &self.sqrt_ii.into_matrix(),
                    &unit_torque,
                    0.0,
                );
            }
        }

        *j_id += SPATIAL_DIM * 2;
        unit_force.dot(&self.linvel) + unit_torque.gdot(self.angvel)
    }
}

impl JointVelocityConstraintBuilder<Real> {
    pub fn lock_jacobians_generic(
        &self,
        _params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        writeback_id: WritebackId,
        lin_jac: Vector<Real>,
        ang_jac1: SVector<Real, ANG_DIM>,
        ang_jac2: SVector<Real, ANG_DIM>,
    ) -> JointGenericVelocityConstraint {
        let is_rigid_body1 = mb1.is_none();
        let is_rigid_body2 = mb2.is_none();

        let ndofs1 = mb1.map(|(m, _)| m.ndofs()).unwrap_or(SPATIAL_DIM);
        let ndofs2 = mb2.map(|(m, _)| m.ndofs()).unwrap_or(SPATIAL_DIM);

        let j_id1 = *j_id;
        let vel1 = if let Some((mb1, link_id1)) = mb1 {
            mb1.fill_jacobians(link_id1, lin_jac, ang_jac1, j_id, jacobians)
                .1
        } else {
            body1.fill_jacobians(lin_jac, ang_jac1, j_id, jacobians)
        };

        let j_id2 = *j_id;
        let vel2 = if let Some((mb2, link_id2)) = mb2 {
            mb2.fill_jacobians(link_id2, lin_jac, ang_jac2, j_id, jacobians)
                .1
        } else {
            body2.fill_jacobians(lin_jac, ang_jac2, j_id, jacobians)
        };

        if is_rigid_body1 {
            let ang_j_id1 = j_id1 + DIM;
            let ang_wj_id1 = j_id1 + DIM + ndofs1;
            let (mut j, wj) = jacobians.rows_range_pair_mut(
                ang_j_id1..ang_j_id1 + ANG_DIM,
                ang_wj_id1..ang_wj_id1 + ANG_DIM,
            );
            j.copy_from(&wj);
        }

        if is_rigid_body2 {
            let ang_j_id2 = j_id2 + DIM;
            let ang_wj_id2 = j_id2 + DIM + ndofs2;
            let (mut j, wj) = jacobians.rows_range_pair_mut(
                ang_j_id2..ang_j_id2 + ANG_DIM,
                ang_wj_id2..ang_wj_id2 + ANG_DIM,
            );
            j.copy_from(&wj);
        }

        let rhs_wo_bias = vel2 - vel1;

        let mj_lambda1 = mb1.map(|m| m.0.solver_id).unwrap_or(body1.mj_lambda[0]);
        let mj_lambda2 = mb2.map(|m| m.0.solver_id).unwrap_or(body2.mj_lambda[0]);

        JointGenericVelocityConstraint {
            is_rigid_body1,
            is_rigid_body2,
            mj_lambda1,
            mj_lambda2,
            ndofs1,
            j_id1,
            ndofs2,
            j_id2,
            joint_id,
            impulse: 0.0,
            impulse_bounds: [-Real::MAX, Real::MAX],
            inv_lhs: 0.0,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            cfm_coeff: 0.0,
            cfm_gain: 0.0,
            writeback_id,
        }
    }

    pub fn lock_linear_generic(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointGenericVelocityConstraint {
        let lin_jac = self.basis.column(locked_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(locked_axis).into_owned();
        let ang_jac2 = self.cmat2_basis.column(locked_axis).into_owned();

        let mut c = self.lock_jacobians_generic(
            params,
            jacobians,
            j_id,
            joint_id,
            body1,
            body2,
            mb1,
            mb2,
            writeback_id,
            lin_jac,
            ang_jac1,
            ang_jac2,
        );

        let erp_inv_dt = params.joint_erp_inv_dt();
        let rhs_bias = lin_jac.dot(&self.lin_err) * erp_inv_dt;
        c.rhs += rhs_bias;
        c
    }

    pub fn limit_linear_generic(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        limited_axis: usize,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointGenericVelocityConstraint {
        let lin_jac = self.basis.column(limited_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(limited_axis).into_owned();
        let ang_jac2 = self.cmat2_basis.column(limited_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic(
            params,
            jacobians,
            j_id,
            joint_id,
            body1,
            body2,
            mb1,
            mb2,
            writeback_id,
            lin_jac,
            ang_jac1,
            ang_jac2,
        );

        let dist = self.lin_err.dot(&lin_jac);
        let min_enabled = dist < limits[0];
        let max_enabled = limits[1] < dist;

        let erp_inv_dt = params.joint_erp_inv_dt();
        let rhs_bias = ((dist - limits[1]).max(0.0) - (limits[0] - dist).max(0.0)) * erp_inv_dt;
        constraint.rhs += rhs_bias;
        constraint.impulse_bounds = [
            min_enabled as u32 as Real * -Real::MAX,
            max_enabled as u32 as Real * Real::MAX,
        ];

        constraint
    }

    pub fn motor_linear_generic(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        motor_axis: usize,
        motor_params: &MotorParameters<Real>,
        writeback_id: WritebackId,
    ) -> JointGenericVelocityConstraint {
        let lin_jac = self.basis.column(motor_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(motor_axis).into_owned();
        let ang_jac2 = self.cmat2_basis.column(motor_axis).into_owned();

        // TODO: do we need the same trick as for the non-generic constraint?
        // if locked_ang_axes & (1 << motor_axis) != 0 {
        //     // FIXME: check that this also works for cases
        //     // whene not all the angular axes are locked.
        //     constraint.ang_jac1.fill(0.0);
        //     constraint.ang_jac2.fill(0.0);
        // }

        let mut constraint = self.lock_jacobians_generic(
            params,
            jacobians,
            j_id,
            joint_id,
            body1,
            body2,
            mb1,
            mb2,
            writeback_id,
            lin_jac,
            ang_jac1,
            ang_jac2,
        );

        let mut rhs_wo_bias = 0.0;
        if motor_params.erp_inv_dt != 0.0 {
            let dist = self.lin_err.dot(&lin_jac);
            rhs_wo_bias += (dist - motor_params.target_pos) * motor_params.erp_inv_dt;
        }

        let dvel = lin_jac.dot(&(body2.linvel - body1.linvel))
            + (ang_jac2.gdot(body2.angvel) - ang_jac1.gdot(body1.angvel));
        rhs_wo_bias += dvel - motor_params.target_vel;

        constraint.impulse_bounds = [-motor_params.max_impulse, motor_params.max_impulse];
        constraint.rhs = rhs_wo_bias;
        constraint.rhs_wo_bias = rhs_wo_bias;
        constraint.cfm_coeff = motor_params.cfm_coeff;
        constraint.cfm_gain = motor_params.cfm_gain;
        constraint
    }

    pub fn lock_angular_generic(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointGenericVelocityConstraint {
        let ang_jac = self.ang_basis.column(locked_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic(
            params,
            jacobians,
            j_id,
            joint_id,
            body1,
            body2,
            mb1,
            mb2,
            writeback_id,
            na::zero(),
            ang_jac,
            ang_jac,
        );

        let erp_inv_dt = params.joint_erp_inv_dt();
        #[cfg(feature = "dim2")]
        let rhs_bias = self.ang_err.im * erp_inv_dt;
        #[cfg(feature = "dim3")]
        let rhs_bias = self.ang_err.imag()[locked_axis] * erp_inv_dt;
        constraint.rhs += rhs_bias;
        constraint
    }

    pub fn limit_angular_generic(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        limited_axis: usize,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointGenericVelocityConstraint {
        let ang_jac = self.ang_basis.column(limited_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic(
            params,
            jacobians,
            j_id,
            joint_id,
            body1,
            body2,
            mb1,
            mb2,
            writeback_id,
            na::zero(),
            ang_jac,
            ang_jac,
        );

        let s_limits = [(limits[0] / 2.0).sin(), (limits[1] / 2.0).sin()];
        #[cfg(feature = "dim2")]
        let s_ang = self.ang_err.im;
        #[cfg(feature = "dim3")]
        let s_ang = self.ang_err.imag()[limited_axis];
        let min_enabled = s_ang < s_limits[0];
        let max_enabled = s_limits[1] < s_ang;
        let impulse_bounds = [
            min_enabled as u32 as Real * -Real::MAX,
            max_enabled as u32 as Real * Real::MAX,
        ];

        let erp_inv_dt = params.joint_erp_inv_dt();
        let rhs_bias =
            ((s_ang - s_limits[1]).max(0.0) - (s_limits[0] - s_ang).max(0.0)) * erp_inv_dt;

        constraint.rhs += rhs_bias;
        constraint.impulse_bounds = impulse_bounds;
        constraint
    }

    pub fn motor_angular_generic(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        _motor_axis: usize,
        motor_params: &MotorParameters<Real>,
        writeback_id: WritebackId,
    ) -> JointGenericVelocityConstraint {
        // let mut ang_jac = self.ang_basis.column(motor_axis).into_owned();
        #[cfg(feature = "dim2")]
        let ang_jac = na::Vector1::new(1.0);
        #[cfg(feature = "dim3")]
        let ang_jac = self.basis.column(_motor_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic(
            params,
            jacobians,
            j_id,
            joint_id,
            body1,
            body2,
            mb1,
            mb2,
            writeback_id,
            na::zero(),
            ang_jac,
            ang_jac,
        );

        let mut rhs_wo_bias = 0.0;
        if motor_params.erp_inv_dt != 0.0 {
            #[cfg(feature = "dim2")]
            let s_ang_dist = self.ang_err.im;
            #[cfg(feature = "dim3")]
            let s_ang_dist = self.ang_err.imag()[_motor_axis];
            let s_target_ang = motor_params.target_pos.sin();
            rhs_wo_bias += (s_ang_dist - s_target_ang) * motor_params.erp_inv_dt;
        }

        let dvel = ang_jac.gdot(body2.angvel) - ang_jac.gdot(body1.angvel);
        rhs_wo_bias += dvel - motor_params.target_vel;

        constraint.rhs_wo_bias = rhs_wo_bias;
        constraint.rhs = rhs_wo_bias;
        constraint.cfm_coeff = motor_params.cfm_coeff;
        constraint.cfm_gain = motor_params.cfm_gain;
        constraint.impulse_bounds = [-motor_params.max_impulse, motor_params.max_impulse];
        constraint
    }

    pub fn finalize_generic_constraints(
        jacobians: &mut DVector<Real>,
        constraints: &mut [JointGenericVelocityConstraint],
    ) {
        // TODO: orthogonalization doesn’t seem to give good results for multibodies?
        const ORTHOGONALIZE: bool = false;
        let len = constraints.len();

        if len == 0 {
            return;
        }

        let ndofs1 = constraints[0].ndofs1;
        let ndofs2 = constraints[0].ndofs2;

        // Use the modified Gramm-Schmidt orthogonalization.
        for j in 0..len {
            let c_j = &mut constraints[j];

            let jac_j1 = jacobians.rows(c_j.j_id1, ndofs1);
            let jac_j2 = jacobians.rows(c_j.j_id2, ndofs2);
            let w_jac_j1 = jacobians.rows(c_j.j_id1 + ndofs1, ndofs1);
            let w_jac_j2 = jacobians.rows(c_j.j_id2 + ndofs2, ndofs2);

            let dot_jj = jac_j1.dot(&w_jac_j1) + jac_j2.dot(&w_jac_j2);
            let cfm_gain = dot_jj * c_j.cfm_coeff + c_j.cfm_gain;
            let inv_dot_jj = crate::utils::simd_inv(dot_jj);
            c_j.inv_lhs = crate::utils::simd_inv(dot_jj + cfm_gain); // Don’t forget to update the inv_lhs.
            c_j.cfm_gain = cfm_gain;

            if c_j.impulse_bounds != [-Real::MAX, Real::MAX] {
                // Don't remove constraints with limited forces from the others
                // because they may not deliver the necessary forces to fulfill
                // the removed parts of other constraints.
                continue;
            }

            if !ORTHOGONALIZE {
                continue;
            }

            for i in (j + 1)..len {
                let (c_i, c_j) = constraints.index_mut_const(i, j);

                let jac_i1 = jacobians.rows(c_i.j_id1, ndofs1);
                let jac_i2 = jacobians.rows(c_i.j_id2, ndofs2);
                let w_jac_j1 = jacobians.rows(c_j.j_id1 + ndofs1, ndofs1);
                let w_jac_j2 = jacobians.rows(c_j.j_id2 + ndofs2, ndofs2);

                let dot_ij = jac_i1.dot(&w_jac_j1) + jac_i2.dot(&w_jac_j2);
                let coeff = dot_ij * inv_dot_jj;

                let (mut jac_i, jac_j) = jacobians.rows_range_pair_mut(
                    c_i.j_id1..c_i.j_id1 + 2 * (ndofs1 + ndofs2),
                    c_j.j_id1..c_j.j_id1 + 2 * (ndofs1 + ndofs2),
                );

                jac_i.axpy(-coeff, &jac_j, 1.0);

                c_i.rhs_wo_bias -= c_j.rhs_wo_bias * coeff;
                c_i.rhs -= c_j.rhs * coeff;
            }
        }
    }
}

impl JointVelocityConstraintBuilder<Real> {
    pub fn lock_jacobians_generic_ground(
        &self,
        _params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        (mb2, link_id2): (&Multibody, usize),
        writeback_id: WritebackId,
        lin_jac: Vector<Real>,
        ang_jac1: SVector<Real, ANG_DIM>,
        ang_jac2: SVector<Real, ANG_DIM>,
    ) -> JointGenericVelocityGroundConstraint {
        let ndofs2 = mb2.ndofs();

        let vel1 = lin_jac.dot(&body1.linvel) + ang_jac1.gdot(body1.angvel);

        let j_id2 = *j_id;
        let vel2 = mb2
            .fill_jacobians(link_id2, lin_jac, ang_jac2, j_id, jacobians)
            .1;
        let rhs_wo_bias = vel2 - vel1;

        let mj_lambda2 = mb2.solver_id;

        JointGenericVelocityGroundConstraint {
            mj_lambda2,
            ndofs2,
            j_id2,
            joint_id,
            impulse: 0.0,
            impulse_bounds: [-Real::MAX, Real::MAX],
            inv_lhs: 0.0,
            rhs: rhs_wo_bias,
            rhs_wo_bias,
            cfm_coeff: 0.0,
            cfm_gain: 0.0,
            writeback_id,
        }
    }

    pub fn lock_linear_generic_ground(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        mb2: (&Multibody, usize),
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointGenericVelocityGroundConstraint {
        let lin_jac = self.basis.column(locked_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(locked_axis).into_owned();
        let ang_jac2 = self.cmat2_basis.column(locked_axis).into_owned();

        let mut c = self.lock_jacobians_generic_ground(
            params,
            jacobians,
            j_id,
            joint_id,
            body1,
            mb2,
            writeback_id,
            lin_jac,
            ang_jac1,
            ang_jac2,
        );

        let erp_inv_dt = params.joint_erp_inv_dt();
        let rhs_bias = lin_jac.dot(&self.lin_err) * erp_inv_dt;
        c.rhs += rhs_bias;
        c
    }

    pub fn limit_linear_generic_ground(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        mb2: (&Multibody, usize),
        limited_axis: usize,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointGenericVelocityGroundConstraint {
        let lin_jac = self.basis.column(limited_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(limited_axis).into_owned();
        let ang_jac2 = self.cmat2_basis.column(limited_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic_ground(
            params,
            jacobians,
            j_id,
            joint_id,
            body1,
            mb2,
            writeback_id,
            lin_jac,
            ang_jac1,
            ang_jac2,
        );

        let dist = self.lin_err.dot(&lin_jac);
        let min_enabled = dist < limits[0];
        let max_enabled = limits[1] < dist;

        let erp_inv_dt = params.joint_erp_inv_dt();
        let rhs_bias = ((dist - limits[1]).max(0.0) - (limits[0] - dist).max(0.0)) * erp_inv_dt;
        constraint.rhs += rhs_bias;
        constraint.impulse_bounds = [
            min_enabled as u32 as Real * -Real::MAX,
            max_enabled as u32 as Real * Real::MAX,
        ];

        constraint
    }

    pub fn motor_linear_generic_ground(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>, // TODO: we shouldn’t need this.
        mb2: (&Multibody, usize),
        motor_axis: usize,
        motor_params: &MotorParameters<Real>,
        writeback_id: WritebackId,
    ) -> JointGenericVelocityGroundConstraint {
        let lin_jac = self.basis.column(motor_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(motor_axis).into_owned();
        let ang_jac2 = self.cmat2_basis.column(motor_axis).into_owned();

        // TODO: do we need the same trick as for the non-generic constraint?
        // if locked_ang_axes & (1 << motor_axis) != 0 {
        //     // FIXME: check that this also works for cases
        //     // whene not all the angular axes are locked.
        //     constraint.ang_jac1.fill(0.0);
        //     constraint.ang_jac2.fill(0.0);
        // }

        let mut constraint = self.lock_jacobians_generic_ground(
            params,
            jacobians,
            j_id,
            joint_id,
            body1,
            mb2,
            writeback_id,
            lin_jac,
            ang_jac1,
            ang_jac2,
        );

        let mut rhs_wo_bias = 0.0;
        if motor_params.erp_inv_dt != 0.0 {
            let dist = self.lin_err.dot(&lin_jac);
            rhs_wo_bias += (dist - motor_params.target_pos) * motor_params.erp_inv_dt;
        }

        let dvel = lin_jac.dot(&(body2.linvel - body1.linvel))
            + (ang_jac2.gdot(body2.angvel) - ang_jac1.gdot(body1.angvel));
        rhs_wo_bias += dvel - motor_params.target_vel;

        constraint.impulse_bounds = [-motor_params.max_impulse, motor_params.max_impulse];
        constraint.rhs = rhs_wo_bias;
        constraint.rhs_wo_bias = rhs_wo_bias;
        constraint.cfm_coeff = motor_params.cfm_coeff;
        constraint.cfm_gain = motor_params.cfm_gain;
        constraint
    }

    pub fn lock_angular_generic_ground(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        mb2: (&Multibody, usize),
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointGenericVelocityGroundConstraint {
        let ang_jac = self.ang_basis.column(locked_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic_ground(
            params,
            jacobians,
            j_id,
            joint_id,
            body1,
            mb2,
            writeback_id,
            na::zero(),
            ang_jac,
            ang_jac,
        );

        let erp_inv_dt = params.joint_erp_inv_dt();
        #[cfg(feature = "dim2")]
        let rhs_bias = self.ang_err.im * erp_inv_dt;
        #[cfg(feature = "dim3")]
        let rhs_bias = self.ang_err.imag()[locked_axis] * erp_inv_dt;
        constraint.rhs += rhs_bias;
        constraint
    }

    pub fn limit_angular_generic_ground(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        mb2: (&Multibody, usize),
        limited_axis: usize,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointGenericVelocityGroundConstraint {
        let ang_jac = self.ang_basis.column(limited_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic_ground(
            params,
            jacobians,
            j_id,
            joint_id,
            body1,
            mb2,
            writeback_id,
            na::zero(),
            ang_jac,
            ang_jac,
        );

        let s_limits = [(limits[0] / 2.0).sin(), (limits[1] / 2.0).sin()];
        #[cfg(feature = "dim2")]
        let s_ang = self.ang_err.im;
        #[cfg(feature = "dim3")]
        let s_ang = self.ang_err.imag()[limited_axis];
        let min_enabled = s_ang < s_limits[0];
        let max_enabled = s_limits[1] < s_ang;
        let impulse_bounds = [
            min_enabled as u32 as Real * -Real::MAX,
            max_enabled as u32 as Real * Real::MAX,
        ];

        let erp_inv_dt = params.joint_erp_inv_dt();
        let rhs_bias =
            ((s_ang - s_limits[1]).max(0.0) - (s_limits[0] - s_ang).max(0.0)) * erp_inv_dt;

        constraint.rhs += rhs_bias;
        constraint.impulse_bounds = impulse_bounds;
        constraint
    }

    pub fn motor_angular_generic_ground(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &SolverBody<Real, 1>,
        body2: &SolverBody<Real, 1>, // TODO: we shouldn’t need this.
        mb2: (&Multibody, usize),
        _motor_axis: usize,
        motor_params: &MotorParameters<Real>,
        writeback_id: WritebackId,
    ) -> JointGenericVelocityGroundConstraint {
        // let mut ang_jac = self.ang_basis.column(_motor_axis).into_owned();
        #[cfg(feature = "dim2")]
        let ang_jac = na::Vector1::new(1.0);
        #[cfg(feature = "dim3")]
        let ang_jac = self.basis.column(_motor_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic_ground(
            params,
            jacobians,
            j_id,
            joint_id,
            body1,
            mb2,
            writeback_id,
            na::zero(),
            ang_jac,
            ang_jac,
        );

        let mut rhs = 0.0;
        if motor_params.erp_inv_dt != 0.0 {
            #[cfg(feature = "dim2")]
            let s_ang_dist = self.ang_err.im;
            #[cfg(feature = "dim3")]
            let s_ang_dist = self.ang_err.imag()[_motor_axis];
            let s_target_ang = motor_params.target_pos.sin();
            rhs += (s_ang_dist - s_target_ang) * motor_params.erp_inv_dt;
        }

        let dvel = ang_jac.gdot(body2.angvel) - ang_jac.gdot(body1.angvel);
        rhs += dvel - motor_params.target_vel;

        constraint.rhs_wo_bias = rhs;
        constraint.rhs = rhs;
        constraint.cfm_coeff = motor_params.cfm_coeff;
        constraint.cfm_gain = motor_params.cfm_gain;
        constraint.impulse_bounds = [-motor_params.max_impulse, motor_params.max_impulse];
        constraint
    }

    pub fn finalize_generic_constraints_ground(
        jacobians: &mut DVector<Real>,
        constraints: &mut [JointGenericVelocityGroundConstraint],
    ) {
        // TODO: orthogonalization doesn’t seem to give good results for multibodies?
        const ORTHOGONALIZE: bool = false;
        let len = constraints.len();

        if len == 0 {
            return;
        }

        let ndofs2 = constraints[0].ndofs2;

        // Use the modified Gramm-Schmidt orthogonalization.
        for j in 0..len {
            let c_j = &mut constraints[j];

            let jac_j2 = jacobians.rows(c_j.j_id2, ndofs2);
            let w_jac_j2 = jacobians.rows(c_j.j_id2 + ndofs2, ndofs2);

            let dot_jj = jac_j2.dot(&w_jac_j2);
            let cfm_gain = dot_jj * c_j.cfm_coeff + c_j.cfm_gain;
            let inv_dot_jj = crate::utils::simd_inv(dot_jj);
            c_j.inv_lhs = crate::utils::simd_inv(dot_jj + cfm_gain); // Don’t forget to update the inv_lhs.
            c_j.cfm_gain = cfm_gain;

            if c_j.impulse_bounds != [-Real::MAX, Real::MAX] {
                // Don't remove constraints with limited forces from the others
                // because they may not deliver the necessary forces to fulfill
                // the removed parts of other constraints.
                continue;
            }

            if !ORTHOGONALIZE {
                continue;
            }

            for i in (j + 1)..len {
                let (c_i, c_j) = constraints.index_mut_const(i, j);

                let jac_i2 = jacobians.rows(c_i.j_id2, ndofs2);
                let w_jac_j2 = jacobians.rows(c_j.j_id2 + ndofs2, ndofs2);

                let dot_ij = jac_i2.dot(&w_jac_j2);
                let coeff = dot_ij * inv_dot_jj;

                let (mut jac_i, jac_j) = jacobians.rows_range_pair_mut(
                    c_i.j_id2..c_i.j_id2 + 2 * ndofs2,
                    c_j.j_id2..c_j.j_id2 + 2 * ndofs2,
                );

                jac_i.axpy(-coeff, &jac_j, 1.0);

                c_i.rhs_wo_bias -= c_j.rhs_wo_bias * coeff;
                c_i.rhs -= c_j.rhs * coeff;
            }
        }
    }
}
