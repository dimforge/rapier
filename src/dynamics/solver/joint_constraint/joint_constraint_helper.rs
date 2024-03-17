pub(self) mod math {
    pub use crate::math::*;
    pub type RealConst = Real;

    pub const LANES: usize = 1;
}

#[path = "./joint_constraint_helper_template.rs"]
mod joint_constraint_helper_template;

use crate::utils::{SimdBasis, SimdDot};
pub use joint_constraint_helper_template::JointConstraintHelperTemplate as JointConstraintHelper;

use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::{
    JointFixedSolverBody, JointOneBodyConstraint, JointTwoBodyConstraint, WritebackId,
};
use crate::dynamics::solver::joint_constraint::JointSolverBody;
use crate::dynamics::{IntegrationParameters, JointIndex};
use crate::math::*;

impl JointConstraintHelper {
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
            lin_jac: Default::default(),
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
            lin_jac: Default::default(),
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
