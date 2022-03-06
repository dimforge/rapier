#![allow(missing_docs)] // For downcast.

use crate::dynamics::joint::MultibodyLink;
use crate::dynamics::solver::{
    AnyJointVelocityConstraint, JointGenericVelocityGroundConstraint, WritebackId,
};
use crate::dynamics::{IntegrationParameters, JointMotor, Multibody};
use crate::math::Real;
use na::DVector;

/// Initializes and generate the velocity constraints applicable to the multibody links attached
/// to this multibody_joint.
pub fn unit_joint_limit_constraint(
    params: &IntegrationParameters,
    multibody: &Multibody,
    link: &MultibodyLink,
    limits: [Real; 2],
    curr_pos: Real,
    dof_id: usize,
    j_id: &mut usize,
    jacobians: &mut DVector<Real>,
    constraints: &mut Vec<AnyJointVelocityConstraint>,
    insert_at: &mut Option<usize>,
) {
    let ndofs = multibody.ndofs();
    let joint_velocity = multibody.joint_velocity(link);

    let min_enabled = curr_pos < limits[0];
    let max_enabled = limits[1] < curr_pos;
    let erp_inv_dt = params.joint_erp_inv_dt();
    let cfm_coeff = params.joint_cfm_coeff();
    let rhs_bias = ((curr_pos - limits[1]).max(0.0) - (limits[0] - curr_pos).max(0.0)) * erp_inv_dt;
    let rhs_wo_bias = joint_velocity[dof_id];

    let dof_j_id = *j_id + dof_id + link.assembly_id;
    jacobians.rows_mut(*j_id, ndofs * 2).fill(0.0);
    jacobians[dof_j_id] = 1.0;
    jacobians[dof_j_id + ndofs] = 1.0;
    multibody
        .inv_augmented_mass()
        .solve_mut(&mut jacobians.rows_mut(*j_id + ndofs, ndofs));

    let lhs = jacobians[dof_j_id + ndofs]; // = J^t * M^-1 J
    let impulse_bounds = [
        min_enabled as u32 as Real * -Real::MAX,
        max_enabled as u32 as Real * Real::MAX,
    ];

    let constraint = JointGenericVelocityGroundConstraint {
        mj_lambda2: multibody.solver_id,
        ndofs2: ndofs,
        j_id2: *j_id,
        joint_id: usize::MAX,
        impulse: 0.0,
        impulse_bounds,
        inv_lhs: crate::utils::inv(lhs),
        rhs: rhs_wo_bias + rhs_bias,
        rhs_wo_bias,
        cfm_coeff,
        cfm_gain: 0.0,
        writeback_id: WritebackId::Limit(dof_id),
    };

    if let Some(at) = insert_at {
        constraints[*at] = AnyJointVelocityConstraint::JointGenericGroundConstraint(constraint);
        *at += 1;
    } else {
        constraints.push(AnyJointVelocityConstraint::JointGenericGroundConstraint(
            constraint,
        ));
    }
    *j_id += 2 * ndofs;
}

/// Initializes and generate the velocity constraints applicable to the multibody links attached
/// to this multibody_joint.
pub fn unit_joint_motor_constraint(
    params: &IntegrationParameters,
    multibody: &Multibody,
    link: &MultibodyLink,
    motor: &JointMotor,
    curr_pos: Real,
    limits: Option<[Real; 2]>,
    dof_id: usize,
    j_id: &mut usize,
    jacobians: &mut DVector<Real>,
    constraints: &mut Vec<AnyJointVelocityConstraint>,
    insert_at: &mut Option<usize>,
) {
    let inv_dt = params.inv_dt();
    let ndofs = multibody.ndofs();
    let joint_velocity = multibody.joint_velocity(link);

    let motor_params = motor.motor_params(params.dt);

    let dof_j_id = *j_id + dof_id + link.assembly_id;
    jacobians.rows_mut(*j_id, ndofs * 2).fill(0.0);
    jacobians[dof_j_id] = 1.0;
    jacobians[dof_j_id + ndofs] = 1.0;
    multibody
        .inv_augmented_mass()
        .solve_mut(&mut jacobians.rows_mut(*j_id + ndofs, ndofs));

    let lhs = jacobians[dof_j_id + ndofs]; // = J^t * M^-1 J
    let impulse_bounds = [-motor_params.max_impulse, motor_params.max_impulse];

    let mut rhs_wo_bias = 0.0;
    if motor_params.erp_inv_dt != 0.0 {
        rhs_wo_bias += (curr_pos - motor_params.target_pos) * motor_params.erp_inv_dt;
    }

    let mut target_vel = motor_params.target_vel;
    if let Some(limits) = limits {
        target_vel = target_vel.clamp(
            (limits[0] - curr_pos) * inv_dt,
            (limits[1] - curr_pos) * inv_dt,
        );
    };

    let dvel = joint_velocity[dof_id];
    rhs_wo_bias += dvel - target_vel;

    let constraint = JointGenericVelocityGroundConstraint {
        mj_lambda2: multibody.solver_id,
        ndofs2: ndofs,
        j_id2: *j_id,
        joint_id: usize::MAX,
        impulse: 0.0,
        impulse_bounds,
        cfm_coeff: motor_params.cfm_coeff,
        cfm_gain: motor_params.cfm_gain,
        inv_lhs: crate::utils::inv(lhs),
        rhs: rhs_wo_bias,
        rhs_wo_bias,
        writeback_id: WritebackId::Limit(dof_id),
    };

    if let Some(at) = insert_at {
        constraints[*at] = AnyJointVelocityConstraint::JointGenericGroundConstraint(constraint);
        *at += 1;
    } else {
        constraints.push(AnyJointVelocityConstraint::JointGenericGroundConstraint(
            constraint,
        ));
    }
    *j_id += 2 * ndofs;
}
