use crate::dynamics::solver::MotorParameters;
use crate::dynamics::solver::joint_constraint::joint_generic_constraint::{
    JointGenericOneBodyConstraint, JointGenericTwoBodyConstraint,
};
use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::{
    JointFixedSolverBody, WritebackId,
};
use crate::dynamics::solver::joint_constraint::{JointSolverBody, JointTwoBodyConstraintHelper};
use crate::dynamics::{
    GenericJoint, ImpulseJoint, IntegrationParameters, JointIndex, Multibody, MultibodyJointSet,
    MultibodyLinkId, RigidBodySet,
};
use crate::math::{ANG_DIM, DIM, Real, SPATIAL_DIM, Vector};
use crate::utils;
use crate::utils::IndexMut2;
use crate::utils::SimdDot;
use na::{DVector, SVector};

use crate::dynamics::solver::ConstraintsCounts;
use crate::dynamics::solver::solver_body::SolverBody;
#[cfg(feature = "dim3")]
use crate::utils::SimdAngularInertia;
#[cfg(feature = "dim2")]
use na::Vector1;
use parry::math::Isometry;

#[derive(Copy, Clone)]
enum LinkOrBody {
    Link(MultibodyLinkId),
    Body(usize),
}

#[derive(Copy, Clone)]
pub struct JointGenericTwoBodyConstraintBuilder {
    link1: LinkOrBody,
    link2: LinkOrBody,
    joint_id: JointIndex,
    joint: GenericJoint,
    j_id: usize,
    // These are solver body for both joints, except that
    // the world_com is actually in local-space.
    local_body1: JointSolverBody<Real, 1>,
    local_body2: JointSolverBody<Real, 1>,
    multibodies_ndof: usize,
    constraint_id: usize,
}

impl JointGenericTwoBodyConstraintBuilder {
    pub fn invalid() -> Self {
        Self {
            link1: LinkOrBody::Body(usize::MAX),
            link2: LinkOrBody::Body(usize::MAX),
            joint_id: JointIndex::MAX,
            joint: GenericJoint::default(),
            j_id: usize::MAX,
            local_body1: JointSolverBody::invalid(),
            local_body2: JointSolverBody::invalid(),
            multibodies_ndof: usize::MAX,
            constraint_id: usize::MAX,
        }
    }

    pub fn generate(
        joint_id: JointIndex,
        joint: &ImpulseJoint,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        out_builder: &mut Self,
        j_id: &mut usize,
        jacobians: &mut DVector<Real>,
        out_constraint_id: &mut usize,
    ) {
        let starting_j_id = *j_id;
        let rb1 = &bodies[joint.body1];
        let rb2 = &bodies[joint.body2];

        let local_body1 = JointSolverBody {
            im: rb1.mprops.effective_inv_mass,
            sqrt_ii: rb1.mprops.effective_world_inv_inertia_sqrt,
            world_com: rb1.mprops.local_mprops.local_com,
            solver_vel: [rb1.ids.active_set_offset],
        };
        let local_body2 = JointSolverBody {
            im: rb2.mprops.effective_inv_mass,
            sqrt_ii: rb2.mprops.effective_world_inv_inertia_sqrt,
            world_com: rb2.mprops.local_mprops.local_com,
            solver_vel: [rb2.ids.active_set_offset],
        };

        let mut multibodies_ndof = 0;
        let link1 = match multibodies.rigid_body_link(joint.body1) {
            Some(link) => {
                multibodies_ndof += multibodies[link.multibody].ndofs();
                LinkOrBody::Link(*link)
            }
            None => {
                multibodies_ndof += SPATIAL_DIM;
                LinkOrBody::Body(rb2.ids.active_set_offset)
            }
        };
        let link2 = match multibodies.rigid_body_link(joint.body2) {
            Some(link) => {
                multibodies_ndof += multibodies[link.multibody].ndofs();
                LinkOrBody::Link(*link)
            }
            None => {
                multibodies_ndof += SPATIAL_DIM;
                LinkOrBody::Body(rb2.ids.active_set_offset)
            }
        };

        if multibodies_ndof == 0 {
            // Both multibodies are fixed, don’t generate any constraint.
            out_builder.multibodies_ndof = multibodies_ndof;
            return;
        }

        // For each solver contact we generate up to SPATIAL_DIM constraints, and each
        // constraints appends the multibodies jacobian and weighted jacobians.
        // Also note that for impulse_joints, the rigid-bodies will also add their jacobians
        // to the generic DVector.
        // TODO: is this count correct when we take both motors and limits into account?
        let required_jacobian_len = *j_id + multibodies_ndof * 2 * SPATIAL_DIM;

        // TODO: use a more precise increment.
        *j_id += multibodies_ndof * 2 * SPATIAL_DIM;

        if jacobians.nrows() < required_jacobian_len && !cfg!(feature = "parallel") {
            jacobians.resize_vertically_mut(required_jacobian_len, 0.0);
        }

        *out_builder = Self {
            link1,
            link2,
            joint_id,
            joint: joint.data,
            j_id: starting_j_id,
            local_body1,
            local_body2,
            multibodies_ndof,
            constraint_id: *out_constraint_id,
        };

        *out_constraint_id += ConstraintsCounts::from_joint(joint).num_constraints;
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        multibodies: &MultibodyJointSet,
        bodies: &[SolverBody],
        jacobians: &mut DVector<Real>,
        out: &mut [JointGenericTwoBodyConstraint],
    ) {
        if self.multibodies_ndof == 0 {
            // The joint is between two static bodies, no constraint needed.
            return;
        }

        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?
        let pos1;
        let pos2;
        let mb1;
        let mb2;

        match self.link1 {
            LinkOrBody::Link(link) => {
                let mb = &multibodies[link.multibody];
                pos1 = &mb.link(link.id).unwrap().local_to_world;
                mb1 = Some((mb, link.id));
            }
            LinkOrBody::Body(body1) => {
                pos1 = &bodies[body1].position;
                mb1 = None;
            }
        };
        match self.link2 {
            LinkOrBody::Link(link) => {
                let mb = &multibodies[link.multibody];
                pos2 = &mb.link(link.id).unwrap().local_to_world;
                mb2 = Some((mb, link.id));
            }
            LinkOrBody::Body(body2) => {
                pos2 = &bodies[body2].position;
                mb2 = None;
            }
        };

        let frame1 = pos1 * self.joint.local_frame1;
        let frame2 = pos2 * self.joint.local_frame2;

        let joint_body1 = JointSolverBody {
            world_com: pos1 * self.local_body1.world_com, // the world_com was stored in local-space.
            ..self.local_body1
        };
        let joint_body2 = JointSolverBody {
            world_com: pos2 * self.local_body2.world_com, // the world_com was stored in local-space.
            ..self.local_body2
        };

        let mut j_id = self.j_id;

        JointGenericTwoBodyConstraint::lock_axes(
            params,
            self.joint_id,
            &joint_body1,
            &joint_body2,
            mb1,
            mb2,
            &frame1,
            &frame2,
            &self.joint,
            jacobians,
            &mut j_id,
            &mut out[self.constraint_id..],
        );
    }
}

#[derive(Copy, Clone)]
#[allow(clippy::large_enum_variant)]
pub enum JointGenericOneBodyConstraintBuilder {
    Internal(JointGenericVelocityOneBodyInternalConstraintBuilder),
    External(JointGenericVelocityOneBodyExternalConstraintBuilder),
    Empty,
}

#[derive(Copy, Clone)]
pub struct JointGenericVelocityOneBodyInternalConstraintBuilder {
    link: MultibodyLinkId,
    j_id: usize,
    constraint_id: usize,
}

impl JointGenericOneBodyConstraintBuilder {
    pub fn invalid() -> Self {
        Self::Empty
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        multibodies: &MultibodyJointSet,
        bodies: &[SolverBody],
        jacobians: &mut DVector<Real>,
        out: &mut [JointGenericOneBodyConstraint],
    ) {
        match self {
            Self::Empty => {}
            Self::Internal(builder) => builder.update(params, multibodies, jacobians, out),
            Self::External(builder) => builder.update(params, multibodies, bodies, jacobians, out),
        }
    }
}

impl JointGenericVelocityOneBodyInternalConstraintBuilder {
    pub fn num_constraints(multibodies: &MultibodyJointSet, link_id: &MultibodyLinkId) -> usize {
        let multibody = &multibodies[link_id.multibody];
        let link = multibody.link(link_id.id).unwrap();
        link.joint().num_velocity_constraints()
    }

    pub fn generate(
        multibodies: &MultibodyJointSet,
        link_id: &MultibodyLinkId,
        out_builder: &mut JointGenericOneBodyConstraintBuilder,
        j_id: &mut usize,
        jacobians: &mut DVector<Real>,
        out_constraint_id: &mut usize,
    ) {
        let multibody = &multibodies[link_id.multibody];
        let link = multibody.link(link_id.id).unwrap();
        let num_constraints = link.joint().num_velocity_constraints();

        if num_constraints == 0 {
            return;
        }

        *out_builder = JointGenericOneBodyConstraintBuilder::Internal(Self {
            link: *link_id,
            j_id: *j_id,
            constraint_id: *out_constraint_id,
        });

        *j_id += num_constraints * multibody.ndofs() * 2;
        if jacobians.nrows() < *j_id {
            jacobians.resize_vertically_mut(*j_id, 0.0);
        }

        *out_constraint_id += num_constraints;
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        multibodies: &MultibodyJointSet,
        jacobians: &mut DVector<Real>,
        out: &mut [JointGenericOneBodyConstraint],
    ) {
        let mb = &multibodies[self.link.multibody];
        let link = mb.link(self.link.id).unwrap();
        link.joint().velocity_constraints(
            params,
            mb,
            link,
            self.j_id,
            jacobians,
            &mut out[self.constraint_id..],
        );
    }
}

#[derive(Copy, Clone)]
pub struct JointGenericVelocityOneBodyExternalConstraintBuilder {
    body1: JointFixedSolverBody<Real>,
    frame1: Isometry<Real>,
    link2: MultibodyLinkId,
    joint_id: JointIndex,
    joint: GenericJoint,
    j_id: usize,
    constraint_id: usize,
    // These are solver body for both joints, except that
    // the world_com is actually in local-space.
    local_body2: JointSolverBody<Real, 1>,
}

impl JointGenericVelocityOneBodyExternalConstraintBuilder {
    pub fn generate(
        joint_id: JointIndex,
        joint: &ImpulseJoint,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        out_builder: &mut JointGenericOneBodyConstraintBuilder,
        j_id: &mut usize,
        jacobians: &mut DVector<Real>,
        out_constraint_id: &mut usize,
    ) {
        let mut joint_data = joint.data;
        let mut handle1 = joint.body1;
        let mut handle2 = joint.body2;
        let flipped = !bodies[handle2].is_dynamic();

        if flipped {
            std::mem::swap(&mut handle1, &mut handle2);
            joint_data.flip();
        }

        let rb1 = &bodies[handle1];
        let rb2 = &bodies[handle2];

        let frame1 = rb1.pos.position * joint_data.local_frame1;

        let starting_j_id = *j_id;

        let body1 = JointFixedSolverBody {
            linvel: rb1.vels.linvel,
            angvel: rb1.vels.angvel,
            world_com: rb1.mprops.world_com,
        };
        let local_body2 = JointSolverBody {
            im: rb2.mprops.effective_inv_mass,
            sqrt_ii: rb2.mprops.effective_world_inv_inertia_sqrt,
            world_com: rb2.mprops.local_mprops.local_com,
            solver_vel: [rb2.ids.active_set_offset],
        };

        let link2 = *multibodies.rigid_body_link(handle2).unwrap();
        let mb2 = &multibodies[link2.multibody];
        let multibodies_ndof = mb2.ndofs();

        if multibodies_ndof == 0 {
            // The multibody is fixed, don’t generate any constraint.
            *out_builder = JointGenericOneBodyConstraintBuilder::Empty;
            return;
        }

        // For each solver contact we generate up to SPATIAL_DIM constraints, and each
        // constraints appends the multibodies jacobian and weighted jacobians.
        // Also note that for impulse_joints, the rigid-bodies will also add their jacobians
        // to the generic DVector.
        // TODO: is this count correct when we take both motors and limits into account?
        let required_jacobian_len = *j_id + multibodies_ndof * 2 * SPATIAL_DIM;
        // TODO: use a more precise increment.
        *j_id += multibodies_ndof * 2 * SPATIAL_DIM;

        if jacobians.nrows() < required_jacobian_len && !cfg!(feature = "parallel") {
            jacobians.resize_vertically_mut(required_jacobian_len, 0.0);
        }

        *out_builder = JointGenericOneBodyConstraintBuilder::External(Self {
            body1,
            link2,
            joint_id,
            joint: joint_data,
            j_id: starting_j_id,
            frame1,
            local_body2,
            constraint_id: *out_constraint_id,
        });

        *out_constraint_id += ConstraintsCounts::from_joint(joint).num_constraints;
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        multibodies: &MultibodyJointSet,
        _bodies: &[SolverBody],
        jacobians: &mut DVector<Real>,
        out: &mut [JointGenericOneBodyConstraint],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?
        let mb2 = &multibodies[self.link2.multibody];
        let pos2 = &mb2.link(self.link2.id).unwrap().local_to_world;
        let frame2 = pos2 * self.joint.local_frame2;

        let joint_body2 = JointSolverBody {
            world_com: pos2 * self.local_body2.world_com, // the world_com was stored in local-space.
            ..self.local_body2
        };

        let mut j_id = self.j_id;

        JointGenericOneBodyConstraint::lock_axes(
            params,
            self.joint_id,
            &self.body1,
            &joint_body2,
            (mb2, self.link2.id),
            &self.frame1,
            &frame2,
            &self.joint,
            jacobians,
            &mut j_id,
            &mut out[self.constraint_id..],
        );
    }
}

impl JointSolverBody<Real, 1> {
    pub fn fill_jacobians(
        &self,
        unit_force: Vector<Real>,
        unit_torque: SVector<Real, ANG_DIM>,
        j_id: &mut usize,
        jacobians: &mut DVector<Real>,
    ) {
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
    }
}

impl JointTwoBodyConstraintHelper<Real> {
    pub fn lock_jacobians_generic(
        &self,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &JointSolverBody<Real, 1>,
        body2: &JointSolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        writeback_id: WritebackId,
        lin_jac: Vector<Real>,
        ang_jac1: SVector<Real, ANG_DIM>,
        ang_jac2: SVector<Real, ANG_DIM>,
    ) -> JointGenericTwoBodyConstraint {
        let is_rigid_body1 = mb1.is_none();
        let is_rigid_body2 = mb2.is_none();

        let ndofs1 = mb1.map(|(m, _)| m.ndofs()).unwrap_or(SPATIAL_DIM);
        let ndofs2 = mb2.map(|(m, _)| m.ndofs()).unwrap_or(SPATIAL_DIM);

        let j_id1 = *j_id;
        if let Some((mb1, link_id1)) = mb1 {
            mb1.fill_jacobians(link_id1, lin_jac, ang_jac1, j_id, jacobians);
        } else {
            body1.fill_jacobians(lin_jac, ang_jac1, j_id, jacobians);
        };

        let j_id2 = *j_id;
        if let Some((mb2, link_id2)) = mb2 {
            mb2.fill_jacobians(link_id2, lin_jac, ang_jac2, j_id, jacobians);
        } else {
            body2.fill_jacobians(lin_jac, ang_jac2, j_id, jacobians);
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

        let rhs_wo_bias = 0.0;

        let solver_vel1 = mb1.map(|m| m.0.solver_id).unwrap_or(body1.solver_vel[0]);
        let solver_vel2 = mb2.map(|m| m.0.solver_id).unwrap_or(body2.solver_vel[0]);

        JointGenericTwoBodyConstraint {
            is_rigid_body1,
            is_rigid_body2,
            solver_vel1,
            solver_vel2,
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
        body1: &JointSolverBody<Real, 1>,
        body2: &JointSolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointGenericTwoBodyConstraint {
        let lin_jac = self.basis.column(locked_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(locked_axis).into_owned();
        let ang_jac2 = self.cmat2_basis.column(locked_axis).into_owned();

        let mut c = self.lock_jacobians_generic(
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
        body1: &JointSolverBody<Real, 1>,
        body2: &JointSolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        limited_axis: usize,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointGenericTwoBodyConstraint {
        let lin_jac = self.basis.column(limited_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(limited_axis).into_owned();
        let ang_jac2 = self.cmat2_basis.column(limited_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic(
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
        let min_enabled = dist <= limits[0];
        let max_enabled = limits[1] <= dist;

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
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &JointSolverBody<Real, 1>,
        body2: &JointSolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        motor_axis: usize,
        motor_params: &MotorParameters<Real>,
        writeback_id: WritebackId,
    ) -> JointGenericTwoBodyConstraint {
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

        rhs_wo_bias += -motor_params.target_vel;

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
        body1: &JointSolverBody<Real, 1>,
        body2: &JointSolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        _locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointGenericTwoBodyConstraint {
        #[cfg(feature = "dim2")]
        let ang_jac = Vector1::new(1.0);
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(_locked_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic(
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
        let rhs_bias = self.ang_err.imag()[_locked_axis] * erp_inv_dt;
        constraint.rhs += rhs_bias;
        constraint
    }

    pub fn limit_angular_generic(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &JointSolverBody<Real, 1>,
        body2: &JointSolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        _limited_axis: usize,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointGenericTwoBodyConstraint {
        #[cfg(feature = "dim2")]
        let ang_jac = Vector1::new(1.0);
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(_limited_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic(
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
        let s_ang = (self.ang_err.angle() / 2.0).sin();
        #[cfg(feature = "dim3")]
        let s_ang = self.ang_err.imag()[_limited_axis];
        let min_enabled = s_ang <= s_limits[0];
        let max_enabled = s_limits[1] <= s_ang;
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
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &JointSolverBody<Real, 1>,
        body2: &JointSolverBody<Real, 1>,
        mb1: Option<(&Multibody, usize)>,
        mb2: Option<(&Multibody, usize)>,
        _motor_axis: usize,
        motor_params: &MotorParameters<Real>,
        writeback_id: WritebackId,
    ) -> JointGenericTwoBodyConstraint {
        #[cfg(feature = "dim2")]
        let ang_jac = na::Vector1::new(1.0);
        #[cfg(feature = "dim3")]
        let ang_jac = self.basis.column(_motor_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic(
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
            let s_ang_dist = (self.ang_err.angle() / 2.0).sin();
            #[cfg(feature = "dim3")]
            let s_ang_dist = self.ang_err.imag()[_motor_axis];
            let s_target_ang = (motor_params.target_pos / 2.0).sin();
            rhs_wo_bias += utils::smallest_abs_diff_between_sin_angles(s_ang_dist, s_target_ang)
                * motor_params.erp_inv_dt;
        }

        rhs_wo_bias += -motor_params.target_vel;

        constraint.rhs_wo_bias = rhs_wo_bias;
        constraint.rhs = rhs_wo_bias;
        constraint.cfm_coeff = motor_params.cfm_coeff;
        constraint.cfm_gain = motor_params.cfm_gain;
        constraint.impulse_bounds = [-motor_params.max_impulse, motor_params.max_impulse];
        constraint
    }

    pub fn finalize_generic_constraints(
        jacobians: &mut DVector<Real>,
        constraints: &mut [JointGenericTwoBodyConstraint],
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

impl JointTwoBodyConstraintHelper<Real> {
    pub fn lock_jacobians_generic_one_body(
        &self,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &JointFixedSolverBody<Real>,
        (mb2, link_id2): (&Multibody, usize),
        writeback_id: WritebackId,
        lin_jac: Vector<Real>,
        ang_jac1: SVector<Real, ANG_DIM>,
        ang_jac2: SVector<Real, ANG_DIM>,
    ) -> JointGenericOneBodyConstraint {
        let ndofs2 = mb2.ndofs();

        let proj_vel1 = lin_jac.dot(&body1.linvel) + ang_jac1.gdot(body1.angvel);
        let j_id2 = *j_id;
        mb2.fill_jacobians(link_id2, lin_jac, ang_jac2, j_id, jacobians);
        let rhs_wo_bias = -proj_vel1;

        let solver_vel2 = mb2.solver_id;

        JointGenericOneBodyConstraint {
            solver_vel2,
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

    pub fn lock_linear_generic_one_body(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &JointFixedSolverBody<Real>,
        mb2: (&Multibody, usize),
        locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointGenericOneBodyConstraint {
        let lin_jac = self.basis.column(locked_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(locked_axis).into_owned();
        let ang_jac2 = self.cmat2_basis.column(locked_axis).into_owned();

        let mut c = self.lock_jacobians_generic_one_body(
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

    pub fn limit_linear_generic_one_body(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &JointFixedSolverBody<Real>,
        mb2: (&Multibody, usize),
        limited_axis: usize,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointGenericOneBodyConstraint {
        let lin_jac = self.basis.column(limited_axis).into_owned();
        let ang_jac1 = self.cmat1_basis.column(limited_axis).into_owned();
        let ang_jac2 = self.cmat2_basis.column(limited_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic_one_body(
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
        let min_enabled = dist <= limits[0];
        let max_enabled = limits[1] <= dist;

        let erp_inv_dt = params.joint_erp_inv_dt();
        let rhs_bias = ((dist - limits[1]).max(0.0) - (limits[0] - dist).max(0.0)) * erp_inv_dt;
        constraint.rhs += rhs_bias;
        constraint.impulse_bounds = [
            min_enabled as u32 as Real * -Real::MAX,
            max_enabled as u32 as Real * Real::MAX,
        ];

        constraint
    }

    pub fn motor_linear_generic_one_body(
        &self,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &JointFixedSolverBody<Real>,
        mb2: (&Multibody, usize),
        motor_axis: usize,
        motor_params: &MotorParameters<Real>,
        writeback_id: WritebackId,
    ) -> JointGenericOneBodyConstraint {
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

        let mut constraint = self.lock_jacobians_generic_one_body(
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

        let proj_vel1 = -lin_jac.dot(&body1.linvel) - ang_jac1.gdot(body1.angvel);
        rhs_wo_bias += proj_vel1 - motor_params.target_vel;

        constraint.impulse_bounds = [-motor_params.max_impulse, motor_params.max_impulse];
        constraint.rhs = rhs_wo_bias;
        constraint.rhs_wo_bias = rhs_wo_bias;
        constraint.cfm_coeff = motor_params.cfm_coeff;
        constraint.cfm_gain = motor_params.cfm_gain;
        constraint
    }

    pub fn lock_angular_generic_one_body(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &JointFixedSolverBody<Real>,
        mb2: (&Multibody, usize),
        _locked_axis: usize,
        writeback_id: WritebackId,
    ) -> JointGenericOneBodyConstraint {
        #[cfg(feature = "dim2")]
        let ang_jac = Vector1::new(1.0);
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(_locked_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic_one_body(
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
        let rhs_bias = self.ang_err.imag()[_locked_axis] * erp_inv_dt;
        constraint.rhs += rhs_bias;
        constraint
    }

    pub fn limit_angular_generic_one_body(
        &self,
        params: &IntegrationParameters,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &JointFixedSolverBody<Real>,
        mb2: (&Multibody, usize),
        _limited_axis: usize,
        limits: [Real; 2],
        writeback_id: WritebackId,
    ) -> JointGenericOneBodyConstraint {
        #[cfg(feature = "dim2")]
        let ang_jac = Vector1::new(1.0);
        #[cfg(feature = "dim3")]
        let ang_jac = self.ang_basis.column(_limited_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic_one_body(
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
        let s_ang = (self.ang_err.angle() / 2.0).sin();
        #[cfg(feature = "dim3")]
        let s_ang = self.ang_err.imag()[_limited_axis];
        let min_enabled = s_ang <= s_limits[0];
        let max_enabled = s_limits[1] <= s_ang;
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

    pub fn motor_angular_generic_one_body(
        &self,
        jacobians: &mut DVector<Real>,
        j_id: &mut usize,
        joint_id: JointIndex,
        body1: &JointFixedSolverBody<Real>,
        mb2: (&Multibody, usize),
        _motor_axis: usize,
        motor_params: &MotorParameters<Real>,
        writeback_id: WritebackId,
    ) -> JointGenericOneBodyConstraint {
        #[cfg(feature = "dim2")]
        let ang_jac = na::Vector1::new(1.0);
        #[cfg(feature = "dim3")]
        let ang_jac = self.basis.column(_motor_axis).into_owned();

        let mut constraint = self.lock_jacobians_generic_one_body(
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
            let s_ang_dist = (self.ang_err.angle() / 2.0).sin();
            #[cfg(feature = "dim3")]
            let s_ang_dist = self.ang_err.imag()[_motor_axis];
            let s_target_ang = (motor_params.target_pos / 2.0).sin();
            rhs += utils::smallest_abs_diff_between_sin_angles(s_ang_dist, s_target_ang)
                * motor_params.erp_inv_dt;
        }

        let proj_vel1 = -ang_jac.gdot(body1.angvel);
        rhs += proj_vel1 - motor_params.target_vel;

        constraint.rhs_wo_bias = rhs;
        constraint.rhs = rhs;
        constraint.cfm_coeff = motor_params.cfm_coeff;
        constraint.cfm_gain = motor_params.cfm_gain;
        constraint.impulse_bounds = [-motor_params.max_impulse, motor_params.max_impulse];
        constraint
    }

    pub fn finalize_generic_constraints_one_body(
        jacobians: &mut DVector<Real>,
        constraints: &mut [JointGenericOneBodyConstraint],
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
