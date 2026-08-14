//! Staged-solver entry points on [`ContactConstraintsSet`] and
//! [`VelocitySolver`]: generic (multibody) constraint init and the
//! multibody-only halves of buffer init and position integration.

use crate::dynamics::solver::SolverVel;
use crate::dynamics::solver::VelocitySolver;
use crate::dynamics::solver::contact_constraint::{
    ContactConstraintsSet, GenericContactConstraint, GenericContactConstraintBuilder,
};
use crate::dynamics::solver::manifold_store::ManifoldStore;
use crate::dynamics::solver::solver_contact_graph::SolverContactGraph;
#[cfg(feature = "dim3")]
use crate::dynamics::solver::velocity_solver::GyroParams;
use crate::dynamics::{IntegrationParameters, MultibodyJointSet, RigidBodyHandle, RigidBodySet};
use parry::math::SIMD_WIDTH;

impl ContactConstraintsSet {
    /// Builds the generic (multibody) contact constraints for the staged solver.
    pub(super) fn compute_generic_constraints(
        &mut self,
        bodies: &RigidBodySet,
        multibody_joints: &MultibodyJointSet,
        graph: &SolverContactGraph,
        store: &ManifoldStore,
        jacobian_id: &mut usize,
    ) {
        let generic = graph.generic();
        let total_num_constraints = generic.len();
        self.generic_velocity_constraints_builder.resize(
            total_num_constraints,
            GenericContactConstraintBuilder::invalid(),
        );
        self.generic_velocity_constraints
            .resize(total_num_constraints, GenericContactConstraint::invalid());

        for (curr_id, r) in generic.iter().enumerate() {
            let manifold = store.get(*r);
            GenericContactConstraintBuilder::generate(
                *r,
                manifold,
                bodies,
                multibody_joints,
                &mut self.generic_velocity_constraints_builder[curr_id],
                &mut self.generic_velocity_constraints[curr_id],
                &mut self.generic_jacobians,
                jacobian_id,
            );
        }
    }
}

impl VelocitySolver {
    /// Initializes the solver buffers and multibody solver state; the plain rigid-body
    /// initialization is excluded, the staged solver performs it in parallel in its first stage.
    pub(super) fn init_solver_buffers_and_multibodies(
        &mut self,
        params: &IntegrationParameters,
        island_bodies: &[RigidBodyHandle],
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
    ) {
        self.multibody_roots.clear();
        self.solver_bodies.clear();

        let aligned_solver_bodies_len = island_bodies.len().div_ceil(SIMD_WIDTH) * SIMD_WIDTH;
        self.solver_bodies.resize(aligned_solver_bodies_len);

        self.solver_vels_increment.clear();
        self.solver_vels_increment
            .resize(aligned_solver_bodies_len, SolverVel::zero());

        // Reset every step so multibody/padding slots (which the body-copy stage
        // skips) read as gyro-disabled.
        #[cfg(feature = "dim3")]
        {
            self.solver_gyro.clear();
            self.solver_gyro
                .resize(aligned_solver_bodies_len, GyroParams::default());
        }

        // Assign solver ids to multibodies, and collect the relevant roots.
        let mut multibody_solver_id = 0;
        for handle in island_bodies {
            if let Some(link) = multibodies.rigid_body_link(*handle).copied() {
                let multibody = multibodies
                    .get_multibody_mut_internal(link.multibody)
                    .unwrap();

                if link.id == 0 || link.id == 1 && !multibody.root_is_dynamic {
                    multibody.solver_id = multibody_solver_id;
                    multibody_solver_id += multibody.ndofs() as u32;
                    self.multibody_roots.push(link);
                }
            }
        }

        // Resize in place and zero the rows instead of allocating two fresh
        // vectors every step (reallocation only happens when the total DoF
        // count grows).
        let total_ndofs = multibody_solver_id as usize;
        self.generic_solver_vels_increment
            .resize_vertically_mut(total_ndofs, 0.0);
        self.generic_solver_vels_increment.fill(0.0);
        self.generic_solver_vels
            .resize_vertically_mut(total_ndofs, 0.0);
        self.generic_solver_vels.fill(0.0);

        for link in &self.multibody_roots {
            let multibody = multibodies
                .get_multibody_mut_internal(link.multibody)
                .unwrap();
            multibody.update_velocities(bodies);
            multibody.update_mass_matrix(params.dt, bodies);
            multibody.update_acceleration(params.dt, bodies);

            let mut solver_vels_incr = self
                .generic_solver_vels_increment
                .rows_mut(multibody.solver_id as usize, multibody.ndofs());
            let mut solver_vels = self
                .generic_solver_vels
                .rows_mut(multibody.solver_id as usize, multibody.ndofs());

            solver_vels_incr.axpy(params.dt, &multibody.accelerations, 0.0);
            solver_vels.copy_from(&multibody.velocities);
        }
    }

    /// The multibody part of `integrate_positions`, used by the staged solver
    /// (regular solver bodies are integrated in parallel by the workers).
    pub(super) fn integrate_multibody_positions(
        &mut self,
        params: &IntegrationParameters,
        is_last_substep: bool,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
    ) {
        for link in &self.multibody_roots {
            let multibody = multibodies
                .get_multibody_mut_internal(link.multibody)
                .unwrap();
            let solver_vels = self
                .generic_solver_vels
                .rows(multibody.solver_id as usize, multibody.ndofs());
            multibody.velocities.copy_from(&solver_vels);
            multibody.integrate(params.dt);
            multibody.forward_kinematics(bodies, false);
            multibody.update_rigid_bodies_internal(bodies, !is_last_substep, true, false);

            if !is_last_substep {
                multibody.update_velocities(bodies);
                multibody.update_mass_matrix(params.dt, bodies);
                multibody.update_acceleration(params.dt, bodies);

                let mut solver_vels_incr = self
                    .generic_solver_vels_increment
                    .rows_mut(multibody.solver_id as usize, multibody.ndofs());
                solver_vels_incr.axpy(params.dt, &multibody.accelerations, 0.0);
            }
        }
    }
}
