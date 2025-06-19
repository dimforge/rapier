use crate::dynamics::solver::categorization::categorize_joints;
use crate::dynamics::solver::solver_body::SolverBody;
use crate::dynamics::solver::solver_vel::SolverVel;
use crate::dynamics::solver::{
    JointConstraintTypes, JointGenericOneBodyConstraint, JointGenericOneBodyConstraintBuilder,
    JointGenericTwoBodyConstraint, JointGenericTwoBodyConstraintBuilder,
    JointGenericVelocityOneBodyExternalConstraintBuilder,
    JointGenericVelocityOneBodyInternalConstraintBuilder, SolverConstraintsSet, reset_buffer,
};
use crate::dynamics::{
    IntegrationParameters, IslandManager, JointGraphEdge, JointIndex, MultibodyJointSet,
    RigidBodySet,
};
use na::DVector;
use parry::math::Real;

use crate::dynamics::solver::joint_constraint::joint_constraint_builder::{
    JointOneBodyConstraintBuilder, JointTwoBodyConstraintBuilder,
};
#[cfg(feature = "simd-is-enabled")]
use {
    crate::dynamics::solver::joint_constraint::joint_constraint_builder::{
        JointOneBodyConstraintBuilderSimd, JointTwoBodyConstraintBuilderSimd,
    },
    crate::math::SIMD_WIDTH,
};

pub type JointConstraintsSet = SolverConstraintsSet<JointConstraintTypes>;

impl JointConstraintsSet {
    pub fn init(
        &mut self,
        island_id: usize,
        islands: &IslandManager,
        bodies: &RigidBodySet,
        multibody_joints: &MultibodyJointSet,
        impulse_joints: &[JointGraphEdge],
        joint_constraint_indices: &[JointIndex],
    ) {
        // Generate constraints for impulse_joints.
        self.two_body_interactions.clear();
        self.one_body_interactions.clear();
        self.generic_two_body_interactions.clear();
        self.generic_one_body_interactions.clear();

        categorize_joints(
            bodies,
            multibody_joints,
            impulse_joints,
            joint_constraint_indices,
            &mut self.one_body_interactions,
            &mut self.two_body_interactions,
            &mut self.generic_one_body_interactions,
            &mut self.generic_two_body_interactions,
        );

        self.clear_constraints();
        self.clear_builders();

        self.interaction_groups.clear_groups();
        self.interaction_groups.group_joints(
            island_id,
            islands,
            bodies,
            impulse_joints,
            &self.two_body_interactions,
        );

        self.one_body_interaction_groups.clear_groups();
        self.one_body_interaction_groups.group_joints(
            island_id,
            islands,
            bodies,
            impulse_joints,
            &self.one_body_interactions,
        );
        // NOTE: uncomment this do disable SIMD joint resolution.
        // self.interaction_groups
        //     .nongrouped_interactions
        //     .append(&mut self.interaction_groups.simd_interactions);
        // self.one_body_interaction_groups
        //     .nongrouped_interactions
        //     .append(&mut self.one_body_interaction_groups.simd_interactions);

        let mut j_id = 0;
        self.compute_joint_constraints(bodies, impulse_joints);
        #[cfg(feature = "simd-is-enabled")]
        {
            self.simd_compute_joint_constraints(bodies, impulse_joints);
        }
        self.compute_generic_joint_constraints(bodies, multibody_joints, impulse_joints, &mut j_id);

        self.compute_joint_one_body_constraints(bodies, impulse_joints);
        #[cfg(feature = "simd-is-enabled")]
        {
            self.simd_compute_joint_one_body_constraints(bodies, impulse_joints);
        }
        self.compute_generic_one_body_joint_constraints(
            island_id,
            islands,
            bodies,
            multibody_joints,
            impulse_joints,
            &mut j_id,
        );
    }

    fn compute_joint_one_body_constraints(
        &mut self,
        bodies: &RigidBodySet,
        joints_all: &[JointGraphEdge],
    ) {
        let total_num_builders = self
            .one_body_interaction_groups
            .nongrouped_interactions
            .len();

        unsafe {
            reset_buffer(
                &mut self.velocity_one_body_constraints_builder,
                total_num_builders,
            );
        }

        let mut num_constraints = 0;
        for (joint_i, builder) in self
            .one_body_interaction_groups
            .nongrouped_interactions
            .iter()
            .zip(self.velocity_one_body_constraints_builder.iter_mut())
        {
            let joint = &joints_all[*joint_i].weight;
            JointOneBodyConstraintBuilder::generate(
                joint,
                bodies,
                *joint_i,
                builder,
                &mut num_constraints,
            );
        }

        unsafe {
            reset_buffer(&mut self.velocity_one_body_constraints, num_constraints);
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    fn simd_compute_joint_one_body_constraints(
        &mut self,
        bodies: &RigidBodySet,
        joints_all: &[JointGraphEdge],
    ) {
        let total_num_builders =
            self.one_body_interaction_groups.simd_interactions.len() / SIMD_WIDTH;

        unsafe {
            reset_buffer(
                &mut self.simd_velocity_one_body_constraints_builder,
                total_num_builders,
            );
        }

        let mut num_constraints = 0;
        for (joints_i, builder) in self
            .one_body_interaction_groups
            .simd_interactions
            .chunks_exact(SIMD_WIDTH)
            .zip(self.simd_velocity_one_body_constraints_builder.iter_mut())
        {
            let joints_id = gather![|ii| joints_i[ii]];
            let impulse_joints = gather![|ii| &joints_all[joints_i[ii]].weight];
            JointOneBodyConstraintBuilderSimd::generate(
                impulse_joints,
                bodies,
                joints_id,
                builder,
                &mut num_constraints,
            );
        }

        unsafe {
            reset_buffer(
                &mut self.simd_velocity_one_body_constraints,
                num_constraints,
            );
        }
    }

    fn compute_joint_constraints(&mut self, bodies: &RigidBodySet, joints_all: &[JointGraphEdge]) {
        let total_num_builders = self.interaction_groups.nongrouped_interactions.len();

        unsafe {
            reset_buffer(&mut self.velocity_constraints_builder, total_num_builders);
        }

        let mut num_constraints = 0;
        for (joint_i, builder) in self
            .interaction_groups
            .nongrouped_interactions
            .iter()
            .zip(self.velocity_constraints_builder.iter_mut())
        {
            let joint = &joints_all[*joint_i].weight;
            JointTwoBodyConstraintBuilder::generate(
                joint,
                bodies,
                *joint_i,
                builder,
                &mut num_constraints,
            );
        }

        unsafe {
            reset_buffer(&mut self.velocity_constraints, num_constraints);
        }
    }

    fn compute_generic_joint_constraints(
        &mut self,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        joints_all: &[JointGraphEdge],
        j_id: &mut usize,
    ) {
        let total_num_builders = self.generic_two_body_interactions.len();
        self.generic_velocity_constraints_builder.resize(
            total_num_builders,
            JointGenericTwoBodyConstraintBuilder::invalid(),
        );

        let mut num_constraints = 0;
        for (joint_i, builder) in self
            .generic_two_body_interactions
            .iter()
            .zip(self.generic_velocity_constraints_builder.iter_mut())
        {
            let joint = &joints_all[*joint_i].weight;
            JointGenericTwoBodyConstraintBuilder::generate(
                *joint_i,
                joint,
                bodies,
                multibodies,
                builder,
                j_id,
                &mut self.generic_jacobians,
                &mut num_constraints,
            );
        }

        self.generic_velocity_constraints
            .resize(num_constraints, JointGenericTwoBodyConstraint::invalid());
    }

    fn compute_generic_one_body_joint_constraints(
        &mut self,
        island_id: usize,
        islands: &IslandManager,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        joints_all: &[JointGraphEdge],
        j_id: &mut usize,
    ) {
        let mut total_num_builders = self.generic_one_body_interactions.len();

        for handle in islands.active_island(island_id) {
            if let Some(link_id) = multibodies.rigid_body_link(*handle) {
                if JointGenericVelocityOneBodyInternalConstraintBuilder::num_constraints(
                    multibodies,
                    link_id,
                ) > 0
                {
                    total_num_builders += 1;
                }
            }
        }

        self.generic_velocity_one_body_constraints_builder.resize(
            total_num_builders,
            JointGenericOneBodyConstraintBuilder::invalid(),
        );

        let mut num_constraints = 0;
        for (joint_i, builder) in self.generic_one_body_interactions.iter().zip(
            self.generic_velocity_one_body_constraints_builder
                .iter_mut(),
        ) {
            let joint = &joints_all[*joint_i].weight;
            JointGenericVelocityOneBodyExternalConstraintBuilder::generate(
                *joint_i,
                joint,
                bodies,
                multibodies,
                builder,
                j_id,
                &mut self.generic_jacobians,
                &mut num_constraints,
            );
        }

        let mut curr_builder = self.generic_one_body_interactions.len();
        for handle in islands.active_island(island_id) {
            if curr_builder >= self.generic_velocity_one_body_constraints_builder.len() {
                break; // No more builder need to be generated.
            }

            if let Some(link_id) = multibodies.rigid_body_link(*handle) {
                let prev_num_constraints = num_constraints;
                JointGenericVelocityOneBodyInternalConstraintBuilder::generate(
                    multibodies,
                    link_id,
                    &mut self.generic_velocity_one_body_constraints_builder[curr_builder],
                    j_id,
                    &mut self.generic_jacobians,
                    &mut num_constraints,
                );
                if num_constraints != prev_num_constraints {
                    curr_builder += 1;
                }
            }
        }

        self.generic_velocity_one_body_constraints
            .resize(num_constraints, JointGenericOneBodyConstraint::invalid());
    }

    #[cfg(feature = "simd-is-enabled")]
    fn simd_compute_joint_constraints(
        &mut self,
        bodies: &RigidBodySet,
        joints_all: &[JointGraphEdge],
    ) {
        let total_num_builders = self.interaction_groups.simd_interactions.len() / SIMD_WIDTH;

        unsafe {
            reset_buffer(
                &mut self.simd_velocity_constraints_builder,
                total_num_builders,
            );
        }

        let mut num_constraints = 0;
        for (joints_i, builder) in self
            .interaction_groups
            .simd_interactions
            .chunks_exact(SIMD_WIDTH)
            .zip(self.simd_velocity_constraints_builder.iter_mut())
        {
            let joints_id = gather![|ii| joints_i[ii]];
            let impulse_joints = gather![|ii| &joints_all[joints_i[ii]].weight];
            JointTwoBodyConstraintBuilderSimd::generate(
                impulse_joints,
                bodies,
                joints_id,
                builder,
                &mut num_constraints,
            );
        }

        unsafe {
            reset_buffer(&mut self.simd_velocity_constraints, num_constraints);
        }
    }

    #[profiling::function]
    pub fn solve(
        &mut self,
        solver_vels: &mut [SolverVel<Real>],
        generic_solver_vels: &mut DVector<Real>,
    ) {
        let (jac, constraints) = self.iter_constraints_mut();
        for mut c in constraints {
            c.solve(jac, solver_vels, generic_solver_vels);
        }
    }

    pub fn solve_wo_bias(
        &mut self,
        solver_vels: &mut [SolverVel<Real>],
        generic_solver_vels: &mut DVector<Real>,
    ) {
        let (jac, constraints) = self.iter_constraints_mut();
        for mut c in constraints {
            c.remove_bias();
            c.solve(jac, solver_vels, generic_solver_vels);
        }
    }

    pub fn writeback_impulses(&mut self, joints_all: &mut [JointGraphEdge]) {
        let (_, constraints) = self.iter_constraints_mut();
        for mut c in constraints {
            c.writeback_impulses(joints_all);
        }
    }

    #[profiling::function]
    pub fn update(
        &mut self,
        params: &IntegrationParameters,
        multibodies: &MultibodyJointSet,
        solver_bodies: &[SolverBody],
    ) {
        for builder in &mut self.generic_velocity_constraints_builder {
            builder.update(
                params,
                multibodies,
                solver_bodies,
                &mut self.generic_jacobians,
                &mut self.generic_velocity_constraints,
            );
        }

        for builder in &mut self.generic_velocity_one_body_constraints_builder {
            builder.update(
                params,
                multibodies,
                solver_bodies,
                &mut self.generic_jacobians,
                &mut self.generic_velocity_one_body_constraints,
            );
        }

        for builder in &mut self.velocity_constraints_builder {
            builder.update(params, solver_bodies, &mut self.velocity_constraints);
        }

        #[cfg(feature = "simd-is-enabled")]
        for builder in &mut self.simd_velocity_constraints_builder {
            builder.update(params, solver_bodies, &mut self.simd_velocity_constraints);
        }

        for builder in &mut self.velocity_one_body_constraints_builder {
            builder.update(
                params,
                solver_bodies,
                &mut self.velocity_one_body_constraints,
            );
        }

        #[cfg(feature = "simd-is-enabled")]
        for builder in &mut self.simd_velocity_one_body_constraints_builder {
            builder.update(
                params,
                solver_bodies,
                &mut self.simd_velocity_one_body_constraints,
            );
        }
    }
}
