use super::{JointConstraintsSet, VelocitySolver, VelocitySolver2};
use crate::counters::Counters;
use crate::dynamics::solver::contact_constraint::ContactConstraintsSet;
use crate::dynamics::{IntegrationParameters, JointGraphEdge, JointIndex, RigidBodySet};
use crate::dynamics::{IslandManager, IslandManager2};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::prelude::MultibodyJointSet;
use parry::math::Real;

pub struct IslandSolver2 {
    contact_constraints: ContactConstraintsSet,
    joint_constraints: JointConstraintsSet,
    velocity_solver: VelocitySolver2,
}

impl Default for IslandSolver2 {
    fn default() -> Self {
        Self::new()
    }
}

impl IslandSolver2 {
    pub fn new() -> Self {
        Self {
            contact_constraints: ContactConstraintsSet::new(),
            joint_constraints: JointConstraintsSet::new(),
            velocity_solver: VelocitySolver2::new(),
        }
    }

    #[profiling::function]
    pub fn init_and_solve(
        &mut self,
        island_id: usize,
        counters: &mut Counters,
        base_params: &IntegrationParameters,
        islands: &IslandManager2,
        bodies: &mut RigidBodySet,
        manifolds: &mut [&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
        impulse_joints: &mut [JointGraphEdge],
        joint_indices: &[JointIndex],
        multibodies: &mut MultibodyJointSet,
    ) {
        counters.solver.velocity_assembly_time.resume();
        let num_solver_iterations = base_params.num_solver_iterations.get();
        // FIXME: + islands.active_island_additional_solver_iterations(island_id);

        let mut params = *base_params;
        params.dt /= num_solver_iterations as Real;

        /*
         *
         * Below this point, the `params` is using the "small step" settings.
         *
         */
        // INIT
        self.velocity_solver
            .init_solver_velocities_and_solver_bodies(
                &params,
                island_id,
                islands,
                bodies,
                multibodies,
            );
        self.velocity_solver.init_constraints(
            island_id,
            islands,
            bodies,
            multibodies,
            manifolds,
            manifold_indices,
            impulse_joints,
            joint_indices,
            &mut self.contact_constraints,
            &mut self.joint_constraints,
        );
        counters.solver.velocity_assembly_time.pause();

        // SOLVE
        counters.solver.velocity_resolution_time.resume();
        self.velocity_solver.solve_constraints(
            &params,
            num_solver_iterations,
            bodies,
            multibodies,
            &mut self.contact_constraints,
            &mut self.joint_constraints,
        );
        counters.solver.velocity_resolution_time.pause();

        // WRITEBACK
        counters.solver.velocity_writeback_time.resume();
        self.joint_constraints.writeback_impulses(impulse_joints);
        self.contact_constraints.writeback_impulses(manifolds);
        self.velocity_solver.writeback_bodies(
            base_params,
            num_solver_iterations,
            islands,
            island_id,
            bodies,
            multibodies,
        );
        counters.solver.velocity_writeback_time.pause();
    }
}
