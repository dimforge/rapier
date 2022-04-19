use super::VelocitySolver;
use crate::counters::Counters;
use crate::dynamics::solver::{
    AnyJointVelocityConstraint, AnyVelocityConstraint, SolverConstraints,
};
use crate::dynamics::IslandManager;
use crate::dynamics::{IntegrationParameters, JointGraphEdge, JointIndex, RigidBodySet};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::prelude::MultibodyJointSet;

pub struct IslandSolver {
    contact_constraints: SolverConstraints<AnyVelocityConstraint>,
    joint_constraints: SolverConstraints<AnyJointVelocityConstraint>,
    velocity_solver: VelocitySolver,
}

impl Default for IslandSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl IslandSolver {
    pub fn new() -> Self {
        Self {
            contact_constraints: SolverConstraints::new(),
            joint_constraints: SolverConstraints::new(),
            velocity_solver: VelocitySolver::new(),
        }
    }

    pub fn init_and_solve(
        &mut self,
        island_id: usize,
        counters: &mut Counters,
        params: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        manifolds: &mut [&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
        impulse_joints: &mut [JointGraphEdge],
        joint_indices: &[JointIndex],
        multibody_joints: &mut MultibodyJointSet,
    ) {
        // Init the solver id for multibody_joints.
        // We need that for building the constraints.
        let mut solver_id = 0;
        for (_, multibody) in multibody_joints.multibodies.iter_mut() {
            multibody.solver_id = solver_id;
            solver_id += multibody.ndofs();
        }

        counters.solver.velocity_assembly_time.resume();
        self.contact_constraints.init(
            island_id,
            params,
            islands,
            bodies,
            multibody_joints,
            manifolds,
            manifold_indices,
        );
        self.joint_constraints.init(
            island_id,
            params,
            islands,
            bodies,
            multibody_joints,
            impulse_joints,
            joint_indices,
        );
        counters.solver.velocity_assembly_time.pause();

        counters.solver.velocity_resolution_time.resume();
        self.velocity_solver.solve(
            island_id,
            params,
            islands,
            bodies,
            multibody_joints,
            manifolds,
            impulse_joints,
            &mut self.contact_constraints.velocity_constraints,
            &self.contact_constraints.generic_jacobians,
            &mut self.joint_constraints.velocity_constraints,
            &self.joint_constraints.generic_jacobians,
        );
        counters.solver.velocity_resolution_time.pause();
    }
}
