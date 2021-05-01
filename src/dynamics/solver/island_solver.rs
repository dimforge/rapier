use super::{PositionSolver, VelocitySolver};
use crate::counters::Counters;
use crate::data::{BundleSet, ComponentSet, ComponentSetMut};
use crate::dynamics::solver::{
    AnyJointPositionConstraint, AnyJointVelocityConstraint, AnyPositionConstraint,
    AnyVelocityConstraint, SolverConstraints,
};
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, RigidBodyDamping, RigidBodyForces,
    RigidBodyIds, RigidBodyMassProps, RigidBodyPosition, RigidBodyType,
};
use crate::dynamics::{IslandManager, RigidBodyVelocity};
use crate::geometry::{ContactManifold, ContactManifoldIndex};

pub struct IslandSolver {
    contact_constraints: SolverConstraints<AnyVelocityConstraint, AnyPositionConstraint>,
    joint_constraints: SolverConstraints<AnyJointVelocityConstraint, AnyJointPositionConstraint>,
    velocity_solver: VelocitySolver,
    position_solver: PositionSolver,
}

impl IslandSolver {
    pub fn new() -> Self {
        Self {
            contact_constraints: SolverConstraints::new(),
            joint_constraints: SolverConstraints::new(),
            velocity_solver: VelocitySolver::new(),
            position_solver: PositionSolver::new(),
        }
    }

    pub fn solve_position_constraints<Bodies>(
        &mut self,
        island_id: usize,
        islands: &IslandManager,
        counters: &mut Counters,
        params: &IntegrationParameters,
        bodies: &mut Bodies,
    ) where
        Bodies: ComponentSet<RigidBodyIds> + ComponentSetMut<RigidBodyPosition>,
    {
        counters.solver.position_resolution_time.resume();
        self.position_solver.solve(
            island_id,
            params,
            islands,
            bodies,
            &self.contact_constraints.position_constraints,
            &self.joint_constraints.position_constraints,
        );
        counters.solver.position_resolution_time.pause();
    }

    pub fn init_constraints_and_solve_velocity_constraints<Bodies>(
        &mut self,
        island_id: usize,
        counters: &mut Counters,
        params: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &mut Bodies,
        manifolds: &mut [&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
        joints: &mut [JointGraphEdge],
        joint_indices: &[JointIndex],
    ) where
        Bodies: ComponentSet<RigidBodyForces>
            + ComponentSetMut<RigidBodyPosition>
            + ComponentSetMut<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyDamping>
            + ComponentSet<RigidBodyIds>
            + ComponentSet<RigidBodyType>,
    {
        let has_constraints = manifold_indices.len() != 0 || joint_indices.len() != 0;

        if has_constraints {
            counters.solver.velocity_assembly_time.resume();
            self.contact_constraints.init(
                island_id,
                params,
                islands,
                bodies,
                manifolds,
                manifold_indices,
            );
            self.joint_constraints
                .init(island_id, params, islands, bodies, joints, joint_indices);
            counters.solver.velocity_assembly_time.pause();

            counters.solver.velocity_resolution_time.resume();
            self.velocity_solver.solve(
                island_id,
                params,
                islands,
                bodies,
                manifolds,
                joints,
                &mut self.contact_constraints.velocity_constraints,
                &mut self.joint_constraints.velocity_constraints,
            );
            counters.solver.velocity_resolution_time.pause();

            counters.solver.velocity_update_time.resume();

            for handle in islands.active_island(island_id) {
                let (poss, vels, damping, mprops): (
                    &RigidBodyPosition,
                    &RigidBodyVelocity,
                    &RigidBodyDamping,
                    &RigidBodyMassProps,
                ) = bodies.index_bundle(handle.0);

                let mut new_poss = *poss;
                let new_vels = vels.apply_damping(params.dt, damping);
                new_poss.next_position =
                    vels.integrate(params.dt, &poss.position, &mprops.mass_properties.local_com);

                bodies.set_internal(handle.0, new_vels);
                bodies.set_internal(handle.0, new_poss);
            }

            counters.solver.velocity_update_time.pause();
        } else {
            self.contact_constraints.clear();
            self.joint_constraints.clear();
            counters.solver.velocity_update_time.resume();

            for handle in islands.active_island(island_id) {
                // Since we didn't run the velocity solver we need to integrate the accelerations here
                let (poss, vels, forces, damping, mprops): (
                    &RigidBodyPosition,
                    &RigidBodyVelocity,
                    &RigidBodyForces,
                    &RigidBodyDamping,
                    &RigidBodyMassProps,
                ) = bodies.index_bundle(handle.0);

                let mut new_poss = *poss;
                let new_vels = forces
                    .integrate(params.dt, vels, mprops)
                    .apply_damping(params.dt, &damping);
                new_poss.next_position =
                    vels.integrate(params.dt, &poss.position, &mprops.mass_properties.local_com);

                bodies.set_internal(handle.0, new_vels);
                bodies.set_internal(handle.0, new_poss);
            }
            counters.solver.velocity_update_time.pause();
        }
    }
}
