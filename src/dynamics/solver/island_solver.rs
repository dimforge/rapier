use super::VelocitySolver;
use crate::counters::Counters;
use crate::data::{BundleSet, ComponentSet, ComponentSetMut};
use crate::dynamics::solver::{
    AnyGenericVelocityConstraint, AnyJointVelocityConstraint, AnyVelocityConstraint,
    SolverConstraints,
};
use crate::dynamics::{
    IntegrationParameters, JointGraphEdge, JointIndex, RigidBodyDamping, RigidBodyForces,
    RigidBodyIds, RigidBodyMassProps, RigidBodyPosition, RigidBodyType,
};
use crate::dynamics::{IslandManager, RigidBodyVelocity};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::prelude::{MultibodyJointSet, RigidBodyActivation};

pub struct IslandSolver {
    contact_constraints: SolverConstraints<AnyVelocityConstraint, AnyGenericVelocityConstraint>,
    joint_constraints: SolverConstraints<AnyJointVelocityConstraint, ()>,
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

    pub fn init_and_solve<Bodies>(
        &mut self,
        island_id: usize,
        counters: &mut Counters,
        params: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &mut Bodies,
        manifolds: &mut [&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
        impulse_joints: &mut [JointGraphEdge],
        joint_indices: &[JointIndex],
        multibody_joints: &mut MultibodyJointSet,
    ) where
        Bodies: ComponentSet<RigidBodyForces>
            + ComponentSetMut<RigidBodyPosition>
            + ComponentSetMut<RigidBodyVelocity>
            + ComponentSetMut<RigidBodyMassProps>
            + ComponentSetMut<RigidBodyActivation>
            + ComponentSet<RigidBodyDamping>
            + ComponentSet<RigidBodyIds>
            + ComponentSet<RigidBodyType>,
    {
        let mut has_constraints = manifold_indices.len() != 0 || joint_indices.len() != 0;
        if !has_constraints {
            // Check if the multibody_joints have internal constraints.
            for handle in islands.active_island(island_id) {
                if let Some(link) = multibody_joints.rigid_body_link(*handle) {
                    let multibody = multibody_joints.get_multibody(link.multibody).unwrap();

                    if (link.id == 0 || link.id == 1 && !multibody.root_is_dynamic)
                        && multibody.has_active_internal_constraints()
                    {
                        has_constraints = true;
                        break;
                    }
                }
            }
        }

        if has_constraints {
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
                &mut self.contact_constraints.generic_velocity_constraints,
                &self.contact_constraints.generic_jacobians,
                &mut self.joint_constraints.velocity_constraints,
                &self.joint_constraints.generic_jacobians,
            );
            counters.solver.velocity_resolution_time.pause();
        } else {
            self.contact_constraints.clear();
            self.joint_constraints.clear();
            counters.solver.velocity_update_time.resume();

            for handle in islands.active_island(island_id) {
                if let Some(link) = multibody_joints.rigid_body_link(*handle).copied() {
                    let multibody = multibody_joints
                        .get_multibody_mut_internal(link.multibody)
                        .unwrap();

                    if link.id == 0 || link.id == 1 && !multibody.root_is_dynamic {
                        let accels = &multibody.accelerations;
                        multibody.velocities.axpy(params.dt, accels, 1.0);
                        multibody.integrate(params.dt);
                        multibody.forward_kinematics(bodies, false);
                    }
                } else {
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
                        vels.integrate(params.dt, &poss.position, &mprops.local_mprops.local_com);

                    bodies.set_internal(handle.0, new_vels);
                    bodies.set_internal(handle.0, new_poss);
                }
            }
            counters.solver.velocity_update_time.pause();
        }
    }
}
