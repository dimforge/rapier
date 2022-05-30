use super::AnyJointVelocityConstraint;
use crate::dynamics::{
    solver::{AnyVelocityConstraint, DeltaVel},
    IntegrationParameters, IslandManager, JointGraphEdge, MultibodyJointSet, RigidBodySet,
};
use crate::geometry::ContactManifold;
use crate::math::Real;
use crate::utils::WAngularInertia;
use na::DVector;

pub(crate) struct VelocitySolver {
    pub mj_lambdas: Vec<DeltaVel<Real>>,
    pub generic_mj_lambdas: DVector<Real>,
}

impl VelocitySolver {
    pub fn new() -> Self {
        Self {
            mj_lambdas: Vec::new(),
            generic_mj_lambdas: DVector::zeros(0),
        }
    }

    pub fn solve(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
        manifolds_all: &mut [&mut ContactManifold],
        joints_all: &mut [JointGraphEdge],
        contact_constraints: &mut [AnyVelocityConstraint],
        generic_contact_jacobians: &DVector<Real>,
        joint_constraints: &mut [AnyJointVelocityConstraint],
        generic_joint_jacobians: &DVector<Real>,
    ) {
        self.mj_lambdas.clear();
        self.mj_lambdas
            .resize(islands.active_island(island_id).len(), DeltaVel::zero());

        let total_multibodies_ndofs = multibodies.multibodies.iter().map(|m| m.1.ndofs()).sum();
        self.generic_mj_lambdas = DVector::zeros(total_multibodies_ndofs);

        // Initialize delta-velocities (`mj_lambdas`) with external forces (gravity etc):
        for handle in islands.active_island(island_id) {
            if let Some(link) = multibodies.rigid_body_link(*handle).copied() {
                let multibody = multibodies
                    .get_multibody_mut_internal(link.multibody)
                    .unwrap();

                if link.id == 0 || link.id == 1 && !multibody.root_is_dynamic {
                    let mut mj_lambdas = self
                        .generic_mj_lambdas
                        .rows_mut(multibody.solver_id, multibody.ndofs());
                    mj_lambdas.axpy(params.dt, &multibody.accelerations, 0.0);
                }
            } else {
                let rb = &bodies[*handle];
                let dvel = &mut self.mj_lambdas[rb.ids.active_set_offset];

                // NOTE: `dvel.angular` is actually storing angular velocity delta multiplied
                //       by the square root of the inertia tensor:
                dvel.angular +=
                    rb.mprops.effective_world_inv_inertia_sqrt * rb.forces.torque * params.dt;
                dvel.linear +=
                    rb.forces.force.component_mul(&rb.mprops.effective_inv_mass) * params.dt;
            }
        }

        /*
         * Solve constraints.
         */
        for i in 0..params.max_velocity_iterations {
            let solve_friction = params.interleave_restitution_and_friction_resolution
                && params.max_velocity_friction_iterations + i >= params.max_velocity_iterations;

            for constraint in &mut *joint_constraints {
                constraint.solve(
                    generic_joint_jacobians,
                    &mut self.mj_lambdas[..],
                    &mut self.generic_mj_lambdas,
                );
            }

            for constraint in &mut *contact_constraints {
                constraint.solve(
                    generic_contact_jacobians,
                    &mut self.mj_lambdas[..],
                    &mut self.generic_mj_lambdas,
                    true,
                    false,
                );
            }

            if solve_friction {
                for constraint in &mut *contact_constraints {
                    constraint.solve(
                        generic_contact_jacobians,
                        &mut self.mj_lambdas[..],
                        &mut self.generic_mj_lambdas,
                        false,
                        true,
                    );
                }
            }
        }

        let remaining_friction_iterations =
            if !params.interleave_restitution_and_friction_resolution {
                params.max_velocity_friction_iterations
            } else if params.max_velocity_friction_iterations > params.max_velocity_iterations {
                params.max_velocity_friction_iterations - params.max_velocity_iterations
            } else {
                0
            };

        for _ in 0..remaining_friction_iterations {
            for constraint in &mut *contact_constraints {
                constraint.solve(
                    generic_contact_jacobians,
                    &mut self.mj_lambdas[..],
                    &mut self.generic_mj_lambdas,
                    false,
                    true,
                );
            }
        }

        // Integrate positions.
        for handle in islands.active_island(island_id) {
            if let Some(link) = multibodies.rigid_body_link(*handle).copied() {
                let multibody = multibodies
                    .get_multibody_mut_internal(link.multibody)
                    .unwrap();

                if link.id == 0 || link.id == 1 && !multibody.root_is_dynamic {
                    let mj_lambdas = self
                        .generic_mj_lambdas
                        .rows(multibody.solver_id, multibody.ndofs());
                    let prev_vels = multibody.velocities.clone(); // FIXME: avoid allocations.
                    multibody.velocities += mj_lambdas;
                    multibody.integrate(params.dt);
                    multibody.forward_kinematics(bodies, false);
                    multibody.velocities = prev_vels;
                }
            } else {
                let rb = bodies.index_mut_internal(*handle);

                let dvel = self.mj_lambdas[rb.ids.active_set_offset];
                let dangvel = rb
                    .mprops
                    .effective_world_inv_inertia_sqrt
                    .transform_vector(dvel.angular);

                // Update positions.
                let mut new_pos = rb.pos;
                let mut new_vels = rb.vels;
                new_vels.linvel += dvel.linear;
                new_vels.angvel += dangvel;
                new_vels = new_vels.apply_damping(params.dt, &rb.damping);
                new_pos.next_position = new_vels.integrate(
                    params.dt,
                    &rb.pos.position,
                    &rb.mprops.local_mprops.local_com,
                );
                rb.integrated_vels = new_vels;
                rb.pos = new_pos;
            }
        }

        for joint in &mut *joint_constraints {
            joint.remove_bias_from_rhs();
        }
        for constraint in &mut *contact_constraints {
            constraint.remove_bias_from_rhs();
        }

        for _ in 0..params.max_stabilization_iterations {
            for constraint in &mut *joint_constraints {
                constraint.solve(
                    generic_joint_jacobians,
                    &mut self.mj_lambdas[..],
                    &mut self.generic_mj_lambdas,
                );
            }

            for constraint in &mut *contact_constraints {
                constraint.solve(
                    generic_contact_jacobians,
                    &mut self.mj_lambdas[..],
                    &mut self.generic_mj_lambdas,
                    true,
                    false,
                );
            }

            for constraint in &mut *contact_constraints {
                constraint.solve(
                    generic_contact_jacobians,
                    &mut self.mj_lambdas[..],
                    &mut self.generic_mj_lambdas,
                    false,
                    true,
                );
            }
        }

        // Update velocities.
        for handle in islands.active_island(island_id) {
            if let Some(link) = multibodies.rigid_body_link(*handle).copied() {
                let multibody = multibodies
                    .get_multibody_mut_internal(link.multibody)
                    .unwrap();

                if link.id == 0 || link.id == 1 && !multibody.root_is_dynamic {
                    let mj_lambdas = self
                        .generic_mj_lambdas
                        .rows(multibody.solver_id, multibody.ndofs());
                    multibody.velocities += mj_lambdas;
                }
            } else {
                let rb = bodies.index_mut_internal(*handle);
                let dvel = self.mj_lambdas[rb.ids.active_set_offset];
                let dangvel = rb
                    .mprops
                    .effective_world_inv_inertia_sqrt
                    .transform_vector(dvel.angular);

                rb.vels.linvel += dvel.linear;
                rb.vels.angvel += dangvel;
                rb.vels = rb.vels.apply_damping(params.dt, &rb.damping);
            }
        }

        // Write impulses back into the manifold structures.
        for constraint in &*joint_constraints {
            constraint.writeback_impulses(joints_all);
        }

        for constraint in &*contact_constraints {
            constraint.writeback_impulses(manifolds_all);
        }
    }
}
