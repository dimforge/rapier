use super::{AnyJointVelocityConstraint, AnyVelocityConstraint, DeltaVel, ThreadContext};
use crate::concurrent_loop;
use crate::dynamics::{
    solver::ParallelSolverConstraints, IntegrationParameters, IslandManager, JointGraphEdge,
    MultibodyJointSet, RigidBodySet,
};
use crate::geometry::ContactManifold;
use crate::math::Real;
use crate::utils::WAngularInertia;

use na::DVector;
use std::sync::atomic::Ordering;

pub(crate) struct ParallelVelocitySolver {
    pub mj_lambdas: Vec<DeltaVel<Real>>,
    pub generic_mj_lambdas: DVector<Real>,
}

impl ParallelVelocitySolver {
    pub fn new() -> Self {
        Self {
            mj_lambdas: Vec::new(),
            generic_mj_lambdas: DVector::zeros(0),
        }
    }

    pub fn solve(
        &mut self,
        thread: &ThreadContext,
        params: &IntegrationParameters,
        island_id: usize,
        islands: &IslandManager,
        bodies: &mut RigidBodySet,
        multibodies: &mut MultibodyJointSet,
        manifolds_all: &mut [&mut ContactManifold],
        joints_all: &mut [JointGraphEdge],
        contact_constraints: &mut ParallelSolverConstraints<AnyVelocityConstraint>,
        joint_constraints: &mut ParallelSolverConstraints<AnyJointVelocityConstraint>,
    ) {
        let mut start_index = thread
            .solve_interaction_index
            .fetch_add(thread.batch_size, Ordering::SeqCst);
        let mut batch_size = thread.batch_size;
        let contact_descs = &contact_constraints.constraint_descs[..];
        let joint_descs = &joint_constraints.constraint_descs[..];
        let mut target_num_desc = 0;
        let mut shift = 0;

        // Each thread will concurrently grab thread.batch_size constraint desc to
        // solve. If the batch size is large enough to cross the boundary of
        // a parallel_desc_group, we have to wait util the current group is finished
        // before starting the next one.
        macro_rules! solve {
            ($part: expr, $($solve_args: expr),*) => {
                for group in $part.parallel_desc_groups.windows(2) {
                    let num_descs_in_group = group[1] - group[0];
                    target_num_desc += num_descs_in_group;

                    while start_index < group[1] {
                        let end_index = (start_index + batch_size).min(group[1]);

                        // TODO: remove the first branch case?
                        let constraints = if end_index == $part.constraint_descs.len() {
                            &mut $part.velocity_constraints
                                [$part.constraint_descs[start_index].0..]
                        } else {
                            &mut $part.velocity_constraints
                                [$part.constraint_descs[start_index].0
                                ..$part.constraint_descs[end_index].0]
                        };

                        for constraint in constraints {
                            constraint.solve(
                                $($solve_args),*
                            );
                        }

                        let num_solved = end_index - start_index;
                        batch_size -= num_solved;

                        thread
                            .num_solved_interactions
                            .fetch_add(num_solved, Ordering::SeqCst);

                        if batch_size == 0 {
                            start_index = thread
                                .solve_interaction_index
                                .fetch_add(thread.batch_size, Ordering::SeqCst);
                            start_index -= shift;
                            batch_size = thread.batch_size;
                        } else {
                            start_index += num_solved;
                        }
                    }
                    ThreadContext::lock_until_ge(
                        &thread.num_solved_interactions,
                        target_num_desc,
                    );
                }
            };
        }

        /*
         * Solve constraints.
         */
        {
            for i in 0..params.max_velocity_iterations {
                let solve_friction = params.interleave_restitution_and_friction_resolution
                    && params.max_velocity_friction_iterations + i
                        >= params.max_velocity_iterations;
                // Solve joints.
                solve!(
                    joint_constraints,
                    &joint_constraints.generic_jacobians,
                    &mut self.mj_lambdas,
                    &mut self.generic_mj_lambdas
                );
                shift += joint_descs.len();
                start_index -= joint_descs.len();

                // Solve rigid-body contacts.
                solve!(
                    contact_constraints,
                    &contact_constraints.generic_jacobians,
                    &mut self.mj_lambdas,
                    &mut self.generic_mj_lambdas,
                    true,
                    false
                );
                shift += contact_descs.len();
                start_index -= contact_descs.len();

                // Solve generic rigid-body contacts.
                solve!(
                    contact_constraints,
                    &contact_constraints.generic_jacobians,
                    &mut self.mj_lambdas,
                    &mut self.generic_mj_lambdas,
                    true,
                    false
                );
                shift += contact_descs.len();
                start_index -= contact_descs.len();

                if solve_friction {
                    solve!(
                        contact_constraints,
                        &contact_constraints.generic_jacobians,
                        &mut self.mj_lambdas,
                        &mut self.generic_mj_lambdas,
                        false,
                        true
                    );
                    shift += contact_descs.len();
                    start_index -= contact_descs.len();
                }
            }

            // Solve the remaining friction iterations.
            let remaining_friction_iterations =
                if !params.interleave_restitution_and_friction_resolution {
                    params.max_velocity_friction_iterations
                } else if params.max_velocity_friction_iterations > params.max_velocity_iterations {
                    params.max_velocity_friction_iterations - params.max_velocity_iterations
                } else {
                    0
                };

            for _ in 0..remaining_friction_iterations {
                solve!(
                    contact_constraints,
                    &contact_constraints.generic_jacobians,
                    &mut self.mj_lambdas,
                    &mut self.generic_mj_lambdas,
                    false,
                    true
                );
                shift += contact_descs.len();
                start_index -= contact_descs.len();
            }
        }

        // Integrate positions.
        {
            let island_range = islands.active_island_range(island_id);
            let active_bodies = &islands.active_dynamic_set[island_range];

            concurrent_loop! {
                let batch_size = thread.batch_size;
                for handle in active_bodies[thread.body_integration_pos_index, thread.num_integrated_pos_bodies] {
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
                        let dangvel = rb.mprops
                            .effective_world_inv_inertia_sqrt
                            .transform_vector(dvel.angular);

                        // Update positions.
                        let mut new_vels = rb.vels;
                        new_vels.linvel += dvel.linear;
                        new_vels.angvel += dangvel;
                        new_vels = new_vels.apply_damping(params.dt, &rb.damping);
                        rb.pos.next_position = new_vels.integrate(
                            params.dt,
                            &rb.pos.position,
                            &rb.mprops.local_mprops.local_com,
                        );
                    }
                }
            }

            ThreadContext::lock_until_ge(&thread.num_integrated_pos_bodies, active_bodies.len());
        }

        // Remove bias from constraints.
        {
            let joint_constraints = &mut joint_constraints.velocity_constraints;
            let contact_constraints = &mut contact_constraints.velocity_constraints;

            crate::concurrent_loop! {
                 let batch_size = thread.batch_size;
                 for constraint in &mut joint_constraints[thread.joint_rm_bias_index] {
                     constraint.remove_bias_from_rhs();
                 }
            }
            crate::concurrent_loop! {
                 let batch_size = thread.batch_size;
                 for constraint in &mut contact_constraints[thread.impulse_rm_bias_index] {
                     constraint.remove_bias_from_rhs();
                 }
            }
        }

        // Stabiliziton resolution.
        {
            for _ in 0..params.max_stabilization_iterations {
                solve!(
                    joint_constraints,
                    &joint_constraints.generic_jacobians,
                    &mut self.mj_lambdas,
                    &mut self.generic_mj_lambdas
                );
                shift += joint_descs.len();
                start_index -= joint_descs.len();

                solve!(
                    contact_constraints,
                    &contact_constraints.generic_jacobians,
                    &mut self.mj_lambdas,
                    &mut self.generic_mj_lambdas,
                    true,
                    false
                );
                shift += contact_descs.len();
                start_index -= contact_descs.len();

                solve!(
                    contact_constraints,
                    &contact_constraints.generic_jacobians,
                    &mut self.mj_lambdas,
                    &mut self.generic_mj_lambdas,
                    false,
                    true
                );
                shift += contact_descs.len();
                start_index -= contact_descs.len();
            }
        }

        // Update velocities.
        {
            let island_range = islands.active_island_range(island_id);
            let active_bodies = &islands.active_dynamic_set[island_range];

            concurrent_loop! {
                let batch_size = thread.batch_size;
                for handle in active_bodies[thread.body_integration_vel_index, thread.num_integrated_vel_bodies] {
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
                        let dangvel = rb.mprops
                            .effective_world_inv_inertia_sqrt
                            .transform_vector(dvel.angular);
                        rb.vels.linvel += dvel.linear;
                        rb.vels.angvel += dangvel;
                        rb.vels = rb.vels.apply_damping(params.dt, &rb.damping);
                    }
                }
            }
        }

        /*
         * Writeback impulses.
         */
        let joint_constraints = &joint_constraints.velocity_constraints;
        let contact_constraints = &contact_constraints.velocity_constraints;

        crate::concurrent_loop! {
             let batch_size = thread.batch_size;
             for constraint in joint_constraints[thread.joint_writeback_index] {
                 constraint.writeback_impulses(joints_all);
             }
        }
        crate::concurrent_loop! {
             let batch_size = thread.batch_size;
             for constraint in contact_constraints[thread.impulse_writeback_index] {
                 constraint.writeback_impulses(manifolds_all);
             }
        }
    }
}
