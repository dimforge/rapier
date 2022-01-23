use super::{AnyJointVelocityConstraint, AnyVelocityConstraint, DeltaVel, ThreadContext};
use crate::dynamics::solver::generic_velocity_constraint::GenericVelocityConstraint;
use crate::dynamics::solver::ParallelSolverConstraints;
use crate::dynamics::{IntegrationParameters, JointGraphEdge};
use crate::geometry::ContactManifold;
use crate::math::Real;
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
        manifolds_all: &mut [&mut ContactManifold],
        joints_all: &mut [JointGraphEdge],
        contact_constraints: &mut ParallelSolverConstraints<
            AnyVelocityConstraint,
            GenericVelocityConstraint,
        >,
        joint_constraints: &mut ParallelSolverConstraints<AnyJointVelocityConstraint, ()>,
    ) {
        if contact_constraints.constraint_descs.is_empty()
            && joint_constraints.constraint_descs.is_empty()
        {
            return;
        }

        /*
         * Solve constraints.
         */
        {
            // Each thread will concurrently grab thread.batch_size constraint desc to
            // solve. If the batch size is large enough for to cross the boundary of
            // a parallel_desc_group, we have to wait util the current group is finished
            // before starting the next one.
            let mut start_index = thread
                .solve_interaction_index
                .fetch_add(thread.batch_size, Ordering::SeqCst);
            let mut batch_size = thread.batch_size;
            let contact_descs = &contact_constraints.constraint_descs[..];
            let joint_descs = &joint_constraints.constraint_descs[..];
            let mut target_num_desc = 0;
            let mut shift = 0;
            let cfm_factor = params.cfm_factor();

            for _ in 0..params.max_velocity_iterations {
                macro_rules! solve {
                    ($part: expr, $($solve_args: expr),*) => {
                        // ImpulseJoint groups.
                        for group in $part.parallel_desc_groups.windows(2) {
                            let num_descs_in_group = group[1] - group[0];

                            target_num_desc += num_descs_in_group;

                            while start_index < group[1] {
                                let end_index = (start_index + batch_size).min(group[1]);

                                let constraints = if end_index == $part.constraint_descs.len() {
                                    &mut $part.velocity_constraints
                                        [$part.constraint_descs[start_index].0..]
                                } else {
                                    &mut $part.velocity_constraints[$part.constraint_descs
                                        [start_index]
                                        .0
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
                    cfm_factor,
                    &mut self.mj_lambdas,
                    true,
                    true
                );
                shift += contact_descs.len();
                start_index -= contact_descs.len();
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
