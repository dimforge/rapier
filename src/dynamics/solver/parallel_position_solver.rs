use super::{AnyJointPositionConstraint, AnyPositionConstraint, ThreadContext};
use crate::dynamics::solver::{
    AnyJointVelocityConstraint, AnyVelocityConstraint, ParallelSolverConstraints,
};
use crate::dynamics::IntegrationParameters;
use crate::math::{Isometry, Real};
use std::sync::atomic::Ordering;

pub(crate) struct ParallelPositionSolver;

impl ParallelPositionSolver {
    pub fn solve(
        thread: &ThreadContext,
        params: &IntegrationParameters,
        positions: &mut [Isometry<Real>],
        contact_constraints: &mut ParallelSolverConstraints<
            AnyVelocityConstraint,
            AnyPositionConstraint,
        >,
        joint_constraints: &mut ParallelSolverConstraints<
            AnyJointVelocityConstraint,
            AnyJointPositionConstraint,
        >,
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
            // a palallel_desc_group, we have to wait util the current group is finished
            // before starting the next one.
            let mut start_index = thread
                .solve_position_interaction_index
                .fetch_add(thread.batch_size, Ordering::SeqCst);
            let mut batch_size = thread.batch_size;
            let contact_descs = &contact_constraints.constraint_descs[..];
            let joint_descs = &joint_constraints.constraint_descs[..];
            let mut target_num_desc = 0;
            let mut shift = 0;

            for _ in 0..params.max_position_iterations {
                macro_rules! solve {
                    ($part: expr) => {
                        // Joint groups.
                        for group in $part.parallel_desc_groups.windows(2) {
                            let num_descs_in_group = group[1] - group[0];
                            target_num_desc += num_descs_in_group;

                            while start_index < group[1] {
                                let end_index = (start_index + batch_size).min(group[1]);

                                let constraints = if end_index == $part.constraint_descs.len() {
                                    &mut $part.position_constraints
                                        [$part.constraint_descs[start_index].0..]
                                } else {
                                    &mut $part.position_constraints[$part.constraint_descs
                                        [start_index]
                                        .0
                                        ..$part.constraint_descs[end_index].0]
                                };

                                for constraint in constraints {
                                    constraint.solve(params, positions);
                                }

                                let num_solved = end_index - start_index;
                                batch_size -= num_solved;

                                thread
                                    .num_solved_position_interactions
                                    .fetch_add(num_solved, Ordering::SeqCst);

                                if batch_size == 0 {
                                    start_index = thread
                                        .solve_position_interaction_index
                                        .fetch_add(thread.batch_size, Ordering::SeqCst);
                                    start_index -= shift;
                                    batch_size = thread.batch_size;
                                } else {
                                    start_index += num_solved;
                                }
                            }
                            ThreadContext::lock_until_ge(
                                &thread.num_solved_position_interactions,
                                target_num_desc,
                            );
                        }
                    };
                }

                solve!(joint_constraints);
                shift += joint_descs.len();
                start_index -= joint_descs.len();
                solve!(contact_constraints);
                shift += contact_descs.len();
                start_index -= contact_descs.len();
            }
        }
    }
}
