use super::ParallelInteractionGroups;
use super::{AnyJointVelocityConstraint, AnyVelocityConstraint, DeltaVel, ThreadContext};
use crate::dynamics::solver::categorization::{categorize_contacts, categorize_joints};
use crate::dynamics::solver::{InteractionGroups, VelocityConstraint, VelocityGroundConstraint};
use crate::dynamics::{IntegrationParameters, JointGraphEdge, RigidBodySet};
use crate::geometry::ContactManifold;
#[cfg(feature = "simd-is-enabled")]
use crate::{
    dynamics::solver::{WVelocityConstraint, WVelocityGroundConstraint},
    simd::SIMD_WIDTH,
};
use std::sync::atomic::Ordering;

pub(crate) enum VelocityConstraintDesc {
    NongroundNongrouped(usize),
    GroundNongrouped(usize),
    #[cfg(feature = "simd-is-enabled")]
    NongroundGrouped([usize; SIMD_WIDTH]),
    #[cfg(feature = "simd-is-enabled")]
    GroundGrouped([usize; SIMD_WIDTH]),
}

pub(crate) struct ParallelSolverConstraints<Constraint> {
    pub not_ground_interactions: Vec<usize>,
    pub ground_interactions: Vec<usize>,
    pub interaction_groups: InteractionGroups,
    pub ground_interaction_groups: InteractionGroups,
    pub constraints: Vec<Constraint>,
    pub constraint_descs: Vec<(usize, VelocityConstraintDesc)>,
    pub parallel_desc_groups: Vec<usize>,
}

impl<Constraint> ParallelSolverConstraints<Constraint> {
    pub fn new() -> Self {
        Self {
            not_ground_interactions: Vec::new(),
            ground_interactions: Vec::new(),
            interaction_groups: InteractionGroups::new(),
            ground_interaction_groups: InteractionGroups::new(),
            constraints: Vec::new(),
            constraint_descs: Vec::new(),
            parallel_desc_groups: Vec::new(),
        }
    }
}

macro_rules! impl_init_constraints_group {
    ($Constraint: ty, $Interaction: ty, $categorize: ident, $group: ident, $num_active_constraints: path, $empty_constraint: expr $(, $weight: ident)*) => {
        impl ParallelSolverConstraints<$Constraint> {
            pub fn init_constraints_groups(
                &mut self,
                island_id: usize,
                bodies: &RigidBodySet,
                interactions: &mut [$Interaction],
                interaction_groups: &ParallelInteractionGroups,
            ) {
                let mut total_num_constraints = 0;
                let num_groups = interaction_groups.num_groups();

                self.interaction_groups.clear_groups();
                self.ground_interaction_groups.clear_groups();
                self.parallel_desc_groups.clear();
                self.constraint_descs.clear();
                self.parallel_desc_groups.push(0);

                for i in 0..num_groups {
                    let group = interaction_groups.group(i);

                    self.not_ground_interactions.clear();
                    self.ground_interactions.clear();
                    $categorize(
                        bodies,
                        interactions,
                        group,
                        &mut self.ground_interactions,
                        &mut self.not_ground_interactions,
                    );

                    #[cfg(feature = "simd-is-enabled")]
                    let start_grouped = self.interaction_groups.grouped_interactions.len();
                    let start_nongrouped = self.interaction_groups.nongrouped_interactions.len();
                    #[cfg(feature = "simd-is-enabled")]
                    let start_grouped_ground = self.ground_interaction_groups.grouped_interactions.len();
                    let start_nongrouped_ground = self.ground_interaction_groups.nongrouped_interactions.len();

                    self.interaction_groups.$group(
                        island_id,
                        bodies,
                        interactions,
                        &self.not_ground_interactions,
                    );
                    self.ground_interaction_groups.$group(
                        island_id,
                        bodies,
                        interactions,
                        &self.ground_interactions,
                    );

                    // Compute constraint indices.
                    for interaction_i in &self.interaction_groups.nongrouped_interactions[start_nongrouped..] {
                        let interaction = &mut interactions[*interaction_i]$(.$weight)*;
                        interaction.constraint_index = total_num_constraints;
                        self.constraint_descs.push((
                            total_num_constraints,
                            VelocityConstraintDesc::NongroundNongrouped(*interaction_i),
                        ));
                        total_num_constraints += $num_active_constraints(interaction);
                    }

                    #[cfg(feature = "simd-is-enabled")]
                    for interaction_i in
                        self.interaction_groups.grouped_interactions[start_grouped..].chunks(SIMD_WIDTH)
                    {
                        let interaction = &mut interactions[interaction_i[0]]$(.$weight)*;
                        interaction.constraint_index = total_num_constraints;
                        self.constraint_descs.push((
                            total_num_constraints,
                            VelocityConstraintDesc::NongroundGrouped(
                                array![|ii| interaction_i[ii]; SIMD_WIDTH],
                            ),
                        ));
                        total_num_constraints += $num_active_constraints(interaction);
                    }

                    for interaction_i in
                        &self.ground_interaction_groups.nongrouped_interactions[start_nongrouped_ground..]
                    {
                        let interaction = &mut interactions[*interaction_i]$(.$weight)*;
                        interaction.constraint_index = total_num_constraints;
                        self.constraint_descs.push((
                            total_num_constraints,
                            VelocityConstraintDesc::GroundNongrouped(*interaction_i),
                        ));
                        total_num_constraints += $num_active_constraints(interaction);
                    }

                    #[cfg(feature = "simd-is-enabled")]
                    for interaction_i in self.ground_interaction_groups.grouped_interactions
                        [start_grouped_ground..]
                        .chunks(SIMD_WIDTH)
                    {
                        let interaction = &mut interactions[interaction_i[0]]$(.$weight)*;
                        interaction.constraint_index = total_num_constraints;
                        self.constraint_descs.push((
                            total_num_constraints,
                            VelocityConstraintDesc::GroundGrouped(
                                array![|ii| interaction_i[ii]; SIMD_WIDTH],
                            ),
                        ));
                        total_num_constraints += $num_active_constraints(interaction);
                    }

                    self.parallel_desc_groups.push(self.constraint_descs.len());
                }

                // Resize the constraints set.
                self.constraints.clear();
                self.constraints
                    .resize_with(total_num_constraints, || $empty_constraint)
            }
        }
    }
}

impl_init_constraints_group!(
    AnyVelocityConstraint,
    &mut ContactManifold,
    categorize_contacts,
    group_manifolds,
    VelocityConstraint::num_active_constraints,
    AnyVelocityConstraint::Empty
);

impl_init_constraints_group!(
    AnyJointVelocityConstraint,
    JointGraphEdge,
    categorize_joints,
    group_joints,
    AnyJointVelocityConstraint::num_active_constraints,
    AnyJointVelocityConstraint::Empty,
    weight
);

impl ParallelSolverConstraints<AnyVelocityConstraint> {
    fn fill_constraints(
        &mut self,
        thread: &ThreadContext,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
    ) {
        let descs = &self.constraint_descs;

        crate::concurrent_loop! {
            let batch_size = thread.batch_size;
            for desc in descs[thread.constraint_initialization_index, thread.num_initialized_constraints] {
                match &desc.1 {
                    VelocityConstraintDesc::NongroundNongrouped(manifold_id) => {
                        let manifold = &*manifolds_all[*manifold_id];
                        VelocityConstraint::generate(params, *manifold_id, manifold, bodies, &mut self.constraints, false);
                    }
                    VelocityConstraintDesc::GroundNongrouped(manifold_id) => {
                        let manifold = &*manifolds_all[*manifold_id];
                        VelocityGroundConstraint::generate(params, *manifold_id, manifold, bodies, &mut self.constraints, false);
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    VelocityConstraintDesc::NongroundGrouped(manifold_id) => {
                        let manifolds = array![|ii| &*manifolds_all[manifold_id[ii]]; SIMD_WIDTH];
                        WVelocityConstraint::generate(params, *manifold_id, manifolds, bodies, &mut self.constraints, false);
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    VelocityConstraintDesc::GroundGrouped(manifold_id) => {
                        let manifolds = array![|ii| &*manifolds_all[manifold_id[ii]]; SIMD_WIDTH];
                        WVelocityGroundConstraint::generate(params, *manifold_id, manifolds, bodies, &mut self.constraints, false);
                    }
                }
            }
        }
    }
}

impl ParallelSolverConstraints<AnyJointVelocityConstraint> {
    fn fill_constraints(
        &mut self,
        thread: &ThreadContext,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        joints_all: &[JointGraphEdge],
    ) {
        let descs = &self.constraint_descs;

        crate::concurrent_loop! {
            let batch_size = thread.batch_size;
            for desc in descs[thread.joint_constraint_initialization_index, thread.num_initialized_joint_constraints] {
                match &desc.1 {
                    VelocityConstraintDesc::NongroundNongrouped(joint_id) => {
                        let joint = &joints_all[*joint_id].weight;
                        let constraint = AnyJointVelocityConstraint::from_joint(params, *joint_id, joint, bodies);
                        self.constraints[joint.constraint_index] = constraint;
                    }
                    VelocityConstraintDesc::GroundNongrouped(joint_id) => {
                        let joint = &joints_all[*joint_id].weight;
                        let constraint = AnyJointVelocityConstraint::from_joint_ground(params, *joint_id, joint, bodies);
                        self.constraints[joint.constraint_index] = constraint;
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    VelocityConstraintDesc::NongroundGrouped(joint_id) => {
                        let joints = array![|ii| &joints_all[joint_id[ii]].weight; SIMD_WIDTH];
                        let constraint = AnyJointVelocityConstraint::from_wide_joint(params, *joint_id, joints, bodies);
                        self.constraints[joints[0].data.constraint_index] = constraint;
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    VelocityConstraintDesc::GroundGrouped(joint_id) => {
                        let joints = array![|ii| &joints_all[joint_id[ii]].weight; SIMD_WIDTH];
                        let constraint = AnyJointVelocityConstraint::from_wide_joint_ground(params, *joint_id, joints, bodies);
                        self.constraints[joints[0].data.constraint_index] = constraint;
                    }
                }
            }
        }
    }
}

pub(crate) struct ParallelVelocitySolver {
    part: ParallelSolverConstraints<AnyVelocityConstraint>,
    joint_part: ParallelSolverConstraints<AnyJointVelocityConstraint>,
}

impl ParallelVelocitySolver {
    pub fn new() -> Self {
        Self {
            part: ParallelSolverConstraints::new(),
            joint_part: ParallelSolverConstraints::new(),
        }
    }

    pub fn init_constraint_groups(
        &mut self,
        island_id: usize,
        bodies: &RigidBodySet,
        manifolds: &mut [&mut ContactManifold],
        manifold_groups: &ParallelInteractionGroups,
        joints: &mut [JointGraphEdge],
        joint_groups: &ParallelInteractionGroups,
    ) {
        self.part
            .init_constraints_groups(island_id, bodies, manifolds, manifold_groups);
        self.joint_part
            .init_constraints_groups(island_id, bodies, joints, joint_groups);
    }

    pub fn fill_constraints(
        &mut self,
        thread: &ThreadContext,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds: &[&mut ContactManifold],
        joints: &[JointGraphEdge],
    ) {
        self.part
            .fill_constraints(thread, params, bodies, manifolds);
        self.joint_part
            .fill_constraints(thread, params, bodies, joints);
        ThreadContext::lock_until_ge(
            &thread.num_initialized_constraints,
            self.part.constraint_descs.len(),
        );
        ThreadContext::lock_until_ge(
            &thread.num_initialized_joint_constraints,
            self.joint_part.constraint_descs.len(),
        );
    }

    pub fn solve_constraints(
        &mut self,
        thread: &ThreadContext,
        params: &IntegrationParameters,
        manifolds_all: &mut [&mut ContactManifold],
        joints_all: &mut [JointGraphEdge],
        mj_lambdas: &mut [DeltaVel<Real>],
    ) {
        if self.part.constraint_descs.len() == 0 && self.joint_part.constraint_descs.len() == 0 {
            return;
        }

        /*
         * Warmstart constraints.
         */
        {
            // Each thread will concurrently grab thread.batch_size constraint desc to
            // solve. If the batch size is large enough for to cross the boundary of
            // a parallel_desc_group, we have to wait util the current group is finished
            // before starting the next one.
            let mut target_num_desc = 0;
            let mut start_index = thread
                .warmstart_contact_index
                .fetch_add(thread.batch_size, Ordering::SeqCst);
            let mut batch_size = thread.batch_size;
            let mut shift = 0;

            macro_rules! warmstart(
                ($part: expr) => {
                    for group in $part.parallel_desc_groups.windows(2) {
                        let num_descs_in_group = group[1] - group[0];
                        target_num_desc += num_descs_in_group;

                        while start_index < group[1] {
                            let end_index = (start_index + batch_size).min(group[1]);

                            let constraints = if end_index == $part.constraint_descs.len() {
                                &mut $part.constraints[$part.constraint_descs[start_index].0..]
                            } else {
                                &mut $part.constraints[$part.constraint_descs[start_index].0..$part.constraint_descs[end_index].0]
                            };

                            for constraint in constraints {
                                constraint.warmstart(mj_lambdas);
                            }

                            let num_solved = end_index - start_index;
                            batch_size -= num_solved;

                            thread
                                .num_warmstarted_contacts
                                .fetch_add(num_solved, Ordering::SeqCst);

                            if batch_size == 0 {
                                start_index = thread
                                    .warmstart_contact_index
                                    .fetch_add(thread.batch_size, Ordering::SeqCst);
                                start_index -= shift;
                                batch_size = thread.batch_size;
                            } else {
                                start_index += num_solved;
                            }
                        }

                        ThreadContext::lock_until_ge(&thread.num_warmstarted_contacts, target_num_desc);
                    }
                }
            );

            warmstart!(self.joint_part);
            shift = self.joint_part.constraint_descs.len();
            start_index -= shift;
            warmstart!(self.part);
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
            let contact_descs = &self.part.constraint_descs[..];
            let joint_descs = &self.joint_part.constraint_descs[..];
            let mut target_num_desc = 0;
            let mut shift = 0;

            for _ in 0..params.max_velocity_iterations {
                macro_rules! solve {
                    ($part: expr) => {
                        // Joint groups.
                        for group in $part.parallel_desc_groups.windows(2) {
                            let num_descs_in_group = group[1] - group[0];

                            target_num_desc += num_descs_in_group;

                            while start_index < group[1] {
                                let end_index = (start_index + batch_size).min(group[1]);

                                let constraints = if end_index == $part.constraint_descs.len() {
                                    &mut $part.constraints[$part.constraint_descs[start_index].0..]
                                } else {
                                    &mut $part.constraints[$part.constraint_descs[start_index].0
                                        ..$part.constraint_descs[end_index].0]
                                };

                                //                                println!(
                                //                                    "Solving a constraint {:?}.",
                                //                                    rayon::current_thread_index()
                                //                                );
                                for constraint in constraints {
                                    constraint.solve(mj_lambdas);
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

                solve!(self.joint_part);
                shift += joint_descs.len();
                start_index -= joint_descs.len();
                solve!(self.part);
                shift += contact_descs.len();
                start_index -= contact_descs.len();
            }
        }

        /*
         * Writeback impulses.
         */
        let joint_constraints = &self.joint_part.constraints;
        let contact_constraints = &self.part.constraints;
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
