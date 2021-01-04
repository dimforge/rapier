use super::ParallelInteractionGroups;
use super::{AnyJointPositionConstraint, AnyPositionConstraint, ThreadContext};
use crate::dynamics::solver::categorization::{categorize_joints, categorize_position_contacts};
use crate::dynamics::solver::{InteractionGroups, PositionConstraint, PositionGroundConstraint};
use crate::dynamics::{IntegrationParameters, JointGraphEdge, RigidBodySet};
use crate::geometry::ContactManifold;
use crate::math::{Isometry, Real};
#[cfg(feature = "simd-is-enabled")]
use crate::{
    dynamics::solver::{WPositionConstraint, WPositionGroundConstraint},
    simd::SIMD_WIDTH,
};
use std::sync::atomic::Ordering;

pub(crate) enum PositionConstraintDesc {
    NongroundNongrouped(usize),
    GroundNongrouped(usize),
    #[cfg(feature = "simd-is-enabled")]
    NongroundGrouped([usize; SIMD_WIDTH]),
    #[cfg(feature = "simd-is-enabled")]
    GroundGrouped([usize; SIMD_WIDTH]),
}

pub(crate) struct ParallelPositionSolverContactPart {
    pub point_point: Vec<usize>,
    pub plane_point: Vec<usize>,
    pub ground_point_point: Vec<usize>,
    pub ground_plane_point: Vec<usize>,
    pub interaction_groups: InteractionGroups,
    pub ground_interaction_groups: InteractionGroups,
    pub constraints: Vec<AnyPositionConstraint>,
    pub constraint_descs: Vec<(usize, PositionConstraintDesc)>,
    pub parallel_desc_groups: Vec<usize>,
}

pub(crate) struct ParallelPositionSolverJointPart {
    pub not_ground_interactions: Vec<usize>,
    pub ground_interactions: Vec<usize>,
    pub interaction_groups: InteractionGroups,
    pub ground_interaction_groups: InteractionGroups,
    pub constraints: Vec<AnyJointPositionConstraint>,
    pub constraint_descs: Vec<(usize, PositionConstraintDesc)>,
    pub parallel_desc_groups: Vec<usize>,
}

impl ParallelPositionSolverContactPart {
    pub fn new() -> Self {
        Self {
            point_point: Vec::new(),
            plane_point: Vec::new(),
            ground_point_point: Vec::new(),
            ground_plane_point: Vec::new(),
            interaction_groups: InteractionGroups::new(),
            ground_interaction_groups: InteractionGroups::new(),
            constraints: Vec::new(),
            constraint_descs: Vec::new(),
            parallel_desc_groups: Vec::new(),
        }
    }
}
impl ParallelPositionSolverJointPart {
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

impl ParallelPositionSolverJointPart {
    pub fn init_constraints_groups(
        &mut self,
        island_id: usize,
        bodies: &RigidBodySet,
        joints: &mut [JointGraphEdge],
        joint_groups: &ParallelInteractionGroups,
    ) {
        let mut total_num_constraints = 0;
        let num_groups = joint_groups.num_groups();

        self.interaction_groups.clear_groups();
        self.ground_interaction_groups.clear_groups();
        self.parallel_desc_groups.clear();
        self.constraint_descs.clear();
        self.parallel_desc_groups.push(0);

        for i in 0..num_groups {
            let group = joint_groups.group(i);

            self.not_ground_interactions.clear();
            self.ground_interactions.clear();
            categorize_joints(
                bodies,
                joints,
                group,
                &mut self.ground_interactions,
                &mut self.not_ground_interactions,
            );

            #[cfg(feature = "simd-is-enabled")]
            let start_grouped = self.interaction_groups.grouped_interactions.len();
            let start_nongrouped = self.interaction_groups.nongrouped_interactions.len();
            #[cfg(feature = "simd-is-enabled")]
            let start_grouped_ground = self.ground_interaction_groups.grouped_interactions.len();
            let start_nongrouped_ground =
                self.ground_interaction_groups.nongrouped_interactions.len();

            self.interaction_groups.group_joints(
                island_id,
                bodies,
                joints,
                &self.not_ground_interactions,
            );
            self.ground_interaction_groups.group_joints(
                island_id,
                bodies,
                joints,
                &self.ground_interactions,
            );

            // Compute constraint indices.
            for interaction_i in
                &self.interaction_groups.nongrouped_interactions[start_nongrouped..]
            {
                let joint = &mut joints[*interaction_i].weight;
                joint.position_constraint_index = total_num_constraints;
                self.constraint_descs.push((
                    total_num_constraints,
                    PositionConstraintDesc::NongroundNongrouped(*interaction_i),
                ));
                total_num_constraints +=
                    AnyJointPositionConstraint::num_active_constraints(joint, false);
            }

            #[cfg(feature = "simd-is-enabled")]
            for interaction_i in
                self.interaction_groups.grouped_interactions[start_grouped..].chunks(SIMD_WIDTH)
            {
                let joint = &mut joints[interaction_i[0]].weight;
                joint.position_constraint_index = total_num_constraints;
                self.constraint_descs.push((
                    total_num_constraints,
                    PositionConstraintDesc::NongroundGrouped(
                        array![|ii| interaction_i[ii]; SIMD_WIDTH],
                    ),
                ));
                total_num_constraints +=
                    AnyJointPositionConstraint::num_active_constraints(joint, true);
            }

            for interaction_i in
                &self.ground_interaction_groups.nongrouped_interactions[start_nongrouped_ground..]
            {
                let joint = &mut joints[*interaction_i].weight;
                joint.position_constraint_index = total_num_constraints;
                self.constraint_descs.push((
                    total_num_constraints,
                    PositionConstraintDesc::GroundNongrouped(*interaction_i),
                ));
                total_num_constraints +=
                    AnyJointPositionConstraint::num_active_constraints(joint, false);
            }

            #[cfg(feature = "simd-is-enabled")]
            for interaction_i in self.ground_interaction_groups.grouped_interactions
                [start_grouped_ground..]
                .chunks(SIMD_WIDTH)
            {
                let joint = &mut joints[interaction_i[0]].weight;
                joint.position_constraint_index = total_num_constraints;
                self.constraint_descs.push((
                    total_num_constraints,
                    PositionConstraintDesc::GroundGrouped(
                        array![|ii| interaction_i[ii]; SIMD_WIDTH],
                    ),
                ));
                total_num_constraints +=
                    AnyJointPositionConstraint::num_active_constraints(joint, true);
            }

            self.parallel_desc_groups.push(self.constraint_descs.len());
        }

        // Resize the constraints set.
        self.constraints.clear();
        self.constraints
            .resize_with(total_num_constraints, || AnyJointPositionConstraint::Empty)
    }

    fn fill_constraints(
        &mut self,
        thread: &ThreadContext,
        bodies: &RigidBodySet,
        joints_all: &[JointGraphEdge],
    ) {
        let descs = &self.constraint_descs;

        crate::concurrent_loop! {
            let batch_size = thread.batch_size;
            for desc in descs[thread.position_joint_constraint_initialization_index, thread.num_initialized_position_joint_constraints] {
                match &desc.1 {
                    PositionConstraintDesc::NongroundNongrouped(joint_id) => {
                        let joint = &joints_all[*joint_id].weight;
                        let constraint = AnyJointPositionConstraint::from_joint(
                            joint,
                            bodies,
                        );
                        self.constraints[joint.position_constraint_index] = constraint;
                    }
                    PositionConstraintDesc::GroundNongrouped(joint_id) => {
                        let joint = &joints_all[*joint_id].weight;
                        let constraint = AnyJointPositionConstraint::from_joint_ground(
                            joint,
                            bodies,
                        );
                        self.constraints[joint.position_constraint_index] = constraint;
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    PositionConstraintDesc::NongroundGrouped(joint_id) => {
                        let joints = array![|ii| &joints_all[joint_id[ii]].weight; SIMD_WIDTH];
                        if let Some(constraint) = AnyJointPositionConstraint::from_wide_joint(
                            joints, bodies,
                        ) {
                            self.constraints[joints[0].position_constraint_index] = constraint
                        } else {
                            for ii in 0..SIMD_WIDTH {
                                let constraint = AnyJointPositionConstraint::from_joint(joints[ii], bodies);
                                self.constraints[joints[0].position_constraint_index + ii] = constraint;
                            }
                        }
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    PositionConstraintDesc::GroundGrouped(joint_id) => {
                        let joints = array![|ii| &joints_all[joint_id[ii]].weight; SIMD_WIDTH];
                        if let Some(constraint) = AnyJointPositionConstraint::from_wide_joint_ground(
                            joints, bodies,
                        ) {
                            self.constraints[joints[0].position_constraint_index] = constraint
                        } else {
                            for ii in 0..SIMD_WIDTH {
                                let constraint = AnyJointPositionConstraint::from_joint_ground(joints[ii], bodies);
                                self.constraints[joints[0].position_constraint_index + ii] = constraint;
                            }
                        }
                    }
                }
            }
        }
    }
}

impl ParallelPositionSolverContactPart {
    pub fn init_constraints_groups(
        &mut self,
        island_id: usize,
        bodies: &RigidBodySet,
        manifolds: &mut [&mut ContactManifold],
        manifold_groups: &ParallelInteractionGroups,
    ) {
        let mut total_num_constraints = 0;
        let num_groups = manifold_groups.num_groups();

        self.interaction_groups.clear_groups();
        self.ground_interaction_groups.clear_groups();
        self.parallel_desc_groups.clear();
        self.constraint_descs.clear();
        self.parallel_desc_groups.push(0);

        for i in 0..num_groups {
            let group = manifold_groups.group(i);

            self.ground_point_point.clear();
            self.ground_plane_point.clear();
            self.point_point.clear();
            self.plane_point.clear();
            categorize_position_contacts(
                bodies,
                manifolds,
                group,
                &mut self.ground_point_point,
                &mut self.ground_plane_point,
                &mut self.point_point,
                &mut self.plane_point,
            );

            #[cfg(feature = "simd-is-enabled")]
            let start_grouped = self.interaction_groups.grouped_interactions.len();
            let start_nongrouped = self.interaction_groups.nongrouped_interactions.len();
            #[cfg(feature = "simd-is-enabled")]
            let start_grouped_ground = self.ground_interaction_groups.grouped_interactions.len();
            let start_nongrouped_ground =
                self.ground_interaction_groups.nongrouped_interactions.len();

            self.interaction_groups.group_manifolds(
                island_id,
                bodies,
                manifolds,
                &self.point_point,
            );
            self.interaction_groups.group_manifolds(
                island_id,
                bodies,
                manifolds,
                &self.plane_point,
            );
            self.ground_interaction_groups.group_manifolds(
                island_id,
                bodies,
                manifolds,
                &self.ground_point_point,
            );
            self.ground_interaction_groups.group_manifolds(
                island_id,
                bodies,
                manifolds,
                &self.ground_plane_point,
            );

            // Compute constraint indices.
            for interaction_i in
                &self.interaction_groups.nongrouped_interactions[start_nongrouped..]
            {
                let manifold = &mut *manifolds[*interaction_i];
                manifold.position_constraint_index = total_num_constraints;
                self.constraint_descs.push((
                    total_num_constraints,
                    PositionConstraintDesc::NongroundNongrouped(*interaction_i),
                ));
                total_num_constraints += PositionConstraint::num_active_constraints(manifold);
            }

            #[cfg(feature = "simd-is-enabled")]
            for interaction_i in
                self.interaction_groups.grouped_interactions[start_grouped..].chunks(SIMD_WIDTH)
            {
                let manifold = &mut *manifolds[interaction_i[0]];
                manifold.position_constraint_index = total_num_constraints;
                self.constraint_descs.push((
                    total_num_constraints,
                    PositionConstraintDesc::NongroundGrouped(
                        array![|ii| interaction_i[ii]; SIMD_WIDTH],
                    ),
                ));
                total_num_constraints += PositionConstraint::num_active_constraints(manifold);
            }

            for interaction_i in
                &self.ground_interaction_groups.nongrouped_interactions[start_nongrouped_ground..]
            {
                let manifold = &mut *manifolds[*interaction_i];
                manifold.position_constraint_index = total_num_constraints;
                self.constraint_descs.push((
                    total_num_constraints,
                    PositionConstraintDesc::GroundNongrouped(*interaction_i),
                ));
                total_num_constraints += PositionConstraint::num_active_constraints(manifold);
            }

            #[cfg(feature = "simd-is-enabled")]
            for interaction_i in self.ground_interaction_groups.grouped_interactions
                [start_grouped_ground..]
                .chunks(SIMD_WIDTH)
            {
                let manifold = &mut *manifolds[interaction_i[0]];
                manifold.position_constraint_index = total_num_constraints;
                self.constraint_descs.push((
                    total_num_constraints,
                    PositionConstraintDesc::GroundGrouped(
                        array![|ii| interaction_i[ii]; SIMD_WIDTH],
                    ),
                ));
                total_num_constraints += PositionConstraint::num_active_constraints(manifold);
            }

            self.parallel_desc_groups.push(self.constraint_descs.len());
        }

        // Resize the constraints set.
        self.constraints.clear();
        self.constraints
            .resize_with(total_num_constraints, || AnyPositionConstraint::Empty)
    }

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
            for desc in descs[thread.position_constraint_initialization_index, thread.num_initialized_position_constraints] {
                match &desc.1 {
                    PositionConstraintDesc::NongroundNongrouped(manifold_id) => {
                        let manifold = &*manifolds_all[*manifold_id];
                        PositionConstraint::generate(
                            params,
                            manifold,
                            bodies,
                            &mut self.constraints,
                            false,
                        );
                    }
                    PositionConstraintDesc::GroundNongrouped(manifold_id) => {
                        let manifold = &*manifolds_all[*manifold_id];
                        PositionGroundConstraint::generate(
                            params,
                            manifold,
                            bodies,
                            &mut self.constraints,
                            false,
                        );
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    PositionConstraintDesc::NongroundGrouped(manifold_id) => {
                        let manifolds = array![|ii| &*manifolds_all[manifold_id[ii]]; SIMD_WIDTH];
                        WPositionConstraint::generate(
                            params,
                            manifolds,
                            bodies,
                            &mut self.constraints,
                            false,
                        );
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    PositionConstraintDesc::GroundGrouped(manifold_id) => {
                        let manifolds = array![|ii| &*manifolds_all[manifold_id[ii]]; SIMD_WIDTH];
                        WPositionGroundConstraint::generate(
                            params,
                            manifolds,
                            bodies,
                            &mut self.constraints,
                            false,
                        );
                    }
                }
            }
        }
    }
}

pub(crate) struct ParallelPositionSolver {
    part: ParallelPositionSolverContactPart,
    joint_part: ParallelPositionSolverJointPart,
}

impl ParallelPositionSolver {
    pub fn new() -> Self {
        Self {
            part: ParallelPositionSolverContactPart::new(),
            joint_part: ParallelPositionSolverJointPart::new(),
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
        self.joint_part.fill_constraints(thread, bodies, joints);
        ThreadContext::lock_until_ge(
            &thread.num_initialized_position_constraints,
            self.part.constraint_descs.len(),
        );
        ThreadContext::lock_until_ge(
            &thread.num_initialized_position_joint_constraints,
            self.joint_part.constraint_descs.len(),
        );
    }

    pub fn solve_constraints(
        &mut self,
        thread: &ThreadContext,
        params: &IntegrationParameters,
        positions: &mut [Isometry<Real>],
    ) {
        if self.part.constraint_descs.len() == 0 {
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
            let contact_descs = &self.part.constraint_descs[..];
            let joint_descs = &self.joint_part.constraint_descs[..];
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
                                    &mut $part.constraints[$part.constraint_descs[start_index].0..]
                                } else {
                                    &mut $part.constraints[$part.constraint_descs[start_index].0
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

                solve!(self.joint_part);
                shift += joint_descs.len();
                start_index -= joint_descs.len();
                solve!(self.part);
                shift += contact_descs.len();
                start_index -= contact_descs.len();
            }
        }
    }
}
