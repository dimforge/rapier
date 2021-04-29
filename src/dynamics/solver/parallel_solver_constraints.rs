use super::ParallelInteractionGroups;
use super::{AnyJointVelocityConstraint, AnyVelocityConstraint, ThreadContext};
use crate::data::ComponentSet;
use crate::dynamics::solver::categorization::{categorize_contacts, categorize_joints};
use crate::dynamics::solver::{
    AnyJointPositionConstraint, AnyPositionConstraint, InteractionGroups, PositionConstraint,
    PositionGroundConstraint, VelocityConstraint, VelocityGroundConstraint,
};
use crate::dynamics::{
    IntegrationParameters, IslandManager, JointGraphEdge, RigidBodyIds, RigidBodyMassProps,
    RigidBodyPosition, RigidBodyType, RigidBodyVelocity,
};
use crate::geometry::ContactManifold;
#[cfg(feature = "simd-is-enabled")]
use crate::{
    dynamics::solver::{
        WPositionConstraint, WPositionGroundConstraint, WVelocityConstraint,
        WVelocityGroundConstraint,
    },
    math::SIMD_WIDTH,
};
use std::sync::atomic::Ordering;

// pub fn init_constraint_groups(
//     &mut self,
//     island_id: usize,
//     bodies: &impl ComponentSet<RigidBody>,
//     manifolds: &mut [&mut ContactManifold],
//     manifold_groups: &ParallelInteractionGroups,
//     joints: &mut [JointGraphEdge],
//     joint_groups: &ParallelInteractionGroups,
// ) {
//     self.part
//         .init_constraints_groups(island_id, bodies, manifolds, manifold_groups);
//     self.joint_part
//         .init_constraints_groups(island_id, bodies, joints, joint_groups);
// }

pub(crate) enum ConstraintDesc {
    NongroundNongrouped(usize),
    GroundNongrouped(usize),
    #[cfg(feature = "simd-is-enabled")]
    NongroundGrouped([usize; SIMD_WIDTH]),
    #[cfg(feature = "simd-is-enabled")]
    GroundGrouped([usize; SIMD_WIDTH]),
}

pub(crate) struct ParallelSolverConstraints<VelocityConstraint, PositionConstraint> {
    pub not_ground_interactions: Vec<usize>,
    pub ground_interactions: Vec<usize>,
    pub interaction_groups: InteractionGroups,
    pub ground_interaction_groups: InteractionGroups,
    pub velocity_constraints: Vec<VelocityConstraint>,
    pub position_constraints: Vec<PositionConstraint>,
    pub constraint_descs: Vec<(usize, ConstraintDesc)>,
    pub parallel_desc_groups: Vec<usize>,
}

impl<VelocityConstraint, PositionConstraint>
    ParallelSolverConstraints<VelocityConstraint, PositionConstraint>
{
    pub fn new() -> Self {
        Self {
            not_ground_interactions: Vec::new(),
            ground_interactions: Vec::new(),
            interaction_groups: InteractionGroups::new(),
            ground_interaction_groups: InteractionGroups::new(),
            velocity_constraints: Vec::new(),
            position_constraints: Vec::new(),
            constraint_descs: Vec::new(),
            parallel_desc_groups: Vec::new(),
        }
    }
}

macro_rules! impl_init_constraints_group {
    ($VelocityConstraint: ty, $PositionConstraint: ty, $Interaction: ty,
     $categorize: ident, $group: ident,
     $data: ident$(.$constraint_index: ident)*,
     $num_active_constraints: path, $empty_velocity_constraint: expr, $empty_position_constraint: expr $(, $weight: ident)*) => {
        impl ParallelSolverConstraints<$VelocityConstraint, $PositionConstraint> {
            pub fn init_constraint_groups<Bodies>(
                &mut self,
                island_id: usize,
                islands: &IslandManager,
                bodies: &Bodies,
                interactions: &mut [$Interaction],
                interaction_groups: &ParallelInteractionGroups,
            ) where Bodies: ComponentSet<RigidBodyType> + ComponentSet<RigidBodyIds> {
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
                        islands,
                        bodies,
                        interactions,
                        &self.not_ground_interactions,
                    );
                    self.ground_interaction_groups.$group(
                        island_id,
                        islands,
                        bodies,
                        interactions,
                        &self.ground_interactions,
                    );

                    // Compute constraint indices.
                    for interaction_i in &self.interaction_groups.nongrouped_interactions[start_nongrouped..] {
                        let interaction = &mut interactions[*interaction_i]$(.$weight)*;
                        interaction.$data$(.$constraint_index)* = total_num_constraints;
                        self.constraint_descs.push((
                            total_num_constraints,
                            ConstraintDesc::NongroundNongrouped(*interaction_i),
                        ));
                        total_num_constraints += $num_active_constraints(interaction);
                    }

                    #[cfg(feature = "simd-is-enabled")]
                    for interaction_i in
                        self.interaction_groups.grouped_interactions[start_grouped..].chunks(SIMD_WIDTH)
                    {
                        let interaction = &mut interactions[interaction_i[0]]$(.$weight)*;
                        interaction.$data$(.$constraint_index)* = total_num_constraints;
                        self.constraint_descs.push((
                            total_num_constraints,
                            ConstraintDesc::NongroundGrouped(
                                gather![|ii| interaction_i[ii]],
                            ),
                        ));
                        total_num_constraints += $num_active_constraints(interaction);
                    }

                    for interaction_i in
                        &self.ground_interaction_groups.nongrouped_interactions[start_nongrouped_ground..]
                    {
                        let interaction = &mut interactions[*interaction_i]$(.$weight)*;
                        interaction.$data$(.$constraint_index)* = total_num_constraints;
                        self.constraint_descs.push((
                            total_num_constraints,
                            ConstraintDesc::GroundNongrouped(*interaction_i),
                        ));
                        total_num_constraints += $num_active_constraints(interaction);
                    }

                    #[cfg(feature = "simd-is-enabled")]
                    for interaction_i in self.ground_interaction_groups.grouped_interactions
                        [start_grouped_ground..]
                        .chunks(SIMD_WIDTH)
                    {
                        let interaction = &mut interactions[interaction_i[0]]$(.$weight)*;
                        interaction.$data$(.$constraint_index)* = total_num_constraints;
                        self.constraint_descs.push((
                            total_num_constraints,
                            ConstraintDesc::GroundGrouped(
                                gather![|ii| interaction_i[ii]],
                            ),
                        ));
                        total_num_constraints += $num_active_constraints(interaction);
                    }

                    self.parallel_desc_groups.push(self.constraint_descs.len());
                }

                // Resize the constraint sets.
                self.velocity_constraints.clear();
                self.velocity_constraints
                    .resize_with(total_num_constraints, || $empty_velocity_constraint);
                self.position_constraints.clear();
                self.position_constraints
                    .resize_with(total_num_constraints, || $empty_position_constraint);
            }
        }
    }
}

impl_init_constraints_group!(
    AnyVelocityConstraint,
    AnyPositionConstraint,
    &mut ContactManifold,
    categorize_contacts,
    group_manifolds,
    data.constraint_index,
    VelocityConstraint::num_active_constraints,
    AnyVelocityConstraint::Empty,
    AnyPositionConstraint::Empty
);

impl_init_constraints_group!(
    AnyJointVelocityConstraint,
    AnyJointPositionConstraint,
    JointGraphEdge,
    categorize_joints,
    group_joints,
    constraint_index,
    AnyJointVelocityConstraint::num_active_constraints,
    AnyJointVelocityConstraint::Empty,
    AnyJointPositionConstraint::Empty,
    weight
);

impl ParallelSolverConstraints<AnyVelocityConstraint, AnyPositionConstraint> {
    pub fn fill_constraints<Bodies>(
        &mut self,
        thread: &ThreadContext,
        params: &IntegrationParameters,
        bodies: &Bodies,
        manifolds_all: &[&mut ContactManifold],
    ) where
        Bodies: ComponentSet<RigidBodyIds>
            + ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>,
    {
        let descs = &self.constraint_descs;

        crate::concurrent_loop! {
            let batch_size = thread.batch_size;
            for desc in descs[thread.constraint_initialization_index, thread.num_initialized_constraints] {
                match &desc.1 {
                    ConstraintDesc::NongroundNongrouped(manifold_id) => {
                        let manifold = &*manifolds_all[*manifold_id];
                        VelocityConstraint::generate(params, *manifold_id, manifold, bodies, &mut self.velocity_constraints, false);
                        PositionConstraint::generate(params, manifold, bodies, &mut self.position_constraints, false);
                    }
                    ConstraintDesc::GroundNongrouped(manifold_id) => {
                        let manifold = &*manifolds_all[*manifold_id];
                        VelocityGroundConstraint::generate(params, *manifold_id, manifold, bodies, &mut self.velocity_constraints, false);
                        PositionGroundConstraint::generate(params, manifold, bodies, &mut self.position_constraints, false);
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    ConstraintDesc::NongroundGrouped(manifold_id) => {
                        let manifolds = gather![|ii| &*manifolds_all[manifold_id[ii]]];
                        WVelocityConstraint::generate(params, *manifold_id, manifolds, bodies, &mut self.velocity_constraints, false);
                        WPositionConstraint::generate(params, manifolds, bodies, &mut self.position_constraints, false);
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    ConstraintDesc::GroundGrouped(manifold_id) => {
                        let manifolds = gather![|ii| &*manifolds_all[manifold_id[ii]]];
                        WVelocityGroundConstraint::generate(params, *manifold_id, manifolds, bodies, &mut self.velocity_constraints, false);
                        WPositionGroundConstraint::generate(params, manifolds, bodies, &mut self.position_constraints, false);
                    }
                }
            }
        }
    }
}

impl ParallelSolverConstraints<AnyJointVelocityConstraint, AnyJointPositionConstraint> {
    pub fn fill_constraints<Bodies>(
        &mut self,
        thread: &ThreadContext,
        params: &IntegrationParameters,
        bodies: &Bodies,
        joints_all: &[JointGraphEdge],
    ) where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>
            + ComponentSet<RigidBodyType>,
    {
        let descs = &self.constraint_descs;

        crate::concurrent_loop! {
            let batch_size = thread.batch_size;
            for desc in descs[thread.joint_constraint_initialization_index, thread.num_initialized_joint_constraints] {
                match &desc.1 {
                    ConstraintDesc::NongroundNongrouped(joint_id) => {
                        let joint = &joints_all[*joint_id].weight;
                        let velocity_constraint = AnyJointVelocityConstraint::from_joint(params, *joint_id, joint, bodies);
                        let position_constraint = AnyJointPositionConstraint::from_joint(joint, bodies);
                        self.velocity_constraints[joint.constraint_index] = velocity_constraint;
                        self.position_constraints[joint.constraint_index] = position_constraint;
                    }
                    ConstraintDesc::GroundNongrouped(joint_id) => {
                        let joint = &joints_all[*joint_id].weight;
                        let velocity_constraint = AnyJointVelocityConstraint::from_joint_ground(params, *joint_id, joint, bodies);
                        let position_constraint = AnyJointPositionConstraint::from_joint_ground(joint, bodies);
                        self.velocity_constraints[joint.constraint_index] = velocity_constraint;
                        self.position_constraints[joint.constraint_index] = position_constraint;
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    ConstraintDesc::NongroundGrouped(joint_id) => {
                        let joints = gather![|ii| &joints_all[joint_id[ii]].weight];
                        let velocity_constraint = AnyJointVelocityConstraint::from_wide_joint(params, *joint_id, joints, bodies);
                        let position_constraint = AnyJointPositionConstraint::from_wide_joint(joints, bodies);
                        self.velocity_constraints[joints[0].constraint_index] = velocity_constraint;
                        self.position_constraints[joints[0].constraint_index] = position_constraint;
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    ConstraintDesc::GroundGrouped(joint_id) => {
                        let joints = gather![|ii| &joints_all[joint_id[ii]].weight];
                        let velocity_constraint = AnyJointVelocityConstraint::from_wide_joint_ground(params, *joint_id, joints, bodies);
                        let position_constraint = AnyJointPositionConstraint::from_wide_joint_ground(joints, bodies);
                        self.velocity_constraints[joints[0].constraint_index] = velocity_constraint;
                        self.position_constraints[joints[0].constraint_index] = position_constraint;
                    }
                }
            }
        }
    }
}
