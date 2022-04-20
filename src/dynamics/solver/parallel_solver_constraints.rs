use super::ParallelInteractionGroups;
use super::{AnyJointVelocityConstraint, AnyVelocityConstraint, ThreadContext};
use crate::dynamics::solver::categorization::{categorize_contacts, categorize_joints};
use crate::dynamics::solver::generic_velocity_constraint::GenericVelocityConstraint;
use crate::dynamics::solver::{
    GenericVelocityGroundConstraint, InteractionGroups, VelocityConstraint,
    VelocityGroundConstraint,
};
use crate::dynamics::{
    ImpulseJoint, IntegrationParameters, IslandManager, JointGraphEdge, MultibodyIndex,
    MultibodyJointSet, RigidBodyHandle, RigidBodySet,
};
use crate::geometry::ContactManifold;
use crate::math::{Real, SPATIAL_DIM};
#[cfg(feature = "simd-is-enabled")]
use crate::{
    dynamics::solver::{WVelocityConstraint, WVelocityGroundConstraint},
    math::SIMD_WIDTH,
};
use na::DVector;
use std::sync::atomic::Ordering;

// pub fn init_constraint_groups(
//     &mut self,
//     island_id: usize,
//     bodies: &impl ComponentSet<RigidBody>,
//     manifolds: &mut [&mut ContactManifold],
//     manifold_groups: &ParallelInteractionGroups,
//     impulse_joints: &mut [JointGraphEdge],
//     joint_groups: &ParallelInteractionGroups,
// ) {
//     self.part
//         .init_constraints_groups(island_id, bodies, manifolds, manifold_groups);
//     self.joint_part
//         .init_constraints_groups(island_id, bodies, impulse_joints, joint_groups);
// }

pub(crate) enum ConstraintDesc {
    NongroundNongrouped(usize),
    GroundNongrouped(usize),
    #[cfg(feature = "simd-is-enabled")]
    NongroundGrouped([usize; SIMD_WIDTH]),
    #[cfg(feature = "simd-is-enabled")]
    GroundGrouped([usize; SIMD_WIDTH]),
    GenericNongroundNongrouped(usize, usize),
    GenericGroundNongrouped(usize, usize),
    GenericMultibodyInternal(MultibodyIndex, usize),
}

pub(crate) struct ParallelSolverConstraints<VelocityConstraint> {
    pub generic_jacobians: DVector<Real>,
    pub not_ground_interactions: Vec<usize>,
    pub ground_interactions: Vec<usize>,
    pub generic_not_ground_interactions: Vec<usize>,
    pub generic_ground_interactions: Vec<usize>,
    pub interaction_groups: InteractionGroups,
    pub ground_interaction_groups: InteractionGroups,
    pub velocity_constraints: Vec<VelocityConstraint>,
    pub constraint_descs: Vec<(usize, ConstraintDesc)>,
    pub parallel_desc_groups: Vec<usize>,
}

impl<VelocityConstraint> ParallelSolverConstraints<VelocityConstraint> {
    pub fn new() -> Self {
        Self {
            generic_jacobians: DVector::zeros(0),
            not_ground_interactions: vec![],
            ground_interactions: vec![],
            generic_not_ground_interactions: vec![],
            generic_ground_interactions: vec![],
            interaction_groups: InteractionGroups::new(),
            ground_interaction_groups: InteractionGroups::new(),
            velocity_constraints: vec![],
            constraint_descs: vec![],
            parallel_desc_groups: vec![],
        }
    }
}

macro_rules! impl_init_constraints_group {
    ($VelocityConstraint: ty, $Interaction: ty,
     $categorize: ident, $group: ident,
     $body1: ident,
     $body2: ident,
     $generate_internal_constraints: expr,
     $num_active_constraints_and_jacobian_lines: path,
     $empty_velocity_constraint: expr $(, $weight: ident)*) => {
        impl ParallelSolverConstraints<$VelocityConstraint> {
            pub fn init_constraint_groups(
                &mut self,
                island_id: usize,
                islands: &IslandManager,
                bodies: &RigidBodySet,
                multibodies: &MultibodyJointSet,
                interactions: &mut [$Interaction],
                interaction_groups: &ParallelInteractionGroups,
                j_id: &mut usize,
            )  {
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
                    self.generic_not_ground_interactions.clear();
                    self.generic_ground_interactions.clear();

                    $categorize(
                        bodies,
                        multibodies,
                        interactions,
                        group,
                        &mut self.ground_interactions,
                        &mut self.not_ground_interactions,
                        &mut self.generic_ground_interactions,
                        &mut self.generic_not_ground_interactions,
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
                        self.constraint_descs.push((
                            total_num_constraints,
                            ConstraintDesc::NongroundNongrouped(*interaction_i),
                        ));
                        total_num_constraints += $num_active_constraints_and_jacobian_lines(interaction).0;
                    }

                    #[cfg(feature = "simd-is-enabled")]
                    for interaction_i in
                        self.interaction_groups.grouped_interactions[start_grouped..].chunks(SIMD_WIDTH)
                    {
                        let interaction = &mut interactions[interaction_i[0]]$(.$weight)*;
                        self.constraint_descs.push((
                            total_num_constraints,
                            ConstraintDesc::NongroundGrouped(
                                gather![|ii| interaction_i[ii]],
                            ),
                        ));
                        total_num_constraints += $num_active_constraints_and_jacobian_lines(interaction).0;
                    }

                    for interaction_i in
                        &self.ground_interaction_groups.nongrouped_interactions[start_nongrouped_ground..]
                    {
                        let interaction = &mut interactions[*interaction_i]$(.$weight)*;
                        self.constraint_descs.push((
                            total_num_constraints,
                            ConstraintDesc::GroundNongrouped(*interaction_i),
                        ));
                        total_num_constraints += $num_active_constraints_and_jacobian_lines(interaction).0;
                    }

                    #[cfg(feature = "simd-is-enabled")]
                    for interaction_i in self.ground_interaction_groups.grouped_interactions
                        [start_grouped_ground..]
                        .chunks(SIMD_WIDTH)
                    {
                        let interaction = &mut interactions[interaction_i[0]]$(.$weight)*;
                        self.constraint_descs.push((
                            total_num_constraints,
                            ConstraintDesc::GroundGrouped(
                                gather![|ii| interaction_i[ii]],
                            ),
                        ));
                        total_num_constraints += $num_active_constraints_and_jacobian_lines(interaction).0;
                    }

                    let multibody_ndofs = |handle| {
                        if let Some(link) = multibodies.rigid_body_link(handle).copied() {
                            let multibody = multibodies
                                .get_multibody(link.multibody)
                                .unwrap();
                            multibody.ndofs()
                        } else {
                            SPATIAL_DIM
                        }
                    };

                    for interaction_i in &self.generic_not_ground_interactions[..] {
                        let interaction = &mut interactions[*interaction_i]$(.$weight)*;
                        self.constraint_descs.push((
                            total_num_constraints,
                            ConstraintDesc::GenericNongroundNongrouped(*interaction_i, *j_id),
                        ));
                        let (num_constraints, num_jac_lines) = $num_active_constraints_and_jacobian_lines(interaction);
                        let ndofs1 = $body1(interaction).map(multibody_ndofs).unwrap_or(0);
                        let ndofs2 = $body2(interaction).map(multibody_ndofs).unwrap_or(0);

                        *j_id += (ndofs1 + ndofs2) * 2 * num_jac_lines;
                        total_num_constraints += num_constraints;
                    }

                    for interaction_i in &self.generic_ground_interactions[..] {
                        let interaction = &mut interactions[*interaction_i]$(.$weight)*;
                        self.constraint_descs.push((
                            total_num_constraints,
                            ConstraintDesc::GenericGroundNongrouped(*interaction_i, *j_id),
                        ));

                        let (num_constraints, num_jac_lines) = $num_active_constraints_and_jacobian_lines(interaction);
                        let ndofs1 = $body1(interaction).map(multibody_ndofs).unwrap_or(0);
                        let ndofs2 = $body2(interaction).map(multibody_ndofs).unwrap_or(0);

                        *j_id += (ndofs1 + ndofs2) * 2 * num_jac_lines;
                        total_num_constraints += num_constraints;
                    }

                    self.parallel_desc_groups.push(self.constraint_descs.len());
                }

                if $generate_internal_constraints {
                    let mut had_any_internal_constraint = false;
                    for handle in islands.active_island(island_id) {
                        if let Some(link) = multibodies.rigid_body_link(*handle) {
                            let multibody = multibodies.get_multibody(link.multibody).unwrap();
                            if link.id == 0 || link.id == 1 && !multibody.root_is_dynamic {
                                let (num_constraints, num_jac_lines) = multibody.num_active_internal_constraints_and_jacobian_lines();
                                let ndofs = multibody.ndofs();

                                self.constraint_descs.push((
                                    total_num_constraints,
                                    ConstraintDesc::GenericMultibodyInternal(link.multibody, *j_id)
                                ));

                                *j_id += ndofs * 2 * num_jac_lines;
                                total_num_constraints += num_constraints;
                                had_any_internal_constraint = true;
                            }
                        }
                    }

                    if had_any_internal_constraint {
                        self.parallel_desc_groups.push(self.constraint_descs.len());
                    }
                }

                // Resize the constraint sets.
                self.velocity_constraints.clear();
                self.velocity_constraints
                    .resize_with(total_num_constraints, || $empty_velocity_constraint);
            }
        }
    }
}

fn joint_body1(joint: &ImpulseJoint) -> Option<RigidBodyHandle> {
    Some(joint.body1)
}
fn joint_body2(joint: &ImpulseJoint) -> Option<RigidBodyHandle> {
    Some(joint.body2)
}
fn manifold_body1(manifold: &ContactManifold) -> Option<RigidBodyHandle> {
    manifold.data.rigid_body1
}
fn manifold_body2(manifold: &ContactManifold) -> Option<RigidBodyHandle> {
    manifold.data.rigid_body2
}

impl_init_constraints_group!(
    AnyVelocityConstraint,
    &mut ContactManifold,
    categorize_contacts,
    group_manifolds,
    manifold_body1,
    manifold_body2,
    false,
    VelocityConstraint::num_active_constraints_and_jacobian_lines,
    AnyVelocityConstraint::Empty
);

impl_init_constraints_group!(
    AnyJointVelocityConstraint,
    JointGraphEdge,
    categorize_joints,
    group_joints,
    joint_body1,
    joint_body2,
    true,
    AnyJointVelocityConstraint::num_active_constraints_and_jacobian_lines,
    AnyJointVelocityConstraint::Empty,
    weight
);

impl ParallelSolverConstraints<AnyVelocityConstraint> {
    pub fn fill_constraints(
        &mut self,
        thread: &ThreadContext,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        manifolds_all: &[&mut ContactManifold],
    ) {
        let descs = &self.constraint_descs;

        crate::concurrent_loop! {
            let batch_size = thread.batch_size;
            for desc in descs[thread.constraint_initialization_index, thread.num_initialized_constraints] {
                match &desc.1 {
                    ConstraintDesc::NongroundNongrouped(manifold_id) => {
                        let manifold = &*manifolds_all[*manifold_id];
                        VelocityConstraint::generate(params, *manifold_id, manifold, bodies, &mut self.velocity_constraints, Some(desc.0));
                    }
                    ConstraintDesc::GroundNongrouped(manifold_id) => {
                        let manifold = &*manifolds_all[*manifold_id];
                        VelocityGroundConstraint::generate(params, *manifold_id, manifold, bodies, &mut self.velocity_constraints, Some(desc.0));
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    ConstraintDesc::NongroundGrouped(manifold_id) => {
                        let manifolds = gather![|ii| &*manifolds_all[manifold_id[ii]]];
                        WVelocityConstraint::generate(params, *manifold_id, manifolds, bodies, &mut self.velocity_constraints, Some(desc.0));
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    ConstraintDesc::GroundGrouped(manifold_id) => {
                        let manifolds = gather![|ii| &*manifolds_all[manifold_id[ii]]];
                        WVelocityGroundConstraint::generate(params, *manifold_id, manifolds, bodies, &mut self.velocity_constraints, Some(desc.0));
                    }
                    ConstraintDesc::GenericNongroundNongrouped(manifold_id, j_id) => {
                        let mut j_id = *j_id;
                        let manifold = &*manifolds_all[*manifold_id];
                        GenericVelocityConstraint::generate(params, *manifold_id, manifold, bodies, multibodies,  &mut self.velocity_constraints, &mut self.generic_jacobians, &mut j_id, Some(desc.0));
                    }
                    ConstraintDesc::GenericGroundNongrouped(manifold_id, j_id) => {
                        let mut j_id = *j_id;
                        let manifold = &*manifolds_all[*manifold_id];
                        GenericVelocityGroundConstraint::generate(params, *manifold_id, manifold, bodies, multibodies, &mut self.velocity_constraints, &mut self.generic_jacobians, &mut j_id, Some(desc.0));
                    }
                    ConstraintDesc::GenericMultibodyInternal(..) => unreachable!()
                }
            }
        }
    }
}

impl ParallelSolverConstraints<AnyJointVelocityConstraint> {
    pub fn fill_constraints(
        &mut self,
        thread: &ThreadContext,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        joints_all: &[JointGraphEdge],
    ) {
        let descs = &self.constraint_descs;

        crate::concurrent_loop! {
            let batch_size = thread.batch_size;
            for desc in descs[thread.joint_constraint_initialization_index, thread.num_initialized_joint_constraints] {
                match &desc.1 {
                    ConstraintDesc::NongroundNongrouped(joint_id) => {
                        let joint = &joints_all[*joint_id].weight;
                        AnyJointVelocityConstraint::from_joint(params, *joint_id, joint, bodies, multibodies, &mut 0, &mut self.generic_jacobians, &mut self.velocity_constraints, Some(desc.0));
                    }
                    ConstraintDesc::GroundNongrouped(joint_id) => {
                        let joint = &joints_all[*joint_id].weight;
                        AnyJointVelocityConstraint::from_joint_ground(params, *joint_id, joint, bodies, multibodies, &mut 0, &mut self.generic_jacobians, &mut self.velocity_constraints, Some(desc.0));
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    ConstraintDesc::NongroundGrouped(joint_id) => {
                        let impulse_joints = gather![|ii| &joints_all[joint_id[ii]].weight];
                        AnyJointVelocityConstraint::from_wide_joint(params, *joint_id, impulse_joints, bodies, &mut self.velocity_constraints, Some(desc.0));
                    }
                    #[cfg(feature = "simd-is-enabled")]
                    ConstraintDesc::GroundGrouped(joint_id) => {
                        let impulse_joints = gather![|ii| &joints_all[joint_id[ii]].weight];
                        AnyJointVelocityConstraint::from_wide_joint_ground(params, *joint_id, impulse_joints, bodies, &mut self.velocity_constraints, Some(desc.0));
                    }
                    ConstraintDesc::GenericNongroundNongrouped(joint_id, j_id) => {
                        let mut j_id = *j_id;
                        let joint = &joints_all[*joint_id].weight;
                        AnyJointVelocityConstraint::from_joint(params, *joint_id, joint, bodies, multibodies, &mut j_id, &mut self.generic_jacobians, &mut self.velocity_constraints, Some(desc.0));
                    }
                    ConstraintDesc::GenericGroundNongrouped(joint_id, j_id) => {
                        let mut j_id = *j_id;
                        let joint = &joints_all[*joint_id].weight;
                        AnyJointVelocityConstraint::from_joint_ground(params, *joint_id, joint, bodies, multibodies, &mut j_id, &mut self.generic_jacobians, &mut self.velocity_constraints, Some(desc.0));
                    }
                    ConstraintDesc::GenericMultibodyInternal(multibody_id, j_id) => {
                        let mut j_id = *j_id;
                        let multibody = multibodies.get_multibody(*multibody_id).unwrap();
                        multibody.generate_internal_constraints(params, &mut j_id, &mut self.generic_jacobians, &mut self.velocity_constraints, Some(desc.0));
                    }
                }
            }
        }
    }
}
