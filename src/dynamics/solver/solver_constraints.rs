use super::{
    AnyJointVelocityConstraint, InteractionGroups, VelocityConstraint, VelocityGroundConstraint,
};
#[cfg(feature = "simd-is-enabled")]
use super::{
    WPositionConstraint, WPositionGroundConstraint, WVelocityConstraint,
    WVelocityConstraintWithManifoldFriction, WVelocityGroundConstraint,
    WVelocityGroundConstraintWithManifoldFriction,
};
use crate::dynamics::solver::categorization::{categorize_contacts, categorize_joints};
use crate::dynamics::solver::{
    AnyJointPositionConstraint, AnyPositionConstraint, PositionConstraint,
    PositionGroundConstraint, VelocityConstraintWithManifoldFriction,
    VelocityGroundConstraintWithManifoldFriction,
};
use crate::dynamics::{
    solver::AnyVelocityConstraint, IntegrationParameters, JointGraphEdge, JointIndex, RigidBodySet,
};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
#[cfg(feature = "simd-is-enabled")]
use crate::math::SIMD_WIDTH;

pub(crate) struct SolverConstraints<VelocityConstraint, PositionConstraint> {
    pub not_ground_interactions: Vec<usize>,
    pub ground_interactions: Vec<usize>,
    pub interaction_groups: InteractionGroups,
    pub ground_interaction_groups: InteractionGroups,
    pub velocity_constraints: Vec<VelocityConstraint>,
    pub position_constraints: Vec<PositionConstraint>,
}

impl<VelocityConstraint, PositionConstraint>
    SolverConstraints<VelocityConstraint, PositionConstraint>
{
    pub fn new() -> Self {
        Self {
            not_ground_interactions: Vec::new(),
            ground_interactions: Vec::new(),
            interaction_groups: InteractionGroups::new(),
            ground_interaction_groups: InteractionGroups::new(),
            velocity_constraints: Vec::new(),
            position_constraints: Vec::new(),
        }
    }
}

impl SolverConstraints<AnyVelocityConstraint, AnyPositionConstraint> {
    pub fn init_constraint_groups(
        &mut self,
        island_id: usize,
        bodies: &RigidBodySet,
        manifolds: &[&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
    ) {
        self.not_ground_interactions.clear();
        self.ground_interactions.clear();
        categorize_contacts(
            bodies,
            manifolds,
            manifold_indices,
            &mut self.ground_interactions,
            &mut self.not_ground_interactions,
        );

        self.interaction_groups.clear_groups();
        self.interaction_groups.group_manifolds(
            island_id,
            bodies,
            manifolds,
            &self.not_ground_interactions,
        );

        self.ground_interaction_groups.clear_groups();
        self.ground_interaction_groups.group_manifolds(
            island_id,
            bodies,
            manifolds,
            &self.ground_interactions,
        );

        // NOTE: uncomment this do disable SIMD contact resolution.
        //        self.interaction_groups
        //            .nongrouped_interactions
        //            .append(&mut self.interaction_groups.grouped_interactions);
        //        self.ground_interaction_groups
        //            .nongrouped_interactions
        //            .append(&mut self.ground_interaction_groups.grouped_interactions);
    }

    pub fn init(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds: &[&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
    ) {
        self.velocity_constraints.clear();
        self.position_constraints.clear();

        self.init_constraint_groups(island_id, bodies, manifolds, manifold_indices);

        #[cfg(feature = "simd-is-enabled")]
        {
            self.compute_grouped_constraints(params, bodies, manifolds);
        }
        self.compute_nongrouped_constraints(params, bodies, manifolds);
        #[cfg(feature = "simd-is-enabled")]
        {
            self.compute_grouped_ground_constraints(params, bodies, manifolds);
        }
        self.compute_nongrouped_ground_constraints(params, bodies, manifolds);
    }

    #[cfg(feature = "simd-is-enabled")]
    fn compute_grouped_constraints(
        &mut self,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
    ) {
        for manifolds_i in self
            .interaction_groups
            .grouped_interactions
            .chunks_exact(SIMD_WIDTH)
        {
            let manifold_id = array![|ii| manifolds_i[ii]; SIMD_WIDTH];
            let manifolds = array![|ii| &*manifolds_all[manifolds_i[ii]]; SIMD_WIDTH];
            WVelocityConstraintWithManifoldFriction::generate(
                params,
                manifold_id,
                manifolds,
                bodies,
                &mut self.velocity_constraints,
                true,
            );

            if params.positionErp != 0.0 {
                WPositionConstraint::generate(
                    params,
                    manifolds,
                    bodies,
                    &mut self.position_constraints,
                    true,
                );
            }
        }
    }

    fn compute_nongrouped_constraints(
        &mut self,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
    ) {
        for manifold_i in &self.interaction_groups.nongrouped_interactions {
            let manifold = &manifolds_all[*manifold_i];
            VelocityConstraintWithManifoldFriction::generate(
                params,
                *manifold_i,
                manifold,
                bodies,
                &mut self.velocity_constraints,
                true,
            );

            if params.positionErp != 0.0 {
                PositionConstraint::generate(
                    params,
                    manifold,
                    bodies,
                    &mut self.position_constraints,
                    true,
                );
            }
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    fn compute_grouped_ground_constraints(
        &mut self,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
    ) {
        for manifolds_i in self
            .ground_interaction_groups
            .grouped_interactions
            .chunks_exact(SIMD_WIDTH)
        {
            let manifold_id = array![|ii| manifolds_i[ii]; SIMD_WIDTH];
            let manifolds = array![|ii| &*manifolds_all[manifolds_i[ii]]; SIMD_WIDTH];
            WVelocityGroundConstraintWithManifoldFriction::generate(
                params,
                manifold_id,
                manifolds,
                bodies,
                &mut self.velocity_constraints,
                true,
            );

            if params.positionErp != 0.0 {
                WPositionGroundConstraint::generate(
                    params,
                    manifolds,
                    bodies,
                    &mut self.position_constraints,
                    true,
                );
            }
        }
    }

    fn compute_nongrouped_ground_constraints(
        &mut self,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
    ) {
        for manifold_i in &self.ground_interaction_groups.nongrouped_interactions {
            let manifold = &manifolds_all[*manifold_i];
            VelocityGroundConstraintWithManifoldFriction::generate(
                params,
                *manifold_i,
                manifold,
                bodies,
                &mut self.velocity_constraints,
                true,
            );

            if params.positionErp != 0.0 {
                PositionGroundConstraint::generate(
                    params,
                    manifold,
                    bodies,
                    &mut self.position_constraints,
                    true,
                )
            }
        }
    }
}

impl SolverConstraints<AnyJointVelocityConstraint, AnyJointPositionConstraint> {
    pub fn init(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        joints: &[JointGraphEdge],
        joint_constraint_indices: &[JointIndex],
    ) {
        // Generate constraints for joints.
        self.not_ground_interactions.clear();
        self.ground_interactions.clear();
        categorize_joints(
            bodies,
            joints,
            joint_constraint_indices,
            &mut self.ground_interactions,
            &mut self.not_ground_interactions,
        );

        self.velocity_constraints.clear();
        self.position_constraints.clear();

        self.interaction_groups.clear_groups();
        self.interaction_groups.group_joints(
            island_id,
            bodies,
            joints,
            &self.not_ground_interactions,
        );

        self.ground_interaction_groups.clear_groups();
        self.ground_interaction_groups.group_joints(
            island_id,
            bodies,
            joints,
            &self.ground_interactions,
        );
        // NOTE: uncomment this do disable SIMD joint resolution.
        // self.interaction_groups
        //     .nongrouped_interactions
        //     .append(&mut self.interaction_groups.grouped_interactions);
        // self.ground_interaction_groups
        //     .nongrouped_interactions
        //     .append(&mut self.ground_interaction_groups.grouped_interactions);

        self.compute_nongrouped_joint_ground_constraints(params, bodies, joints);
        #[cfg(feature = "simd-is-enabled")]
        {
            self.compute_grouped_joint_ground_constraints(params, bodies, joints);
        }
        self.compute_nongrouped_joint_constraints(params, bodies, joints);
        #[cfg(feature = "simd-is-enabled")]
        {
            self.compute_grouped_joint_constraints(params, bodies, joints);
        }
    }

    fn compute_nongrouped_joint_ground_constraints(
        &mut self,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        joints_all: &[JointGraphEdge],
    ) {
        for joint_i in &self.ground_interaction_groups.nongrouped_interactions {
            let joint = &joints_all[*joint_i].weight;
            let vel_constraint =
                AnyJointVelocityConstraint::from_joint_ground(params, *joint_i, joint, bodies);
            self.velocity_constraints.push(vel_constraint);
            let pos_constraint = AnyJointPositionConstraint::from_joint_ground(joint, bodies);
            self.position_constraints.push(pos_constraint);
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    fn compute_grouped_joint_ground_constraints(
        &mut self,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        joints_all: &[JointGraphEdge],
    ) {
        for joints_i in self
            .ground_interaction_groups
            .grouped_interactions
            .chunks_exact(SIMD_WIDTH)
        {
            let joints_id = array![|ii| joints_i[ii]; SIMD_WIDTH];
            let joints = array![|ii| &joints_all[joints_i[ii]].weight; SIMD_WIDTH];
            let vel_constraint = AnyJointVelocityConstraint::from_wide_joint_ground(
                params, joints_id, joints, bodies,
            );
            self.velocity_constraints.push(vel_constraint);

            let pos_constraint = AnyJointPositionConstraint::from_wide_joint_ground(joints, bodies);
            self.position_constraints.push(pos_constraint);
        }
    }

    fn compute_nongrouped_joint_constraints(
        &mut self,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        joints_all: &[JointGraphEdge],
    ) {
        for joint_i in &self.interaction_groups.nongrouped_interactions {
            let joint = &joints_all[*joint_i].weight;
            let vel_constraint =
                AnyJointVelocityConstraint::from_joint(params, *joint_i, joint, bodies);
            self.velocity_constraints.push(vel_constraint);
            let pos_constraint = AnyJointPositionConstraint::from_joint(joint, bodies);
            self.position_constraints.push(pos_constraint);
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    fn compute_grouped_joint_constraints(
        &mut self,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        joints_all: &[JointGraphEdge],
    ) {
        for joints_i in self
            .interaction_groups
            .grouped_interactions
            .chunks_exact(SIMD_WIDTH)
        {
            let joints_id = array![|ii| joints_i[ii]; SIMD_WIDTH];
            let joints = array![|ii| &joints_all[joints_i[ii]].weight; SIMD_WIDTH];
            let vel_constraint =
                AnyJointVelocityConstraint::from_wide_joint(params, joints_id, joints, bodies);
            self.velocity_constraints.push(vel_constraint);

            let pos_constraint = AnyJointPositionConstraint::from_wide_joint(joints, bodies);
            self.position_constraints.push(pos_constraint);
        }
    }
}
