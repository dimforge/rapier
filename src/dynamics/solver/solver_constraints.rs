use super::{
    AnyJointVelocityConstraint, InteractionGroups, VelocityConstraint, VelocityGroundConstraint,
};
#[cfg(feature = "simd-is-enabled")]
use super::{WVelocityConstraint, WVelocityGroundConstraint};
use crate::data::ComponentSet;
use crate::dynamics::solver::categorization::{categorize_contacts, categorize_joints};
use crate::dynamics::solver::generic_velocity_ground_constraint::GenericVelocityGroundConstraint;
use crate::dynamics::solver::AnyGenericVelocityConstraint;
use crate::dynamics::solver::GenericVelocityConstraint;
use crate::dynamics::{
    solver::AnyVelocityConstraint, IntegrationParameters, JointGraphEdge, JointIndex,
    MultibodyJointSet, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition, RigidBodyType,
};
use crate::dynamics::{IslandManager, RigidBodyVelocity};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::Real;
#[cfg(feature = "simd-is-enabled")]
use crate::math::SIMD_WIDTH;
use na::DVector;

pub(crate) struct SolverConstraints<VelocityConstraint, GenVelocityConstraint> {
    pub generic_jacobians: DVector<Real>,
    pub not_ground_interactions: Vec<usize>,
    pub ground_interactions: Vec<usize>,
    pub generic_not_ground_interactions: Vec<usize>,
    pub generic_ground_interactions: Vec<usize>,
    pub interaction_groups: InteractionGroups,
    pub ground_interaction_groups: InteractionGroups,
    pub velocity_constraints: Vec<VelocityConstraint>,
    pub generic_velocity_constraints: Vec<GenVelocityConstraint>,
}

impl<VelocityConstraint, GenVelocityConstraint>
    SolverConstraints<VelocityConstraint, GenVelocityConstraint>
{
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
            generic_velocity_constraints: vec![],
        }
    }

    pub fn clear(&mut self) {
        self.not_ground_interactions.clear();
        self.ground_interactions.clear();
        self.generic_not_ground_interactions.clear();
        self.generic_ground_interactions.clear();
        self.interaction_groups.clear();
        self.ground_interaction_groups.clear();
        self.velocity_constraints.clear();
        self.generic_velocity_constraints.clear();
    }
}

impl SolverConstraints<AnyVelocityConstraint, AnyGenericVelocityConstraint> {
    pub fn init_constraint_groups<Bodies>(
        &mut self,
        island_id: usize,
        islands: &IslandManager,
        bodies: &Bodies,
        multibody_joints: &MultibodyJointSet,
        manifolds: &[&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
    ) where
        Bodies: ComponentSet<RigidBodyType> + ComponentSet<RigidBodyIds>,
    {
        self.not_ground_interactions.clear();
        self.ground_interactions.clear();
        self.generic_not_ground_interactions.clear();
        self.generic_ground_interactions.clear();

        categorize_contacts(
            bodies,
            multibody_joints,
            manifolds,
            manifold_indices,
            &mut self.ground_interactions,
            &mut self.not_ground_interactions,
            &mut self.generic_ground_interactions,
            &mut self.generic_not_ground_interactions,
        );

        self.interaction_groups.clear_groups();
        self.interaction_groups.group_manifolds(
            island_id,
            islands,
            bodies,
            manifolds,
            &self.not_ground_interactions,
        );

        self.ground_interaction_groups.clear_groups();
        self.ground_interaction_groups.group_manifolds(
            island_id,
            islands,
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

    pub fn init<Bodies>(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &Bodies,
        multibody_joints: &MultibodyJointSet,
        manifolds: &[&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
    ) where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>
            + ComponentSet<RigidBodyType>,
    {
        self.velocity_constraints.clear();
        self.generic_velocity_constraints.clear();

        self.init_constraint_groups(
            island_id,
            islands,
            bodies,
            multibody_joints,
            manifolds,
            manifold_indices,
        );

        let mut jacobian_id = 0;
        #[cfg(feature = "simd-is-enabled")]
        {
            self.compute_grouped_constraints(params, bodies, manifolds);
        }
        self.compute_nongrouped_constraints(params, bodies, manifolds);
        self.compute_generic_constraints(
            params,
            bodies,
            multibody_joints,
            manifolds,
            &mut jacobian_id,
        );

        #[cfg(feature = "simd-is-enabled")]
        {
            self.compute_grouped_ground_constraints(params, bodies, manifolds);
        }
        self.compute_nongrouped_ground_constraints(params, bodies, manifolds);
        self.compute_generic_ground_constraints(
            params,
            bodies,
            multibody_joints,
            manifolds,
            &mut jacobian_id,
        );
    }

    #[cfg(feature = "simd-is-enabled")]
    fn compute_grouped_constraints<Bodies>(
        &mut self,
        params: &IntegrationParameters,
        bodies: &Bodies,
        manifolds_all: &[&mut ContactManifold],
    ) where
        Bodies: ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>,
    {
        for manifolds_i in self
            .interaction_groups
            .grouped_interactions
            .chunks_exact(SIMD_WIDTH)
        {
            let manifold_id = gather![|ii| manifolds_i[ii]];
            let manifolds = gather![|ii| &*manifolds_all[manifolds_i[ii]]];
            WVelocityConstraint::generate(
                params,
                manifold_id,
                manifolds,
                bodies,
                &mut self.velocity_constraints,
                true,
            );
        }
    }

    fn compute_nongrouped_constraints<Bodies>(
        &mut self,
        params: &IntegrationParameters,
        bodies: &Bodies,
        manifolds_all: &[&mut ContactManifold],
    ) where
        Bodies: ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>,
    {
        for manifold_i in &self.interaction_groups.nongrouped_interactions {
            let manifold = &manifolds_all[*manifold_i];
            VelocityConstraint::generate(
                params,
                *manifold_i,
                manifold,
                bodies,
                &mut self.velocity_constraints,
                true,
            );
        }
    }

    fn compute_generic_constraints<Bodies>(
        &mut self,
        params: &IntegrationParameters,
        bodies: &Bodies,
        multibody_joints: &MultibodyJointSet,
        manifolds_all: &[&mut ContactManifold],
        jacobian_id: &mut usize,
    ) where
        Bodies: ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>
            + ComponentSet<RigidBodyType>,
    {
        for manifold_i in &self.generic_not_ground_interactions {
            let manifold = &manifolds_all[*manifold_i];
            GenericVelocityConstraint::generate(
                params,
                *manifold_i,
                manifold,
                bodies,
                multibody_joints,
                &mut self.generic_velocity_constraints,
                &mut self.generic_jacobians,
                jacobian_id,
                true,
            );
        }
    }

    fn compute_generic_ground_constraints<Bodies>(
        &mut self,
        params: &IntegrationParameters,
        bodies: &Bodies,
        multibody_joints: &MultibodyJointSet,
        manifolds_all: &[&mut ContactManifold],
        jacobian_id: &mut usize,
    ) where
        Bodies: ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>
            + ComponentSet<RigidBodyType>,
    {
        for manifold_i in &self.generic_ground_interactions {
            let manifold = &manifolds_all[*manifold_i];
            GenericVelocityGroundConstraint::generate(
                params,
                *manifold_i,
                manifold,
                bodies,
                multibody_joints,
                &mut self.generic_velocity_constraints,
                &mut self.generic_jacobians,
                jacobian_id,
                true,
            );
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    fn compute_grouped_ground_constraints<Bodies>(
        &mut self,
        params: &IntegrationParameters,
        bodies: &Bodies,
        manifolds_all: &[&mut ContactManifold],
    ) where
        Bodies: ComponentSet<RigidBodyIds>
            + ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>,
    {
        for manifolds_i in self
            .ground_interaction_groups
            .grouped_interactions
            .chunks_exact(SIMD_WIDTH)
        {
            let manifold_id = gather![|ii| manifolds_i[ii]];
            let manifolds = gather![|ii| &*manifolds_all[manifolds_i[ii]]];
            WVelocityGroundConstraint::generate(
                params,
                manifold_id,
                manifolds,
                bodies,
                &mut self.velocity_constraints,
                true,
            );
        }
    }

    fn compute_nongrouped_ground_constraints<Bodies>(
        &mut self,
        params: &IntegrationParameters,
        bodies: &Bodies,
        manifolds_all: &[&mut ContactManifold],
    ) where
        Bodies: ComponentSet<RigidBodyIds>
            + ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>,
    {
        for manifold_i in &self.ground_interaction_groups.nongrouped_interactions {
            let manifold = &manifolds_all[*manifold_i];
            VelocityGroundConstraint::generate(
                params,
                *manifold_i,
                manifold,
                bodies,
                &mut self.velocity_constraints,
                true,
            );
        }
    }
}

impl SolverConstraints<AnyJointVelocityConstraint, ()> {
    pub fn init<Bodies>(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        islands: &IslandManager,
        bodies: &Bodies,
        multibody_joints: &MultibodyJointSet,
        impulse_joints: &[JointGraphEdge],
        joint_constraint_indices: &[JointIndex],
    ) where
        Bodies: ComponentSet<RigidBodyType>
            + ComponentSet<RigidBodyIds>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyMassProps>,
    {
        // Generate constraints for impulse_joints.
        self.not_ground_interactions.clear();
        self.ground_interactions.clear();
        self.generic_not_ground_interactions.clear();
        self.generic_ground_interactions.clear();

        categorize_joints(
            bodies,
            multibody_joints,
            impulse_joints,
            joint_constraint_indices,
            &mut self.ground_interactions,
            &mut self.not_ground_interactions,
            &mut self.generic_ground_interactions,
            &mut self.generic_not_ground_interactions,
        );

        self.velocity_constraints.clear();
        self.generic_velocity_constraints.clear();

        self.interaction_groups.clear_groups();
        self.interaction_groups.group_joints(
            island_id,
            islands,
            bodies,
            impulse_joints,
            &self.not_ground_interactions,
        );

        self.ground_interaction_groups.clear_groups();
        self.ground_interaction_groups.group_joints(
            island_id,
            islands,
            bodies,
            impulse_joints,
            &self.ground_interactions,
        );
        // NOTE: uncomment this do disable SIMD joint resolution.
        // self.interaction_groups
        //     .nongrouped_interactions
        //     .append(&mut self.interaction_groups.grouped_interactions);
        // self.ground_interaction_groups
        //     .nongrouped_interactions
        //     .append(&mut self.ground_interaction_groups.grouped_interactions);

        let mut j_id = 0;
        self.compute_nongrouped_joint_constraints(
            params,
            bodies,
            multibody_joints,
            impulse_joints,
            &mut j_id,
        );
        #[cfg(feature = "simd-is-enabled")]
        {
            self.compute_grouped_joint_constraints(params, bodies, impulse_joints);
        }
        self.compute_generic_joint_constraints(
            params,
            bodies,
            multibody_joints,
            impulse_joints,
            &mut j_id,
        );

        self.compute_nongrouped_joint_ground_constraints(
            params,
            bodies,
            multibody_joints,
            impulse_joints,
        );
        #[cfg(feature = "simd-is-enabled")]
        {
            self.compute_grouped_joint_ground_constraints(params, bodies, impulse_joints);
        }
        self.compute_generic_ground_joint_constraints(
            params,
            bodies,
            multibody_joints,
            impulse_joints,
            &mut j_id,
        );
        self.compute_articulation_constraints(
            params,
            island_id,
            islands,
            multibody_joints,
            &mut j_id,
        );
    }

    fn compute_articulation_constraints(
        &mut self,
        params: &IntegrationParameters,
        island_id: usize,
        islands: &IslandManager,
        multibody_joints: &MultibodyJointSet,
        j_id: &mut usize,
    ) {
        for handle in islands.active_island(island_id) {
            if let Some(link) = multibody_joints.rigid_body_link(*handle) {
                let multibody = multibody_joints.get_multibody(link.multibody).unwrap();
                if link.id == 0 || link.id == 1 && !multibody.root_is_dynamic {
                    multibody.generate_internal_constraints(
                        params,
                        j_id,
                        &mut self.generic_jacobians,
                        &mut self.velocity_constraints,
                    )
                }
            }
        }
    }

    fn compute_nongrouped_joint_ground_constraints<Bodies>(
        &mut self,
        params: &IntegrationParameters,
        bodies: &Bodies,
        multibody_joints: &MultibodyJointSet,
        joints_all: &[JointGraphEdge],
    ) where
        Bodies: ComponentSet<RigidBodyType>
            + ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyIds>,
    {
        let mut j_id = 0;
        for joint_i in &self.ground_interaction_groups.nongrouped_interactions {
            let joint = &joints_all[*joint_i].weight;
            AnyJointVelocityConstraint::from_joint_ground(
                params,
                *joint_i,
                joint,
                bodies,
                multibody_joints,
                &mut j_id,
                &mut self.generic_jacobians,
                &mut self.velocity_constraints,
                true,
            );
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    fn compute_grouped_joint_ground_constraints<Bodies>(
        &mut self,
        params: &IntegrationParameters,
        bodies: &Bodies,
        joints_all: &[JointGraphEdge],
    ) where
        Bodies: ComponentSet<RigidBodyType>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>,
    {
        for joints_i in self
            .ground_interaction_groups
            .grouped_interactions
            .chunks_exact(SIMD_WIDTH)
        {
            let joints_id = gather![|ii| joints_i[ii]];
            let impulse_joints = gather![|ii| &joints_all[joints_i[ii]].weight];
            AnyJointVelocityConstraint::from_wide_joint_ground(
                params,
                joints_id,
                impulse_joints,
                bodies,
                &mut self.velocity_constraints,
                true,
            );
        }
    }

    fn compute_nongrouped_joint_constraints<Bodies>(
        &mut self,
        params: &IntegrationParameters,
        bodies: &Bodies,
        multibody_joints: &MultibodyJointSet,
        joints_all: &[JointGraphEdge],
        j_id: &mut usize,
    ) where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>,
    {
        for joint_i in &self.interaction_groups.nongrouped_interactions {
            let joint = &joints_all[*joint_i].weight;
            AnyJointVelocityConstraint::from_joint(
                params,
                *joint_i,
                joint,
                bodies,
                multibody_joints,
                j_id,
                &mut self.generic_jacobians,
                &mut self.velocity_constraints,
                true,
            );
        }
    }

    fn compute_generic_joint_constraints<Bodies>(
        &mut self,
        params: &IntegrationParameters,
        bodies: &Bodies,
        multibody_joints: &MultibodyJointSet,
        joints_all: &[JointGraphEdge],
        j_id: &mut usize,
    ) where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>,
    {
        for joint_i in &self.generic_not_ground_interactions {
            let joint = &joints_all[*joint_i].weight;
            AnyJointVelocityConstraint::from_joint(
                params,
                *joint_i,
                joint,
                bodies,
                multibody_joints,
                j_id,
                &mut self.generic_jacobians,
                &mut self.velocity_constraints,
                true,
            )
        }
    }

    fn compute_generic_ground_joint_constraints<Bodies>(
        &mut self,
        params: &IntegrationParameters,
        bodies: &Bodies,
        multibody_joints: &MultibodyJointSet,
        joints_all: &[JointGraphEdge],
        j_id: &mut usize,
    ) where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyType>
            + ComponentSet<RigidBodyIds>,
    {
        for joint_i in &self.generic_ground_interactions {
            let joint = &joints_all[*joint_i].weight;
            AnyJointVelocityConstraint::from_joint_ground(
                params,
                *joint_i,
                joint,
                bodies,
                multibody_joints,
                j_id,
                &mut self.generic_jacobians,
                &mut self.velocity_constraints,
                true,
            )
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    fn compute_grouped_joint_constraints<Bodies>(
        &mut self,
        params: &IntegrationParameters,
        bodies: &Bodies,
        joints_all: &[JointGraphEdge],
    ) where
        Bodies: ComponentSet<RigidBodyPosition>
            + ComponentSet<RigidBodyVelocity>
            + ComponentSet<RigidBodyMassProps>
            + ComponentSet<RigidBodyIds>,
    {
        for joints_i in self
            .interaction_groups
            .grouped_interactions
            .chunks_exact(SIMD_WIDTH)
        {
            let joints_id = gather![|ii| joints_i[ii]];
            let impulse_joints = gather![|ii| &joints_all[joints_i[ii]].weight];
            AnyJointVelocityConstraint::from_wide_joint(
                params,
                joints_id,
                impulse_joints,
                bodies,
                &mut self.velocity_constraints,
                true,
            );
        }
    }
}
