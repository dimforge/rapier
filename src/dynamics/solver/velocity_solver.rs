use super::{
    AnyJointVelocityConstraint, InteractionGroups, VelocityConstraint, VelocityGroundConstraint,
};
#[cfg(feature = "simd-is-enabled")]
use super::{WVelocityConstraint, WVelocityGroundConstraint};
use crate::dynamics::solver::categorization::{categorize_contacts, categorize_joints};
use crate::dynamics::solver::{
    AnyPositionConstraint, PositionConstraint, PositionGroundConstraint, WPositionConstraint,
    WPositionGroundConstraint,
};
use crate::dynamics::{
    solver::{AnyVelocityConstraint, DeltaVel},
    IntegrationParameters, JointGraphEdge, JointIndex, RigidBodySet,
};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
#[cfg(feature = "simd-is-enabled")]
use crate::math::SIMD_WIDTH;
use crate::utils::WAngularInertia;

pub(crate) struct VelocitySolver {
    pub mj_lambdas: Vec<DeltaVel<f32>>,
    pub contact_part: VelocitySolverPart<AnyVelocityConstraint>,
    pub joint_part: VelocitySolverPart<AnyJointVelocityConstraint>,
}

impl VelocitySolver {
    pub fn new() -> Self {
        Self {
            mj_lambdas: Vec::new(),
            contact_part: VelocitySolverPart::new(),
            joint_part: VelocitySolverPart::new(),
        }
    }

    pub fn init_constraints(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds: &[&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
        joints: &[JointGraphEdge],
        joint_constraint_indices: &[JointIndex],
        position_constraints: &mut Vec<AnyPositionConstraint>,
    ) {
        self.contact_part.init_constraints(
            island_id,
            params,
            bodies,
            manifolds,
            manifold_indices,
            position_constraints,
        );
        self.joint_part.init_constraints(
            island_id,
            params,
            bodies,
            joints,
            joint_constraint_indices,
        )
    }

    pub fn solve_constraints(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        bodies: &mut RigidBodySet,
        manifolds_all: &mut [&mut ContactManifold],
        joints_all: &mut [JointGraphEdge],
    ) {
        self.mj_lambdas.clear();
        self.mj_lambdas
            .resize(bodies.active_island(island_id).len(), DeltaVel::zero());

        /*
         * Warmstart constraints.
         */
        for constraint in &self.joint_part.constraints {
            constraint.warmstart(&mut self.mj_lambdas[..]);
        }

        for constraint in &self.contact_part.constraints {
            constraint.warmstart(&mut self.mj_lambdas[..]);
        }

        /*
         * Solve constraints.
         */
        for _ in 0..params.max_velocity_iterations {
            for constraint in &mut self.joint_part.constraints {
                constraint.solve(&mut self.mj_lambdas[..]);
            }

            for constraint in &mut self.contact_part.constraints {
                constraint.solve(&mut self.mj_lambdas[..]);
            }
        }

        // Update velocities.
        bodies.foreach_active_island_body_mut_internal(island_id, |_, rb| {
            let dvel = self.mj_lambdas[rb.active_set_offset];
            rb.linvel += dvel.linear;
            rb.angvel += rb.world_inv_inertia_sqrt.transform_vector(dvel.angular);
        });

        // Write impulses back into the manifold structures.
        for constraint in &self.joint_part.constraints {
            constraint.writeback_impulses(joints_all);
        }

        for constraint in &self.contact_part.constraints {
            constraint.writeback_impulses(manifolds_all);
        }
    }
}

pub(crate) struct VelocitySolverPart<Constraint> {
    pub not_ground_interactions: Vec<usize>,
    pub ground_interactions: Vec<usize>,
    pub interaction_groups: InteractionGroups,
    pub ground_interaction_groups: InteractionGroups,
    pub constraints: Vec<Constraint>,
}

impl<Constraint> VelocitySolverPart<Constraint> {
    pub fn new() -> Self {
        Self {
            not_ground_interactions: Vec::new(),
            ground_interactions: Vec::new(),
            interaction_groups: InteractionGroups::new(),
            ground_interaction_groups: InteractionGroups::new(),
            constraints: Vec::new(),
        }
    }
}

impl VelocitySolverPart<AnyVelocityConstraint> {
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

    pub fn init_constraints(
        &mut self,
        island_id: usize,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds: &[&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
        position_constraints: &mut Vec<AnyPositionConstraint>,
    ) {
        self.init_constraint_groups(island_id, bodies, manifolds, manifold_indices);
        self.constraints.clear();
        #[cfg(feature = "simd-is-enabled")]
        {
            self.compute_grouped_constraints(params, bodies, manifolds, position_constraints);
        }
        self.compute_nongrouped_constraints(params, bodies, manifolds, position_constraints);
        #[cfg(feature = "simd-is-enabled")]
        {
            self.compute_grouped_ground_constraints(
                params,
                bodies,
                manifolds,
                position_constraints,
            );
        }
        self.compute_nongrouped_ground_constraints(params, bodies, manifolds, position_constraints);
    }

    #[cfg(feature = "simd-is-enabled")]
    fn compute_grouped_constraints(
        &mut self,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
        constraints_all: &mut Vec<AnyPositionConstraint>,
    ) {
        for manifolds_i in self
            .interaction_groups
            .grouped_interactions
            .chunks_exact(SIMD_WIDTH)
        {
            let manifold_id = array![|ii| manifolds_i[ii]; SIMD_WIDTH];
            let manifolds = array![|ii| &*manifolds_all[manifolds_i[ii]]; SIMD_WIDTH];
            WVelocityConstraint::generate(
                params,
                manifold_id,
                manifolds,
                bodies,
                &mut self.constraints,
                true,
            );
            WPositionConstraint::generate(params, manifolds, bodies, constraints_all, true);
        }
    }

    fn compute_nongrouped_constraints(
        &mut self,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
        constraints_all: &mut Vec<AnyPositionConstraint>,
    ) {
        for manifold_i in &self.interaction_groups.nongrouped_interactions {
            let manifold = &manifolds_all[*manifold_i];
            VelocityConstraint::generate(
                params,
                *manifold_i,
                manifold,
                bodies,
                &mut self.constraints,
                true,
            );
            PositionConstraint::generate(params, manifold, bodies, constraints_all, true);
        }
    }

    #[cfg(feature = "simd-is-enabled")]
    fn compute_grouped_ground_constraints(
        &mut self,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
        constraints_all: &mut Vec<AnyPositionConstraint>,
    ) {
        for manifolds_i in self
            .ground_interaction_groups
            .grouped_interactions
            .chunks_exact(SIMD_WIDTH)
        {
            let manifold_id = array![|ii| manifolds_i[ii]; SIMD_WIDTH];
            let manifolds = array![|ii| &*manifolds_all[manifolds_i[ii]]; SIMD_WIDTH];
            WVelocityGroundConstraint::generate(
                params,
                manifold_id,
                manifolds,
                bodies,
                &mut self.constraints,
                true,
            );
            WPositionGroundConstraint::generate(params, manifolds, bodies, constraints_all, true);
        }
    }

    fn compute_nongrouped_ground_constraints(
        &mut self,
        params: &IntegrationParameters,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
        constraints_all: &mut Vec<AnyPositionConstraint>,
    ) {
        for manifold_i in &self.ground_interaction_groups.nongrouped_interactions {
            let manifold = &manifolds_all[*manifold_i];
            VelocityGroundConstraint::generate(
                params,
                *manifold_i,
                manifold,
                bodies,
                &mut self.constraints,
                true,
            );
            PositionGroundConstraint::generate(params, manifold, bodies, constraints_all, true)
        }
    }
}

impl VelocitySolverPart<AnyJointVelocityConstraint> {
    pub fn init_constraints(
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

        self.constraints.clear();

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
            self.constraints.push(vel_constraint);
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
            self.constraints.push(vel_constraint);
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
            self.constraints.push(vel_constraint);
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
            self.constraints.push(vel_constraint);
        }
    }
}
