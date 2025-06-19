use crate::dynamics::solver::categorization::categorize_contacts;
use crate::dynamics::solver::contact_constraint::{
    GenericOneBodyConstraint, GenericOneBodyConstraintBuilder, GenericTwoBodyConstraint,
    GenericTwoBodyConstraintBuilder, OneBodyConstraint, OneBodyConstraintBuilder,
    TwoBodyConstraint, TwoBodyConstraintBuilder,
};
use crate::dynamics::solver::solver_body::SolverBody;
use crate::dynamics::solver::solver_vel::SolverVel;
use crate::dynamics::solver::{ConstraintTypes, SolverConstraintsSet, reset_buffer};
use crate::dynamics::{
    ImpulseJoint, IntegrationParameters, IslandManager, JointAxesMask, MultibodyJointSet,
    RigidBodySet,
};
use crate::geometry::{ContactManifold, ContactManifoldIndex};
use crate::math::{MAX_MANIFOLD_POINTS, Real};
use na::DVector;
use parry::math::DIM;

#[cfg(feature = "simd-is-enabled")]
use {
    crate::dynamics::solver::contact_constraint::{
        OneBodyConstraintSimd, SimdOneBodyConstraintBuilder, TwoBodyConstraintBuilderSimd,
        TwoBodyConstraintSimd,
    },
    crate::math::SIMD_WIDTH,
};

#[derive(Debug)]
pub struct ConstraintsCounts {
    pub num_constraints: usize,
    #[allow(dead_code)] // Keep this around for now. Might be useful once we rework parallelism.
    pub num_jacobian_lines: usize,
}

impl ConstraintsCounts {
    pub fn from_contacts(manifold: &ContactManifold) -> Self {
        let rest = manifold.data.solver_contacts.len() % MAX_MANIFOLD_POINTS != 0;
        Self {
            num_constraints: manifold.data.solver_contacts.len() / MAX_MANIFOLD_POINTS
                + rest as usize,
            num_jacobian_lines: manifold.data.solver_contacts.len() * DIM,
        }
    }

    pub fn from_joint(joint: &ImpulseJoint) -> Self {
        let joint = &joint.data;
        let locked_axes = joint.locked_axes.bits();
        let motor_axes = joint.motor_axes.bits() & !locked_axes;
        let limit_axes = joint.limit_axes.bits() & !locked_axes;
        let coupled_axes = joint.coupled_axes.bits();

        let num_constraints = (motor_axes & !coupled_axes).count_ones() as usize
            + ((motor_axes & coupled_axes) & JointAxesMask::ANG_AXES.bits() != 0) as usize
            + ((motor_axes & coupled_axes) & JointAxesMask::LIN_AXES.bits() != 0) as usize
            + locked_axes.count_ones() as usize
            + (limit_axes & !coupled_axes).count_ones() as usize
            + ((limit_axes & coupled_axes) & JointAxesMask::ANG_AXES.bits() != 0) as usize
            + ((limit_axes & coupled_axes) & JointAxesMask::LIN_AXES.bits() != 0) as usize;
        Self {
            num_constraints,
            num_jacobian_lines: num_constraints,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub(crate) struct ContactConstraintTypes;

impl ConstraintTypes for ContactConstraintTypes {
    type OneBody = OneBodyConstraint;
    type TwoBodies = TwoBodyConstraint;
    type GenericOneBody = GenericOneBodyConstraint;
    type GenericTwoBodies = GenericTwoBodyConstraint;
    #[cfg(feature = "simd-is-enabled")]
    type SimdOneBody = OneBodyConstraintSimd;
    #[cfg(feature = "simd-is-enabled")]
    type SimdTwoBodies = TwoBodyConstraintSimd;

    type BuilderOneBody = OneBodyConstraintBuilder;
    type BuilderTwoBodies = TwoBodyConstraintBuilder;
    type GenericBuilderOneBody = GenericOneBodyConstraintBuilder;
    type GenericBuilderTwoBodies = GenericTwoBodyConstraintBuilder;
    #[cfg(feature = "simd-is-enabled")]
    type SimdBuilderOneBody = SimdOneBodyConstraintBuilder;
    #[cfg(feature = "simd-is-enabled")]
    type SimdBuilderTwoBodies = TwoBodyConstraintBuilderSimd;
}

pub type ContactConstraintsSet = SolverConstraintsSet<ContactConstraintTypes>;

impl ContactConstraintsSet {
    pub fn init_constraint_groups(
        &mut self,
        island_id: usize,
        islands: &IslandManager,
        bodies: &RigidBodySet,
        multibody_joints: &MultibodyJointSet,
        manifolds: &[&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
    ) {
        self.two_body_interactions.clear();
        self.one_body_interactions.clear();
        self.generic_two_body_interactions.clear();
        self.generic_one_body_interactions.clear();

        categorize_contacts(
            bodies,
            multibody_joints,
            manifolds,
            manifold_indices,
            &mut self.one_body_interactions,
            &mut self.two_body_interactions,
            &mut self.generic_one_body_interactions,
            &mut self.generic_two_body_interactions,
        );

        self.interaction_groups.clear_groups();
        self.interaction_groups.group_manifolds(
            island_id,
            islands,
            bodies,
            manifolds,
            &self.two_body_interactions,
        );

        self.one_body_interaction_groups.clear_groups();
        self.one_body_interaction_groups.group_manifolds(
            island_id,
            islands,
            bodies,
            manifolds,
            &self.one_body_interactions,
        );

        // NOTE: uncomment this do disable SIMD contact resolution.
        //        self.interaction_groups
        //            .nongrouped_interactions
        //            .append(&mut self.interaction_groups.simd_interactions);
        //        self.one_body_interaction_groups
        //            .nongrouped_interactions
        //            .append(&mut self.one_body_interaction_groups.simd_interactions);
    }

    pub fn init(
        &mut self,
        island_id: usize,
        islands: &IslandManager,
        bodies: &RigidBodySet,
        multibody_joints: &MultibodyJointSet,
        manifolds: &[&mut ContactManifold],
        manifold_indices: &[ContactManifoldIndex],
    ) {
        self.clear_constraints();
        self.clear_builders();

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
            self.simd_compute_constraints(bodies, manifolds);
        }
        self.compute_constraints(bodies, manifolds);
        self.compute_generic_constraints(bodies, multibody_joints, manifolds, &mut jacobian_id);

        #[cfg(feature = "simd-is-enabled")]
        {
            self.simd_compute_one_body_constraints(bodies, manifolds);
        }
        self.compute_one_body_constraints(bodies, manifolds);
        self.compute_generic_one_body_constraints(
            bodies,
            multibody_joints,
            manifolds,
            &mut jacobian_id,
        );
    }

    #[cfg(feature = "simd-is-enabled")]
    fn simd_compute_constraints(
        &mut self,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
    ) {
        let total_num_constraints = self
            .interaction_groups
            .simd_interactions
            .chunks_exact(SIMD_WIDTH)
            .map(|i| ConstraintsCounts::from_contacts(manifolds_all[i[0]]).num_constraints)
            .sum();

        unsafe {
            reset_buffer(
                &mut self.simd_velocity_constraints_builder,
                total_num_constraints,
            );
            reset_buffer(&mut self.simd_velocity_constraints, total_num_constraints);
        }

        let mut curr_start = 0;

        for manifolds_i in self
            .interaction_groups
            .simd_interactions
            .chunks_exact(SIMD_WIDTH)
        {
            let num_to_add =
                ConstraintsCounts::from_contacts(manifolds_all[manifolds_i[0]]).num_constraints;
            let manifold_id = gather![|ii| manifolds_i[ii]];
            let manifolds = gather![|ii| &*manifolds_all[manifolds_i[ii]]];

            TwoBodyConstraintBuilderSimd::generate(
                manifold_id,
                manifolds,
                bodies,
                &mut self.simd_velocity_constraints_builder[curr_start..],
                &mut self.simd_velocity_constraints[curr_start..],
            );

            curr_start += num_to_add;
        }

        assert_eq!(curr_start, total_num_constraints);
    }

    fn compute_constraints(
        &mut self,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
    ) {
        let total_num_constraints = self
            .interaction_groups
            .nongrouped_interactions
            .iter()
            .map(|i| ConstraintsCounts::from_contacts(manifolds_all[*i]).num_constraints)
            .sum();

        unsafe {
            reset_buffer(
                &mut self.velocity_constraints_builder,
                total_num_constraints,
            );
            reset_buffer(&mut self.velocity_constraints, total_num_constraints);
        }

        let mut curr_start = 0;

        for manifold_i in &self.interaction_groups.nongrouped_interactions {
            let manifold = &manifolds_all[*manifold_i];
            let num_to_add = ConstraintsCounts::from_contacts(manifold).num_constraints;

            TwoBodyConstraintBuilder::generate(
                *manifold_i,
                manifold,
                bodies,
                &mut self.velocity_constraints_builder[curr_start..],
                &mut self.velocity_constraints[curr_start..],
            );

            curr_start += num_to_add;
        }
        assert_eq!(curr_start, total_num_constraints);
    }

    fn compute_generic_constraints(
        &mut self,
        bodies: &RigidBodySet,
        multibody_joints: &MultibodyJointSet,
        manifolds_all: &[&mut ContactManifold],
        jacobian_id: &mut usize,
    ) {
        let total_num_constraints = self
            .generic_two_body_interactions
            .iter()
            .map(|i| ConstraintsCounts::from_contacts(manifolds_all[*i]).num_constraints)
            .sum();

        self.generic_velocity_constraints_builder.resize(
            total_num_constraints,
            GenericTwoBodyConstraintBuilder::invalid(),
        );
        self.generic_velocity_constraints
            .resize(total_num_constraints, GenericTwoBodyConstraint::invalid());

        let mut curr_start = 0;

        for manifold_i in &self.generic_two_body_interactions {
            let manifold = &manifolds_all[*manifold_i];
            let num_to_add = ConstraintsCounts::from_contacts(manifold).num_constraints;

            GenericTwoBodyConstraintBuilder::generate(
                *manifold_i,
                manifold,
                bodies,
                multibody_joints,
                &mut self.generic_velocity_constraints_builder[curr_start..],
                &mut self.generic_velocity_constraints[curr_start..],
                &mut self.generic_jacobians,
                jacobian_id,
            );

            curr_start += num_to_add;
        }

        assert_eq!(curr_start, total_num_constraints);
    }

    fn compute_generic_one_body_constraints(
        &mut self,
        bodies: &RigidBodySet,
        multibody_joints: &MultibodyJointSet,
        manifolds_all: &[&mut ContactManifold],
        jacobian_id: &mut usize,
    ) {
        let total_num_constraints = self
            .generic_one_body_interactions
            .iter()
            .map(|i| ConstraintsCounts::from_contacts(manifolds_all[*i]).num_constraints)
            .sum();
        self.generic_velocity_one_body_constraints_builder.resize(
            total_num_constraints,
            GenericOneBodyConstraintBuilder::invalid(),
        );
        self.generic_velocity_one_body_constraints
            .resize(total_num_constraints, GenericOneBodyConstraint::invalid());

        let mut curr_start = 0;

        for manifold_i in &self.generic_one_body_interactions {
            let manifold = &manifolds_all[*manifold_i];
            let num_to_add = ConstraintsCounts::from_contacts(manifold).num_constraints;

            GenericOneBodyConstraintBuilder::generate(
                *manifold_i,
                manifold,
                bodies,
                multibody_joints,
                &mut self.generic_velocity_one_body_constraints_builder[curr_start..],
                &mut self.generic_velocity_one_body_constraints[curr_start..],
                &mut self.generic_jacobians,
                jacobian_id,
            );

            curr_start += num_to_add;
        }
        assert_eq!(curr_start, total_num_constraints);
    }

    #[cfg(feature = "simd-is-enabled")]
    fn simd_compute_one_body_constraints(
        &mut self,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
    ) {
        let total_num_constraints = self
            .one_body_interaction_groups
            .simd_interactions
            .chunks_exact(SIMD_WIDTH)
            .map(|i| ConstraintsCounts::from_contacts(manifolds_all[i[0]]).num_constraints)
            .sum();

        unsafe {
            reset_buffer(
                &mut self.simd_velocity_one_body_constraints_builder,
                total_num_constraints,
            );
            reset_buffer(
                &mut self.simd_velocity_one_body_constraints,
                total_num_constraints,
            );
        }

        let mut curr_start = 0;

        for manifolds_i in self
            .one_body_interaction_groups
            .simd_interactions
            .chunks_exact(SIMD_WIDTH)
        {
            let num_to_add =
                ConstraintsCounts::from_contacts(manifolds_all[manifolds_i[0]]).num_constraints;
            let manifold_id = gather![|ii| manifolds_i[ii]];
            let manifolds = gather![|ii| &*manifolds_all[manifolds_i[ii]]];
            SimdOneBodyConstraintBuilder::generate(
                manifold_id,
                manifolds,
                bodies,
                &mut self.simd_velocity_one_body_constraints_builder[curr_start..],
                &mut self.simd_velocity_one_body_constraints[curr_start..],
            );
            curr_start += num_to_add;
        }
        assert_eq!(curr_start, total_num_constraints);
    }

    fn compute_one_body_constraints(
        &mut self,
        bodies: &RigidBodySet,
        manifolds_all: &[&mut ContactManifold],
    ) {
        let total_num_constraints = self
            .one_body_interaction_groups
            .nongrouped_interactions
            .iter()
            .map(|i| ConstraintsCounts::from_contacts(manifolds_all[*i]).num_constraints)
            .sum();

        unsafe {
            reset_buffer(
                &mut self.velocity_one_body_constraints_builder,
                total_num_constraints,
            );
            reset_buffer(
                &mut self.velocity_one_body_constraints,
                total_num_constraints,
            );
        }

        let mut curr_start = 0;

        for manifold_i in &self.one_body_interaction_groups.nongrouped_interactions {
            let manifold = &manifolds_all[*manifold_i];
            let num_to_add = ConstraintsCounts::from_contacts(manifold).num_constraints;

            OneBodyConstraintBuilder::generate(
                *manifold_i,
                manifold,
                bodies,
                &mut self.velocity_one_body_constraints_builder[curr_start..],
                &mut self.velocity_one_body_constraints[curr_start..],
            );

            curr_start += num_to_add;
        }
        assert_eq!(curr_start, total_num_constraints);
    }

    pub fn warmstart(
        &mut self,
        solver_vels: &mut [SolverVel<Real>],
        generic_solver_vels: &mut DVector<Real>,
    ) {
        let (jac, constraints) = self.iter_constraints_mut();
        for mut c in constraints {
            c.warmstart(jac, solver_vels, generic_solver_vels);
        }
    }

    #[profiling::function]
    pub fn solve_restitution(
        &mut self,
        solver_vels: &mut [SolverVel<Real>],
        generic_solver_vels: &mut DVector<Real>,
    ) {
        let (jac, constraints) = self.iter_constraints_mut();
        for mut c in constraints {
            c.solve_restitution(jac, solver_vels, generic_solver_vels);
        }
    }

    #[profiling::function]
    pub fn solve_restitution_wo_bias(
        &mut self,
        solver_vels: &mut [SolverVel<Real>],
        generic_solver_vels: &mut DVector<Real>,
    ) {
        let (jac, constraints) = self.iter_constraints_mut();
        for mut c in constraints {
            c.remove_bias();
            c.solve_restitution(jac, solver_vels, generic_solver_vels);
        }
    }

    #[profiling::function]
    pub fn solve_friction(
        &mut self,
        solver_vels: &mut [SolverVel<Real>],
        generic_solver_vels: &mut DVector<Real>,
    ) {
        let (jac, constraints) = self.iter_constraints_mut();
        for mut c in constraints {
            c.solve_friction(jac, solver_vels, generic_solver_vels);
        }
    }

    pub fn writeback_impulses(&mut self, manifolds_all: &mut [&mut ContactManifold]) {
        let (_, constraints) = self.iter_constraints_mut();
        for mut c in constraints {
            c.writeback_impulses(manifolds_all);
        }
    }

    #[profiling::function]
    pub fn update(
        &mut self,
        params: &IntegrationParameters,
        small_step_id: usize,
        multibodies: &MultibodyJointSet,
        solver_bodies: &[SolverBody],
    ) {
        macro_rules! update_contacts(
            ($builders: ident, $constraints: ident) => {
                for (builder, constraint) in self.$builders.iter().zip(self.$constraints.iter_mut()) {
                    builder.update(
                        &params,
                        small_step_id as Real * params.dt,
                        solver_bodies,
                        multibodies,
                        constraint,
                    );
                }
            }
        );

        update_contacts!(
            generic_velocity_constraints_builder,
            generic_velocity_constraints
        );
        update_contacts!(velocity_constraints_builder, velocity_constraints);
        #[cfg(feature = "simd-is-enabled")]
        update_contacts!(simd_velocity_constraints_builder, simd_velocity_constraints);

        update_contacts!(
            generic_velocity_one_body_constraints_builder,
            generic_velocity_one_body_constraints
        );
        update_contacts!(
            velocity_one_body_constraints_builder,
            velocity_one_body_constraints
        );
        #[cfg(feature = "simd-is-enabled")]
        update_contacts!(
            simd_velocity_one_body_constraints_builder,
            simd_velocity_one_body_constraints
        );
    }
}
