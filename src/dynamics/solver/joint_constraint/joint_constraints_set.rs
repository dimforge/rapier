use crate::alloc_prelude::*;
use crate::dynamics::solver::{
    AnyJointConstraintMut, GenericJointConstraint, JointGenericExternalConstraintBuilder,
    JointGenericInternalConstraintBuilder,
};
use crate::dynamics::{JointGraphEdge, MultibodyJointSet, RigidBodySet};
use crate::math::DVector;
use parry::math::Real;

use crate::dynamics::solver::joint_constraint::generic_joint_constraint_builder::GenericJointConstraintBuilder;
use crate::dynamics::solver::joint_constraint::joint_constraint_builder::JointConstraintBuilder;
use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::JointConstraint;
use {
    crate::dynamics::solver::joint_constraint::joint_constraint_builder::JointConstraintBuilderSimd,
    crate::math::{SIMD_WIDTH, SimdReal},
};

pub struct JointConstraintsSet {
    pub generic_jacobians: DVector,
    pub two_body_interactions: Vec<usize>,
    pub generic_two_body_interactions: Vec<usize>,

    pub generic_velocity_constraints: Vec<GenericJointConstraint>,
    pub velocity_constraints: Vec<JointConstraint<Real, 1>>,
    pub simd_velocity_constraints: Vec<JointConstraint<SimdReal, SIMD_WIDTH>>,

    pub generic_velocity_constraints_builder: Vec<GenericJointConstraintBuilder>,
    pub velocity_constraints_builder: Vec<JointConstraintBuilder>,
    pub simd_velocity_constraints_builder: Vec<JointConstraintBuilderSimd>,
}

impl JointConstraintsSet {
    pub fn new() -> Self {
        Self {
            generic_jacobians: DVector::zeros(0),
            two_body_interactions: vec![],
            generic_two_body_interactions: vec![],
            velocity_constraints: vec![],
            generic_velocity_constraints: vec![],
            simd_velocity_constraints: vec![],
            velocity_constraints_builder: vec![],
            generic_velocity_constraints_builder: vec![],
            simd_velocity_constraints_builder: vec![],
        }
    }

    // Returns the generic jacobians and a mutable iterator through all the constraints.
    pub fn iter_constraints_mut(
        &mut self,
    ) -> (&DVector, impl Iterator<Item = AnyJointConstraintMut<'_>>) {
        let jac = &self.generic_jacobians;
        let a = self
            .generic_velocity_constraints
            .iter_mut()
            .map(AnyJointConstraintMut::Generic);
        let b = self
            .velocity_constraints
            .iter_mut()
            .map(AnyJointConstraintMut::Rigid);
        let c = self
            .simd_velocity_constraints
            .iter_mut()
            .map(AnyJointConstraintMut::SimdRigid);
        (jac, a.chain(b).chain(c))
    }
}

impl JointConstraintsSet {
    pub(crate) fn compute_generic_joint_constraints(
        &mut self,
        island_bodies: &[crate::dynamics::RigidBodyHandle],
        bodies: &RigidBodySet,
        multibodies: &MultibodyJointSet,
        joints_all: &[JointGraphEdge],
        j_id: &mut usize,
    ) {
        // Count the internal and external constraints builder.
        let num_external_constraint_builders = self.generic_two_body_interactions.len();
        let mut num_internal_constraint_builders = 0;
        for handle in island_bodies {
            if let Some(link_id) = multibodies.rigid_body_link(*handle) {
                if JointGenericInternalConstraintBuilder::num_constraints(multibodies, link_id) > 0
                {
                    num_internal_constraint_builders += 1;
                }
            }
        }
        let total_num_builders =
            num_external_constraint_builders + num_internal_constraint_builders;

        // Preallocate builders buffer.
        self.generic_velocity_constraints_builder
            .resize(total_num_builders, GenericJointConstraintBuilder::Empty);

        // Generate external constraints builders.
        let mut num_constraints = 0;
        for (joint_i, builder) in self
            .generic_two_body_interactions
            .iter()
            .zip(self.generic_velocity_constraints_builder.iter_mut())
        {
            let joint = &joints_all[*joint_i].weight;
            JointGenericExternalConstraintBuilder::generate(
                *joint_i,
                joint,
                bodies,
                multibodies,
                builder,
                j_id,
                &mut self.generic_jacobians,
                &mut num_constraints,
            );
        }

        // Generate internal constraints builder. They are indexed after the
        let mut curr_builder = self.generic_two_body_interactions.len();
        for handle in island_bodies {
            if curr_builder >= self.generic_velocity_constraints_builder.len() {
                break; // No more builder need to be generated.
            }

            if let Some(link_id) = multibodies.rigid_body_link(*handle) {
                let prev_num_constraints = num_constraints;
                JointGenericInternalConstraintBuilder::generate(
                    multibodies,
                    link_id,
                    &mut self.generic_velocity_constraints_builder[curr_builder],
                    j_id,
                    &mut self.generic_jacobians,
                    &mut num_constraints,
                );
                if num_constraints != prev_num_constraints {
                    curr_builder += 1;
                }
            }
        }

        // Resize constraints buffer now that we know the total count.
        self.generic_velocity_constraints
            .resize(num_constraints, GenericJointConstraint::invalid());
    }

    pub fn writeback_impulses(&mut self, joints_all: &mut [JointGraphEdge]) {
        let (_, constraints) = self.iter_constraints_mut();
        for mut c in constraints {
            c.writeback_impulses(joints_all);
        }
    }
}
