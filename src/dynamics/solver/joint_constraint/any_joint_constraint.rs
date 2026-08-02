use crate::dynamics::JointGraphEdge;
use crate::dynamics::solver::joint_constraint::generic_joint_constraint::GenericJointConstraint;
use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::JointConstraint;
use crate::math::Real;

use crate::math::{SIMD_WIDTH, SimdReal};

#[derive(Debug)]
pub enum AnyJointConstraintMut<'a> {
    Generic(&'a mut GenericJointConstraint),
    Rigid(&'a mut JointConstraint<Real, 1>),
    SimdRigid(&'a mut JointConstraint<SimdReal, SIMD_WIDTH>),
}

impl AnyJointConstraintMut<'_> {
    pub fn writeback_impulses(&mut self, joints_all: &mut [JointGraphEdge]) {
        match self {
            Self::Rigid(c) => c.writeback_impulses(joints_all),
            Self::Generic(c) => c.writeback_impulses(joints_all),
            Self::SimdRigid(c) => c.writeback_impulses(joints_all),
        }
    }
}
