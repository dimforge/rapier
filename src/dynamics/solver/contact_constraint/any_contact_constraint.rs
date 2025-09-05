use crate::dynamics::solver::solver_body::SolverBodies;
use crate::dynamics::solver::{ContactWithCoulombFriction, GenericContactConstraint};
use crate::math::Real;
use na::DVector;

#[cfg(feature = "dim3")]
use crate::dynamics::solver::ContactWithTwistFriction;
use crate::prelude::ContactManifold;

#[derive(Debug)]
pub enum AnyContactConstraintMut<'a> {
    Generic(&'a mut GenericContactConstraint),
    WithCoulombFriction(&'a mut ContactWithCoulombFriction),
    #[cfg(feature = "dim3")]
    WithTwistFriction(&'a mut ContactWithTwistFriction),
}

impl AnyContactConstraintMut<'_> {
    pub fn remove_bias(&mut self) {
        match self {
            Self::Generic(c) => c.remove_cfm_and_bias_from_rhs(),
            Self::WithCoulombFriction(c) => c.remove_cfm_and_bias_from_rhs(),
            #[cfg(feature = "dim3")]
            Self::WithTwistFriction(c) => c.remove_cfm_and_bias_from_rhs(),
        }
    }
    pub fn warmstart(
        &mut self,
        generic_jacobians: &DVector<Real>,
        solver_vels: &mut SolverBodies,
        generic_solver_vels: &mut DVector<Real>,
    ) {
        match self {
            Self::Generic(c) => c.warmstart(generic_jacobians, solver_vels, generic_solver_vels),
            Self::WithCoulombFriction(c) => c.warmstart(solver_vels),
            #[cfg(feature = "dim3")]
            Self::WithTwistFriction(c) => c.warmstart(solver_vels),
        }
    }

    pub fn solve(
        &mut self,
        generic_jacobians: &DVector<Real>,
        bodies: &mut SolverBodies,
        generic_solver_vels: &mut DVector<Real>,
    ) {
        match self {
            Self::Generic(c) => c.solve(generic_jacobians, bodies, generic_solver_vels, true, true),
            Self::WithCoulombFriction(c) => c.solve(bodies, true, true),
            #[cfg(feature = "dim3")]
            Self::WithTwistFriction(c) => c.solve(bodies, true, true),
        }
    }

    pub fn writeback_impulses(&mut self, manifolds_all: &mut [&mut ContactManifold]) {
        match self {
            Self::Generic(c) => c.writeback_impulses(manifolds_all),
            Self::WithCoulombFriction(c) => c.writeback_impulses(manifolds_all),
            #[cfg(feature = "dim3")]
            Self::WithTwistFriction(c) => c.writeback_impulses(manifolds_all),
        }
    }
}
