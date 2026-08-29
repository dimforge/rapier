//! The FEM soft-body solver, an opt-in alternative to the soft constraints: a
//! [`SoftBodySolver::Fem`](crate::dynamics::SoftBodySolver::Fem) body solves `A Δv = b` with
//! `A = M + h D + h² K` each substep; constraints stay in the staged solver, acting via `A⁻¹Jᵀ`.

mod soft_fem_set;
mod soft_fem_skyline;
mod soft_fem_sparse;
mod system;

pub(crate) use soft_fem_set::SoftFemSet;
