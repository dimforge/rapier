#[cfg(not(feature = "parallel"))]
pub(crate) use self::island_solver::IslandSolver;
#[cfg(feature = "parallel")]
pub(crate) use self::parallel_island_solver::{ParallelIslandSolver, ThreadContext};
#[cfg(feature = "parallel")]
pub(self) use self::parallel_solver_constraints::ParallelSolverConstraints;
#[cfg(feature = "parallel")]
pub(self) use self::parallel_velocity_solver::ParallelVelocitySolver;
#[cfg(not(feature = "parallel"))]
pub(self) use self::solver_constraints::SolverConstraints;
#[cfg(not(feature = "parallel"))]
pub(self) use self::velocity_solver::VelocitySolver;
pub(self) use delta_vel::DeltaVel;
pub(self) use generic_velocity_constraint::*;
pub(self) use generic_velocity_constraint_element::*;
pub(self) use generic_velocity_ground_constraint::*;
pub(self) use interaction_groups::*;
pub(crate) use joint_constraint::MotorParameters;
pub use joint_constraint::*;
pub(self) use velocity_constraint::*;
pub(self) use velocity_constraint_element::*;
#[cfg(feature = "simd-is-enabled")]
pub(self) use velocity_constraint_wide::*;
pub(self) use velocity_ground_constraint::*;
pub(self) use velocity_ground_constraint_element::*;
#[cfg(feature = "simd-is-enabled")]
pub(self) use velocity_ground_constraint_wide::*;

mod categorization;
mod delta_vel;
mod generic_velocity_constraint;
mod generic_velocity_constraint_element;
mod generic_velocity_ground_constraint;
mod generic_velocity_ground_constraint_element;
mod interaction_groups;
#[cfg(not(feature = "parallel"))]
mod island_solver;
mod joint_constraint;
#[cfg(feature = "parallel")]
mod parallel_island_solver;
#[cfg(feature = "parallel")]
mod parallel_solver_constraints;
#[cfg(feature = "parallel")]
mod parallel_velocity_solver;
#[cfg(not(feature = "parallel"))]
mod solver_constraints;
mod velocity_constraint;
mod velocity_constraint_element;
#[cfg(feature = "simd-is-enabled")]
mod velocity_constraint_wide;
mod velocity_ground_constraint;
mod velocity_ground_constraint_element;
#[cfg(feature = "simd-is-enabled")]
mod velocity_ground_constraint_wide;
#[cfg(not(feature = "parallel"))]
mod velocity_solver;
