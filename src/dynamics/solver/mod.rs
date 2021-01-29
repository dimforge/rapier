#[cfg(not(feature = "parallel"))]
pub(crate) use self::island_solver::IslandSolver;
#[cfg(feature = "parallel")]
pub(crate) use self::parallel_island_solver::{ParallelIslandSolver, ThreadContext};
#[cfg(feature = "parallel")]
pub(self) use self::parallel_position_solver::ParallelPositionSolver;
#[cfg(feature = "parallel")]
pub(self) use self::parallel_solver_constraints::ParallelSolverConstraints;
#[cfg(feature = "parallel")]
pub(self) use self::parallel_velocity_solver::ParallelVelocitySolver;
#[cfg(not(feature = "parallel"))]
pub(self) use self::position_solver::PositionSolver;
#[cfg(not(feature = "parallel"))]
pub(self) use self::solver_constraints::SolverConstraints;
#[cfg(not(feature = "parallel"))]
pub(self) use self::velocity_solver::VelocitySolver;
pub(self) use delta_vel::DeltaVel;
pub(self) use interaction_groups::*;
pub(self) use joint_constraint::*;
pub(self) use position_constraint::*;
#[cfg(feature = "simd-is-enabled")]
pub(self) use position_constraint_wide::*;
pub(self) use position_ground_constraint::*;
#[cfg(feature = "simd-is-enabled")]
pub(self) use position_ground_constraint_wide::*;
pub(self) use velocity_constraint::*;
#[cfg(feature = "simd-is-enabled")]
pub(self) use velocity_constraint_wide::*;
pub(self) use velocity_ground_constraint::*;
#[cfg(feature = "simd-is-enabled")]
pub(self) use velocity_ground_constraint_wide::*;

mod categorization;
mod delta_vel;
mod interaction_groups;
#[cfg(not(feature = "parallel"))]
mod island_solver;
mod joint_constraint;
#[cfg(feature = "parallel")]
mod parallel_island_solver;
#[cfg(feature = "parallel")]
mod parallel_position_solver;
#[cfg(feature = "parallel")]
mod parallel_solver_constraints;
#[cfg(feature = "parallel")]
mod parallel_velocity_solver;
mod position_constraint;
#[cfg(feature = "simd-is-enabled")]
mod position_constraint_wide;
mod position_ground_constraint;
#[cfg(feature = "simd-is-enabled")]
mod position_ground_constraint_wide;
#[cfg(not(feature = "parallel"))]
mod position_solver;
#[cfg(not(feature = "parallel"))]
mod solver_constraints;
mod velocity_constraint;
#[cfg(feature = "simd-is-enabled")]
mod velocity_constraint_wide;
mod velocity_ground_constraint;
#[cfg(feature = "simd-is-enabled")]
mod velocity_ground_constraint_wide;
#[cfg(not(feature = "parallel"))]
mod velocity_solver;
