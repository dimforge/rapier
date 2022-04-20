//! Structures related to dynamics: bodies, impulse_joints, etc.

pub use self::ccd::CCDSolver;
pub use self::coefficient_combine_rule::CoefficientCombineRule;
pub use self::integration_parameters::IntegrationParameters;
pub use self::island_manager::IslandManager;
pub(crate) use self::joint::JointGraphEdge;
pub(crate) use self::joint::JointIndex;
pub use self::joint::*;
pub use self::rigid_body_components::*;
#[cfg(not(feature = "parallel"))]
pub(crate) use self::solver::IslandSolver;
#[cfg(feature = "parallel")]
pub(crate) use self::solver::ParallelIslandSolver;
pub use parry::mass_properties::MassProperties;

pub use self::rigid_body::{RigidBody, RigidBodyBuilder};
pub use self::rigid_body_set::{BodyPair, RigidBodySet};

mod ccd;
mod coefficient_combine_rule;
mod integration_parameters;
mod island_manager;
mod joint;
mod rigid_body_components;
mod solver;

mod rigid_body;
mod rigid_body_set;
