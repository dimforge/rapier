//! Structures related to dynamics: bodies, impulse_joints, etc.

#[cfg(feature = "alloc")]
pub use self::ccd::CCDSolver;
pub use self::coefficient_combine_rule::CoefficientCombineRule;
pub use self::integration_parameters::{IntegrationParameters, SpringCoefficients};
#[cfg(feature = "alloc")]
pub use self::island_manager::IslandManager;

#[cfg(feature = "dim3")]
pub use self::integration_parameters::FrictionModel;

#[cfg(feature = "alloc")]
pub(crate) use self::joint::JointGraphEdge;
#[cfg(feature = "alloc")]
pub(crate) use self::joint::JointIndex;
pub use self::joint::*;
#[cfg(feature = "alloc")]
pub use self::rigid_body_components::*;
pub use self::rigid_body_handle::RigidBodyHandle;
#[cfg(feature = "alloc")]
pub(crate) use self::rigid_body_set::ModifiedRigidBodies;
// #[cfg(not(feature = "parallel"))]
#[cfg(feature = "alloc")]
pub(crate) use self::solver::IslandSolver;
// #[cfg(feature = "parallel")]
// pub(crate) use self::solver::ParallelIslandSolver;
pub use parry::mass_properties::MassProperties;

#[cfg(feature = "alloc")]
pub use self::rigid_body::{RigidBody, RigidBodyBuilder};
#[cfg(feature = "alloc")]
pub use self::rigid_body_set::{BodyPair, RigidBodySet};

#[cfg(feature = "alloc")]
mod ccd;
mod coefficient_combine_rule;
mod integration_parameters;
#[cfg(feature = "alloc")]
mod island_manager;
mod joint;
#[cfg(feature = "alloc")]
mod rigid_body_components;
mod rigid_body_handle;
#[cfg(feature = "alloc")]
mod solver;

#[cfg(feature = "alloc")]
mod rigid_body;
#[cfg(feature = "alloc")]
mod rigid_body_set;
