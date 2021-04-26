//! Structures related to dynamics: bodies, joints, etc.

pub use self::ccd::CCDSolver;
pub use self::coefficient_combine_rule::CoefficientCombineRule;
pub use self::integration_parameters::IntegrationParameters;
pub use self::island_manager::IslandManager;
pub(crate) use self::joint::JointGraphEdge;
pub(crate) use self::joint::JointIndex;
#[cfg(feature = "dim3")]
pub use self::joint::RevoluteJoint;
pub use self::joint::{
    BallJoint,
    FixedJoint,
    Joint,
    JointHandle,
    JointParams,
    JointSet,
    PrismaticJoint,
    SpringModel, // GenericJoint
};
pub use self::rigid_body_components::*;
#[cfg(not(feature = "parallel"))]
pub(crate) use self::solver::IslandSolver;
#[cfg(feature = "parallel")]
pub(crate) use self::solver::ParallelIslandSolver;
pub use parry::mass_properties::MassProperties;

#[cfg(feature = "default-sets")]
pub use self::rigid_body::{RigidBody, RigidBodyBuilder};
#[cfg(feature = "default-sets")]
pub use self::rigid_body_set::{BodyPair, RigidBodySet};

mod ccd;
mod coefficient_combine_rule;
mod integration_parameters;
mod island_manager;
mod joint;
mod rigid_body_components;
mod solver;

#[cfg(feature = "default-sets")]
mod rigid_body;
#[cfg(feature = "default-sets")]
mod rigid_body_set;
