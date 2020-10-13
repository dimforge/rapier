//! Structures related to dynamics: bodies, joints, etc.

pub use self::integration_parameters::IntegrationParameters;
pub(crate) use self::joint::JointIndex;
#[cfg(feature = "dim3")]
pub use self::joint::RevoluteJoint;
pub use self::joint::{
    BallJoint, FixedJoint, Joint, JointHandle, JointParams, JointSet, PrismaticJoint,
};
pub use self::mass_properties::MassProperties;
pub use self::rigid_body::{ActivationStatus, BodyStatus, RigidBody, RigidBodyBuilder};
pub use self::rigid_body_set::{BodyPair, RigidBodyHandle, RigidBodyMut, RigidBodySet};
// #[cfg(not(feature = "parallel"))]
pub(crate) use self::joint::JointGraphEdge;
#[cfg(not(feature = "parallel"))]
pub(crate) use self::solver::IslandSolver;
#[cfg(feature = "parallel")]
pub(crate) use self::solver::ParallelIslandSolver;

mod integration_parameters;
mod joint;
mod mass_properties;
mod mass_properties_ball;
mod mass_properties_capsule;
mod mass_properties_cuboid;
mod mass_properties_cylinder;
#[cfg(feature = "dim2")]
mod mass_properties_polygon;
mod rigid_body;
mod rigid_body_set;
mod solver;
