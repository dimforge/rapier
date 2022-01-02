pub use self::fixed_joint::FixedJoint;
pub use self::impulse_joint::*;
pub use self::joint_data::*;
pub use self::motor_model::MotorModel;
pub use self::multibody_joint::*;
pub use self::prismatic_joint::PrismaticJoint;
pub use self::revolute_joint::RevoluteJoint;

#[cfg(feature = "dim3")]
pub use self::spherical_joint::SphericalJoint;

mod fixed_joint;
mod impulse_joint;
mod joint_data;
mod motor_model;
mod multibody_joint;
mod prismatic_joint;
mod revolute_joint;

#[cfg(feature = "dim3")]
mod spherical_joint;
