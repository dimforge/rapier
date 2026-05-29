//! Conversion from `mjcf_rs::Model` into rapier rigid-bodies, colliders,
//! joints, and post-insertion runtime helpers.
//!
//! The driver lives in [`conversion`]; sub-areas (geom shape building,
//! mass-property derivation, joint-frame construction, pose helpers) live
//! in sibling modules and add their own `impl Conversion` blocks.

mod conversion;
mod geom;
mod handles;
mod insert;
mod joint;
mod mass;
mod options;
mod parse;
mod runtime;
mod types;

pub use handles::{
    MjcfActuatorHandle, MjcfBodyHandle, MjcfColliderHandle, MjcfJointHandle, MjcfRobotHandles,
};
pub use options::{ContactFilterMode, MjcfLoaderOptions, MjcfMultibodyOptions};
pub use runtime::MjcfSensorValue;
pub use types::{
    MjcfActuatorBinding, MjcfBody, MjcfEqualityJoint, MjcfJoint, MjcfRobot, MjcfSensorBinding,
    MjcfVisualMesh, SensorObjectRef,
};
