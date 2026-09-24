//! C ABI shared by the four Rapier dimensions/precisions. See c/README.md.
#![deny(unsafe_op_in_unsafe_fn)]
#![allow(clippy::missing_safety_doc, clippy::too_many_arguments)]
use rapier::math::{AngVector, Pose, Real, Rotation, Vector};
use rapier::prelude::*;
use rapier_c_macros::rapier_export;
mod config_data;
pub use config_data::*;
mod joint_desc;
pub use joint_desc::*;
mod soft_desc;
pub use soft_desc::*;
mod world_queries;
pub use world_queries::*;
mod descriptors;
pub use descriptors::*;
mod control;
mod dynamics;
mod error;
mod geometry;
mod joints;
mod objects;
mod pipeline;
mod queries;
mod soft_body;
mod types;
pub use control::*;
pub use dynamics::*;
pub use error::*;
pub use geometry::*;
pub use joints::*;
pub use objects::*;
pub use pipeline::*;
pub use queries::*;
pub use soft_body::*;
pub use types::*;

mod extra;
pub use extra::*;

#[cfg(test)]
mod tests;

mod render;
pub use render::*;

#[cfg(all(feature = "robotics", feature = "dim3", feature = "f32"))]
mod robotics;
#[cfg(all(feature = "robotics", feature = "dim3", feature = "f32"))]
pub use robotics::*;

#[cfg(test)]
mod pod_tests;

mod handle_access;
pub use handle_access::*;

mod array_views;
pub use array_views::*;

mod shape_desc;
pub use shape_desc::*;

mod soft_recipes;
pub use soft_recipes::*;

mod scoped_access;
pub use scoped_access::*;

mod joint_access;
pub use joint_access::*;

mod geometry_views;
pub use geometry_views::*;

mod world;
pub use world::*;
mod read_access;
pub use read_access::*;

#[cfg(test)]
mod world_tests;

mod return_values;
pub use return_values::*;

mod handle_world;
use handle_world::*;

#[cfg(test)]
mod owner_handle_tests;
