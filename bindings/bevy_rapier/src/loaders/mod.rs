//! Loaders creating `bevy_rapier` entities from robot descriptions (URDF, MJCF) and colliders
//! from mesh files.
//!
//! Each loader is enabled by the cargo feature of the same name:
//! - `urdf`: [`urdf::spawn_urdf_robot`] spawns a robot loaded from an URDF file.
//! - `mjcf`: [`mjcf::spawn_mjcf_model`] spawns a model loaded from a MuJoCo MJCF file.
//! - `meshloader`: [`Collider::from_mesh_file`](crate::geometry::Collider::from_mesh_file)
//!   builds colliders from STL, Collada or Wavefront files.
//!
//! The robot loaders don't insert anything into the physics scene directly: they spawn entities
//! with regular `bevy_rapier` components (rigid-bodies, colliders, joints), so the usual
//! component synchronization stays authoritative.

#[cfg(any(feature = "urdf", feature = "mjcf"))]
mod common;
#[cfg(feature = "meshloader")]
pub mod meshloader;
#[cfg(feature = "mjcf")]
pub mod mjcf;
#[cfg(feature = "urdf")]
pub mod urdf;

#[cfg(any(feature = "urdf", feature = "mjcf"))]
pub use common::LoaderError;

#[cfg(test)]
mod tests;
