//! The `SoftBodySet`: storage of the soft bodies, their insertion and removal, and their clusters.
mod soft_body_set;
mod soft_body_set_clusters;
mod soft_body_set_insert_remove;
mod soft_body_set_proxies;
mod soft_body_set_step_sync;

pub use self::soft_body_set::SoftBodySet;
pub(crate) use self::soft_body_set::SoftBodyIslandEvent;
pub(super) use super::{SoftBodyParticleSettings, SoftCollisionMesh};
#[cfg(feature = "dim3")]
pub(super) use super::soft_body_builder;
