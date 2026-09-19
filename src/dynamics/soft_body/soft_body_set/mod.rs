//! The `SoftBodySet`: storage of the soft bodies, their insertion and removal, and their clusters.
mod soft_body_set;
mod soft_body_set_clusters;
mod soft_body_set_insert_remove;
mod soft_body_set_proxies;
mod soft_body_set_split;
mod soft_body_set_step_sync;

pub(crate) use self::soft_body_set::SoftBodyIslandEvent;
pub use self::soft_body_set::SoftBodySet;
#[cfg(feature = "dim3")]
pub(super) use super::soft_body_builder;
pub(super) use super::{SoftBodyParticleSettings, SoftCollisionMesh};
