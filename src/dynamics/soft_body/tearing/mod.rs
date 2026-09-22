//! Tearing and cutting of soft bodies: the cracks opened by duplicating the particles a tear or a
//! cut passes through, the particles a cut inserts into segments, and the surface and coloring
//! rebuild that follows.

mod tearing;
mod tearing_crack;
mod tearing_cut;
mod tearing_event;
mod tearing_helpers;
mod tearing_particle_split;
mod tearing_tables;
mod tearing_validate;
#[cfg(test)]
mod tests;

pub use self::tearing_event::{SoftBodyPiece, SoftBodyTearEvent, SoftClusterSplit, SoftJointMove};
pub(super) use super::collision_mesh;
