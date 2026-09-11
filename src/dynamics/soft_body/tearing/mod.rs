//! Tearing and cutting of soft bodies: the cracks opened by duplicating the particles a tear or a
//! cut passes through, the particles a cut inserts into segments, and the surface and coloring
//! rebuild that follows.

mod tearing;
mod tearing_event;
mod tearing_helpers;
mod tearing_particle_split;
mod tearing_tables;
mod tearing_validate;

pub use self::tearing_event::SoftBodyTearEvent;
pub(super) use super::collision_mesh;
