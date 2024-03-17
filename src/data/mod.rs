//! Data structures modified with guaranteed deterministic behavior after deserialization.

pub use self::arena::{Arena, Index};
pub use self::coarena::Coarena;

#[cfg(feature = "bevy")]
pub use self::entity_arena::EntityArena;

pub mod arena;
mod coarena;
pub(crate) mod graph;
pub mod pubsub;

#[cfg(feature = "bevy")]
mod entity_arena;
