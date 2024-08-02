//! Data structures modified with guaranteed deterministic behavior after deserialization.

pub use self::arena::{Arena, Index};
pub use self::coarena::Coarena;
pub use graph::{Externals, Neighbors, NodeReferences};

pub mod arena;
mod coarena;
pub(crate) mod graph;
pub mod pubsub;
