//! Data structures modified with guaranteed deterministic behavior after deserialization.

pub use self::coarena::Coarena;

pub mod arena;
mod coarena;
pub(crate) mod graph;
pub mod pubsub;
