//! Data structures modified with guaranteed deterministic behavior after deserialization.

pub use self::coarena::Coarena;
pub use self::maybe_serializable_data::MaybeSerializableData;

pub mod arena;
mod coarena;
pub(crate) mod graph;
pub(crate) mod hashmap;
mod maybe_serializable_data;
pub mod pubsub;
