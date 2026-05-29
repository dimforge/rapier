//! Data structures modified with guaranteed deterministic behavior after deserialization.

#[cfg(feature = "alloc")]
pub use self::arena::Arena;
pub use self::arena::Index;
#[cfg(feature = "alloc")]
pub use self::coarena::Coarena;
#[cfg(feature = "alloc")]
pub use self::modified_objects::{HasModifiedFlag, ModifiedObjects};

pub mod arena;
#[cfg(feature = "alloc")]
mod coarena;
#[cfg(feature = "alloc")]
pub(crate) mod graph;
#[cfg(feature = "alloc")]
mod modified_objects;
#[cfg(feature = "alloc")]
pub mod pubsub;
