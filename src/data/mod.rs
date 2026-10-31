//! Data structures modified with guaranteed deterministic behavior after deserialization.

#[cfg(feature = "alloc")]
pub use self::arena::Arena;
pub use self::arena::Index;
#[cfg(feature = "alloc")]
pub use self::coarena::Coarena;
#[cfg(feature = "serde-serialize")]
pub use self::delta::{ChangeTick, Delta, DeltaError, SnapshotDelta};
#[cfg(feature = "alloc")]
pub use self::modified_objects::{HasModifiedFlag, ModifiedObjects};

pub mod arena;
#[cfg(feature = "alloc")]
pub(crate) mod change_stamps;
#[cfg(feature = "alloc")]
pub(crate) mod coarena;
#[cfg(feature = "serde-serialize")]
pub mod delta;
#[cfg(feature = "alloc")]
pub(crate) mod graph;
#[cfg(feature = "alloc")]
mod modified_objects;
#[cfg(feature = "alloc")]
pub mod pubsub;
#[cfg(feature = "alloc")]
pub(crate) mod stamped_vec;
#[cfg(feature = "alloc")]
pub(crate) mod stamped_vec_map;
#[cfg(feature = "alloc")]
pub(crate) mod union_find;
