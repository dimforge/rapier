use downcast_rs::{impl_downcast, DowncastSync};
#[cfg(feature = "serde-serialize")]
use erased_serde::Serialize;

/// Piece of data that may be serializable.
pub trait MaybeSerializableData: DowncastSync {
    /// Convert this shape as a serializable entity.
    #[cfg(feature = "serde-serialize")]
    fn as_serialize(&self) -> Option<(u32, &dyn Serialize)> {
        None
    }

    /// Clones `self`.
    fn clone_dyn(&self) -> Box<dyn MaybeSerializableData>;
}

impl_downcast!(sync MaybeSerializableData);
