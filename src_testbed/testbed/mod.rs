//! Testbed support types: UI/run state, keyboard state, and hover highlighting.

#![allow(clippy::bad_bit_mask)]
#![allow(clippy::unnecessary_cast)]
#![allow(clippy::module_inception)]

pub(crate) mod hover;
pub(crate) mod keys;
pub(crate) mod state;

// Re-export the public testbed-support types.
pub use keys::KeysState;
pub use state::{
    ExampleEntry, RunMode, TestbedActionFlags, TestbedState, TestbedStateFlags, UiTab,
};
