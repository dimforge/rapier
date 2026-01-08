//! Testbed module - visual debugging and example runner for Rapier physics.

#![allow(clippy::bad_bit_mask)]
#![allow(clippy::unnecessary_cast)]
#![allow(clippy::module_inception)]

mod app;
mod graphics_context;
mod hover;
mod keys;
mod state;
mod testbed;

#[cfg(all(feature = "dim3", feature = "other-backends"))]
use crate::physx_backend::PhysxWorld;

// Re-export all public types
pub use app::TestbedApp;
pub use graphics_context::TestbedGraphics;
pub use keys::KeysState;
pub use state::{RunMode, TestbedActionFlags, TestbedState, TestbedStateFlags, UiTab};
pub use testbed::{Example, Testbed};

// Internal re-exports for other modules
pub(crate) use state::{PHYSX_BACKEND_PATCH_FRICTION, PHYSX_BACKEND_TWO_FRICTION_DIR};

/// Backend implementations for other physics engines
#[cfg(feature = "other-backends")]
pub struct OtherBackends {
    #[cfg(feature = "dim3")]
    pub physx: Option<PhysxWorld>,
}

/// Container for testbed plugins
pub struct Plugins(pub Vec<Box<dyn crate::plugin::TestbedPlugin>>);
