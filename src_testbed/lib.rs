#![allow(clippy::too_many_arguments)]

extern crate nalgebra as na;

pub use crate::graphics::GraphicsManager;
pub use crate::harness::plugin::HarnessPlugin;
pub use crate::physics::PhysicsState;
pub use crate::plugin::TestbedPlugin;
pub use crate::testbed::{
    Example, KeysState, RunMode, Testbed, TestbedActionFlags, TestbedApp, TestbedGraphics,
    TestbedState, TestbedStateFlags,
};

// Re-export kiss3d types that users might need
pub use kiss3d::event::{Action, Key, MouseButton, WindowEvent};

// KeyCode alias for backwards compatibility with examples
// Maps to kiss3d Key variants
pub use kiss3d::event::Key as KeyCode;

// Re-export egui for UI
pub use egui;

mod debug_render;
mod graphics;
pub mod harness;
mod mouse;
pub mod physics;
#[cfg(all(feature = "dim3", feature = "other-backends"))]
mod physx_backend;
mod plugin;
mod save;
mod settings;
mod testbed;
pub mod ui;

#[cfg(feature = "dim3")]
pub use kiss3d::camera::OrbitCamera3d as Camera;
#[cfg(feature = "dim2")]
pub use kiss3d::camera::PanZoomCamera2d as Camera;

#[cfg(feature = "dim2")]
pub mod math {
    pub type SimdPose<N> = na::Isometry2<N>;
    pub type SimdVector<N> = na::Vector2<N>;
    pub type Point<N> = na::Point2<N>;
    pub type Translation<N> = na::Translation2<N>;
}

#[cfg(feature = "dim3")]
pub mod math {
    pub type SimdPose<N> = na::Isometry3<N>;
    pub type SimdVector<N> = na::Vector3<N>;
    pub type Point<N> = na::Point3<N>;
    pub type Translation<N> = na::Translation3<N>;
}
