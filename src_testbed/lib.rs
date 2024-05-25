#![allow(clippy::too_many_arguments)]
#![allow(unexpected_cfgs)] // This happens due to the dim2/dim3/f32/f64 cfg.

extern crate nalgebra as na;

#[macro_use]
extern crate bitflags;

#[cfg(feature = "log")]
#[macro_use]
extern crate log;

pub use crate::graphics::{BevyMaterial, GraphicsManager};
pub use crate::harness::plugin::HarnessPlugin;
pub use crate::physics::PhysicsState;
pub use crate::plugin::TestbedPlugin;
pub use crate::testbed::{Testbed, TestbedApp, TestbedGraphics, TestbedState};
pub use bevy::prelude::KeyCode;

#[cfg(all(feature = "dim2", feature = "other-backends"))]
mod box2d_backend;
#[cfg(feature = "dim2")]
mod camera2d;
#[cfg(feature = "dim3")]
mod camera3d;
mod debug_render;
mod graphics;
pub mod harness;
mod mouse;
pub mod objects;
pub mod physics;
#[cfg(all(feature = "dim3", feature = "other-backends"))]
mod physx_backend;
mod plugin;
mod testbed;
mod ui;

#[cfg(feature = "dim2")]
pub mod math {
    pub type Isometry<N> = na::Isometry2<N>;
    pub type Vector<N> = na::Vector2<N>;
    pub type Point<N> = na::Point2<N>;
    pub type Translation<N> = na::Translation2<N>;
}

#[cfg(feature = "dim3")]
pub mod math {
    pub type Isometry<N> = na::Isometry3<N>;
    pub type Vector<N> = na::Vector3<N>;
    pub type Point<N> = na::Point3<N>;
    pub type Translation<N> = na::Translation3<N>;
}
