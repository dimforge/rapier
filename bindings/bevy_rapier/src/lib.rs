//!
//! # Official integration of Rapier to the Bevy game engine
//!
//! Rapier is a set of two Rust crates `rapier2d` and `rapier3d` for efficient cross-platform
//! physics simulation. Its target application include video games, animation, robotics, etc.
//!
//! The `bevy_rapier` project implements two crates `bevy_rapier2d` and `bevy_rapier3d` which
//! define physics plugins for the Bevy game engine.
//!
//! User documentation for `bevy_rapier` is on [the official Rapier site](https://rapier.rs/docs/).
//!
#![warn(missing_docs)]

#[macro_use]
#[cfg(feature = "serde-serialize")]
extern crate serde;

pub extern crate nalgebra as na;
#[cfg(feature = "dim2")]
pub extern crate rapier2d as rapier;
#[cfg(feature = "dim3")]
pub extern crate rapier3d as rapier;
pub use rapier::parry;
/// Type aliases to select the right vector/rotation types based
/// on the dimension used by the engine.
#[cfg(feature = "dim2")]
pub mod math {
    use bevy::math::Vec2;
    /// The real type (f32 or f64).
    pub type Real = rapier::math::Real;
    /// The vector type.
    pub type Vect = Vec2;
    /// The integer vector type.
    pub type IVect = bevy::math::IVec2;
    /// The rotation type (in 2D this is an angle in radians).
    pub type Rot = Real;
    /// The angular vector type (in 2D this is a scalar).
    pub type AngVector = Real;
}

/// Type aliases to select the right vector/rotation types based
/// on the dimension used by the engine.
#[cfg(feature = "dim3")]
pub mod math {
    use bevy::math::{Quat, Vec3};
    /// The real type (f32 or f64).
    pub type Real = rapier::math::Real;
    /// The vector type.
    pub type Vect = Vec3;
    /// The integer vector type.
    pub type IVect = bevy::math::IVec3;
    /// The rotation type.
    pub type Rot = Quat;
    /// The angular vector type.
    pub type AngVector = Vec3;
}

/// Components related to physics dynamics (rigid-bodies, velocities, etc.)
pub mod dynamics;
/// Components related to physics geometry (colliders, collision-groups, etc.)
pub mod geometry;
/// Components and resources related to the physics simulation workflow (events, hooks, etc.)
pub mod pipeline;
/// The physics plugin and systems.
pub mod plugin;

/// Reflection utilities.
pub mod reflect;

#[cfg(feature = "picking-backend")]
pub mod picking_backend;

/// Components related to character control.
pub mod control;
/// The debug-renderer.
#[cfg(any(feature = "debug-render-3d", feature = "debug-render-2d"))]
pub mod render;
/// Miscellaneous helper functions.
pub mod utils;

#[cfg(all(
    feature = "dim3",
    any(feature = "urdf", feature = "mjcf", feature = "meshloader")
))]
pub mod loaders;

/// Groups the most often used types.
pub mod prelude {
    pub use crate::control::*;
    pub use crate::dynamics::*;
    pub use crate::geometry::*;
    pub use crate::math::*;
    #[cfg(feature = "picking-backend")]
    pub use crate::picking_backend::*;
    pub use crate::pipeline::*;
    pub use crate::plugin::context::systemparams::*;
    pub use crate::plugin::context::*;
    pub use crate::plugin::*;
    #[cfg(any(feature = "debug-render-3d", feature = "debug-render-2d"))]
    pub use crate::render::*;
}
