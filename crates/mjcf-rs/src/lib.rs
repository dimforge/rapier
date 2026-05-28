//! ## MJCF parser
//!
//! Pure-Rust parser for the [MuJoCo XML (MJCF) format](https://mujoco.readthedocs.io/en/stable/XMLreference.html).
//!
//! ### Disclaimer
//!
//! Most of this crate — source, tests, and documentation — was produced by
//! an AI coding assistant working iteratively from MJCF reference scenes
//! (primarily the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)),
//! under human direction and review.
//!
//! ### Quick start
//!
//! ```no_run
//! use mjcf_rs::Model;
//! let model = Model::from_file("robot.xml").unwrap();
//! println!("{} bodies", model.bodies_iter().count());
//! ```
//!
//! ### What this crate does
//!
//! - Reads MJCF XML and produces a typed AST.
//! - Resolves `<include file="…"/>` recursively (with cycle detection).
//! - Resolves `<default>` class inheritance — every concrete element has its
//!   final attributes baked in.
//! - Normalizes angle units to radians (`<compiler angle="degree">` is the
//!   MJCF default).
//! - Records `<compiler eulerseq>` so the conversion side can apply Euler
//!   triples in the correct order.
//!
//! ### What this crate does **not** do
//!
//! - Simulate anything. Pair this with `rapier3d-mjcf` for that.
//! - Cover MJCF features that are out of scope for `rapier3d-mjcf`. See the
//!   `rapier3d-mjcf` README for the complete list.

#![warn(missing_docs)]

pub mod assets;
pub mod body;
pub mod compiler;
pub mod contact;
pub mod equality;
mod error;
pub mod extras;
mod loader;
pub mod model;
pub mod types;

#[cfg(feature = "msh")]
pub mod msh;

pub use error::*;
pub use model::*;
pub use types::*;

/// A pose: a 3D rigid-body transform (rotation + translation), backed by
/// [`glamx::DPose3`]. MJCF's several rotation specifications (`quat`,
/// `axisangle`, `euler`, `xyaxes`, `zaxis`) are all collapsed into the
/// quaternion stored here by the loader.
pub use glamx::DPose3 as Pose;
/// Re-export of [`glam`], the math library backing [`Pose`], so
/// downstream crates can name [`glam::DQuat`]/[`glam::DVec3`] without taking a
/// direct dependency on glam.
pub use glamx::glam;
