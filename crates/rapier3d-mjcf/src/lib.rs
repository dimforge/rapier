//! ## MJCF loader for the Rapier physics engine
//!
//! Rapier is a set of 2D and 3D physics engines for games, animation, and
//! robotics. The `rapier3d-mjcf` crate lets you convert a [MuJoCo MJCF XML
//! file](https://mujoco.readthedocs.io/en/stable/XMLreference.html) into a
//! set of rigid-bodies, colliders, and joints, for use with the `rapier3d`
//! physics engine.
//!
//! ### Disclaimer
//!
//! Most of this crate — source, tests, and documentation — was produced by
//! an AI coding assistant working iteratively from MJCF reference scenes
//! (primarily the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)),
//! under human direction and review.
//!
//! ```no_run
//! use rapier3d::prelude::*;
//! use rapier3d_mjcf::{MjcfLoaderOptions, MjcfRobot};
//!
//! let mut bodies = RigidBodySet::new();
//! let mut colliders = ColliderSet::new();
//! let mut impulse_joints = ImpulseJointSet::new();
//!
//! let (robot, _model) =
//!     MjcfRobot::from_file("robot.xml", MjcfLoaderOptions::default()).unwrap();
//! robot.insert_using_impulse_joints(&mut bodies, &mut colliders, &mut impulse_joints);
//! ```
//!
//! See the crate-level [README](https://github.com/dimforge/rapier/blob/master/crates/rapier3d-mjcf/README.md)
//! for a feature matrix.

#![warn(missing_docs)]

mod hooks;
mod loader;

pub use hooks::*;
pub use loader::*;
pub use mjcf_rs;
