//! Shared PyO3 plumbing for the four `_rapier{2,3}d{,_f64}` extension modules.
//!
//! This crate is intentionally **not feature-flagged** on `(dim, scalar)` —
//! Cargo unifies features across workspace members, which would mash dim2 and
//! dim3 into the same build. Instead, the dim-/scalar-generic conversion
//! shims (`PyVector`, `PyPoint`, `PyRotation`, `PyIsometry`, `PyAngVector`)
//! are emitted by the `define_conv_types!` macro that each cdylib invokes
//! with its concrete `Real`/`DIM` parameters.
//!
//! The error tree (`RapierError` and its subclasses) IS dim/scalar-independent
//! and lives here as a concrete module that every cdylib registers.

pub use nalgebra as na;
pub use numpy;
pub use pyo3;
// Re-export the serde/bincode/serde_json crates so the macros emitted by
// `define_serde_types!` can refer to them via stable paths
// (`$crate::bincode`, `$crate::serde_json`).
pub use bincode;
pub use serde_json;

pub mod controllers;
pub mod conv;
pub mod debug_render;
pub mod dynamics;
pub mod errors;
pub mod events_hooks;
pub mod geometry;
pub mod joints;
pub mod loaders;
pub mod math;
pub mod mjcf;
pub mod pipeline;
pub mod serde_io;
pub mod serde_macros;
pub mod urdf;

pub use errors::*;
