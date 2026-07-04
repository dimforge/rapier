//! `_rapier3d` — PyO3 extension module for rapier3d (3D, f32).
//!
//! This crate is a single, concrete 3D/f32 binding. (It was previously split
//! into a `rapier-py-core` macro layer plus per-dimension cdylibs; with only
//! the 3D f32 flavor remaining, everything lives here as ordinary modules.)

// pyo3 0.22's `#[pymethods]` macro emits `unsafe fn` wrappers whose bodies call
// pyo3-internal unsafe functions without an inner `unsafe {}` block. Under the
// 2024 edition that trips `unsafe_op_in_unsafe_fn`. The calls are pyo3-internal
// and correct; the previous macro-based layout hid them via macro hygiene.
#![allow(unsafe_op_in_unsafe_fn)]
// These modules are a faithful, mechanical de-macroization of the former
// `rapier-py-core` macro layer. Clippy never lint-checked the macro-expanded
// code (it skips external-macro output), so the same code now surfaces
// warnings it did not before. The flagged patterns are all intentional:
//   - SCREAMING_CASE enum variants and methods map 1:1 to Python enum members
//     and factory names — renaming them would break the public Python API.
//   - `new`-returning-a-builder, `to_/from_rapier` conventions, and boxed-fn
//     signature types are pyo3 idioms.
//   - the leftover identity `.into()`s come from the f64→f32 flavor collapse.
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(clippy::useless_conversion)]
#![allow(clippy::wrong_self_convention)]
#![allow(clippy::new_ret_no_self)]
#![allow(clippy::type_complexity)]
#![allow(clippy::field_reassign_with_default)]
#![allow(clippy::redundant_closure_call)]
#![allow(clippy::redundant_locals)]

// Re-export the crate dependencies so the modules can refer to them via stable
// `crate::…` paths (a holdover from the former macro layer, kept intentional).
pub use bincode;
pub use nalgebra as na;
pub use numpy;
pub use pyo3;
pub use serde_json;

pub mod conv;
pub mod errors;
pub mod serde_io;

pub mod controllers;
pub mod debug_render;
pub mod dynamics;
pub mod events_hooks;
pub mod geometry;
pub mod joints;
pub mod loaders;
pub mod math;
pub mod pipeline;
pub mod serde_glue;

pub use controllers::*;
pub use conv::*;
pub use debug_render::*;
pub use dynamics::*;
pub use errors::*;
pub use events_hooks::*;
pub use geometry::*;
pub use joints::*;
pub use loaders::*;
pub use math::*;
pub use pipeline::*;

use pyo3::prelude::*;

const RAPIER_PY_VERSION: &str = env!("CARGO_PKG_VERSION");

#[pymodule]
fn _rapier3d(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    errors::register_errors(py, m)?;
    register_math(py, m)?;
    register_geometry(py, m)?;
    register_joints(py, m)?;
    register_dynamics(py, m)?;
    register_pipeline(py, m)?;
    register_events_hooks(py, m)?;
    register_loaders(py, m)?;
    register_controllers(py, m)?;
    register_debug_render(py, m)?;
    m.add("__version__", RAPIER_PY_VERSION)?;
    Ok(())
}
