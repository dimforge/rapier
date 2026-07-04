//! `_rapier3d` — PyO3 extension module for rapier3d (3D, f32).
//!
//! This crate is a single, concrete 3D/f32 binding. (It was previously split
//! into a `rapier-py-core` macro layer plus per-dimension cdylibs; with only
//! the 3D f32 flavor remaining, everything lives here as ordinary modules.)

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

pub mod math;

pub use conv::*;
pub use errors::*;
pub use math::*;

use pyo3::prelude::*;

const RAPIER_PY_VERSION: &str = env!("CARGO_PKG_VERSION");

#[pymodule]
fn _rapier3d(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    errors::register_errors(py, m)?;
    register_math(py, m)?;
    m.add("__version__", RAPIER_PY_VERSION)?;
    Ok(())
}
