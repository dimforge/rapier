//! `_rapier3d` — PyO3 extension module for rapier3d (3D, f32).

use pyo3::prelude::*;

// Bring the rapier engine into the cdylib's namespace under the alias
// `rapier` so the dynamics macro (which uses `rapier::...` paths) compiles.
use rapier3d as rapier;

// Materialize the dim-/scalar-specific conversion adapter newtypes
// (`PyVector`, `PyPoint`, `PyRotation`, `PyIsometry`, `PyAngVector`) plus the
// `Real`/`DIM` aliases in this crate's namespace.
rapier_py_core::define_conv_types!(Real = f32, DIM = 3);

// Materialize the user-facing math `#[pyclass]` types and the
// `register_math` function used below.
rapier_py_core::define_math_types!(DIM = 3);

// Materialize the geometry `#[pyclass]` types (must come BEFORE
// `define_dynamics_types!` because dynamics references
// `ColliderHandle`/`ColliderSet`).
rapier_py_core::define_geometry_types!(DIM = 3);

// Materialize the joint `#[pyclass]` types. Must come BEFORE
// `define_dynamics_types!` because the dynamics macro references
// `ImpulseJointSet` / `MultibodyJointSet` in `RigidBodySet::remove`.
rapier_py_core::define_joints_types!(DIM = 3);

// Materialize the dynamics `#[pyclass]` types and `register_dynamics`.
rapier_py_core::define_dynamics_types!(DIM = 3);

// Materialize the pipeline / query `#[pyclass]` types.
rapier_py_core::define_pipeline_types!(DIM = 3);

// Materialize the event-handler / physics-hooks `#[pyclass]` types.
rapier_py_core::define_events_hooks_types!(DIM = 3);

// Materialize the loader `#[pyclass]` types. 3D-only.
rapier_py_core::define_loaders_types!(DIM = 3);

// Materialize the controller `#[pyclass]` types.
rapier_py_core::define_controllers_types!(DIM = 3);

// Materialize the debug-render `#[pyclass]` types.
rapier_py_core::define_debug_render_types!(DIM = 3);

// Serialization, snapshots. Must run AFTER every other
// `define_*_types!` because it adds additional `#[pymethods]` blocks on the
// existing pyclasses (relies on pyo3's `multiple-pymethods` feature).
rapier_py_core::define_serde_types!(DIM = 3);

const RAPIER_PY_VERSION: &str = env!("CARGO_PKG_VERSION");

#[pymodule]
fn _rapier3d(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    rapier_py_core::errors::register_errors(py, m)?;
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
