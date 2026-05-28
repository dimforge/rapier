//! `_rapier3d_f64` — PyO3 extension module for rapier3d-f64 (3D, f64).

use pyo3::prelude::*;

use rapier3d_f64 as rapier;

rapier_py_core::define_conv_types!(Real = f64, DIM = 3);
rapier_py_core::define_math_types!(DIM = 3);
rapier_py_core::define_geometry_types!(DIM = 3);
rapier_py_core::define_joints_types!(DIM = 3);
rapier_py_core::define_dynamics_types!(DIM = 3);
rapier_py_core::define_pipeline_types!(DIM = 3);
rapier_py_core::define_events_hooks_types!(DIM = 3);
rapier_py_core::define_controllers_types!(DIM = 3);

// debug-render adapter.
rapier_py_core::define_debug_render_types!(DIM = 3);

// must run AFTER every other `define_*_types!`.
rapier_py_core::define_serde_types!(DIM = 3);

const RAPIER_PY_VERSION: &str = env!("CARGO_PKG_VERSION");

#[pymodule]
fn _rapier3d_f64(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    rapier_py_core::errors::register_errors(py, m)?;
    register_math(py, m)?;
    register_geometry(py, m)?;
    register_joints(py, m)?;
    register_dynamics(py, m)?;
    register_pipeline(py, m)?;
    register_events_hooks(py, m)?;
    register_controllers(py, m)?;
    register_debug_render(py, m)?;
    m.add("__version__", RAPIER_PY_VERSION)?;
    Ok(())
}
