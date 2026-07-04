//! The `RapierError` exception tree shared by every cdylib.
//!
//! All concrete error types inherit from `RapierError` (which itself inherits
//! from Python's `Exception`), so users can write
//! `except rapier.RapierError as e:` to catch any rapier-originated failure.

use pyo3::create_exception;
use pyo3::exceptions::PyException;
use pyo3::prelude::*;

create_exception!(
    rapier,
    RapierError,
    PyException,
    "Base exception for all rapier-originated errors."
);
create_exception!(
    rapier,
    InvalidHandle,
    RapierError,
    "Raised when a handle (rigid body, collider, joint, ...) no longer refers to a live object."
);
create_exception!(
    rapier,
    MeshConversionError,
    RapierError,
    "Raised when converting between mesh representations fails (degenerate, non-manifold, ...)."
);
create_exception!(
    rapier,
    UrdfError,
    RapierError,
    "Raised by the URDF loader on parse or structural errors."
);
create_exception!(
    rapier,
    MjcfError,
    RapierError,
    "Raised by the MJCF loader on parse or structural errors."
);
create_exception!(
    rapier,
    SerializationError,
    RapierError,
    "Raised when (de)serializing a physics scene fails."
);
create_exception!(
    rapier,
    QueryFailure,
    RapierError,
    "Raised when a spatial query (raycast, point projection, ...) cannot complete."
);
create_exception!(
    rapier,
    MeshLoaderError,
    RapierError,
    "Raised by the mesh loader (e.g. STL/OBJ ingestion) on parse or IO errors."
);

/// Register every exception class defined in this crate onto the given pymodule.
///
/// Call this exactly once from each cdylib's `#[pymodule]` function.
pub fn register_errors(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("RapierError", py.get_type_bound::<RapierError>())?;
    m.add("InvalidHandle", py.get_type_bound::<InvalidHandle>())?;
    m.add(
        "MeshConversionError",
        py.get_type_bound::<MeshConversionError>(),
    )?;
    m.add("UrdfError", py.get_type_bound::<UrdfError>())?;
    m.add("MjcfError", py.get_type_bound::<MjcfError>())?;
    m.add(
        "SerializationError",
        py.get_type_bound::<SerializationError>(),
    )?;
    m.add("QueryFailure", py.get_type_bound::<QueryFailure>())?;
    m.add("MeshLoaderError", py.get_type_bound::<MeshLoaderError>())?;
    Ok(())
}

/// Convenience macro that each cdylib calls from its `#[pymodule]`:
/// ```ignore
/// #[pymodule]
/// fn _rapier3d(py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
///     rapier_py_core::register_errors!(py, m);
///     Ok(())
/// }
/// ```
#[macro_export]
macro_rules! register_errors {
    ($py:expr, $m:expr) => {
        $crate::errors::register_errors($py, $m)?
    };
}
