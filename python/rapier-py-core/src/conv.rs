//! `FromPyObject` shims for the math primitives every binding consumes.
//!
//! These types are **input adapters**: they implement `FromPyObject` so that
//! `#[pyfunction] fn foo(v: PyVector)` will accept tuples, lists, NumPy
//! arrays, and other PyVector instances of the right shape.
//!
//! The dim/scalar split is handled by the `define_conv_types!` macro at the
//! bottom of this file. Each cdylib invokes it once near the top of its
//! `src/lib.rs` with concrete `Real`/`DIM` parameters; the macro emits
//! locally-defined `PyVector`, `PyPoint`, `PyRotation`, `PyIsometry`,
//! `PyAngVector` types in the cdylib's namespace.
//!
//! Helper functions here are generic and re-used by the macro expansion.

use numpy::PyReadonlyArray1;
use pyo3::prelude::*;
use pyo3::types::PySequence;

/// Extract `expected` floats from a Python tuple/list/sequence.
pub fn extract_floats_from_sequence<R>(obj: &Bound<'_, PyAny>, expected: usize) -> PyResult<Vec<R>>
where
    R: Copy + for<'a> FromPyObject<'a>,
{
    let seq = obj.downcast::<PySequence>().map_err(|_| {
        pyo3::exceptions::PyTypeError::new_err(
            "expected a tuple, list, ndarray, or matching newtype",
        )
    })?;
    let len = seq.len()?;
    if len != expected {
        return Err(pyo3::exceptions::PyValueError::new_err(format!(
            "expected sequence of {expected} floats, got length {len}"
        )));
    }
    let mut out = Vec::with_capacity(expected);
    for i in 0..expected {
        let item = seq.get_item(i)?;
        out.push(item.extract::<R>()?);
    }
    Ok(out)
}

/// Extract `expected` floats from a 1-D NumPy ndarray (accepts f32 or f64
/// dtype and casts to the requested scalar). Returns `None` if `obj` is not
/// an ndarray at all.
pub fn extract_floats_from_ndarray<R>(
    obj: &Bound<'_, PyAny>,
    expected: usize,
) -> Option<PyResult<Vec<R>>>
where
    R: num_from::FromF64 + num_from::FromF32,
{
    if let Ok(arr) = obj.extract::<PyReadonlyArray1<f64>>() {
        let slice_res = arr.as_slice();
        return Some(match slice_res {
            Ok(s) if s.len() == expected => Ok(s.iter().map(|x| R::from_f64(*x)).collect()),
            Ok(s) => Err(pyo3::exceptions::PyValueError::new_err(format!(
                "expected ndarray of length {expected}, got {}",
                s.len()
            ))),
            Err(_) => Err(pyo3::exceptions::PyValueError::new_err(
                "ndarray is not contiguous",
            )),
        });
    }
    if let Ok(arr) = obj.extract::<PyReadonlyArray1<f32>>() {
        let slice_res = arr.as_slice();
        return Some(match slice_res {
            Ok(s) if s.len() == expected => Ok(s.iter().map(|x| R::from_f32(*x)).collect()),
            Ok(s) => Err(pyo3::exceptions::PyValueError::new_err(format!(
                "expected ndarray of length {expected}, got {}",
                s.len()
            ))),
            Err(_) => Err(pyo3::exceptions::PyValueError::new_err(
                "ndarray is not contiguous",
            )),
        });
    }
    None
}

/// Tiny `From{f32,f64}` shim so the helpers can return either scalar.
pub mod num_from {
    pub trait FromF64 {
        fn from_f64(x: f64) -> Self;
    }
    pub trait FromF32 {
        fn from_f32(x: f32) -> Self;
    }

    impl FromF64 for f32 {
        #[inline]
        fn from_f64(x: f64) -> Self {
            x as f32
        }
    }
    impl FromF32 for f32 {
        #[inline]
        fn from_f32(x: f32) -> Self {
            x
        }
    }
    impl FromF64 for f64 {
        #[inline]
        fn from_f64(x: f64) -> Self {
            x
        }
    }
    impl FromF32 for f64 {
        #[inline]
        fn from_f32(x: f32) -> Self {
            x as f64
        }
    }
}

/// Extract `expected` floats from any of: PySequence (tuple/list), 1-D
/// ndarray, or any Python object exposing a `to_tuple()` method that returns
/// a sequence of the right length (this covers the canonical `Vec*`/`Point*`/
/// `Rotation*` math `#[pyclass]` types).
pub fn extract_floats<R>(obj: &Bound<'_, PyAny>, expected: usize) -> PyResult<Vec<R>>
where
    R: Copy + num_from::FromF64 + num_from::FromF32 + for<'a> FromPyObject<'a>,
{
    if let Some(res) = extract_floats_from_ndarray::<R>(obj, expected) {
        return res;
    }
    if let Ok(res) = extract_floats_from_sequence::<R>(obj, expected) {
        return Ok(res);
    }
    // Fall back to duck-typed `.to_tuple()` so that `Vec3`/`Point3`/
    // `Rotation3`/`Quaternion` instances flow through any API that accepts
    // a `PyVector`/`PyPoint`/`PyRotation`.
    if let Ok(meth) = obj.getattr(pyo3::intern!(obj.py(), "to_tuple")) {
        let t = meth.call0()?;
        return extract_floats_from_sequence::<R>(&t, expected);
    }
    Err(pyo3::exceptions::PyTypeError::new_err(
        "expected a tuple, list, ndarray, or a Vec/Point/Rotation-like object",
    ))
}

// ---------------------------------------------------------------------------
// Macro: define_conv_types!
//
// Each cdylib invokes this exactly once with its concrete (Real, DIM) parameters
// to materialize the `PyVector`, `PyPoint`, `PyRotation`, `PyIsometry`,
// `PyAngVector` adapter newtypes in its own namespace.
// ---------------------------------------------------------------------------

/// Emit `Py{Vector,Point,Rotation,Isometry,AngVector}` adapter types specialized
/// for the given `(Real, DIM)` pair. The `dim` token must be the literal `2` or
/// `3` so the macro can pick the right nalgebra constructors.
#[macro_export]
macro_rules! define_conv_types {
    (Real = $real:ty, DIM = 2) => {
        $crate::define_conv_types!(@common $real, 2);

        #[derive(Debug, Clone, Copy)]
        pub struct PyVector(pub $crate::na::Vector2<$real>);
        impl<'py> $crate::pyo3::FromPyObject<'py> for PyVector {
            fn extract_bound(obj: &$crate::pyo3::Bound<'py, $crate::pyo3::PyAny>) -> $crate::pyo3::PyResult<Self> {
                let v: Vec<$real> = $crate::conv::extract_floats(obj, 2)?;
                Ok(PyVector($crate::na::Vector2::new(v[0], v[1])))
            }
        }

        #[derive(Debug, Clone, Copy)]
        pub struct PyPoint(pub $crate::na::Point2<$real>);
        impl<'py> $crate::pyo3::FromPyObject<'py> for PyPoint {
            fn extract_bound(obj: &$crate::pyo3::Bound<'py, $crate::pyo3::PyAny>) -> $crate::pyo3::PyResult<Self> {
                let v: Vec<$real> = $crate::conv::extract_floats(obj, 2)?;
                Ok(PyPoint($crate::na::Point2::new(v[0], v[1])))
            }
        }

        /// In 2D, the angular vector is a bare angle in radians.
        #[derive(Debug, Clone, Copy)]
        pub struct PyAngVector(pub $real);
        impl<'py> $crate::pyo3::FromPyObject<'py> for PyAngVector {
            fn extract_bound(obj: &$crate::pyo3::Bound<'py, $crate::pyo3::PyAny>) -> $crate::pyo3::PyResult<Self> {
                if let Ok(f) = obj.extract::<$real>() {
                    return Ok(PyAngVector(f));
                }
                let v: Vec<$real> = $crate::conv::extract_floats(obj, 1)?;
                Ok(PyAngVector(v[0]))
            }
        }

        #[derive(Debug, Clone, Copy)]
        pub struct PyRotation(pub $crate::na::UnitComplex<$real>);
        impl<'py> $crate::pyo3::FromPyObject<'py> for PyRotation {
            fn extract_bound(obj: &$crate::pyo3::Bound<'py, $crate::pyo3::PyAny>) -> $crate::pyo3::PyResult<Self> {
                if let Ok(f) = obj.extract::<$real>() {
                    return Ok(PyRotation($crate::na::UnitComplex::new(f)));
                }
                let v: Vec<$real> = $crate::conv::extract_floats(obj, 1)?;
                Ok(PyRotation($crate::na::UnitComplex::new(v[0])))
            }
        }

        #[derive(Debug, Clone, Copy)]
        pub struct PyIsometry(pub $crate::na::Isometry2<$real>);
        impl<'py> $crate::pyo3::FromPyObject<'py> for PyIsometry {
            fn extract_bound(obj: &$crate::pyo3::Bound<'py, $crate::pyo3::PyAny>) -> $crate::pyo3::PyResult<Self> {
                // First try `Isometry2`-like objects exposing `translation`
                // and `rotation` attributes (the canonical `#[pyclass]`).
                let py = obj.py();
                if let (Ok(t_obj), Ok(r_obj)) = (
                    obj.getattr($crate::pyo3::intern!(py, "translation")),
                    obj.getattr($crate::pyo3::intern!(py, "rotation")),
                ) {
                    let t: PyVector = t_obj.extract()?;
                    let r: PyRotation = r_obj.extract()?;
                    return Ok(PyIsometry($crate::na::Isometry2::from_parts(
                        $crate::na::Translation2::from(t.0),
                        r.0,
                    )));
                }
                let (t, r): (PyVector, PyRotation) = obj.extract()?;
                Ok(PyIsometry($crate::na::Isometry2::from_parts(
                    $crate::na::Translation2::from(t.0),
                    r.0,
                )))
            }
        }
    };

    (Real = $real:ty, DIM = 3) => {
        $crate::define_conv_types!(@common $real, 3);

        #[derive(Debug, Clone, Copy)]
        pub struct PyVector(pub $crate::na::Vector3<$real>);
        impl<'py> $crate::pyo3::FromPyObject<'py> for PyVector {
            fn extract_bound(obj: &$crate::pyo3::Bound<'py, $crate::pyo3::PyAny>) -> $crate::pyo3::PyResult<Self> {
                let v: Vec<$real> = $crate::conv::extract_floats(obj, 3)?;
                Ok(PyVector($crate::na::Vector3::new(v[0], v[1], v[2])))
            }
        }

        #[derive(Debug, Clone, Copy)]
        pub struct PyPoint(pub $crate::na::Point3<$real>);
        impl<'py> $crate::pyo3::FromPyObject<'py> for PyPoint {
            fn extract_bound(obj: &$crate::pyo3::Bound<'py, $crate::pyo3::PyAny>) -> $crate::pyo3::PyResult<Self> {
                let v: Vec<$real> = $crate::conv::extract_floats(obj, 3)?;
                Ok(PyPoint($crate::na::Point3::new(v[0], v[1], v[2])))
            }
        }

        /// In 3D, the angular vector is a scaled axis-angle (3 floats).
        #[derive(Debug, Clone, Copy)]
        pub struct PyAngVector(pub $crate::na::Vector3<$real>);
        impl<'py> $crate::pyo3::FromPyObject<'py> for PyAngVector {
            fn extract_bound(obj: &$crate::pyo3::Bound<'py, $crate::pyo3::PyAny>) -> $crate::pyo3::PyResult<Self> {
                let v: Vec<$real> = $crate::conv::extract_floats(obj, 3)?;
                Ok(PyAngVector($crate::na::Vector3::new(v[0], v[1], v[2])))
            }
        }

        /// Quaternion convention: `(x, y, z, w)`.
        #[derive(Debug, Clone, Copy)]
        pub struct PyRotation(pub $crate::na::UnitQuaternion<$real>);
        impl<'py> $crate::pyo3::FromPyObject<'py> for PyRotation {
            fn extract_bound(obj: &$crate::pyo3::Bound<'py, $crate::pyo3::PyAny>) -> $crate::pyo3::PyResult<Self> {
                let v: Vec<$real> = $crate::conv::extract_floats(obj, 4)?;
                let q = $crate::na::Quaternion::new(v[3], v[0], v[1], v[2]);
                Ok(PyRotation($crate::na::UnitQuaternion::from_quaternion(q)))
            }
        }

        #[derive(Debug, Clone, Copy)]
        pub struct PyIsometry(pub $crate::na::Isometry3<$real>);
        impl<'py> $crate::pyo3::FromPyObject<'py> for PyIsometry {
            fn extract_bound(obj: &$crate::pyo3::Bound<'py, $crate::pyo3::PyAny>) -> $crate::pyo3::PyResult<Self> {
                let py = obj.py();
                if let (Ok(t_obj), Ok(r_obj)) = (
                    obj.getattr($crate::pyo3::intern!(py, "translation")),
                    obj.getattr($crate::pyo3::intern!(py, "rotation")),
                ) {
                    let t: PyVector = t_obj.extract()?;
                    let r: PyRotation = r_obj.extract()?;
                    return Ok(PyIsometry($crate::na::Isometry3::from_parts(
                        $crate::na::Translation3::from(t.0),
                        r.0,
                    )));
                }
                let (t, r): (PyVector, PyRotation) = obj.extract()?;
                Ok(PyIsometry($crate::na::Isometry3::from_parts(
                    $crate::na::Translation3::from(t.0),
                    r.0,
                )))
            }
        }
    };

    (@common $real:ty, $dim:literal) => {
        /// Scalar type selected for this cdylib.
        pub type Real = $real;
        /// Spatial dimension selected for this cdylib.
        pub const DIM: usize = $dim;
    };
}
