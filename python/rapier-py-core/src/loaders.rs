//! Macro emitting the loader `#[pyclass]`-es per 3D cdylib.
//!
//! Loaders (URDF / MJCF / Mesh).
//!
//! Only 3D cdylibs invoke `define_loaders_types!(DIM = 3)` (the upstream
//! `rapier3d-urdf` / `rapier3d-meshloader` / `rapier3d-mjcf` crates are
//! 3D-only and only exist in the `rapier3d` (f32) variant).
//!
//! Like the other modules, this macro is invoked once per cdylib *after*
//! `define_conv_types!`, `define_math_types!`, `define_geometry_types!`,
//! `define_joints_types!` and `define_dynamics_types!`.
//!
//! `define_loaders_types!` produces `register_loaders(py, m)` for the
//! `#[pymodule]` entry point.
//!
//! ## Layout
//!
//! The mesh loader lives here. The URDF and MJCF loaders are large enough to
//! warrant their own files, mirroring the upstream `rapier3d-urdf` /
//! `rapier3d-mjcf` crate boundaries:
//!
//! - [`crate::urdf`] — emits `__define_loaders_urdf!` + `register_loaders_urdf`.
//! - [`crate::mjcf`] — emits `__define_loaders_mjcf!` + `register_loaders_mjcf`.
//!
//! `__define_loaders_3d!` calls those two macros so all three loaders expand
//! into the same cdylib namespace, and `register_loaders` chains their
//! per-loader registration helpers.
//!
//! ## Notes
//!
//! - `rapier3d-urdf` / `rapier3d-meshloader` / `rapier3d-mjcf` have no
//!   `-f64` upstream variant. The 3D-f64 cdylib therefore does *not* expose
//!   loader symbols.

/// Materialize the loader `#[pyclass]` types for a given 3D cdylib.
#[macro_export]
macro_rules! define_loaders_types {
    (DIM = 3) => {
        $crate::__define_loaders_3d!();
        $crate::__define_loaders_register!();
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __define_loaders_3d {
    () => {
        // ==============================================================
        // Mesh loader
        // ==============================================================

        /// One shape extracted by :func:`load_from_path` or
        /// :func:`load_from_raw_mesh`.
        ///
        /// Bundles the converted :class:`SharedShape`, its world-relative
        /// :class:`Isometry3` pose (taken from any ``visual.origin`` /
        /// ``collision.origin`` element in the source file), and the raw
        /// triangle mesh ``(vertices, indices)`` used to build the shape.
        ///
        /// :ivar shape: Converted collision shape.
        /// :ivar pose: World-relative pose of the shape.
        /// :ivar vertices: ``(N, 3)`` float32 NumPy array of mesh vertices.
        /// :ivar indices: ``(M, 3)`` uint32 NumPy array of triangle indices.
        #[pyclass(name = "LoadedShape", module = "rapier")]
        pub struct LoadedShape {
            #[pyo3(get)]
            pub shape: SharedShape,
            #[pyo3(get)]
            pub pose: Isometry3,
            vertices: Vec<[f32; 3]>,
            indices: Vec<[u32; 3]>,
        }

        #[pymethods]
        impl LoadedShape {
            /// ``(N, 3)`` float32 NumPy array of raw mesh vertex positions.
            #[getter]
            fn vertices<'py>(
                &self,
                py: $crate::pyo3::Python<'py>,
            ) -> $crate::pyo3::Bound<'py, $crate::numpy::PyArray2<f32>> {
                use $crate::numpy::PyArray2;
                let n = self.vertices.len();
                let mut flat: Vec<f32> = Vec::with_capacity(n * 3);
                for v in &self.vertices {
                    flat.extend_from_slice(v);
                }
                PyArray2::from_vec2_bound(
                    py,
                    &self.vertices.iter().map(|v| v.to_vec()).collect::<Vec<_>>(),
                )
                .unwrap_or_else(|_| PyArray2::<f32>::zeros_bound(py, [0, 3], false))
            }

            /// ``(M, 3)`` uint32 NumPy array of triangle face indices.
            #[getter]
            fn indices<'py>(
                &self,
                py: $crate::pyo3::Python<'py>,
            ) -> $crate::pyo3::Bound<'py, $crate::numpy::PyArray2<u32>> {
                use $crate::numpy::PyArray2;
                PyArray2::from_vec2_bound(
                    py,
                    &self.indices.iter().map(|v| v.to_vec()).collect::<Vec<_>>(),
                )
                .unwrap_or_else(|_| PyArray2::<u32>::zeros_bound(py, [0, 3], false))
            }

            /// Return the ``LoadedShape(...)`` repr.
            fn __repr__(&self) -> String {
                format!(
                    "LoadedShape(shape={:?}, n_vertices={}, n_faces={})",
                    self.shape.0.shape_type(),
                    self.vertices.len(),
                    self.indices.len(),
                )
            }
        }

        fn _loaded_shape_from_meshloader(loaded: rapier3d_meshloader::LoadedShape) -> LoadedShape {
            let iso: $crate::na::Isometry<Real, _, 3> = loaded.pose.into();
            LoadedShape {
                shape: SharedShape(loaded.shape),
                pose: Isometry3(iso),
                vertices: loaded.raw_mesh.vertices.clone(),
                indices: loaded.raw_mesh.faces.clone(),
            }
        }

        /// Load shapes from a mesh file on disk.
        ///
        /// The file is parsed (formats supported by ``rapier3d-meshloader``
        /// — typically OBJ, GLTF, STL) into one or more groups; each group
        /// is independently converted into a shape using ``converter``.
        ///
        /// :param path: Path to the source mesh file.
        /// :param converter: :class:`MeshConverter` to use (defaults to
        ///     :attr:`MeshConverter.TriMesh`).
        /// :param scale: Uniform scale applied during conversion.
        /// :returns: A list with one entry per source group, each either a
        ///     :class:`LoadedShape` (success) or a ``MeshConversionError``
        ///     instance (failure).
        /// :raises MeshLoaderError: if the file itself failed to load.
        #[pyfunction]
        #[pyo3(name = "load_from_path")]
        #[pyo3(signature = (path, converter=None, scale=1.0))]
        fn loaders_mesh_load_from_path(
            py: $crate::pyo3::Python<'_>,
            path: &str,
            converter: Option<MeshConverter>,
            scale: Real,
        ) -> $crate::pyo3::PyResult<Vec<$crate::pyo3::PyObject>> {
            use $crate::pyo3::IntoPy;
            let converter =
                converter.unwrap_or(MeshConverter(rapier::geometry::MeshConverter::TriMesh));
            let scale_v = rapier::math::Vector::new(scale, scale, scale);
            let results = rapier3d_meshloader::load_from_path(path, &converter.0, scale_v)
                .map_err(|e| $crate::errors::MeshLoaderError::new_err(format!("{e}")))?;
            let mut out: Vec<$crate::pyo3::PyObject> = Vec::with_capacity(results.len());
            for r in results {
                match r {
                    Ok(loaded) => {
                        let py_loaded = _loaded_shape_from_meshloader(loaded);
                        out.push($crate::pyo3::Py::new(py, py_loaded)?.into_py(py));
                    }
                    Err(e) => {
                        let exc = $crate::errors::MeshConversionError::new_err(format!("{e}"));
                        out.push(exc.value_bound(py).clone().into());
                    }
                }
            }
            Ok(out)
        }

        /// Load a single shape from an in-memory triangle mesh.
        ///
        /// Identical to :func:`load_from_path` except the mesh data comes
        /// from NumPy arrays / sequences directly. No file IO.
        ///
        /// :param vertices: Sequence of 3D vertex positions.
        /// :param indices: Sequence of triangle indices ``(i, j, k)``.
        /// :param converter: :class:`MeshConverter` (defaults to TriMesh).
        /// :param scale: Uniform scale applied during conversion.
        /// :returns: A :class:`LoadedShape`.
        /// :raises MeshConversionError: if the conversion failed.
        #[pyfunction]
        #[pyo3(name = "load_from_raw_mesh")]
        #[pyo3(signature = (vertices, indices, converter=None, scale=1.0))]
        fn loaders_mesh_load_from_raw_mesh(
            _py: $crate::pyo3::Python<'_>,
            vertices: &$crate::pyo3::Bound<'_, $crate::pyo3::PyAny>,
            indices: &$crate::pyo3::Bound<'_, $crate::pyo3::PyAny>,
            converter: Option<MeshConverter>,
            scale: Real,
        ) -> $crate::pyo3::PyResult<LoadedShape> {
            let verts = extract_verts_for_dim(vertices)?;
            let idx = $crate::geometry::extract_indices(indices)?;
            let converter =
                converter.unwrap_or(MeshConverter(rapier::geometry::MeshConverter::TriMesh));
            let mut mesh = mesh_loader::Mesh::default();
            mesh.vertices = verts
                .iter()
                .map(|v| [v.x as f32, v.y as f32, v.z as f32])
                .collect();
            mesh.faces = idx.clone();
            let scale_v = rapier::math::Vector::new(scale, scale, scale);
            let (shape, pose) =
                rapier3d_meshloader::load_from_raw_mesh(&mesh, &converter.0, scale_v)
                    .map_err(|e| $crate::errors::MeshConversionError::new_err(format!("{e}")))?;
            let iso: $crate::na::Isometry<Real, _, 3> = pose.into();
            Ok(LoadedShape {
                shape: SharedShape(shape),
                pose: Isometry3(iso),
                vertices: mesh.vertices,
                indices: mesh.faces,
            })
        }

        // The URDF and MJCF loaders live in their own files; expand them into
        // the same cdylib namespace right after the mesh loader.
        $crate::__define_loaders_urdf!();
        $crate::__define_loaders_mjcf!();
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __define_loaders_register {
    () => {
        pub fn register_loaders(
            _py: $crate::pyo3::Python<'_>,
            m: &$crate::pyo3::Bound<'_, $crate::pyo3::types::PyModule>,
        ) -> $crate::pyo3::PyResult<()> {
            use $crate::pyo3::prelude::*;
            // Mesh loader
            m.add_class::<LoadedShape>()?;
            m.add_function($crate::pyo3::wrap_pyfunction!(
                loaders_mesh_load_from_path,
                m
            )?)?;
            m.add_function($crate::pyo3::wrap_pyfunction!(
                loaders_mesh_load_from_raw_mesh,
                m
            )?)?;
            // URDF + MJCF loaders (defined in `crate::urdf` / `crate::mjcf`).
            register_loaders_urdf(m)?;
            register_loaders_mjcf(m)?;
            Ok(())
        }
    };
}
