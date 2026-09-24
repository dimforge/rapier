//! `BuildFeatures` / `build_features()`: how the loaded extension was built.

use pyo3::prelude::*;

/// How the loaded ``rapier3d`` extension was built (see :func:`build_features`).
#[pyclass(name = "BuildFeatures", module = "rapier", frozen)]
#[derive(Clone)]
pub struct BuildFeatures {
    /// The Cargo profile of the extension: ``"release"`` for an optimized build, ``"debug"``
    /// otherwise (e.g. ``maturin develop`` without ``--release``), which is much slower.
    #[pyo3(get)]
    pub profile: &'static str,
    /// ``True`` if built with the ``determinism`` feature (rapier's ``enhanced-determinism``):
    /// the simulation and the functions of :mod:`rapier3d.math` give bit-identical results on
    /// every platform.
    #[pyo3(get)]
    pub enhanced_determinism: bool,
    /// ``True`` if the parallel stages of a step can run on several worker threads (see
    /// :meth:`PhysicsWorld.set_num_threads`). Always ``True`` for the ``rapier3d`` wheels.
    #[pyo3(get)]
    pub parallel: bool,
    /// ``True`` if built with rapier's ``profiler`` feature: the timings of the enabled
    /// :class:`Counters` are measured (they are zero otherwise).
    #[pyo3(get)]
    pub profiler: bool,
}

#[pymethods]
impl BuildFeatures {
    fn __repr__(&self) -> String {
        let b = |v: bool| if v { "True" } else { "False" };
        format!(
            "BuildFeatures(profile='{}', enhanced_determinism={}, parallel={}, profiler={})",
            self.profile,
            b(self.enhanced_determinism),
            b(self.parallel),
            b(self.profiler)
        )
    }
}

/// How the loaded ``rapier3d`` extension was built: its Cargo profile and the optional engine
/// features compiled in.
///
/// ::
///
///     if rapier3d.build_features().profile != "release":
///         print("Warning: the rapier3d bindings are built without optimizations.")
#[pyfunction]
pub fn build_features() -> BuildFeatures {
    BuildFeatures {
        profile: env!("RAPIER_PY_CARGO_PROFILE"),
        enhanced_determinism: cfg!(feature = "determinism"),
        parallel: true,
        profiler: cfg!(feature = "profiler"),
    }
}

pub fn register_build_info(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<BuildFeatures>()?;
    m.add_function(wrap_pyfunction!(build_features, m)?)?;
    Ok(())
}
