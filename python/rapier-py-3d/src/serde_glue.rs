//! pickle / snapshot glue for the concrete 3D / `f32` binding.
//!
//! This module adds extra `#[pymethods]` blocks (enabled by pyo3's
//! `multiple-pymethods` feature) to attach `to_bytes`, `from_bytes`, and
//! `__reduce__` to the relevant pyclasses, plus `snapshot`, `restore`,
//! `snapshot_json`, and `restore_json` to `PhysicsWorld`.
//!
//! The implementations rely on rapier being compiled with `serde-serialize`,
//! which the `rapier-py-3d` cdylib enables in its Cargo.toml.
//!
//! This used to be generated per-cdylib by a `define_serde_types!(DIM)` macro;
//! with only the 3D/`f32` flavor remaining, the glue is written out directly,
//! with no macros.

use crate::*;
use rapier3d as rapier;

use crate::pyo3::prelude::*;
#[allow(unused_imports)]
use crate::pyo3::types::PyBytes;

// ============================================================
// Handle types — `__reduce__ = (from_raw_parts, (idx, gen))`.
// ============================================================
#[pymethods]
impl RigidBodyHandle {
    /// Pickle support: `(type(self), (index, generation))`.
    fn __reduce__(slf: PyRef<'_, Self>) -> PyResult<(Py<PyAny>, (u32, u32))> {
        let py = slf.py();
        let (i, g) = slf.0.into_raw_parts();
        let typ: Py<PyAny> = py
            .get_type_bound::<RigidBodyHandle>()
            .getattr("from_raw_parts")?
            .unbind();
        Ok((typ, (i, g)))
    }
}

#[pymethods]
impl ColliderHandle {
    /// Pickle support: `(type(self), (index, generation))`.
    fn __reduce__(slf: PyRef<'_, Self>) -> PyResult<(Py<PyAny>, (u32, u32))> {
        let py = slf.py();
        let (i, g) = slf.0.into_raw_parts();
        let typ: Py<PyAny> = py
            .get_type_bound::<ColliderHandle>()
            .getattr("from_raw_parts")?
            .unbind();
        Ok((typ, (i, g)))
    }
}

#[pymethods]
impl ImpulseJointHandle {
    /// Pickle support: `(type(self), (index, generation))`.
    fn __reduce__(slf: PyRef<'_, Self>) -> PyResult<(Py<PyAny>, (u32, u32))> {
        let py = slf.py();
        let (i, g) = slf.0.into_raw_parts();
        let typ: Py<PyAny> = py
            .get_type_bound::<ImpulseJointHandle>()
            .getattr("from_raw_parts")?
            .unbind();
        Ok((typ, (i, g)))
    }
}

#[pymethods]
impl MultibodyJointHandle {
    /// Pickle support: `(type(self), (index, generation))`.
    fn __reduce__(slf: PyRef<'_, Self>) -> PyResult<(Py<PyAny>, (u32, u32))> {
        let py = slf.py();
        let (i, g) = slf.0.into_raw_parts();
        let typ: Py<PyAny> = py
            .get_type_bound::<MultibodyJointHandle>()
            .getattr("from_raw_parts")?
            .unbind();
        Ok((typ, (i, g)))
    }
}

// MultibodyIndex / MultibodyLinkId have a slightly different shape
// (no `from_raw_parts` constructor on the Python side). Provide them.
#[pymethods]
impl MultibodyIndex {
    #[staticmethod]
    fn from_raw_parts(index: u32, generation: u32) -> Self {
        Self(rapier::dynamics::MultibodyIndex(
            rapier::data::arena::Index::from_raw_parts(index, generation),
        ))
    }
    fn into_raw_parts(&self) -> (u32, u32) {
        self.0.0.into_raw_parts()
    }
    fn __reduce__(slf: PyRef<'_, Self>) -> PyResult<(Py<PyAny>, (u32, u32))> {
        let py = slf.py();
        let (i, g) = slf.0.0.into_raw_parts();
        let ctor: Py<PyAny> = py
            .get_type_bound::<MultibodyIndex>()
            .getattr("from_raw_parts")?
            .unbind();
        Ok((ctor, (i, g)))
    }
}

// `MultibodyLinkId` has a private `graph_id` field; we round-trip
// through bincode (it's serde-derived upstream).
#[pymethods]
impl MultibodyLinkId {
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let payload = crate::bincode::serialize(&self.0).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let body = crate::serde_io::unwrap_bincode(blob.as_bytes())?;
        let inner: rapier::dynamics::MultibodyLinkId =
            crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok(Self(inner))
    }
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<MultibodyLinkId>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// ============================================================
// RigidBody / RigidBodyBuilder / Collider / ColliderBuilder
// ============================================================

// RigidBody is a thin view (owned standalone, or a handle into a set),
// so it has no single `body` field for the generic serde macro to read.
// Serialize by reading the underlying body out; deserialize as a fresh
// owned body.
#[pymethods]
impl RigidBody {
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = self.to_owned_body();
        let payload = crate::bincode::serialize(&inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner: rapier::dynamics::RigidBody =
            crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok(RigidBody::new_owned(inner))
    }

    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<RigidBody>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// RigidBodyBuilder is NOT `Serialize`/`Deserialize` upstream. To make
// it picklable, we round-trip through its built form (a RigidBody),
// which is serde-derived. This preserves every field that influences
// the spawned body but discards the "builder vs body" distinction —
// round-tripped builders behave identically when `.build()` is called.
#[pymethods]
impl RigidBodyBuilder {
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let body = self.builder.clone().build();
        let payload = crate::bincode::serialize(&body).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body_buf = crate::serde_io::unwrap_bincode(buf)?;
        let body: rapier::dynamics::RigidBody =
            crate::bincode::deserialize(body_buf).map_err(crate::serde_io::bincode_err)?;
        // RigidBodyBuilder has no `.from_body` constructor — rebuild
        // a fresh builder of the correct type and copy in fields
        // we can read back. (`can_sleep` is not retrievable, and the
        // builder's sleeping/enabled bits aren't observable on a built
        // body either, so we approximate.)
        let mut b = rapier::dynamics::RigidBodyBuilder::new(body.body_type());
        b = b
            .pose(*body.position())
            .linvel(body.linvel())
            .linear_damping(body.linear_damping())
            .angular_damping(body.angular_damping())
            .sleeping(body.is_sleeping())
            .gravity_scale(body.gravity_scale())
            .ccd_enabled(body.is_ccd_enabled())
            .soft_ccd_prediction(body.soft_ccd_prediction())
            .dominance_group(body.dominance_group())
            .additional_solver_iterations(body.additional_solver_iterations())
            .user_data(body.user_data)
            .enabled(body.is_enabled())
            .locked_axes(body.locked_axes());
        b = b.angvel(body.angvel());
        Ok(Self { builder: b })
    }

    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<RigidBodyBuilder>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// Collider is a thin view (owned, or a handle into a set); serialize by
// reading the underlying collider out, deserialize as a fresh owned one.
#[pymethods]
impl Collider {
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = self.to_owned_collider();
        let payload = crate::bincode::serialize(&inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner: rapier::geometry::Collider =
            crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok(Collider::new_owned(inner))
    }

    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<Collider>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

impl ColliderBuilder {
    #[inline]
    pub(crate) fn _from_inner(builder: rapier::geometry::ColliderBuilder) -> Self {
        Self { builder }
    }
}

#[pymethods]
impl ColliderBuilder {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.builder;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((ColliderBuilder::_from_inner)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<ColliderBuilder>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// ============================================================
// Sets (newtype `Self(inner)`).
// ============================================================
#[pymethods]
impl RigidBodySet {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((RigidBodySet)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<RigidBodySet>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl ColliderSet {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((ColliderSet)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<ColliderSet>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl ImpulseJointSet {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((ImpulseJointSet)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<ImpulseJointSet>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl MultibodyJointSet {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((MultibodyJointSet)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<MultibodyJointSet>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// ============================================================
// Parameters / scratch / phases
// ============================================================
#[pymethods]
impl IntegrationParameters {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((IntegrationParameters)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<IntegrationParameters>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl IslandManager {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((IslandManager)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<IslandManager>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl CCDSolver {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((CCDSolver)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<CCDSolver>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl BroadPhaseBvh {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((BroadPhaseBvh)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<BroadPhaseBvh>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl NarrowPhase {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((NarrowPhase)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<NarrowPhase>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// ============================================================
// Mass / interaction / shape types
// ============================================================
#[pymethods]
impl MassProperties {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((MassProperties)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<MassProperties>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl InteractionGroups {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((InteractionGroups)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<InteractionGroups>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl SharedShape {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((SharedShape)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<SharedShape>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// ============================================================
// Joint types (concrete + builders + generic).
// ============================================================
#[pymethods]
impl FixedJoint {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((FixedJoint)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<FixedJoint>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl FixedJointBuilder {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((FixedJointBuilder)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<FixedJointBuilder>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl RevoluteJoint {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((RevoluteJoint)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<RevoluteJoint>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl RevoluteJointBuilder {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((RevoluteJointBuilder)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<RevoluteJointBuilder>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl PrismaticJoint {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((PrismaticJoint)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<PrismaticJoint>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl PrismaticJointBuilder {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((PrismaticJointBuilder)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<PrismaticJointBuilder>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl RopeJoint {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((RopeJoint)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<RopeJoint>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl RopeJointBuilder {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((RopeJointBuilder)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<RopeJointBuilder>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl SpringJoint {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((SpringJoint)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<SpringJoint>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl SpringJointBuilder {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((SpringJointBuilder)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<SpringJointBuilder>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// GenericJoint is a thin view (owned, or a view into an in-set joint's
// data); serialize the underlying value, deserialize as owned.
#[pymethods]
impl GenericJoint {
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = self.to_owned_generic();
        let payload = crate::bincode::serialize(&inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let body = crate::serde_io::unwrap_bincode(blob.as_bytes())?;
        let inner: rapier::dynamics::GenericJoint =
            crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok(GenericJoint::new_owned(inner))
    }
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<GenericJoint>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl GenericJointBuilder {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((GenericJointBuilder)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<GenericJointBuilder>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// Dim-specific joints (SphericalJoint in 3D).
#[pymethods]
impl SphericalJoint {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((SphericalJoint)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<SphericalJoint>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl SphericalJointBuilder {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((SphericalJointBuilder)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<SphericalJointBuilder>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// ============================================================
// Math types — Vec, Point, Rotation, Isometry.
// ============================================================
#[pymethods]
impl Vec3 {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((Vec3)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py.get_type_bound::<Vec3>().getattr("from_bytes")?.unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl Point3 {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((Point3)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<Point3>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl Rotation3 {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((Rotation3)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<Rotation3>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl Isometry3 {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((Isometry3)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<Isometry3>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// ============================================================
// Controllers — try to serialize. Wrap each in `__reduce__` only if
// serde-derivable on the inner. KinematicCharacterController, PdController,
// PidController, DynamicRayCastVehicleController are all
// `Serialize + Deserialize` upstream.
// ============================================================
#[pymethods]
impl KinematicCharacterController {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((KinematicCharacterController)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<KinematicCharacterController>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl PdController {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((PdController)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<PdController>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl PidController {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((PidController)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<PidController>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// `DynamicRayCastVehicleController` + `WheelTuning` are 3D-only upstream.
#[pymethods]
impl DynamicRayCastVehicleController {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((DynamicRayCastVehicleController)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<DynamicRayCastVehicleController>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

#[pymethods]
impl WheelTuning {
    /// Serialize this value to a bincode'd `bytes`. The blob is prefixed
    /// with the 4-byte magic `RPYS` and a 4-byte little-endian version.
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let inner = &self.0;
        let payload = crate::bincode::serialize(inner).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok((WheelTuning)(inner))
    }

    /// Pickle support: `(cls.from_bytes, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.to_bytes(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<WheelTuning>()
            .getattr("from_bytes")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}

// ============================================================
// Pipelines + collectors — REFUSE pickle.
// ============================================================
#[pymethods]
impl PhysicsPipeline {
    fn __reduce__(&self) -> PyResult<()> {
        Err(crate::errors::SerializationError::new_err(
            "PhysicsPipeline is not serializable; it's stateless except for \
             per-step counters. Re-create a PhysicsPipeline after restoring \
             the world.",
        ))
    }
}

#[pymethods]
impl CollisionPipeline {
    fn __reduce__(&self) -> PyResult<()> {
        Err(crate::errors::SerializationError::new_err(
            "CollisionPipeline is not serializable; re-create one after restore.",
        ))
    }
}

#[pymethods]
impl ChannelEventCollector {
    fn __reduce__(&self) -> PyResult<()> {
        Err(crate::errors::SerializationError::new_err(
            "ChannelEventCollector is not serializable; drain events first.",
        ))
    }
}

// ============================================================
// PhysicsWorld — full snapshot / restore + pickle.
//
// Serialized payload is a tuple of the inner rapier values:
//   (bodies, colliders, impulse_joints, multibody_joints,
//    broad_phase, narrow_phase, islands, ccd_solver,
//    integration_parameters, gravity)
//
// `event_handler`, `physics_hooks`, `event_error_policy` and
// `auto_update_query` are NOT serialized — restored worlds are
// returned with `event_handler = None`, `physics_hooks = None`,
// `event_error_policy = "defer"`, `auto_update_query = false`.
// ============================================================

// Borrow-aware shared serialization payload.
// Owned form used for deserialization; borrowed form used for
// serialization to avoid cloning huge component sets.
type _PhysicsWorldOwned = (
    rapier::dynamics::RigidBodySet,
    rapier::geometry::ColliderSet,
    rapier::dynamics::ImpulseJointSet,
    rapier::dynamics::MultibodyJointSet,
    rapier::geometry::BroadPhaseBvh,
    rapier::geometry::NarrowPhase,
    rapier::dynamics::IslandManager,
    rapier::dynamics::CCDSolver,
    rapier::dynamics::IntegrationParameters,
    crate::na::SVector<Real, 3>,
);

#[pymethods]
impl PhysicsWorld {
    /// Take a snapshot of the world's physical state.
    ///
    /// Returns a `bytes` blob containing every internal collection
    /// (rigid bodies, colliders, joints, broad phase, narrow phase,
    /// island manager, CCD solver, integration parameters, gravity).
    ///
    /// User-supplied `event_handler` / `physics_hooks` and the
    /// `event_error_policy` are **NOT** serialized — re-attach them
    /// after `restore`.
    fn snapshot<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let bodies = self.bodies.borrow(py);
        let colliders = self.colliders.borrow(py);
        let ij = self.impulse_joints.borrow(py);
        let mj = self.multibody_joints.borrow(py);
        let bp = self.broad_phase.borrow(py);
        let np = self.narrow_phase.borrow(py);
        let islands = self.islands.borrow(py);
        let ccd = self.ccd_solver.borrow(py);
        let ip = self.integration_parameters.borrow(py);
        let tup = (
            &bodies.0,
            &colliders.0,
            &ij.0,
            &mj.0,
            &bp.0,
            &np.0,
            &islands.0,
            &ccd.0,
            &ip.0,
            &self.gravity.0,
        );
        let payload = crate::bincode::serialize(&tup).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode(payload),
        ))
    }

    /// Re-create a `PhysicsWorld` from a snapshot.
    #[staticmethod]
    fn restore(py: Python<'_>, blob: &Bound<'_, PyBytes>) -> PyResult<Py<PhysicsWorld>> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode(buf)?;
        let owned: _PhysicsWorldOwned =
            crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        let (bs, cs, ijs, mjs, bps, nps, isl, ccd, ip, g) = owned;
        let bodies = Py::new(py, RigidBodySet(bs))?;
        let colliders = Py::new(py, ColliderSet(cs))?;
        let impulse_joints = Py::new(py, ImpulseJointSet(ijs))?;
        let multibody_joints = Py::new(py, MultibodyJointSet(mjs))?;
        let broad_phase = Py::new(py, BroadPhaseBvh(bps))?;
        let narrow_phase = Py::new(py, NarrowPhase(nps))?;
        let islands = Py::new(py, IslandManager(isl))?;
        let ccd_solver = Py::new(py, CCDSolver(ccd))?;
        let integration_parameters = Py::new(py, IntegrationParameters(ip))?;
        let physics_pipeline = Py::new(
            py,
            PhysicsPipeline(rapier::pipeline::PhysicsPipeline::new()),
        )?;
        let query_pipeline = Py::new(
            py,
            QueryPipeline {
                broad_phase: broad_phase.clone_ref(py),
                narrow_phase: narrow_phase.clone_ref(py),
                bodies: bodies.clone_ref(py),
                colliders: colliders.clone_ref(py),
            },
        )?;
        Py::new(
            py,
            PhysicsWorld {
                bodies,
                colliders,
                impulse_joints,
                multibody_joints,
                broad_phase,
                narrow_phase,
                islands,
                ccd_solver,
                integration_parameters,
                physics_pipeline,
                query_pipeline,
                gravity: Vec3(g),
                event_handler: None,
                physics_hooks: None,
                auto_update_query: false,
                event_error_policy: "defer".to_string(),
            },
        )
    }

    /// JSON debug snapshot — slower & much larger but human-readable.
    ///
    /// Returns a JSON string with a self-describing envelope:
    ///   `{"_magic": "RPYS", "_version": N, "payload": <inner>}`.
    fn snapshot_json(&self, py: Python<'_>) -> PyResult<String> {
        let bodies = self.bodies.borrow(py);
        let colliders = self.colliders.borrow(py);
        let ij = self.impulse_joints.borrow(py);
        let mj = self.multibody_joints.borrow(py);
        let bp = self.broad_phase.borrow(py);
        let np = self.narrow_phase.borrow(py);
        let islands = self.islands.borrow(py);
        let ccd = self.ccd_solver.borrow(py);
        let ip = self.integration_parameters.borrow(py);
        let tup = (
            &bodies.0,
            &colliders.0,
            &ij.0,
            &mj.0,
            &bp.0,
            &np.0,
            &islands.0,
            &ccd.0,
            &ip.0,
            &self.gravity.0,
        );
        let payload = crate::serde_json::to_value(tup).map_err(crate::serde_io::json_err)?;
        let env = crate::serde_io::wrap_json(payload);
        crate::serde_json::to_string(&env).map_err(crate::serde_io::json_err)
    }

    /// Restore a world from a JSON snapshot.
    #[staticmethod]
    fn restore_json(py: Python<'_>, s: &str) -> PyResult<Py<PhysicsWorld>> {
        let env: crate::serde_json::Value =
            crate::serde_json::from_str(s).map_err(crate::serde_io::json_err)?;
        let payload = crate::serde_io::unwrap_json(env)?;
        let owned: _PhysicsWorldOwned =
            crate::serde_json::from_value(payload).map_err(crate::serde_io::json_err)?;
        let (bs, cs, ijs, mjs, bps, nps, isl, ccd, ip, g) = owned;
        let bodies = Py::new(py, RigidBodySet(bs))?;
        let colliders = Py::new(py, ColliderSet(cs))?;
        let impulse_joints = Py::new(py, ImpulseJointSet(ijs))?;
        let multibody_joints = Py::new(py, MultibodyJointSet(mjs))?;
        let broad_phase = Py::new(py, BroadPhaseBvh(bps))?;
        let narrow_phase = Py::new(py, NarrowPhase(nps))?;
        let islands = Py::new(py, IslandManager(isl))?;
        let ccd_solver = Py::new(py, CCDSolver(ccd))?;
        let integration_parameters = Py::new(py, IntegrationParameters(ip))?;
        let physics_pipeline = Py::new(
            py,
            PhysicsPipeline(rapier::pipeline::PhysicsPipeline::new()),
        )?;
        let query_pipeline = Py::new(
            py,
            QueryPipeline {
                broad_phase: broad_phase.clone_ref(py),
                narrow_phase: narrow_phase.clone_ref(py),
                bodies: bodies.clone_ref(py),
                colliders: colliders.clone_ref(py),
            },
        )?;
        Py::new(
            py,
            PhysicsWorld {
                bodies,
                colliders,
                impulse_joints,
                multibody_joints,
                broad_phase,
                narrow_phase,
                islands,
                ccd_solver,
                integration_parameters,
                physics_pipeline,
                query_pipeline,
                gravity: Vec3(g),
                event_handler: None,
                physics_hooks: None,
                auto_update_query: false,
                event_error_policy: "defer".to_string(),
            },
        )
    }

    /// Pickle support. Returns `(PhysicsWorld.restore, (snapshot_bytes,))`.
    fn __reduce__<'py>(&self, py: Python<'py>) -> PyResult<(Py<PyAny>, (Bound<'py, PyBytes>,))> {
        let blob = self.snapshot(py)?;
        let ctor: Py<PyAny> = py
            .get_type_bound::<PhysicsWorld>()
            .getattr("restore")?
            .unbind();
        Ok((ctor, (blob,)))
    }
}
