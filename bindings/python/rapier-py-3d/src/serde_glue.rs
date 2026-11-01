//! pickle / snapshot glue for the concrete 3D / `f32` binding.
//!
//! This module adds extra `#[pymethods]` blocks (enabled by pyo3's
//! `multiple-pymethods` feature) to attach `to_bytes`, `from_bytes`, and
//! `__reduce__` to the relevant pyclasses, plus `snapshot`, `restore`,
//! `snapshot_json`, `restore_json` and the snapshot-delta methods to `PhysicsWorld`.
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
        let inner = self.to_owned_body()?;
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
        let inner = self.to_owned_collider()?;
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
            &crate::serde_io::wrap_bincode_versioned(
                crate::serde_io::versions::JOINT_AND_COLLIDER_SETS,
                payload,
            ),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode_versioned(
            buf,
            crate::serde_io::versions::JOINT_AND_COLLIDER_SETS,
        )?;
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
            &crate::serde_io::wrap_bincode_versioned(
                crate::serde_io::versions::JOINT_AND_COLLIDER_SETS,
                payload,
            ),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode_versioned(
            buf,
            crate::serde_io::versions::JOINT_AND_COLLIDER_SETS,
        )?;
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
            &crate::serde_io::wrap_bincode_versioned(
                crate::serde_io::versions::JOINT_AND_COLLIDER_SETS,
                payload,
            ),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body = crate::serde_io::unwrap_bincode_versioned(
            buf,
            crate::serde_io::versions::JOINT_AND_COLLIDER_SETS,
        )?;
        let inner = crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        Ok(MultibodyJointSet::wrap(inner))
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
            &crate::serde_io::wrap_bincode_versioned(crate::serde_io::versions::PHASES, payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body =
            crate::serde_io::unwrap_bincode_versioned(buf, crate::serde_io::versions::PHASES)?;
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
            &crate::serde_io::wrap_bincode_versioned(crate::serde_io::versions::PHASES, payload),
        ))
    }

    /// Deserialize from a `bytes` blob produced by `to_bytes` (or pickle).
    #[staticmethod]
    fn from_bytes(blob: &Bound<'_, PyBytes>) -> PyResult<Self> {
        let buf = blob.as_bytes();
        let body =
            crate::serde_io::unwrap_bincode_versioned(buf, crate::serde_io::versions::PHASES)?;
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
        let inner = self.to_owned_generic()?;
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
    rapier::dynamics::SoftBodySet,
);

/// Rebuilds the derived state deserialization leaves empty, so a restored world is readable
/// before its first step.
fn rebuild_derived_state(parts: &mut crate::pipeline::PhysicsWorldParts) {
    let g = parts.gravity;
    let mut gravity = rapier::math::Vector::new(g.x, g.y, g.z);
    rapier::pipeline::WorldComponentsMut {
        gravity: &mut gravity,
        integration_parameters: &mut parts.integration_parameters,
        islands: &mut parts.islands,
        broad_phase: &mut parts.broad_phase,
        narrow_phase: &mut parts.narrow_phase,
        bodies: &mut parts.bodies,
        colliders: &mut parts.colliders,
        impulse_joints: &mut parts.impulse_joints,
        multibody_joints: &mut parts.multibody_joints,
        soft_bodies: &mut parts.soft_bodies,
    }
    .rebuild_derived_state();
}

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
        let bodies = crate::errors::try_borrow(self.bodies.bind(py))?;
        let colliders = crate::errors::try_borrow(self.colliders.bind(py))?;
        let ij = crate::errors::try_borrow(self.impulse_joints.bind(py))?;
        let mj = crate::errors::try_borrow(self.multibody_joints.bind(py))?;
        let bp = crate::errors::try_borrow(self.broad_phase.bind(py))?;
        let np = crate::errors::try_borrow(self.narrow_phase.bind(py))?;
        let islands = crate::errors::try_borrow(self.islands.bind(py))?;
        let ccd = crate::errors::try_borrow(self.ccd_solver.bind(py))?;
        let ip = crate::errors::try_borrow(self.integration_parameters.bind(py))?;
        let sb = crate::errors::try_borrow(self.soft_bodies.bind(py))?;
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
            &sb.0,
        );
        let payload = crate::bincode::serialize(&tup).map_err(crate::serde_io::bincode_err)?;
        Ok(crate::serde_io::bytes_to_py(
            py,
            &crate::serde_io::wrap_bincode_versioned(crate::serde_io::versions::WORLD, payload),
        ))
    }

    /// Re-create a `PhysicsWorld` from a snapshot.
    #[staticmethod]
    fn restore(py: Python<'_>, blob: &Bound<'_, PyBytes>) -> PyResult<Py<PhysicsWorld>> {
        let buf = blob.as_bytes();
        let body =
            crate::serde_io::unwrap_bincode_versioned(buf, crate::serde_io::versions::WORLD)?;
        let owned: _PhysicsWorldOwned =
            crate::bincode::deserialize(body).map_err(crate::serde_io::bincode_err)?;
        let (bs, cs, ijs, mjs, bps, nps, isl, ccd, ip, g, sbs) = owned;
        let mut parts = crate::pipeline::PhysicsWorldParts {
            bodies: bs,
            colliders: cs,
            impulse_joints: ijs,
            multibody_joints: mjs,
            broad_phase: bps,
            narrow_phase: nps,
            islands: isl,
            ccd_solver: ccd,
            integration_parameters: ip,
            gravity: g,
            soft_bodies: sbs,
        };
        rebuild_derived_state(&mut parts);
        Py::new(py, PhysicsWorld::from_parts(py, parts, false)?)
    }

    /// JSON debug snapshot — slower & much larger but human-readable.
    ///
    /// Returns a JSON string with a self-describing envelope:
    ///   `{"_magic": "RPYS", "_version": N, "payload": <inner>}`.
    fn snapshot_json(&self, py: Python<'_>) -> PyResult<String> {
        let bodies = crate::errors::try_borrow(self.bodies.bind(py))?;
        let colliders = crate::errors::try_borrow(self.colliders.bind(py))?;
        let ij = crate::errors::try_borrow(self.impulse_joints.bind(py))?;
        let mj = crate::errors::try_borrow(self.multibody_joints.bind(py))?;
        let bp = crate::errors::try_borrow(self.broad_phase.bind(py))?;
        let np = crate::errors::try_borrow(self.narrow_phase.bind(py))?;
        let islands = crate::errors::try_borrow(self.islands.bind(py))?;
        let ccd = crate::errors::try_borrow(self.ccd_solver.bind(py))?;
        let ip = crate::errors::try_borrow(self.integration_parameters.bind(py))?;
        let sb = crate::errors::try_borrow(self.soft_bodies.bind(py))?;
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
            &sb.0,
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
        let (bs, cs, ijs, mjs, bps, nps, isl, ccd, ip, g, sbs) = owned;
        let mut parts = crate::pipeline::PhysicsWorldParts {
            bodies: bs,
            colliders: cs,
            impulse_joints: ijs,
            multibody_joints: mjs,
            broad_phase: bps,
            narrow_phase: nps,
            islands: isl,
            ccd_solver: ccd,
            integration_parameters: ip,
            gravity: g,
            soft_bodies: sbs,
        };
        rebuild_derived_state(&mut parts);
        Py::new(py, PhysicsWorld::from_parts(py, parts, false)?)
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

// ============================================================
// PhysicsWorld: snapshot deltas.
//
// Deltas and ticks travel as bincode blobs of rapier's `WorldDelta` / `WorldChangeTick`
// headed by `RPYD` / `RPYT`, produced through `WorldComponentsMut`.
// ============================================================

/// Converts a rapier delta error into a Python `DeltaError` with a `kind` attribute.
fn delta_err(py: Python<'_>, e: rapier::data::DeltaError) -> PyErr {
    use rapier::data::DeltaError as E;
    let kind = match e {
        E::TrackingDisabled => "tracking_disabled",
        E::UnknownBaseline => "unknown_baseline",
        E::BaselineMismatch => "baseline_mismatch",
        E::Malformed => "malformed",
        E::MissingShape => "missing_shape",
    };
    let err = crate::errors::DeltaError::new_err(e.to_string());
    // Setting an attribute on a fresh exception instance cannot fail in practice.
    let _ = err.value_bound(py).setattr("kind", kind);
    err
}

/// Largest delta or tick blob accepted, header included.
///
/// The encoder caps the payload at `MAX_DELTA_BYTES - HEADER_SIZE` and the decoder checks the
/// whole blob against `MAX_DELTA_BYTES`, so every blob this module writes is accepted on decode.
const MAX_DELTA_BYTES: usize = 256 * 1024 * 1024;
/// Largest bincode payload of a delta or tick blob (the blob minus its header).
const MAX_DELTA_PAYLOAD_BYTES: usize = MAX_DELTA_BYTES - crate::serde_io::HEADER_SIZE;

/// Encodes a delta or tick blob of `kind`, with the options and size limit of `delta_from_py`.
///
/// Like `bincode::serialize`, a limited size pass runs first, then the payload is written
/// straight into the Python bytes object, so no intermediate copy of the payload exists.
fn delta_to_py<'py, T: serde::Serialize>(
    py: Python<'py>,
    kind: crate::serde_io::BlobKind,
    value: &T,
) -> PyResult<Bound<'py, PyBytes>> {
    use crate::bincode::Options;
    let options = crate::bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(MAX_DELTA_PAYLOAD_BYTES as u64);
    let payload_len = options.serialized_size(value).map_err(|e| match *e {
        crate::bincode::ErrorKind::SizeLimit => {
            crate::errors::SerializationError::new_err(format!(
                "encoded {} exceeds the 256 MiB limit (header included) accepted on decode",
                kind.name()
            ))
        }
        _ => crate::serde_io::bincode_err(e),
    })? as usize;
    let header = crate::serde_io::blob_header(kind);
    PyBytes::new_bound_with(py, header.len() + payload_len, |buf| {
        let (head, mut body) = buf.split_at_mut(header.len());
        head.copy_from_slice(&header);
        // The size pass already enforced the limit.
        options
            .with_no_limit()
            .serialize_into(&mut body, value)
            .map_err(crate::serde_io::bincode_err)?;
        if !body.is_empty() {
            return Err(crate::errors::SerializationError::new_err(format!(
                "encoded {} is shorter than its computed size",
                kind.name()
            )));
        }
        Ok(())
    })
}

/// The bytes of a delta or tick argument: `bytes`, or a copy of any other buffer-protocol
/// object (`bytearray`, `memoryview`...).
fn blob_bytes<'py>(blob: &Bound<'py, PyAny>) -> PyResult<Bound<'py, PyBytes>> {
    if let Ok(bytes) = blob.downcast::<PyBytes>() {
        return Ok(bytes.clone());
    }
    let view = crate::pyo3::types::PyMemoryView::from_bound(blob)?;
    Ok(view.call_method0("tobytes")?.downcast_into::<PyBytes>()?)
}

/// Decodes a delta or tick blob of `kind`, rejecting oversized blobs and trailing bytes.
fn delta_from_py<T: serde::de::DeserializeOwned>(
    blob: &Bound<'_, PyBytes>,
    kind: crate::serde_io::BlobKind,
) -> PyResult<T> {
    use crate::bincode::Options;
    let bytes = blob.as_bytes();
    if bytes.len() > MAX_DELTA_BYTES {
        return Err(crate::errors::SerializationError::new_err(format!(
            "{} blob exceeds the 256 MiB limit (header included)",
            kind.name()
        )));
    }
    let body = crate::serde_io::unwrap_blob(bytes, kind)?;
    crate::bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(MAX_DELTA_PAYLOAD_BYTES as u64)
        .reject_trailing_bytes()
        .deserialize(body)
        .map_err(crate::serde_io::bincode_err)
}

impl PhysicsWorld {
    fn gravity_vector(&self) -> rapier::math::Vector {
        let g = self.gravity.0;
        rapier::math::Vector::new(g.x, g.y, g.z)
    }

    /// Runs `f` on every component deltas cover, with `gravity` standing in for the world's.
    fn with_components<R>(
        &self,
        py: Python<'_>,
        gravity: &mut rapier::math::Vector,
        f: impl FnOnce(&mut rapier::pipeline::WorldComponentsMut<'_>) -> R,
    ) -> PyResult<R> {
        let mut ip = crate::errors::try_borrow_mut(self.integration_parameters.bind(py))?;
        let mut islands = crate::errors::try_borrow_mut(self.islands.bind(py))?;
        let mut bp = crate::errors::try_borrow_mut(self.broad_phase.bind(py))?;
        let mut np = crate::errors::try_borrow_mut(self.narrow_phase.bind(py))?;
        let mut bodies = crate::errors::try_borrow_mut(self.bodies.bind(py))?;
        let mut colliders = crate::errors::try_borrow_mut(self.colliders.bind(py))?;
        let mut ij = crate::errors::try_borrow_mut(self.impulse_joints.bind(py))?;
        let mut mj = crate::errors::try_borrow_mut(self.multibody_joints.bind(py))?;
        let mut sb = crate::errors::try_borrow_mut(self.soft_bodies.bind(py))?;
        let mut components = rapier::pipeline::WorldComponentsMut {
            gravity,
            integration_parameters: &mut ip.0,
            islands: &mut islands.0,
            broad_phase: &mut bp.0,
            narrow_phase: &mut np.0,
            bodies: &mut bodies.0,
            colliders: &mut colliders.0,
            impulse_joints: &mut ij.0,
            multibody_joints: &mut mj.0,
            soft_bodies: &mut sb.0,
        };
        Ok(f(&mut components))
    }
}

#[pymethods]
impl PhysicsWorld {
    /// Start recording which parts of the world change, for :meth:`delta_since`.
    ///
    /// Call it on the sender before issuing the first tick (with :meth:`change_tick` or
    /// ``delta_since(None)``). Calling it again restarts the record: every tick issued
    /// before is then rejected by :meth:`delta_since`.
    fn enable_change_tracking(&self, py: Python<'_>) -> PyResult<()> {
        let mut g = self.gravity_vector();
        self.with_components(py, &mut g, |c| c.enable_change_tracking())
    }

    /// Stop recording changes, and drop the record.
    ///
    /// :meth:`delta_since` raises :class:`DeltaError` until
    /// :meth:`enable_change_tracking` is called again.
    fn disable_change_tracking(&self, py: Python<'_>) -> PyResult<()> {
        let mut g = self.gravity_vector();
        self.with_components(py, &mut g, |c| c.disable_change_tracking())
    }

    /// A tick naming the world's current state, without building a delta.
    ///
    /// Call it right before taking a :meth:`snapshot` sent to a receiver: the receiver
    /// restores the snapshot, then passes the tick to :meth:`set_delta_baseline`, and
    /// the sender passes it to its next :meth:`delta_since`. Requires
    /// :meth:`enable_change_tracking`.
    ///
    /// :returns: The tick, as opaque bytes.
    /// :raises DeltaError: if change tracking is disabled (``kind ==
    ///     "tracking_disabled"``).
    fn change_tick<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        let mut g = self.gravity_vector();
        let tick = self
            .with_components(py, &mut g, |c| c.change_tick())?
            .map_err(|e| delta_err(py, e))?;
        delta_to_py(py, crate::serde_io::BlobKind::Tick, &tick)
    }

    /// Declare that this world holds the sender's state at ``tick``, so deltas produced
    /// from that tick can be applied.
    ///
    /// Call it on a world restored from a :meth:`snapshot` taken right after the sender's
    /// :meth:`change_tick`. Stepping this world, or :meth:`detect_collisions`, clears the
    /// baseline again; other local edits are not detected and require a full delta.
    ///
    /// :param tick: Bytes returned by the sender's :meth:`change_tick` (``bytes``, or any
    ///     buffer such as ``bytearray`` or ``memoryview``).
    /// :raises SerializationError: if ``tick`` is not a valid tick blob.
    fn set_delta_baseline(&self, py: Python<'_>, tick: &Bound<'_, PyAny>) -> PyResult<()> {
        let tick: rapier::pipeline::WorldChangeTick =
            delta_from_py(&blob_bytes(tick)?, crate::serde_io::BlobKind::Tick)?;
        let mut g = self.gravity_vector();
        self.with_components(py, &mut g, |c| c.set_delta_baseline(tick))
    }

    /// The changes made to the world since a tick a receiver acknowledged.
    ///
    /// Requires :meth:`enable_change_tracking`. The returned delta, applied with
    /// :meth:`apply_delta` to a receiver holding the state of ``since``, leaves it
    /// simulating identically to this world. Its size follows what changed: roughly the
    /// awake bodies, their colliders and contacts.
    ///
    /// Typical replication loop::
    ///
    ///     server.enable_change_tracking()
    ///     delta, acked = server.delta_since(None)   # full state
    ///     client = PhysicsWorld()
    ///     client.apply_delta(delta)
    ///     while True:
    ///         server.step()
    ///         delta, _ = server.delta_since(acked)   # sent to the client
    ///         acked = client.apply_delta(delta)       # sent back to the server
    ///
    /// ``since`` can be any tick a receiver acknowledged, not only the last one, so a
    /// receiver that missed deltas catches up with one delta from its own tick.
    ///
    /// :param since: A tick returned by this method, :meth:`change_tick` or
    ///     :meth:`apply_delta` (``bytes``, or any buffer such as ``bytearray`` or
    ///     ``memoryview``), or ``None`` or an empty buffer for the whole state (bootstraps a
    ///     receiver).
    /// :returns: ``(delta, tick)``: the delta bytes, and the tick (bytes) the receiver
    ///     is at once it applied the delta. Both are opaque and can be sent over the
    ///     network.
    /// :raises DeltaError: if change tracking is disabled (``kind ==
    ///     "tracking_disabled"``), or if ``since`` was not issued by this world since
    ///     tracking was last enabled or is too old (``kind == "unknown_baseline"``: send a
    ///     full state with ``since=None`` instead).
    /// :raises SerializationError: if ``since`` is not a valid tick blob, or the delta would
    ///     exceed the 256 MiB blob limit.
    #[pyo3(signature = (since=None))]
    fn delta_since<'py>(
        &self,
        py: Python<'py>,
        since: Option<&Bound<'py, PyAny>>,
    ) -> PyResult<(Bound<'py, PyBytes>, Bound<'py, PyBytes>)> {
        let since = since.map(blob_bytes).transpose()?;
        // An empty baseline is no baseline (a full delta), as in the C and TypeScript bindings.
        let since: Option<rapier::pipeline::WorldChangeTick> = since
            .filter(|t| !t.as_bytes().is_empty())
            .map(|t| delta_from_py(&t, crate::serde_io::BlobKind::Tick))
            .transpose()?;
        let mut g = self.gravity_vector();
        let (delta, tick) = self
            .with_components(py, &mut g, |c| c.delta_since(since))?
            .map_err(|e| delta_err(py, e))?;
        Ok((
            delta_to_py(py, crate::serde_io::BlobKind::Delta, &delta)?,
            delta_to_py(py, crate::serde_io::BlobKind::Tick, &tick)?,
        ))
    }

    /// Patch a delta produced by :meth:`delta_since` into this world.
    ///
    /// Only apply trusted deltas, produced by the same rapier build: the checks catch
    /// corrupted headers and bytes that do not decode, but a corrupted delta that still
    /// decodes may apply and make a later step panic (``PanicException``).
    ///
    /// An incremental delta requires this world to hold the delta's baseline: the target
    /// of the last delta it applied, or the tick declared with :meth:`set_delta_baseline`,
    /// and not stepped (nor :meth:`detect_collisions`) since. Other local edits of this world
    /// are not detected: resynchronize it with a full delta. A full delta (``since=None``)
    /// can be applied to a world in any state, e.g. a fresh ``PhysicsWorld()``: it resets the
    /// world first. Change tracking is not needed on the receiver.
    ///
    /// Replicated objects keep the sender's handles: a handle received from the sender
    /// (e.g. rebuilt with ``RigidBodyHandle.from_raw_parts``) names the same object here. A
    /// full delta applied to a non-empty world built independently replaces its objects, so
    /// one of its old handles may stay valid but name a different object: apply it to an
    /// empty world or a replica of the same sender.
    ///
    /// :param delta: Bytes returned by :meth:`delta_since` (``bytes``, or any buffer such as
    ///     ``bytearray`` or ``memoryview``).
    /// :returns: The tick (bytes) to acknowledge to the sender, which passes it to its
    ///     next :meth:`delta_since`.
    /// :raises DeltaError: if this world does not hold the delta's baseline (``kind ==
    ///     "baseline_mismatch"``), or lacks a shape the delta references (``"missing_shape"``),
    ///     both checked before anything is written, or if the patch found the delta
    ///     inconsistent with this world (``"malformed"``). Only ``"malformed"``, raised while
    ///     patching, may leave the world partially patched: recover by applying a full delta.
    ///     Not every inconsistency is detected (see above).
    /// :raises SerializationError: if ``delta`` is not a valid delta blob (e.g. a tick
    ///     or a snapshot), the world then being untouched. ``except SerializationError``
    ///     also catches ``DeltaError``: check its ``kind`` for ``"malformed"``.
    fn apply_delta<'py>(
        &mut self,
        py: Python<'py>,
        delta: &Bound<'py, PyAny>,
    ) -> PyResult<Bound<'py, PyBytes>> {
        let delta: rapier::pipeline::WorldDelta =
            delta_from_py(&blob_bytes(delta)?, crate::serde_io::BlobKind::Delta)?;
        let mut g = self.gravity_vector();
        let result = self.with_components(py, &mut g, |c| c.apply_delta(delta))?;
        // Gravity may already be patched when a later component fails, so keep it either way.
        self.gravity = Vec3(crate::na::Vector3::new(g.x, g.y, g.z));
        // Wake-ups queued by local joint edits are not part of the replicated state, so they
        // go once anything was patched (only `Malformed` fails after patching started).
        if matches!(result, Ok(_) | Err(rapier::data::DeltaError::Malformed)) {
            crate::errors::try_borrow_mut(self.multibody_joints.bind(py))?
                .1
                .clear();
        }
        let tick = result.map_err(|e| delta_err(py, e))?;
        delta_to_py(py, crate::serde_io::BlobKind::Tick, &tick)
    }
}
