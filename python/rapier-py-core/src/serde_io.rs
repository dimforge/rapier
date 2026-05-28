//! Shared (dim/scalar-agnostic) helpers used by pickle/snapshot serialization.
//!
//! The on-disk format always begins with an 8-byte header:
//!   - 4 bytes magic = `b"RPYS"`
//!   - 4 bytes little-endian `u32` version (currently 1)
//!
//! Anything that follows is the bincode-serialized payload (or for the JSON
//! debug mode, a JSON object with a `"_rpys"` magic+version field, see below).
//!
//! Mismatched magic or unknown version => `SerializationError`.
//!
//! This module deliberately stays free of `rapier::...` paths so that it can
//! live in the dim/scalar-independent `rapier-py-core`. The dim/scalar-specific
//! glue (the `Py<T>` round-trip on `RigidBody`, `Collider`, `RigidBodySet`,
//! etc., plus `PhysicsWorld.snapshot()`/`restore()`) is emitted by the
//! `define_serde_types!` macro, which is invoked once per cdylib.

use pyo3::prelude::*;
use pyo3::types::PyBytes;

/// Magic header marking a rapier-py blob ("Rapier PYthon Snapshot").
pub const MAGIC: &[u8; 4] = b"RPYS";
/// Current on-disk format version. Bumped on incompatible payload changes.
pub const VERSION: u32 = 1;
/// Total size of the binary header (magic + u32 version).
pub const HEADER_SIZE: usize = 8;

/// Prepend the 8-byte binary header to `payload`.
#[inline]
pub fn wrap_bincode(payload: Vec<u8>) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEADER_SIZE + payload.len());
    out.extend_from_slice(MAGIC);
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.extend_from_slice(&payload);
    out
}

/// Strip and validate the 8-byte binary header. Returns the body.
///
/// Returns `SerializationError` on:
///  - Blob shorter than the header.
///  - Magic mismatch (`!= b"RPYS"`).
///  - Unsupported version (`!= VERSION`).
pub fn unwrap_bincode(blob: &[u8]) -> PyResult<&[u8]> {
    if blob.len() < HEADER_SIZE {
        return Err(crate::errors::SerializationError::new_err(format!(
            "serialized blob too short ({} bytes); expected at least {}",
            blob.len(),
            HEADER_SIZE
        )));
    }
    if &blob[..4] != MAGIC {
        return Err(crate::errors::SerializationError::new_err(format!(
            "bad magic in serialized blob: expected {:?} got {:?}",
            std::str::from_utf8(MAGIC).unwrap_or("?"),
            std::str::from_utf8(&blob[..4]).unwrap_or("<non-utf8>")
        )));
    }
    let mut ver_bytes = [0u8; 4];
    ver_bytes.copy_from_slice(&blob[4..8]);
    let ver = u32::from_le_bytes(ver_bytes);
    if ver != VERSION {
        return Err(crate::errors::SerializationError::new_err(format!(
            "unsupported serialization version {} (this build expects {})",
            ver, VERSION
        )));
    }
    Ok(&blob[HEADER_SIZE..])
}

/// Convert any `bincode::Error` into a `SerializationError`. Used in the
/// `to_bytes`/`from_bytes` impls emitted by the macro.
#[inline]
pub fn bincode_err(e: bincode::Error) -> PyErr {
    crate::errors::SerializationError::new_err(format!("bincode error: {}", e))
}

/// Same idea for `serde_json::Error`.
#[inline]
pub fn json_err(e: serde_json::Error) -> PyErr {
    crate::errors::SerializationError::new_err(format!("json error: {}", e))
}

/// Bytes-from-borrow helper: wrap a `&[u8]` view into a Python `bytes`.
#[inline]
pub fn bytes_to_py<'py>(py: Python<'py>, blob: &[u8]) -> Bound<'py, PyBytes> {
    PyBytes::new_bound(py, blob)
}

/// Magic + version embedded in the JSON envelope as plain string/integer
/// fields so the file is self-describing for human readers.
pub const JSON_MAGIC: &str = "RPYS";

/// Build the JSON envelope `{"_magic": "RPYS", "_version": 1, "payload": <inner>}`.
pub fn wrap_json(payload: serde_json::Value) -> serde_json::Value {
    serde_json::json!({
        "_magic": JSON_MAGIC,
        "_version": VERSION,
        "payload": payload,
    })
}

/// Validate + extract the inner payload from the JSON envelope.
pub fn unwrap_json(envelope: serde_json::Value) -> PyResult<serde_json::Value> {
    let obj = envelope.as_object().ok_or_else(|| {
        crate::errors::SerializationError::new_err(
            "expected JSON object envelope, got something else",
        )
    })?;
    let magic = obj.get("_magic").and_then(|v| v.as_str()).ok_or_else(|| {
        crate::errors::SerializationError::new_err("JSON envelope missing '_magic' field")
    })?;
    if magic != JSON_MAGIC {
        return Err(crate::errors::SerializationError::new_err(format!(
            "bad JSON magic: expected {:?} got {:?}",
            JSON_MAGIC, magic
        )));
    }
    let ver = obj
        .get("_version")
        .and_then(|v| v.as_u64())
        .ok_or_else(|| {
            crate::errors::SerializationError::new_err("JSON envelope missing '_version' field")
        })?;
    if ver as u32 != VERSION {
        return Err(crate::errors::SerializationError::new_err(format!(
            "unsupported JSON serialization version {} (expected {})",
            ver, VERSION
        )));
    }
    obj.get("payload").cloned().ok_or_else(|| {
        crate::errors::SerializationError::new_err("JSON envelope missing 'payload'")
    })
}
