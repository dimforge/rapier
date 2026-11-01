//! Shared (dim/scalar-agnostic) helpers used by pickle/snapshot serialization.
//!
//! Every binary blob begins with an 8-byte header:
//!   - 4 bytes magic naming the blob kind (see [`BlobKind`]): `b"RPYS"` for snapshots and
//!     `to_bytes` blobs, `b"RPYD"` for snapshot deltas, `b"RPYT"` for change ticks
//!   - 4 bytes little-endian `u32` version (currently 2, shared by every kind)
//!
//! Anything that follows is the bincode-serialized payload (or for the JSON
//! debug mode, a JSON object with a `"_rpys"` magic+version field, see below).
//!
//! Mismatched magic or unknown version => `SerializationError`.
//!
//! This module deliberately stays free of `rapier::...` paths. The
//! type-specific glue (the `Py<T>` round-trip on `RigidBody`, `Collider`,
//! `RigidBodySet`, etc., plus `PhysicsWorld.snapshot()`/`restore()`) lives in
//! the `serde_glue` module.

use pyo3::prelude::*;
use pyo3::types::PyBytes;

/// Magic header marking a rapier-py snapshot or `to_bytes` blob ("Rapier PYthon Snapshot").
pub const MAGIC: &[u8; 4] = b"RPYS";
/// Magic header marking a snapshot delta ("Rapier PYthon Delta").
pub const DELTA_MAGIC: &[u8; 4] = b"RPYD";
/// Magic header marking a change tick ("Rapier PYthon Tick").
pub const TICK_MAGIC: &[u8; 4] = b"RPYT";
/// Current on-disk format version. Bumped on incompatible payload changes.
pub const VERSION: u32 = 2;
/// Total size of the binary header (magic + u32 version).
pub const HEADER_SIZE: usize = 8;

/// The kind of a binary blob, as told by its magic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlobKind {
    /// A world snapshot or a `to_bytes` blob (`RPYS`).
    Snapshot,
    /// A snapshot delta (`RPYD`).
    Delta,
    /// A change tick (`RPYT`).
    Tick,
}

impl BlobKind {
    /// The 4-byte magic opening blobs of this kind.
    pub fn magic(self) -> &'static [u8; 4] {
        match self {
            BlobKind::Snapshot => MAGIC,
            BlobKind::Delta => DELTA_MAGIC,
            BlobKind::Tick => TICK_MAGIC,
        }
    }

    /// The kind whose magic is `magic`, if any.
    pub fn from_magic(magic: &[u8]) -> Option<Self> {
        [BlobKind::Snapshot, BlobKind::Delta, BlobKind::Tick]
            .into_iter()
            .find(|kind| kind.magic() == magic)
    }

    /// The name used in error messages, e.g. "delta".
    pub fn name(self) -> &'static str {
        match self {
            BlobKind::Snapshot => "snapshot",
            BlobKind::Delta => "delta",
            BlobKind::Tick => "tick",
        }
    }
}

/// Prepend the 8-byte `RPYS` header to `payload`.
#[inline]
pub fn wrap_bincode(payload: Vec<u8>) -> Vec<u8> {
    wrap_blob(BlobKind::Snapshot, payload)
}

/// Prepend the 8-byte header of `kind` to `payload`.
pub fn wrap_blob(kind: BlobKind, payload: Vec<u8>) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEADER_SIZE + payload.len());
    out.extend_from_slice(kind.magic());
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.extend_from_slice(&payload);
    out
}

/// Strip and validate the 8-byte `RPYS` header. Returns the body.
#[inline]
pub fn unwrap_bincode(blob: &[u8]) -> PyResult<&[u8]> {
    unwrap_blob(blob, BlobKind::Snapshot)
}

/// Strip and validate the 8-byte header of a blob of kind `expected`. Returns the body.
///
/// Returns `SerializationError` on:
///  - Blob shorter than the header.
///  - Magic mismatch, naming the blob's kind when it is another rapier-py blob.
///  - Unsupported version (`!= VERSION`).
pub fn unwrap_blob(blob: &[u8], expected: BlobKind) -> PyResult<&[u8]> {
    if blob.len() < HEADER_SIZE {
        return Err(crate::errors::SerializationError::new_err(format!(
            "serialized blob too short ({} bytes); expected at least {}",
            blob.len(),
            HEADER_SIZE
        )));
    }
    let magic = &blob[..4];
    if magic != expected.magic() {
        let msg = match BlobKind::from_magic(magic) {
            Some(found) => format!("expected a {}, got a {}", expected.name(), found.name()),
            None => format!(
                "bad magic in serialized blob: expected {:?} (a {}) got {:?}",
                std::str::from_utf8(expected.magic()).unwrap_or("?"),
                expected.name(),
                std::str::from_utf8(magic).unwrap_or("<non-utf8>")
            ),
        };
        return Err(crate::errors::SerializationError::new_err(msg));
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

/// Build the JSON envelope `{"_magic": "RPYS", "_version": VERSION, "payload": <inner>}`.
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
