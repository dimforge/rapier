//! Binary deltas between physics snapshots.
//!
//! For per-tick snapshotting (rollback netcode, periodic sync), most of two
//! consecutive snapshots' bytes are identical: sleeping islands, their contact pairs,
//! static colliders and shapes don't change between ticks. [`snapshot_delta`] encodes a
//! target snapshot as copy/insert operations against a baseline snapshot, so the delta
//! size is proportional to what actually changed (roughly the awake set), and
//! [`apply_snapshot_delta`] reconstructs the target byte-for-byte.
//!
//! The encoding is content-based (rsync-style block matching), so it works on any
//! serialization format and tolerates insertions and shifts (e.g. arena growth). The
//! CPU cost of *producing* a delta is still a full serialization plus a linear scan;
//! only the output size is proportional to the change.
//!
//! ```
//! # #[cfg(feature = "serde-serialize")] {
//! # use rapier3d::prelude::*;
//! # use rapier3d::pipeline::{snapshot_delta, apply_snapshot_delta};
//! # let mut world = PhysicsWorld::new();
//! let baseline = bincode::serialize(&world).unwrap();
//! world.step();
//! let target = bincode::serialize(&world).unwrap();
//! let delta = snapshot_delta(&baseline, &target);
//! assert_eq!(apply_snapshot_delta(&baseline, &delta).unwrap(), target);
//! # }
//! ```

use crate::alloc_prelude::*;
use std::collections::HashMap;

/// Size of the blocks matched between the baseline and the target. Smaller blocks find
/// more matches but cost more per-op overhead; 64 keeps the overhead below 20% for
/// fully-changed regions while still matching individual sleeping contact pairs.
const BLOCK: usize = 64;

/// Errors returned by [`apply_snapshot_delta`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SnapshotDeltaError {
    /// The delta refers to a baseline range that does not exist; the baseline passed
    /// to `apply_snapshot_delta` is not the one the delta was produced against.
    #[error("the delta references bytes outside the baseline; wrong baseline snapshot?")]
    BaselineMismatch,
    /// The delta bytes are truncated or malformed.
    #[error("malformed snapshot delta")]
    Malformed,
}

fn write_varint(out: &mut Vec<u8>, mut v: u64) {
    loop {
        let b = (v & 0x7f) as u8;
        v >>= 7;
        if v == 0 {
            out.push(b);
            return;
        }
        out.push(b | 0x80);
    }
}

fn read_varint(bytes: &[u8], pos: &mut usize) -> Result<u64, SnapshotDeltaError> {
    let mut v = 0u64;
    let mut shift = 0u32;
    loop {
        let b = *bytes.get(*pos).ok_or(SnapshotDeltaError::Malformed)?;
        *pos += 1;
        v |= ((b & 0x7f) as u64) << shift;
        if b & 0x80 == 0 {
            return Ok(v);
        }
        shift += 7;
        if shift >= 64 {
            return Err(SnapshotDeltaError::Malformed);
        }
    }
}

/// A weak, cheap rolling hash over one block (a polynomial in an odd multiplier);
/// collisions are resolved by direct byte comparison, so its quality only affects
/// speed, not correctness.
fn block_hash(block: &[u8]) -> u64 {
    let mut h = 0u64;
    for &b in block {
        h = h.wrapping_mul(0x0100_0000_01b3).wrapping_add(b as u64);
    }
    h
}

/// Encodes `target` as a delta against `base`. Apply with [`apply_snapshot_delta`].
///
/// The two inputs may be any byte strings, but the intended use is two snapshots of
/// the same world at nearby ticks, where the unchanged (mostly sleeping) state makes
/// the delta small.
pub fn snapshot_delta(base: &[u8], target: &[u8]) -> Vec<u8> {
    // Index the baseline's aligned blocks. Later blocks overwrite earlier ones on
    // hash collision; correctness is preserved by the byte-compare below.
    let mut index: HashMap<u64, u32> = HashMap::with_capacity(base.len() / BLOCK + 1);
    for (i, block) in base.chunks_exact(BLOCK).enumerate() {
        index.insert(block_hash(block), i as u32);
    }

    let mut out = Vec::new();
    write_varint(&mut out, target.len() as u64);

    // Pending literal bytes, flushed before each copy op and at the end.
    let mut literal_start = 0usize;
    let mut pos = 0usize;
    // Where the previous copy op left off in the baseline: unchanged regions continue
    // from here, so trying it (and the identity-aligned offset) before the block index
    // keeps runs coalesced even when the content is repetitive.
    let mut expected = 0usize;

    let flush_literal = |out: &mut Vec<u8>, from: usize, to: usize| {
        if to > from {
            // Op: literal run. Tag bit 0 = insert.
            write_varint(out, ((to - from) as u64) << 1);
            out.extend_from_slice(&target[from..to]);
        }
    };

    while pos + BLOCK <= target.len() {
        let probe = |bstart: usize| {
            bstart + BLOCK <= base.len()
                && base[bstart..bstart + BLOCK] == target[pos..pos + BLOCK]
        };
        let matched = if probe(expected) {
            Some(expected)
        } else if expected != pos && probe(pos) {
            Some(pos)
        } else {
            let h = block_hash(&target[pos..pos + BLOCK]);
            index.get(&h).and_then(|&bi| {
                let bstart = bi as usize * BLOCK;
                probe(bstart).then_some(bstart)
            })
        };
        let Some(bstart) = matched else {
            pos += 1;
            continue;
        };

        // Extend the match forward beyond the block.
        let mut len = BLOCK;
        while bstart + len < base.len()
            && pos + len < target.len()
            && base[bstart + len] == target[pos + len]
        {
            len += 1;
        }
        // Extend backward into the pending literal run.
        let mut back = 0usize;
        while back < pos - literal_start
            && back < bstart
            && base[bstart - back - 1] == target[pos - back - 1]
        {
            back += 1;
        }

        flush_literal(&mut out, literal_start, pos - back);
        // Op: copy run. Tag bit 1 = copy, followed by the baseline offset.
        write_varint(&mut out, (((len + back) as u64) << 1) | 1);
        write_varint(&mut out, (bstart - back) as u64);

        pos += len;
        literal_start = pos;
        expected = bstart + len;
    }
    flush_literal(&mut out, literal_start, target.len());
    out
}

/// Reconstructs the target snapshot from `base` and a delta produced by
/// [`snapshot_delta`] against that same baseline.
pub fn apply_snapshot_delta(base: &[u8], delta: &[u8]) -> Result<Vec<u8>, SnapshotDeltaError> {
    let mut pos = 0usize;
    let target_len = read_varint(delta, &mut pos)? as usize;
    let mut out = Vec::with_capacity(target_len);

    while pos < delta.len() {
        let op = read_varint(delta, &mut pos)?;
        let len = (op >> 1) as usize;
        if op & 1 == 1 {
            let offset = read_varint(delta, &mut pos)? as usize;
            let src = base
                .get(offset..offset + len)
                .ok_or(SnapshotDeltaError::BaselineMismatch)?;
            out.extend_from_slice(src);
        } else {
            let src = delta
                .get(pos..pos + len)
                .ok_or(SnapshotDeltaError::Malformed)?;
            out.extend_from_slice(src);
            pos += len;
        }
    }

    if out.len() != target_len {
        return Err(SnapshotDeltaError::Malformed);
    }
    Ok(out)
}

#[cfg(test)]
mod test {
    use super::*;
    use std::vec;

    fn roundtrip(base: &[u8], target: &[u8]) -> usize {
        let delta = snapshot_delta(base, target);
        assert_eq!(apply_snapshot_delta(base, &delta).unwrap(), target);
        delta.len()
    }

    #[test]
    fn identical_inputs_give_tiny_delta() {
        let data: Vec<u8> = (0..10_000u32).map(|i| (i * 31) as u8).collect();
        let len = roundtrip(&data, &data);
        assert!(len < 32, "identity delta is {len} bytes");
    }

    #[test]
    fn small_change_gives_small_delta() {
        let base: Vec<u8> = (0..100_000u32).map(|i| (i * 31) as u8).collect();
        let mut target = base.clone();
        for i in 50_000..50_100 {
            target[i] ^= 0xff;
        }
        let len = roundtrip(&base, &target);
        assert!(len < 600, "100-byte change produced a {len}-byte delta");
    }

    #[test]
    fn insertion_shifts_are_matched() {
        let base: Vec<u8> = (0..100_000u32).map(|i| (i * 31) as u8).collect();
        let mut target = base.clone();
        // Insert 5 bytes in the middle: everything after shifts.
        target.splice(50_000..50_000, [1, 2, 3, 4, 5]);
        let len = roundtrip(&base, &target);
        assert!(len < 600, "5-byte insertion produced a {len}-byte delta");
    }

    #[test]
    fn unrelated_inputs_still_apply() {
        let base: Vec<u8> = (0..10_000u32).map(|i| (i * 31) as u8).collect();
        let target: Vec<u8> = (0..12_345u32).map(|i| (i * 17 + 5) as u8).collect();
        roundtrip(&base, &target);
    }

    #[test]
    fn empty_and_tiny_inputs() {
        roundtrip(&[], &[]);
        roundtrip(&[], &[1, 2, 3]);
        roundtrip(&[1, 2, 3], &[]);
        roundtrip(&[1, 2, 3], &[1, 2, 3]);
    }

    #[test]
    fn wrong_baseline_is_detected_or_yields_garbage_of_right_shape() {
        let base: Vec<u8> = vec![0; 100_000];
        let target: Vec<u8> = vec![0; 100_001];
        let delta = snapshot_delta(&base, &target);
        // A shorter baseline must be rejected rather than panicking.
        let err = apply_snapshot_delta(&base[..10], &delta);
        assert!(err.is_err());
    }
}
