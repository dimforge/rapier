//! A hash-map that behaves deterministically when the
//! `enhanced-determinism` feature is enabled.

#[cfg(all(feature = "enhanced-determinism", feature = "serde-serialize"))]
use indexmap::IndexMap as StdHashMap;
#[cfg(all(not(feature = "enhanced-determinism"), feature = "serde-serialize"))]
use std::collections::HashMap as StdHashMap;

/// Serializes only the capacity of a hash-map instead of its actual content.
#[cfg(feature = "serde-serialize")]
pub fn serialize_hashmap_capacity<S: serde::Serializer, K, V, H: std::hash::BuildHasher>(
    map: &StdHashMap<K, V, H>,
    s: S,
) -> Result<S::Ok, S::Error> {
    s.serialize_u64(map.capacity() as u64)
}

/// Creates a new hash-map with its capacity deserialized from `d`.
#[cfg(feature = "serde-serialize")]
pub fn deserialize_hashmap_capacity<
    'de,
    D: serde::Deserializer<'de>,
    K,
    V,
    H: std::hash::BuildHasher + Default,
>(
    d: D,
) -> Result<StdHashMap<K, V, H>, D::Error> {
    struct CapacityVisitor;
    impl<'de> serde::de::Visitor<'de> for CapacityVisitor {
        type Value = u64;

        fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
            write!(formatter, "an integer between 0 and 2^64")
        }

        fn visit_u64<E: serde::de::Error>(self, val: u64) -> Result<Self::Value, E> {
            Ok(val)
        }
    }

    let capacity = d.deserialize_u64(CapacityVisitor)? as usize;
    Ok(StdHashMap::with_capacity_and_hasher(
        capacity,
        Default::default(),
    ))
}

/*
 * FxHasher taken from rustc_hash, except that it does not depend on the pointer size.
 */
#[cfg(feature = "enhanced-determinism")]
pub type FxHashMap32<K, V> = indexmap::IndexMap<K, V, std::hash::BuildHasherDefault<FxHasher32>>;
#[cfg(feature = "enhanced-determinism")]
pub use {self::FxHashMap32 as HashMap, indexmap::map::Entry};
#[cfg(not(feature = "enhanced-determinism"))]
pub use {rustc_hash::FxHashMap as HashMap, std::collections::hash_map::Entry};

const K: u32 = 0x9e3779b9;

// Same as FxHasher, but with the guarantee that the internal hash is
// an u32 instead of something that depends on the platform.
pub struct FxHasher32 {
    hash: u32,
}

impl Default for FxHasher32 {
    #[inline]
    fn default() -> FxHasher32 {
        FxHasher32 { hash: 0 }
    }
}

impl FxHasher32 {
    #[inline]
    fn add_to_hash(&mut self, i: u32) {
        use std::ops::BitXor;
        self.hash = self.hash.rotate_left(5).bitxor(i).wrapping_mul(K);
    }
}

impl std::hash::Hasher for FxHasher32 {
    #[inline]
    fn write(&mut self, mut bytes: &[u8]) {
        use std::convert::TryInto;
        let read_u32 = |bytes: &[u8]| u32::from_ne_bytes(bytes[..4].try_into().unwrap());
        let mut hash = FxHasher32 { hash: self.hash };
        assert!(std::mem::size_of::<u32>() <= 8);
        while bytes.len() >= std::mem::size_of::<u32>() {
            hash.add_to_hash(read_u32(bytes) as u32);
            bytes = &bytes[std::mem::size_of::<u32>()..];
        }
        if (std::mem::size_of::<u32>() > 4) && (bytes.len() >= 4) {
            hash.add_to_hash(u32::from_ne_bytes(bytes[..4].try_into().unwrap()) as u32);
            bytes = &bytes[4..];
        }
        if (std::mem::size_of::<u32>() > 2) && bytes.len() >= 2 {
            hash.add_to_hash(u16::from_ne_bytes(bytes[..2].try_into().unwrap()) as u32);
            bytes = &bytes[2..];
        }
        if (std::mem::size_of::<u32>() > 1) && bytes.len() >= 1 {
            hash.add_to_hash(bytes[0] as u32);
        }
        self.hash = hash.hash;
    }

    #[inline]
    fn write_u8(&mut self, i: u8) {
        self.add_to_hash(i as u32);
    }

    #[inline]
    fn write_u16(&mut self, i: u16) {
        self.add_to_hash(i as u32);
    }

    #[inline]
    fn write_u32(&mut self, i: u32) {
        self.add_to_hash(i as u32);
    }

    #[inline]
    fn write_u64(&mut self, i: u64) {
        self.add_to_hash(i as u32);
        self.add_to_hash((i >> 32) as u32);
    }

    #[inline]
    fn write_usize(&mut self, i: usize) {
        self.add_to_hash(i as u32);
    }

    #[inline]
    fn finish(&self) -> u64 {
        self.hash as u64
    }
}
