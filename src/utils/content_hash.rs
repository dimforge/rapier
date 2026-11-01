//! A serde serializer that hashes a value's serialized content instead of writing it.
//!
//! Used by the collider shape interning and by snapshot deltas to name shapes by content. The
//! key may change across versions but not across platforms: lengths are hashed as `u64` and
//! scalars as little-endian bytes.

use core::fmt::Display;
use serde::ser::{
    Serialize, SerializeMap, SerializeSeq, SerializeStruct, SerializeStructVariant, SerializeTuple,
    SerializeTupleStruct, SerializeTupleVariant, Serializer,
};

/// A 128-bit content key: two independent 64-bit accumulators over the same byte
/// stream. Collisions between distinct shapes would silently merge them in the interned
/// table, so 64 bits is not enough; at 128 bits an accidental collision is negligible
/// (the input is not adversarial: applications hash their own shapes).
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, serde::Serialize, serde::Deserialize)]
pub(crate) struct ContentKey(u64, u64);

/// Hashes `value`'s serialized content. Returns `None` only if the value's `Serialize`
/// impl itself errors (a custom shape refusing serialization); callers fall back to
/// identity-only grouping for such values.
pub(crate) fn content_key<T: Serialize + ?Sized>(value: &T) -> Option<ContentKey> {
    let mut hasher = ContentHasher::new();
    value.serialize(&mut hasher).ok()?;
    Some(hasher.finish())
}

struct ContentHasher {
    // FNV-1a over 64-bit words.
    h1: u64,
    // Multiply-add mix with a golden-ratio constant, independent enough of FNV for
    // accidental-collision purposes.
    h2: u64,
    /// The bytes of the word being filled, little-endian.
    word: u64,
    /// The number of bytes written so far.
    len: u64,
}

impl ContentHasher {
    fn new() -> Self {
        Self {
            h1: 0xcbf2_9ce4_8422_2325,
            h2: 0x9e37_79b9_7f4a_7c15,
            word: 0,
            len: 0,
        }
    }

    #[inline]
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.word |= (b as u64) << (8 * (self.len % 8));
            self.len += 1;
            if self.len % 8 == 0 {
                self.mix(self.word);
                self.word = 0;
            }
        }
    }

    #[inline]
    fn mix(&mut self, w: u64) {
        // The xor-shift carries the multiply's high bits back down, so no bit flip can cancel.
        self.h1 = (self.h1 ^ w).wrapping_mul(0x100_0000_01b3);
        self.h1 ^= self.h1 >> 29;
        self.h2 = self
            .h2
            .wrapping_mul(0x9e37_79b9_7f4a_7c15)
            .wrapping_add(w ^ (self.h2 >> 47));
    }

    /// Mixes the partial last word and the length (so streams differing only by trailing zero
    /// bytes hash apart), then avalanches both halves.
    fn finish(mut self) -> ContentKey {
        let (word, len) = (self.word, self.len);
        self.mix(word);
        self.mix(len);
        ContentKey(fmix64(self.h1), fmix64(self.h2))
    }

    /// A one-byte tag distinguishing the serde call that produced the following bytes,
    /// so e.g. `Some(0u8)` and `1u8` cannot hash alike.
    #[inline]
    fn tag(&mut self, t: u8) {
        self.write(&[t]);
    }

    #[inline]
    fn u64(&mut self, v: u64) {
        self.write(&v.to_le_bytes());
    }
}

/// MurmurHash3's 64-bit finalizer.
fn fmix64(mut k: u64) -> u64 {
    k ^= k >> 33;
    k = k.wrapping_mul(0xff51_afd7_ed55_8ccd);
    k ^= k >> 33;
    k = k.wrapping_mul(0xc4ce_b9fe_1a85_ec53);
    k ^ (k >> 33)
}

#[derive(Debug)]
pub(crate) struct ContentHashError;

impl Display for ContentHashError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("content hashing failed")
    }
}

impl std::error::Error for ContentHashError {}

impl serde::ser::Error for ContentHashError {
    fn custom<T: Display>(_: T) -> Self {
        ContentHashError
    }
}

impl<'a> Serializer for &'a mut ContentHasher {
    type Ok = ();
    type Error = ContentHashError;
    type SerializeSeq = Self;
    type SerializeTuple = Self;
    type SerializeTupleStruct = Self;
    type SerializeTupleVariant = Self;
    type SerializeMap = Self;
    type SerializeStruct = Self;
    type SerializeStructVariant = Self;

    fn serialize_bool(self, v: bool) -> Result<(), ContentHashError> {
        self.tag(1);
        self.write(&[v as u8]);
        Ok(())
    }
    fn serialize_i8(self, v: i8) -> Result<(), ContentHashError> {
        self.tag(2);
        self.write(&v.to_le_bytes());
        Ok(())
    }
    fn serialize_i16(self, v: i16) -> Result<(), ContentHashError> {
        self.tag(3);
        self.write(&v.to_le_bytes());
        Ok(())
    }
    fn serialize_i32(self, v: i32) -> Result<(), ContentHashError> {
        self.tag(4);
        self.write(&v.to_le_bytes());
        Ok(())
    }
    fn serialize_i64(self, v: i64) -> Result<(), ContentHashError> {
        self.tag(5);
        self.write(&v.to_le_bytes());
        Ok(())
    }
    fn serialize_u8(self, v: u8) -> Result<(), ContentHashError> {
        self.tag(6);
        self.write(&v.to_le_bytes());
        Ok(())
    }
    fn serialize_u16(self, v: u16) -> Result<(), ContentHashError> {
        self.tag(7);
        self.write(&v.to_le_bytes());
        Ok(())
    }
    fn serialize_u32(self, v: u32) -> Result<(), ContentHashError> {
        self.tag(8);
        self.write(&v.to_le_bytes());
        Ok(())
    }
    fn serialize_u64(self, v: u64) -> Result<(), ContentHashError> {
        self.tag(9);
        self.write(&v.to_le_bytes());
        Ok(())
    }
    fn serialize_i128(self, v: i128) -> Result<(), ContentHashError> {
        self.tag(10);
        self.write(&v.to_le_bytes());
        Ok(())
    }
    fn serialize_u128(self, v: u128) -> Result<(), ContentHashError> {
        self.tag(11);
        self.write(&v.to_le_bytes());
        Ok(())
    }
    fn serialize_f32(self, v: f32) -> Result<(), ContentHashError> {
        self.tag(12);
        self.write(&v.to_bits().to_le_bytes());
        Ok(())
    }
    fn serialize_f64(self, v: f64) -> Result<(), ContentHashError> {
        self.tag(13);
        self.write(&v.to_bits().to_le_bytes());
        Ok(())
    }
    fn serialize_char(self, v: char) -> Result<(), ContentHashError> {
        self.tag(14);
        self.write(&(v as u32).to_le_bytes());
        Ok(())
    }
    fn serialize_str(self, v: &str) -> Result<(), ContentHashError> {
        self.tag(15);
        self.u64(v.len() as u64);
        self.write(v.as_bytes());
        Ok(())
    }
    fn serialize_bytes(self, v: &[u8]) -> Result<(), ContentHashError> {
        self.tag(16);
        self.u64(v.len() as u64);
        self.write(v);
        Ok(())
    }
    fn serialize_none(self) -> Result<(), ContentHashError> {
        self.tag(17);
        Ok(())
    }
    fn serialize_some<T: Serialize + ?Sized>(self, value: &T) -> Result<(), ContentHashError> {
        self.tag(18);
        value.serialize(self)
    }
    fn serialize_unit(self) -> Result<(), ContentHashError> {
        self.tag(19);
        Ok(())
    }
    fn serialize_unit_struct(self, name: &'static str) -> Result<(), ContentHashError> {
        self.tag(20);
        self.serialize_str(name)
    }
    fn serialize_unit_variant(
        self,
        name: &'static str,
        variant_index: u32,
        _variant: &'static str,
    ) -> Result<(), ContentHashError> {
        self.tag(21);
        self.serialize_str(name)?;
        self.u64(variant_index as u64);
        Ok(())
    }
    fn serialize_newtype_struct<T: Serialize + ?Sized>(
        self,
        name: &'static str,
        value: &T,
    ) -> Result<(), ContentHashError> {
        self.tag(22);
        self.serialize_str(name)?;
        value.serialize(self)
    }
    fn serialize_newtype_variant<T: Serialize + ?Sized>(
        self,
        name: &'static str,
        variant_index: u32,
        _variant: &'static str,
        value: &T,
    ) -> Result<(), ContentHashError> {
        self.tag(23);
        self.serialize_str(name)?;
        self.u64(variant_index as u64);
        value.serialize(self)
    }
    fn serialize_seq(self, len: Option<usize>) -> Result<Self, ContentHashError> {
        self.tag(24);
        self.u64(len.map(|l| l as u64).unwrap_or(u64::MAX));
        Ok(self)
    }
    fn serialize_tuple(self, len: usize) -> Result<Self, ContentHashError> {
        self.tag(25);
        self.u64(len as u64);
        Ok(self)
    }
    fn serialize_tuple_struct(
        self,
        name: &'static str,
        len: usize,
    ) -> Result<Self, ContentHashError> {
        self.tag(26);
        self.serialize_str(name)?;
        self.u64(len as u64);
        Ok(self)
    }
    fn serialize_tuple_variant(
        self,
        name: &'static str,
        variant_index: u32,
        _variant: &'static str,
        len: usize,
    ) -> Result<Self, ContentHashError> {
        self.tag(27);
        self.serialize_str(name)?;
        self.u64(variant_index as u64);
        self.u64(len as u64);
        Ok(self)
    }
    fn serialize_map(self, len: Option<usize>) -> Result<Self, ContentHashError> {
        self.tag(28);
        self.u64(len.map(|l| l as u64).unwrap_or(u64::MAX));
        Ok(self)
    }
    fn serialize_struct(self, name: &'static str, len: usize) -> Result<Self, ContentHashError> {
        self.tag(29);
        self.serialize_str(name)?;
        self.u64(len as u64);
        Ok(self)
    }
    fn serialize_struct_variant(
        self,
        name: &'static str,
        variant_index: u32,
        _variant: &'static str,
        len: usize,
    ) -> Result<Self, ContentHashError> {
        self.tag(30);
        self.serialize_str(name)?;
        self.u64(variant_index as u64);
        self.u64(len as u64);
        Ok(self)
    }

    fn is_human_readable(&self) -> bool {
        false
    }
}

impl SerializeSeq for &mut ContentHasher {
    type Ok = ();
    type Error = ContentHashError;
    fn serialize_element<T: Serialize + ?Sized>(
        &mut self,
        value: &T,
    ) -> Result<(), ContentHashError> {
        value.serialize(&mut **self)
    }
    fn end(self) -> Result<(), ContentHashError> {
        Ok(())
    }
}

impl SerializeTuple for &mut ContentHasher {
    type Ok = ();
    type Error = ContentHashError;
    fn serialize_element<T: Serialize + ?Sized>(
        &mut self,
        value: &T,
    ) -> Result<(), ContentHashError> {
        value.serialize(&mut **self)
    }
    fn end(self) -> Result<(), ContentHashError> {
        Ok(())
    }
}

impl SerializeTupleStruct for &mut ContentHasher {
    type Ok = ();
    type Error = ContentHashError;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        value: &T,
    ) -> Result<(), ContentHashError> {
        value.serialize(&mut **self)
    }
    fn end(self) -> Result<(), ContentHashError> {
        Ok(())
    }
}

impl SerializeTupleVariant for &mut ContentHasher {
    type Ok = ();
    type Error = ContentHashError;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        value: &T,
    ) -> Result<(), ContentHashError> {
        value.serialize(&mut **self)
    }
    fn end(self) -> Result<(), ContentHashError> {
        Ok(())
    }
}

impl SerializeMap for &mut ContentHasher {
    type Ok = ();
    type Error = ContentHashError;
    fn serialize_key<T: Serialize + ?Sized>(&mut self, key: &T) -> Result<(), ContentHashError> {
        key.serialize(&mut **self)
    }
    fn serialize_value<T: Serialize + ?Sized>(
        &mut self,
        value: &T,
    ) -> Result<(), ContentHashError> {
        value.serialize(&mut **self)
    }
    fn end(self) -> Result<(), ContentHashError> {
        Ok(())
    }
}

impl SerializeStruct for &mut ContentHasher {
    type Ok = ();
    type Error = ContentHashError;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        _key: &'static str,
        value: &T,
    ) -> Result<(), ContentHashError> {
        value.serialize(&mut **self)
    }
    fn end(self) -> Result<(), ContentHashError> {
        Ok(())
    }
}

impl SerializeStructVariant for &mut ContentHasher {
    type Ok = ();
    type Error = ContentHashError;
    fn serialize_field<T: Serialize + ?Sized>(
        &mut self,
        _key: &'static str,
        value: &T,
    ) -> Result<(), ContentHashError> {
        value.serialize(&mut **self)
    }
    fn end(self) -> Result<(), ContentHashError> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::content_key;
    use crate::alloc_prelude::*;

    #[test]
    fn equal_content_hashes_equal() {
        let a = vec![1.0f32, 2.0, 3.0];
        assert_eq!(content_key(&a), content_key(&a.clone()));
    }

    #[test]
    fn different_content_hashes_apart() {
        assert_ne!(
            content_key(&vec![1.0f32, 2.0]),
            content_key(&vec![2.0f32, 1.0])
        );
        assert_ne!(content_key(&vec![0u8]), content_key(&vec![0u8, 0]));
        // The serde call is hashed too: these have the same bytes.
        assert_ne!(content_key(&Some(0u8)), content_key(&1u8));
        assert_ne!(content_key(&1u32), content_key(&1i32));
    }

    #[test]
    fn top_bit_flips_hash_apart() {
        // Flipping the top bit of two consecutive words cancels out in a plain xor-multiply.
        let key = |words: &[u64]| {
            let mut hasher = super::ContentHasher::new();
            for w in words {
                hasher.mix(*w);
            }
            hasher.finish()
        };
        let (a, b) = (key(&[1, 2, 3]), key(&[1 ^ (1 << 63), 2 ^ (1 << 63), 3]));
        assert!(a.0 != b.0 && a.1 != b.1);
    }

    #[test]
    fn keys_do_not_depend_on_the_platform() {
        // Lengths are hashed as `u64` and scalars little-endian: this key holds on every target.
        let key = content_key(&(vec![7u16; 3], 0.5f32, usize::MAX >> 1 > 0)).unwrap();
        assert_eq!(
            key,
            super::ContentKey(8780902226574128264, 11824366988356022364)
        );
    }
}
