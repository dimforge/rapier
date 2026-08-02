//! Miscellaneous utilities.

#[cfg(not(target_arch = "spirv"))]
mod angular_inertia_ops;
mod component_mul;
mod copysign;
mod cross_product;
mod cross_product_matrix;
mod dot_product;
mod fp_flags;
mod index_mut2;
mod matrix_column;
mod orthonormal_basis;
#[cfg(not(target_arch = "spirv"))]
mod pos_ops;
#[cfg(all(feature = "alloc", not(target_arch = "spirv")))]
mod prefetch;
#[cfg(not(target_arch = "spirv"))]
mod rotation_ops;
#[cfg(not(target_arch = "spirv"))]
mod scalar_type;
mod simd_real_copy;
mod simd_select;

pub use component_mul::ComponentMul;
pub use copysign::CopySign;
pub use index_mut2::IndexMut2;
pub use matrix_column::MatrixColumn;
pub use orthonormal_basis::OrthonormalBasis;
#[cfg(not(target_arch = "spirv"))]
pub use pos_ops::PoseOps;
#[cfg(all(feature = "alloc", not(target_arch = "spirv")))]
pub(crate) use prefetch::prefetch_read;
#[cfg(not(target_arch = "spirv"))]
pub use rotation_ops::RotationOps;
#[cfg(not(target_arch = "spirv"))]
pub use scalar_type::ScalarType;
pub use simd_real_copy::SimdRealCopy;
pub use simd_select::SimdSelect;

#[cfg(not(target_arch = "spirv"))]
pub use angular_inertia_ops::AngularInertiaOps;
pub use cross_product::CrossProduct;
pub use cross_product_matrix::CrossProductMatrix;
pub use dot_product::{DotProduct, SimdLength};
#[allow(unused_imports)]
pub(crate) use fp_flags::DisableFloatingPointExceptionsFlags;

#[cfg(feature = "alloc")]
use crate::math::SIMD_WIDTH;
#[cfg(not(target_arch = "spirv"))]
use crate::math::SimdVector;
use crate::math::{Real, Vector};
#[cfg(all(feature = "dim2", not(target_arch = "spirv")))]
use na::Matrix2;
#[cfg(all(feature = "dim3", not(target_arch = "spirv")))]
use na::Matrix3;

/// Dimension minus one (1 for 2D, 2 for 3D).
#[cfg(feature = "dim2")]
pub const DIM_MINUS_ONE: usize = 1;
/// Dimension minus one (1 for 2D, 2 for 3D).
#[cfg(feature = "dim3")]
pub const DIM_MINUS_ONE: usize = 2;

/// Try to normalize a vector and return both the normalized vector and the original length.
///
/// Returns `None` if the vector's length is below the threshold.
/// This is the glam equivalent of nalgebra's `Unit::try_new_and_get`.
pub fn try_normalize_and_get_length(v: Vector, threshold: Real) -> Option<(Vector, Real)> {
    let len = v.length();
    if len > threshold {
        Some((v / len, len))
    } else {
        None
    }
}

/// Convert glam Vector to nalgebra `SimdVector<Real>`
#[cfg(not(target_arch = "spirv"))]
#[inline]
pub fn vect_to_na(v: Vector) -> SimdVector<Real> {
    v.into()
}

#[cfg(not(target_arch = "spirv"))]
use crate::math::Matrix;

/// Convert glam Matrix to nalgebra `Matrix2<Real>` (2D matrix)
#[cfg(all(feature = "dim2", not(target_arch = "spirv")))]
#[inline]
pub fn mat_to_na(m: Matrix) -> Matrix2<Real> {
    m.into()
}

/// Convert glam Matrix to nalgebra `Matrix3<Real>` (3D matrix)
#[cfg(all(feature = "dim3", not(target_arch = "spirv")))]
#[inline]
pub fn mat_to_na(m: Matrix) -> Matrix3<Real> {
    Matrix3::new(
        m.x_axis.x, m.y_axis.x, m.z_axis.x, m.x_axis.y, m.y_axis.y, m.z_axis.y, m.x_axis.z,
        m.y_axis.z, m.z_axis.z,
    )
}

const INV_EPSILON: Real = 1.0e-20;

#[allow(dead_code)]
pub(crate) fn inv(val: Real) -> Real {
    if (-INV_EPSILON..=INV_EPSILON).contains(&val) {
        0.0
    } else {
        1.0 / val
    }
}

#[allow(dead_code)]
pub(crate) fn simd_inv<N: SimdRealCopy>(val: N) -> N {
    let eps = N::splat(INV_EPSILON);
    N::zero().select(val.simd_gt(-eps) & val.simd_lt(eps), N::one() / val)
}

#[allow(dead_code)]
pub(crate) fn select_other<T: PartialEq>(pair: (T, T), elt: T) -> T {
    if pair.0 == elt { pair.1 } else { pair.0 }
}

/// `Sync`, unless the `unsync-callbacks` feature says otherwise.
#[cfg(not(feature = "unsync-callbacks"))]
pub trait MaybeSync: Sync {}
#[cfg(not(feature = "unsync-callbacks"))]
impl<T: Sync + ?Sized> MaybeSync for T {}

/// See the non-`unsync-callbacks` variant of this trait.
#[cfg(feature = "unsync-callbacks")]
pub trait MaybeSync {}
#[cfg(feature = "unsync-callbacks")]
impl<T: ?Sized> MaybeSync for T {}

/// Removes a key from a [`parry::utils::hashmap::HashMap`] without caring about the
/// resulting entry order.
///
/// The map is an `IndexMap` under `enhanced-determinism` and a `hashbrown` map otherwise,
/// and only the former has (and demands) the order-explicit `swap_remove`. Its order still
/// only depends on the sequence of operations, so swapping stays deterministic.
#[cfg(feature = "alloc")]
pub(crate) fn hashmap_remove<K, V>(
    map: &mut parry::utils::hashmap::HashMap<K, V>,
    key: &K,
) -> Option<V>
where
    K: core::hash::Hash + Eq,
{
    #[cfg(feature = "enhanced-determinism")]
    return map.swap_remove(key);
    #[cfg(not(feature = "enhanced-determinism"))]
    return map.remove(key);
}

/// A raw pointer to an array of `T` that can be shared across threads.
///
/// Safety: this is only sound if each element is accessed by at most one
/// thread at a time (threads own disjoint sets of indices).
#[cfg(feature = "parallel")]
#[derive(Copy, Clone)]
pub(crate) struct SyncPtr<T>(pub *mut T);
#[cfg(feature = "parallel")]
unsafe impl<T: Send> Send for SyncPtr<T> {}
#[cfg(feature = "parallel")]
unsafe impl<T: Send> Sync for SyncPtr<T> {}

#[cfg(feature = "parallel")]
impl<T> SyncPtr<T> {
    /// Pointer to the `i`-th element. Safety: mutating through it is only sound
    /// under the struct-level contract (disjoint per-thread element indices).
    pub(crate) fn add(&self, i: usize) -> *mut T {
        unsafe { self.0.add(i) }
    }
}

/// Calculate the difference with smallest absolute value between the two given values.
pub fn smallest_abs_diff_between_sin_angles<N: SimdRealCopy>(a: N, b: N) -> N {
    // Select the smallest path among the two angles to reach the target.
    let s_err = a - b;
    let sgn = s_err.simd_signum();
    let s_err_complement = s_err - sgn * N::splat(2.0);
    let s_err_is_smallest = s_err.simd_abs().simd_lt(s_err_complement.simd_abs());
    s_err.select(s_err_is_smallest, s_err_complement)
}

/// Calculate the difference with smallest absolute value between the two given angles.
pub fn smallest_abs_diff_between_angles<N: SimdRealCopy>(a: N, b: N) -> N {
    // Select the smallest path among the two angles to reach the target.
    let s_err = a - b;
    let sgn = s_err.simd_signum();
    let s_err_complement = s_err - sgn * N::simd_two_pi();
    let s_err_is_smallest = s_err.simd_abs().simd_lt(s_err_complement.simd_abs());
    s_err.select(s_err_is_smallest, s_err_complement)
}

/// A single solver body's 4-scalar storage block, used to reinterpret a scalar
/// `SolverVel`/`SolverPose`/`SolverContact` as fixed 4-wide chunks for the
/// AoS↔SoA gather/scatter transpose.
///
/// This is deliberately **always** 4 lanes, independent of [`SIMD_WIDTH`]: it
/// describes one body's data layout, not the SIMD lane count. At f32 and the
/// default 4-lane width it is exactly `SimdReal`; at 8 lanes `SimdReal` widens
/// to 256-bit while a per-body block stays 128-bit.
#[cfg(all(feature = "alloc", feature = "f32"))]
pub(crate) type SolverBlock = simba::simd::WideF32x4;
/// See [`SolverBlock`]. `wide::f64x4` is 32-byte aligned, which would over-align
/// a block past the 16-byte AoS rows the scalar structs are laid out in, so the
/// f64 build keeps the plain-array block.
#[cfg(all(feature = "alloc", feature = "f64"))]
pub(crate) type SolverBlock = simba::simd::AutoF64x4;

/// One body's block as the plain array the `aos!` gather hands over — what
/// [`SolverBlock`] wraps, and what the transpose below operates on.
#[cfg(all(feature = "alloc", feature = "f32"))]
pub(crate) type RawBlock = wide::f32x4;
/// See [`RawBlock`].
#[cfg(all(feature = "alloc", feature = "f64"))]
pub(crate) type RawBlock = [Real; 4];

/// A 4x4 block transpose. Pure data movement — no arithmetic — so both
/// implementations below are bit-exact and interchangeable.
#[cfg(all(feature = "alloc", feature = "f32"))]
#[inline(always)]
fn transpose4(data: [RawBlock; 4]) -> [RawBlock; 4] {
    wide::f32x4::transpose(data)
}

/// See [`transpose4`].
#[cfg(all(feature = "alloc", feature = "f64"))]
#[inline(always)]
fn transpose4(data: [RawBlock; 4]) -> [RawBlock; 4] {
    let [
        [a0, a1, a2, a3],
        [b0, b1, b2, b3],
        [c0, c1, c2, c3],
        [d0, d1, d2, d3],
    ] = data;
    [
        [a0, b0, c0, d0],
        [a1, b1, c1, d1],
        [a2, b2, c2, d2],
        [a3, b3, c3, d3],
    ]
}

/// Transposes `SIMD_WIDTH` bodies' blocks (AoS) into 4 SoA lane-vectors, one per
/// float field. Inverse of [`transpose_wide_inv`].
///
/// At 4 lanes this is a single [`transpose4`]. At 8 lanes it does two 4x4
/// transposes (bodies 0–3 / 4–7) and concatenates each field's two halves.
#[cfg(feature = "alloc")]
#[inline(always)]
pub(crate) fn transpose_wide(aos: [RawBlock; SIMD_WIDTH]) -> [crate::math::SimdReal; 4] {
    #[cfg(not(feature = "simd8"))]
    {
        unsafe { core::mem::transmute(transpose4(aos)) }
    }
    #[cfg(feature = "simd8")]
    {
        let lo = transpose4([aos[0], aos[1], aos[2], aos[3]]);
        let hi = transpose4([aos[4], aos[5], aos[6], aos[7]]);
        // Field j spans body 0..8: lanes 0..4 from the low half, 4..8 from the high.
        core::array::from_fn(|j| unsafe {
            core::mem::transmute::<[RawBlock; 2], crate::math::SimdReal>([lo[j], hi[j]])
        })
    }
}

/// Transposes 4 SoA lane-vectors back into `SIMD_WIDTH` bodies' blocks (AoS).
/// Inverse of [`transpose_wide`].
#[cfg(feature = "alloc")]
#[inline(always)]
pub(crate) fn transpose_wide_inv(soa: [crate::math::SimdReal; 4]) -> [RawBlock; SIMD_WIDTH] {
    #[cfg(not(feature = "simd8"))]
    {
        transpose4(unsafe {
            core::mem::transmute::<[crate::math::SimdReal; 4], [RawBlock; 4]>(soa)
        })
    }
    #[cfg(feature = "simd8")]
    {
        // Split each 8-lane field into its low/high 4-lane halves.
        let split: [[RawBlock; 2]; 4] = unsafe { core::mem::transmute(soa) };
        let lo: [RawBlock; 4] = core::array::from_fn(|j| split[j][0]);
        let hi: [RawBlock; 4] = core::array::from_fn(|j| split[j][1]);
        let aos_lo = transpose4(lo); // bodies 0..4
        let aos_hi = transpose4(hi); // bodies 4..8
        core::array::from_fn(|i| if i < 4 { aos_lo[i] } else { aos_hi[i - 4] })
    }
}

/// Helpers around serialization.
#[cfg(feature = "serde-serialize")]
pub mod serde {
    use crate::alloc_prelude::*;
    use core::iter::FromIterator;
    use serde::{Deserialize, Serialize};

    /// Serializes to a `Vec<(K, V)>`.
    ///
    /// Useful for [`std::collections::HashMap`] with a non-string key,
    /// which is unsupported by [`serde_json`](https://docs.rs/serde_json/).
    pub fn serialize_to_vec_tuple<
        'a,
        S: serde::Serializer,
        T: IntoIterator<Item = (&'a K, &'a V)>,
        K: Serialize + 'a,
        V: Serialize + 'a,
    >(
        target: T,
        s: S,
    ) -> Result<S::Ok, S::Error> {
        let container: Vec<_> = target.into_iter().collect();
        serde::Serialize::serialize(&container, s)
    }

    /// Serializes to a `Vec<(K, V)>` ordered by `key`, whatever order the container
    /// iterates in.
    pub fn serialize_sorted_to_vec_tuple<
        'a,
        S: serde::Serializer,
        T: IntoIterator<Item = (&'a K, &'a V)>,
        K: Serialize + 'a,
        V: Serialize + 'a,
        O: Ord,
    >(
        target: T,
        key: impl Fn(&K) -> O,
        s: S,
    ) -> Result<S::Ok, S::Error> {
        let mut container: Vec<_> = target.into_iter().collect();
        container.sort_unstable_by_key(|(a, _)| key(a));
        serde::Serialize::serialize(&container, s)
    }

    /// Deserializes from a `Vec<(K, V)>`.
    ///
    /// Useful for [`std::collections::HashMap`] with a non-string key,
    /// which is unsupported by [`serde_json`](https://docs.rs/serde_json/).
    pub fn deserialize_from_vec_tuple<
        'de,
        D: serde::Deserializer<'de>,
        T: FromIterator<(K, V)>,
        K: Deserialize<'de>,
        V: Deserialize<'de>,
    >(
        d: D,
    ) -> Result<T, D::Error> {
        let hashmap_as_vec: Vec<(K, V)> = Deserialize::deserialize(d)?;
        Ok(T::from_iter(hashmap_as_vec))
    }

    #[cfg(test)]
    mod test {
        use crate::alloc_prelude::*;
        use std::collections::HashMap;

        /// This test uses serde_json because json doesn't support non string
        /// keys in hashmaps, which requires a custom serialization.
        #[test]
        fn serde_json_hashmap() {
            #[derive(Serialize, Deserialize, PartialEq, Eq, Debug)]
            struct Test {
                #[cfg_attr(
                    feature = "serde-serialize",
                    serde(
                        serialize_with = "crate::utils::serde::serialize_to_vec_tuple",
                        deserialize_with = "crate::utils::serde::deserialize_from_vec_tuple"
                    )
                )]
                pub map: HashMap<usize, String>,
            }

            let s = Test {
                map: [(42, "Forty-Two".to_string())].into(),
            };
            let j = serde_json::to_string(&s).unwrap();
            assert_eq!(&j, "{\"map\":[[42,\"Forty-Two\"]]}");
            let p: Test = serde_json::from_str(&j).unwrap();
            assert_eq!(&p, &s);
        }
    }
}
