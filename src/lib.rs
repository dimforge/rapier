//! # Rapier
//!
//! Rapier is a set of two Rust crates `rapier2d` and `rapier3d` for efficient cross-platform
//! physics simulation. It target application include video games, animation, robotics, etc.
//!
//! Rapier has some unique features for collaborative applications:
//! - The ability to snapshot the state of the physics engine, and restore it later.
//! - The ability to run a perfectly deterministic simulation on different machine, as long as they
//! are compliant with the IEEE 754-2008 floating point standard.

#![deny(missing_docs)]

pub extern crate nalgebra as na;
#[cfg(feature = "dim2")]
pub extern crate ncollide2d as ncollide;
#[cfg(feature = "dim3")]
pub extern crate ncollide3d as ncollide;
#[cfg(feature = "serde")]
#[macro_use]
extern crate serde;
#[macro_use]
extern crate approx;
extern crate num_traits as num;
// #[macro_use]
// extern crate array_macro;

#[cfg(feature = "parallel")]
pub use rayon;

#[cfg(all(
    feature = "simd-is-enabled",
    not(feature = "simd-stable"),
    not(feature = "simd-nightly")
))]
std::compile_error!("The `simd-is-enabled` feature should not be enabled explicitly. Please enable the `simd-stable` or the `simd-nightly` feature instead.");
#[cfg(all(feature = "simd-is-enabled", feature = "enhanced-determinism"))]
std::compile_error!(
    "SIMD cannot be enabled when the `enhanced-determinism` feature is also enabled."
);

macro_rules! enable_flush_to_zero(
    () => {
        let _flush_to_zero = crate::utils::FlushToZeroDenormalsAreZeroFlags::flush_denormal_to_zero();
    }
);

macro_rules! array(
    ($callback: expr; SIMD_WIDTH) => {
        {
            #[inline(always)]
            #[allow(dead_code)]
            fn create_arr<T>(mut callback: impl FnMut(usize) -> T) -> [T; SIMD_WIDTH] {
                [callback(0usize), callback(1usize), callback(2usize), callback(3usize)]

                // [callback(0usize), callback(1usize), callback(2usize), callback(3usize),
                //  callback(4usize), callback(5usize), callback(6usize), callback(7usize)]

                // [callback(0usize), callback(1usize), callback(2usize), callback(3usize),
                //  callback(4usize), callback(5usize), callback(6usize), callback(7usize),
                //  callback(8usize), callback(9usize), callback(10usize), callback(11usize),
                //  callback(12usize), callback(13usize), callback(14usize), callback(15usize)]
            }

            create_arr($callback)
        }
    }
);

#[allow(unused_macros)]
macro_rules! par_iter {
    ($t: expr) => {{
        #[cfg(not(feature = "parallel"))]
        let it = $t.iter();

        #[cfg(feature = "parallel")]
        let it = $t.par_iter();
        it
    }};
}

macro_rules! par_iter_mut {
    ($t: expr) => {{
        #[cfg(not(feature = "parallel"))]
        let it = $t.iter_mut();

        #[cfg(feature = "parallel")]
        let it = $t.par_iter_mut();
        it
    }};
}

// macro_rules! par_chunks_mut {
//     ($t: expr, $sz: expr) => {{
//         #[cfg(not(feature = "parallel"))]
//         let it = $t.chunks_mut($sz);
//
//         #[cfg(feature = "parallel")]
//         let it = $t.par_chunks_mut($sz);
//         it
//     }};
// }

#[allow(unused_macros)]
macro_rules! try_ret {
    ($val: expr) => {
        try_ret!($val, ())
    };
    ($val: expr, $ret: expr) => {
        if let Some(val) = $val {
            val
        } else {
            return $ret;
        }
    };
}

// macro_rules! try_continue {
//     ($val: expr) => {
//         if let Some(val) = $val {
//             val
//         } else {
//             continue;
//         }
//     };
// }

pub(crate) const INVALID_U32: u32 = u32::MAX;
pub(crate) const INVALID_U64: u64 = u64::MAX;
pub(crate) const INVALID_USIZE: usize = INVALID_U32 as usize;

pub mod counters;
pub mod data;
pub mod dynamics;
pub mod geometry;
pub mod pipeline;
pub mod utils;

#[cfg(feature = "dim2")]
/// Math primitives used throughout Rapier.
pub mod math {
    pub use super::simd::*;
    use na::{Isometry2, Matrix2, Point2, Translation2, UnitComplex, Vector2, Vector3, U1, U2};

    /// The dimension of the physics simulated by this crate.
    pub const DIM: usize = 2;
    /// The maximum number of point a contact manifold can hold.
    pub const MAX_MANIFOLD_POINTS: usize = 2;
    /// The dimension of the physics simulated by this crate, given as a type-level-integer.
    pub type Dim = U2;
    /// The maximum number of angular degrees of freedom of a rigid body given as a type-level-integer.
    pub type AngDim = U1;
    /// A 2D isometry, i.e., a rotation followed by a translation.
    pub type Isometry<N> = Isometry2<N>;
    /// A 2D vector.
    pub type Vector<N> = Vector2<N>;
    /// A scalar used for angular velocity.
    ///
    /// This is called `AngVector` for coherence with the 3D version of this crate.
    pub type AngVector<N> = N;
    /// A 2D point.
    pub type Point<N> = Point2<N>;
    /// A 2D rotation expressed as an unit complex number.
    pub type Rotation<N> = UnitComplex<N>;
    /// A 2D translation.
    pub type Translation<N> = Translation2<N>;
    /// The angular inertia of a rigid body.
    pub type AngularInertia<N> = N;
    /// The principal angular inertia of a rigid body.
    pub type PrincipalAngularInertia<N> = N;
    /// A matrix that represent the cross product with a given vector.
    pub type CrossMatrix<N> = Vector2<N>;
    /// A 2x2 matrix.
    pub type Matrix<N> = Matrix2<N>;
    /// A vector with a dimension equal to the maximum number of degrees of freedom of a rigid body.
    pub type SpacialVector<N> = Vector3<N>;
    /// A 2D symmetric-definite-positive matrix.
    pub type SdpMatrix<N> = crate::utils::SdpMatrix2<N>;
}

#[cfg(feature = "dim3")]
/// Math primitives used throughout Rapier.
pub mod math {
    pub use super::simd::*;
    use na::{Isometry3, Matrix3, Point3, Translation3, UnitQuaternion, Vector3, Vector6, U3};

    /// The dimension of the physics simulated by this crate.
    pub const DIM: usize = 3;
    /// The maximum number of point a contact manifold can hold.
    pub const MAX_MANIFOLD_POINTS: usize = 4;
    /// The dimension of the physics simulated by this crate, given as a type-level-integer.
    pub type Dim = U3;
    /// The maximum number of angular degrees of freedom of a rigid body given as a type-level-integer.
    pub type AngDim = U3;
    /// A 3D isometry, i.e., a rotation followed by a translation.
    pub type Isometry<N> = Isometry3<N>;
    /// A 3D vector.
    pub type Vector<N> = Vector3<N>;
    /// An axis-angle vector used for angular velocity.
    pub type AngVector<N> = Vector3<N>;
    /// A 3D point.
    pub type Point<N> = Point3<N>;
    /// A 3D rotation expressed as an unit quaternion.
    pub type Rotation<N> = UnitQuaternion<N>;
    /// A 3D translation.
    pub type Translation<N> = Translation3<N>;
    /// The angular inertia of a rigid body.
    pub type AngularInertia<N> = crate::utils::SdpMatrix3<N>;
    /// The principal angular inertia of a rigid body.
    pub type PrincipalAngularInertia<N> = Vector3<N>;
    /// A matrix that represent the cross product with a given vector.
    pub type CrossMatrix<N> = Matrix3<N>;
    /// A 3x3 matrix.
    pub type Matrix<N> = Matrix3<N>;
    /// A vector with a dimension equal to the maximum number of degrees of freedom of a rigid body.
    pub type SpacialVector<N> = Vector6<N>;
    /// A 3D symmetric-definite-positive matrix.
    pub type SdpMatrix<N> = crate::utils::SdpMatrix3<N>;
}

#[cfg(not(feature = "simd-is-enabled"))]
mod simd {
    use simba::simd::{AutoBoolx4, AutoF32x4};
    /// The number of lanes of a SIMD number.
    pub const SIMD_WIDTH: usize = 4;
    /// SIMD_WIDTH - 1
    pub const SIMD_LAST_INDEX: usize = 3;
    /// A SIMD float with SIMD_WIDTH lanes.
    pub type SimdFloat = AutoF32x4;
    /// A SIMD bool with SIMD_WIDTH lanes.
    pub type SimdBool = AutoBoolx4;
}

#[cfg(feature = "simd-is-enabled")]
mod simd {
    #[allow(unused_imports)]
    #[cfg(feature = "simd-nightly")]
    use simba::simd::{f32x16, f32x4, f32x8, m32x16, m32x4, m32x8, u8x16, u8x4, u8x8};
    #[cfg(feature = "simd-stable")]
    use simba::simd::{WideBoolF32x4, WideF32x4};

    /// The number of lanes of a SIMD number.
    pub const SIMD_WIDTH: usize = 4;
    /// SIMD_WIDTH - 1
    pub const SIMD_LAST_INDEX: usize = 3;
    #[cfg(not(feature = "simd-nightly"))]
    /// A SIMD float with SIMD_WIDTH lanes.
    pub type SimdFloat = WideF32x4;
    #[cfg(not(feature = "simd-nightly"))]
    /// A SIMD bool with SIMD_WIDTH lanes.
    pub type SimdBool = WideBoolF32x4;
    #[cfg(feature = "simd-nightly")]
    /// A SIMD float with SIMD_WIDTH lanes.
    pub type SimdFloat = f32x4;
    #[cfg(feature = "simd-nightly")]
    /// A bool float with SIMD_WIDTH lanes.
    pub type SimdBool = m32x4;

    // pub const SIMD_WIDTH: usize = 8;
    // pub const SIMD_LAST_INDEX: usize = 7;
    // pub type SimdFloat = f32x8;
    // pub type SimdBool = m32x8;

    // pub const SIMD_WIDTH: usize = 16;
    // pub const SIMD_LAST_INDEX: usize = 15;
    // pub type SimdFloat = f32x16;
    // pub type SimdBool = m32x16;
}
