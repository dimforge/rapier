//! # Rapier
//!
//! Rapier is a set of two Rust crates `rapier2d` and `rapier3d` for efficient cross-platform
//! physics simulation. It target application include video games, animation, robotics, etc.
//!
//! Rapier has some unique features for collaborative applications:
//! - The ability to snapshot the state of the physics engine, and restore it later.
//! - The ability to run a perfectly deterministic simulation on different machine, as long as they
//!   are compliant with the IEEE 754-2008 floating point standard.
//!
//! User documentation for Rapier is on [the official Rapier site](https://rapier.rs/docs/).

#![deny(bare_trait_objects)]
#![warn(missing_docs)]
#![allow(clippy::too_many_arguments)]
#![allow(clippy::needless_range_loop)] // TODO: remove this? I find that in the math code using indices adds clarity.
#![allow(clippy::module_inception)]

#[cfg(all(feature = "dim2", feature = "f32"))]
pub extern crate parry2d as parry;
#[cfg(all(feature = "dim2", feature = "f64"))]
pub extern crate parry2d_f64 as parry;
#[cfg(all(feature = "dim3", feature = "f32"))]
pub extern crate parry3d as parry;
#[cfg(all(feature = "dim3", feature = "f64"))]
pub extern crate parry3d_f64 as parry;

pub extern crate nalgebra as na;
#[cfg(feature = "serde-serialize")]
#[macro_use]
extern crate serde;
extern crate num_traits as num;

#[cfg(feature = "parallel")]
pub use rayon;

#[cfg(all(
    feature = "simd-is-enabled",
    not(feature = "simd-stable"),
    not(feature = "simd-nightly")
))]
std::compile_error!(
    "The `simd-is-enabled` feature should not be enabled explicitly. Please enable the `simd-stable` or the `simd-nightly` feature instead."
);
#[cfg(all(feature = "simd-is-enabled", feature = "enhanced-determinism"))]
std::compile_error!(
    "SIMD cannot be enabled when the `enhanced-determinism` feature is also enabled."
);

macro_rules! enable_flush_to_zero(
    () => {
        let _flush_to_zero = crate::utils::FlushToZeroDenormalsAreZeroFlags::flush_denormal_to_zero();
    }
);

#[cfg(feature = "simd-is-enabled")]
macro_rules! gather(
    ($callback: expr) => {
        {
            #[inline(always)]
            #[allow(dead_code)]
            fn create_arr<T>(mut callback: impl FnMut(usize) -> T) -> [T; SIMD_WIDTH] {
                [callback(0usize), callback(1usize), callback(2usize), callback(3usize)]
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
pub(crate) const INVALID_USIZE: usize = INVALID_U32 as usize;

/// The string version of Rapier.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

pub mod control;
pub mod counters;
pub mod data;
pub mod dynamics;
pub mod geometry;
pub mod pipeline;
pub mod utils;

/// Elementary mathematical entities (vectors, matrices, isometries, etc).
pub mod math {
    pub use parry::math::*;

    /*
     * 2D
     */
    /// Max number of pairs of contact points from the same
    /// contact manifold that can be solved as part of a
    /// single contact constraint.
    #[cfg(feature = "dim2")]
    pub const MAX_MANIFOLD_POINTS: usize = 2;

    /// The type of a constraint Jacobian in twist coordinates.
    #[cfg(feature = "dim2")]
    pub type Jacobian<N> = na::Matrix3xX<N>;

    /// The type of a slice of the constraint Jacobian in twist coordinates.
    #[cfg(feature = "dim2")]
    pub type JacobianView<'a, N> = na::MatrixView3xX<'a, N>;

    /// The type of a mutable slice of the constraint Jacobian in twist coordinates.
    #[cfg(feature = "dim2")]
    pub type JacobianViewMut<'a, N> = na::MatrixViewMut3xX<'a, N>;

    /// The type of impulse applied for friction constraints.
    #[cfg(feature = "dim2")]
    pub type TangentImpulse<N> = na::Vector1<N>;

    /// The maximum number of possible rotations and translations of a rigid body.
    #[cfg(feature = "dim2")]
    pub const SPATIAL_DIM: usize = 3;

    /// The maximum number of rotational degrees of freedom of a rigid-body.
    #[cfg(feature = "dim2")]
    pub const ANG_DIM: usize = 1;

    /*
     * 3D
     */
    /// Max number of pairs of contact points from the same
    /// contact manifold that can be solved as part of a
    /// single contact constraint.
    #[cfg(feature = "dim3")]
    pub const MAX_MANIFOLD_POINTS: usize = 4;

    /// The type of a constraint Jacobian in twist coordinates.
    #[cfg(feature = "dim3")]
    pub type Jacobian<N> = na::Matrix6xX<N>;

    /// The type of a slice of the constraint Jacobian in twist coordinates.
    #[cfg(feature = "dim3")]
    pub type JacobianView<'a, N> = na::MatrixView6xX<'a, N>;

    /// The type of a mutable slice of the constraint Jacobian in twist coordinates.
    #[cfg(feature = "dim3")]
    pub type JacobianViewMut<'a, N> = na::MatrixViewMut6xX<'a, N>;

    /// The type of impulse applied for friction constraints.
    #[cfg(feature = "dim3")]
    pub type TangentImpulse<N> = na::Vector2<N>;

    /// The maximum number of possible rotations and translations of a rigid body.
    #[cfg(feature = "dim3")]
    pub const SPATIAL_DIM: usize = 6;

    /// The maximum number of rotational degrees of freedom of a rigid-body.
    #[cfg(feature = "dim3")]
    pub const ANG_DIM: usize = 3;
}

/// Prelude containing the common types defined by Rapier.
pub mod prelude {
    pub use crate::dynamics::*;
    pub use crate::geometry::*;
    pub use crate::math::*;
    pub use crate::pipeline::*;
    pub use na::{DMatrix, DVector, point, vector};
    pub extern crate nalgebra;
}
