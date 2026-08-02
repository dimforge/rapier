use crate::alloc_prelude::*;
use crate::dynamics::RigidBody;
use crate::math::{Real, SPATIAL_DIM, Vector};
use crate::utils::{RotationOps, ScalarType};
use core::ops::{AddAssign, Sub, SubAssign};
use na::{DVectorView, DVectorViewMut};
use parry::math::{Pose, Rotation, SIMD_WIDTH, SimdReal};

use crate::utils::{SolverBlock, transpose_wide, transpose_wide_inv};

macro_rules! aos(
    ($data_repr: ident [ $idx: ident ] . $data_n: ident, $fallback: ident) => {
        // One 128-bit block per lane (`SIMD_WIDTH` inferred from the consumer).
        core::array::from_fn(|k| {
            if ($idx[k] as usize) < $data_repr.len() {
                $data_repr[$idx[k] as usize].$data_n.0
            } else {
                $fallback.$data_n.0
            }
        })
    }
);

macro_rules! scatter(
    ($data: ident [ $idx: ident [ $i: expr ] ] = [$($aos: ident),*]) => {
       unsafe {
            #[allow(clippy::missing_transmute_annotations)] // Different macro calls transmute to different types
            if ($idx[$i] as usize) < $data.len() {
                $data[$idx[$i] as usize] = core::mem::transmute([$($aos[$i]),*]);
            }
        }
    }
);

/// Per-solver-body flag: bypass the angular speed cap. Kept in a
/// `SolverBodies.flags` byte parallel to `vels`/`poses` so the scalar integrate-positions loop
/// can read it without disturbing the SIMD-gathered `SolverVel`/`SolverPose` layouts.
pub(crate) const SOLVER_BODY_ALLOW_FAST_ROTATION: u8 = 1;

#[derive(Default)]
pub struct SolverBodies {
    pub vels: Vec<SolverVel<Real>>,
    pub poses: Vec<SolverPose<Real>>,
    /// Per-body flag bytes (see `SOLVER_BODY_ALLOW_FAST_ROTATION`), indexed like `vels`/`poses`.
    pub flags: Vec<u8>,
}

impl SolverBodies {
    pub fn clear(&mut self) {
        self.vels.clear();
        self.poses.clear();
        self.flags.clear();
    }

    pub fn resize(&mut self, sz: usize) {
        self.vels.resize(sz, Default::default());
        self.poses.resize(sz, Default::default());
        self.flags.resize(sz, 0);
    }

    pub fn len(&self) -> usize {
        self.vels.len()
    }

    /// Panics if any non-world (`!= u32::MAX`) solver-body id is out of range. Part of the
    /// `solver-bounds-checks` guards: a stale id (e.g. a contact-graph maintenance bug) would
    /// silently gather the world-body fallback instead of the real body — this turns that into
    /// a clean panic. Called once per chunk at constraint generation (NOT per solve iteration).
    #[cfg(feature = "solver-bounds-checks")]
    #[inline]
    pub fn assert_ids_in_range(&self, idx: [u32; SIMD_WIDTH]) {
        let len = self.len();
        for id in idx {
            assert!(
                id == u32::MAX || (id as usize) < len,
                "stale solver-body id {id} (solver bodies: {len}) — solver contact graph corruption"
            );
        }
    }

    // TODO: add a SIMD version?
    pub fn copy_from(&mut self, i: usize, rb: &RigidBody) {
        let poses = &mut self.poses[i];
        let vels = &mut self.vels[i];

        self.flags[i] = if rb.ccd.allow_fast_rotation {
            SOLVER_BODY_ALLOW_FAST_ROTATION
        } else {
            0
        };

        // NOTE: gyroscopic forces (3D) are applied per-substep by the staged
        // solver's velocity-increment stage, not here.
        #[cfg(feature = "dim2")]
        {
            vels.angular = rb.vels.angvel;
        }
        #[cfg(feature = "dim3")]
        {
            vels.angular = rb.angvel();
        }
        vels.linear = rb.vels.linvel;

        let pose = rb
            .pos
            .position
            .prepend_translation(rb.mprops.local_mprops.local_com);
        poses.rotation = pose.rotation;
        poses.translation = pose.translation;

        // A sleeping body only reaches the solver as a frontier solver body (partial-island
        // sleep): a kinematic-like read-only wall at its real pose, so its mass properties
        // must read as infinite despite its dynamic body type.
        if rb.is_dynamic_or_kinematic() && !rb.is_sleeping() {
            poses.ii = rb.mprops.effective_world_inv_inertia;
            poses.im = rb.mprops.effective_inv_mass;
        } else {
            poses.ii = Default::default();
            poses.im = Default::default();
        }
    }

    #[inline]
    pub fn gather_vels(&self, idx: [u32; SIMD_WIDTH]) -> SolverVel<SimdReal> {
        SolverVel::gather(&self.vels, idx)
    }

    #[inline]
    pub fn get_vel(&self, i: u32) -> SolverVel<Real> {
        self.vels.get(i as usize).copied().unwrap_or_default()
    }

    #[inline]
    pub fn scatter_vels(&mut self, idx: [u32; SIMD_WIDTH], vels: SolverVel<SimdReal>) {
        vels.scatter(&mut self.vels, idx);
    }

    #[inline]
    pub fn set_vel(&mut self, i: u32, vel: SolverVel<Real>) {
        if (i as usize) < self.vels.len() {
            self.vels[i as usize] = vel;
        }
    }

    #[inline]
    pub fn get_pose(&self, i: u32) -> SolverPose<Real> {
        self.poses.get(i as usize).copied().unwrap_or_default()
    }

    #[inline]
    pub fn gather_poses(&self, idx: [u32; SIMD_WIDTH]) -> SolverPose<SimdReal> {
        SolverPose::gather(&self.poses, idx)
    }

    /// Gathers only the transform part (rotation + translation) of the solver poses — half (2D)
    /// or less (3D) of [`Self::gather_poses`]'s transposition work; for kernels that keep the
    /// (step-constant) mass properties in their own constraint storage.
    #[inline]
    pub fn gather_transforms(&self, idx: [u32; SIMD_WIDTH]) -> SolverTransform<SimdReal> {
        SolverTransform::gather(&self.poses, idx)
    }
}

// Total 7/13
#[repr(C)]
#[repr(align(16))]
#[derive(Copy, Clone, Default)]
pub struct SolverVel<T: ScalarType> {
    pub linear: T::Vector,     // 2/3
    pub angular: T::AngVector, // 1/3
    // TODO: explicit padding are useful for static assertions.
    //       But might be wasteful for the SolverVel<SimdReal>
    //       specialization.
    #[cfg(feature = "dim2")]
    padding: [T; 1],
    #[cfg(feature = "dim3")]
    padding: [T; 2],
}

#[repr(C)]
struct SolverVelRepr {
    data0: SolverBlock,
    #[cfg(feature = "dim3")]
    data1: SolverBlock,
}

impl SolverVelRepr {
    pub fn zero() -> Self {
        Self {
            data0: na::zero(),
            #[cfg(feature = "dim3")]
            data1: na::zero(),
        }
    }
}

impl SolverVel<SimdReal> {
    #[inline]
    pub fn gather(data: &[SolverVel<Real>], idx: [u32; SIMD_WIDTH]) -> Self {
        // TODO: double-check that the compiler is using simd loads and
        //       isn’t generating useless copies.

        let zero = SolverVelRepr::zero();
        let data_repr: &[SolverVelRepr] = unsafe { core::mem::transmute(data) };

        #[cfg(feature = "dim2")]
        {
            let aos = aos!(data_repr[idx].data0, zero);
            let soa = transpose_wide(aos);
            unsafe { core::mem::transmute(soa) }
        }

        #[cfg(feature = "dim3")]
        {
            let aos0 = aos!(data_repr[idx].data0, zero);
            let soa0 = transpose_wide(aos0);
            let aos1 = aos!(data_repr[idx].data1, zero);
            let soa1 = transpose_wide(aos1);
            unsafe { core::mem::transmute((soa0, soa1)) }
        }
    }

    #[inline]
    #[cfg(feature = "dim2")]
    pub fn scatter(self, data: &mut [SolverVel<Real>], idx: [u32; SIMD_WIDTH]) {
        // TODO: double-check that the compiler is using simd loads and no useless copies.
        let soa: [SimdReal; 4] = unsafe { core::mem::transmute(self) };
        let aos = transpose_wide_inv(soa);
        for i in 0..SIMD_WIDTH {
            scatter!(data[idx[i]] = [aos]);
        }
    }

    #[inline]
    #[cfg(feature = "dim3")]
    pub fn scatter(self, data: &mut [SolverVel<Real>], idx: [u32; SIMD_WIDTH]) {
        let soa: [[SimdReal; 4]; 2] = unsafe { core::mem::transmute(self) };
        // TODO: double-check that the compiler is using simd loads and no useless copies.
        let aos0 = transpose_wide_inv(soa[0]);
        let aos1 = transpose_wide_inv(soa[1]);
        for i in 0..SIMD_WIDTH {
            scatter!(data[idx[i]] = [aos0, aos1]);
        }
    }
}

// Total: 7/16
#[repr(C)]
#[repr(align(16))]
#[derive(Copy, Clone)]
pub struct SolverPose<N: ScalarType> {
    /// Positional change of the rigid-body’s center of mass.
    pub rotation: N::Rotation, // 2/4
    pub translation: N::Vector, // 2/3
    pub ii: N::AngInertia,      // 1/6
    pub im: N::Vector,          // 2/3
    #[cfg(feature = "dim2")]
    pub padding: [N; 1],
}

impl SolverPose<Real> {
    pub fn pose(&self) -> Pose {
        Pose::from_parts(self.translation, self.rotation)
    }
}

impl SolverPose<SimdReal> {
    pub fn pose(&self) -> <SimdReal as ScalarType>::Pose {
        <SimdReal as ScalarType>::Pose::from_parts(self.translation.into(), self.rotation)
    }
}

impl<N: ScalarType> SolverPose<N> {
    #[inline]
    pub fn transform_point(&self, pt: N::Vector) -> N::Vector {
        self.rotation * pt + self.translation
    }

    #[inline]
    pub fn inverse_transform_point(&self, pt: N::Vector) -> N::Vector {
        self.rotation.inverse() * (pt - self.translation)
    }
}

/// The transform part of a [`SolverPose`], gathered without its mass properties. The field
/// order deliberately matches the head of [`SolverPose`] so the SIMD gather can transpose only
/// the leading pose blocks.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct SolverTransform<N: ScalarType> {
    pub rotation: N::Rotation,
    pub translation: N::Vector,
}

impl<N: ScalarType> SolverTransform<N> {
    #[inline]
    pub fn transform_point(&self, pt: N::Vector) -> N::Vector {
        self.rotation * pt + self.translation
    }
}

impl SolverTransform<SimdReal> {
    #[inline]
    pub fn gather(data: &[SolverPose<Real>], idx: [u32; SIMD_WIDTH]) -> Self {
        let identity = SolverPoseRepr::identity();
        let data_repr: &[SolverPoseRepr] = unsafe { core::mem::transmute(data) };

        #[cfg(feature = "dim2")]
        {
            // In 2D the (rotation, translation) pair is exactly the first SIMD
            // block of the pose: one transposition instead of two.
            let aos0 = aos!(data_repr[idx].data0, identity);
            let soa0 = transpose_wide(aos0);
            unsafe { core::mem::transmute(soa0) }
        }

        #[cfg(feature = "dim3")]
        {
            // In 3D the quaternion + translation live in the first two SIMD
            // blocks of the pose: two transpositions instead of four.
            let aos0 = aos!(data_repr[idx].data0, identity);
            let aos1 = aos!(data_repr[idx].data1, identity);
            let soa0 = transpose_wide(aos0);
            let soa1 = transpose_wide(aos1);

            #[repr(C)]
            struct TransformAndPad {
                transform: SolverTransform<SimdReal>,
                // First inertia element sharing the second pose block.
                _ii_xx: SimdReal,
            }
            let repr: TransformAndPad = unsafe { core::mem::transmute([soa0, soa1]) };
            repr.transform
        }
    }
}

impl Default for SolverPose<Real> {
    #[inline]
    fn default() -> Self {
        Self {
            rotation: Rotation::IDENTITY,
            translation: Vector::ZERO,
            ii: Default::default(),
            im: Default::default(),
            #[cfg(feature = "dim2")]
            padding: Default::default(),
        }
    }
}

#[repr(C)]
struct SolverPoseRepr {
    data0: SolverBlock,
    data1: SolverBlock,
    #[cfg(feature = "dim3")]
    data2: SolverBlock,
    #[cfg(feature = "dim3")]
    data3: SolverBlock,
}

impl SolverPoseRepr {
    pub fn identity() -> Self {
        // TODO PERF: will the compiler handle this efficiently and generate
        //            everything at compile-time?
        unsafe { core::mem::transmute(SolverPose::default()) }
    }
}

impl SolverPose<SimdReal> {
    #[inline]
    pub fn gather(data: &[SolverPose<Real>], idx: [u32; SIMD_WIDTH]) -> Self {
        // TODO: double-check that the compiler is using simd loads and
        //       isn’t generating useless copies.

        let identity = SolverPoseRepr::identity();
        let data_repr: &[SolverPoseRepr] = unsafe { core::mem::transmute(data) };

        #[cfg(feature = "dim2")]
        {
            let aos0 = aos!(data_repr[idx].data0, identity);
            let aos1 = aos!(data_repr[idx].data1, identity);
            let soa0 = transpose_wide(aos0);
            let soa1 = transpose_wide(aos1);
            unsafe { core::mem::transmute([soa0, soa1]) }
        }

        #[cfg(feature = "dim3")]
        {
            let aos0 = aos!(data_repr[idx].data0, identity);
            let aos1 = aos!(data_repr[idx].data1, identity);
            let aos2 = aos!(data_repr[idx].data2, identity);
            let aos3 = aos!(data_repr[idx].data3, identity);
            let soa0 = transpose_wide(aos0);
            let soa1 = transpose_wide(aos1);
            let soa2 = transpose_wide(aos2);
            let soa3 = transpose_wide(aos3);
            unsafe { core::mem::transmute([soa0, soa1, soa2, soa3]) }
        }
    }
}

impl<N: ScalarType> SolverVel<N> {
    pub fn as_slice(&self) -> &[N; SPATIAL_DIM] {
        unsafe { core::mem::transmute(self) }
    }

    pub fn as_mut_slice(&mut self) -> &mut [N; SPATIAL_DIM] {
        unsafe { core::mem::transmute(self) }
    }

    pub fn as_vector_slice(&self) -> DVectorView<'_, N> {
        DVectorView::from_slice(&self.as_slice()[..], SPATIAL_DIM)
    }

    pub fn as_vector_slice_mut(&mut self) -> DVectorViewMut<'_, N> {
        DVectorViewMut::from_slice(&mut self.as_mut_slice()[..], SPATIAL_DIM)
    }
}

impl<N: ScalarType> SolverVel<N> {
    pub fn zero() -> Self {
        Self {
            linear: Default::default(),
            angular: Default::default(),
            #[cfg(feature = "dim2")]
            padding: [na::zero(); 1],
            #[cfg(feature = "dim3")]
            padding: [na::zero(); 2],
        }
    }
}

impl<N: ScalarType> AddAssign for SolverVel<N> {
    fn add_assign(&mut self, rhs: Self) {
        self.linear += rhs.linear;
        self.angular += rhs.angular;
    }
}

impl<N: ScalarType> SubAssign for SolverVel<N> {
    fn sub_assign(&mut self, rhs: Self) {
        self.linear -= rhs.linear;
        self.angular -= rhs.angular;
    }
}

impl<N: ScalarType> Sub for SolverVel<N> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        SolverVel {
            linear: self.linear - rhs.linear,
            angular: self.angular - rhs.angular,
            padding: self.padding,
        }
    }
}
