use crate::math::{AngVector, Vector, SPATIAL_DIM};
use na::{DVectorSlice, DVectorSliceMut};
use na::{Scalar, SimdRealField};
use std::ops::AddAssign;

#[derive(Copy, Clone, Debug)]
#[repr(C)]
//#[repr(align(64))]
pub struct DeltaVel<N: Scalar + Copy> {
    pub linear: Vector<N>,
    pub angular: AngVector<N>,
}

impl<N: Scalar + Copy> DeltaVel<N> {
    pub fn as_slice(&self) -> &[N; SPATIAL_DIM] {
        unsafe { std::mem::transmute(self) }
    }

    pub fn as_mut_slice(&mut self) -> &mut [N; SPATIAL_DIM] {
        unsafe { std::mem::transmute(self) }
    }

    pub fn as_vector_slice(&self) -> DVectorSlice<N> {
        DVectorSlice::from_slice(&self.as_slice()[..], SPATIAL_DIM)
    }

    pub fn as_vector_slice_mut(&mut self) -> DVectorSliceMut<N> {
        DVectorSliceMut::from_slice(&mut self.as_mut_slice()[..], SPATIAL_DIM)
    }
}

impl<N: SimdRealField + Copy> DeltaVel<N> {
    pub fn zero() -> Self {
        Self {
            linear: na::zero(),
            angular: na::zero(),
        }
    }
}

impl<N: SimdRealField + Copy> AddAssign for DeltaVel<N> {
    fn add_assign(&mut self, rhs: Self) {
        self.linear += rhs.linear;
        self.angular += rhs.angular;
    }
}
