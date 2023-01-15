use crate::math::{AngVector, Vector, SPATIAL_DIM};
use crate::utils::WReal;
use na::{DVectorView, DVectorViewMut, Scalar};
use std::ops::{AddAssign, Sub};

#[derive(Copy, Clone, Debug, Default)]
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

    pub fn as_vector_slice(&self) -> DVectorView<N> {
        DVectorView::from_slice(&self.as_slice()[..], SPATIAL_DIM)
    }

    pub fn as_vector_slice_mut(&mut self) -> DVectorViewMut<N> {
        DVectorViewMut::from_slice(&mut self.as_mut_slice()[..], SPATIAL_DIM)
    }
}

impl<N: WReal> DeltaVel<N> {
    pub fn zero() -> Self {
        Self {
            linear: na::zero(),
            angular: na::zero(),
        }
    }
}

impl<N: WReal> AddAssign for DeltaVel<N> {
    fn add_assign(&mut self, rhs: Self) {
        self.linear += rhs.linear;
        self.angular += rhs.angular;
    }
}

impl<N: WReal> Sub for DeltaVel<N> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        DeltaVel {
            linear: self.linear - rhs.linear,
            angular: self.angular - rhs.angular,
        }
    }
}
