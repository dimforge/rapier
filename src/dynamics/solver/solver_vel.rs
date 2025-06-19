use crate::math::{AngVector, SPATIAL_DIM, Vector};
use crate::utils::SimdRealCopy;
use na::{DVectorView, DVectorViewMut, Scalar};
use std::ops::{AddAssign, Sub, SubAssign};

#[derive(Copy, Clone, Debug, Default)]
#[repr(C)]
//#[repr(align(64))]
pub struct SolverVel<N: Scalar + Copy> {
    // The linear velocity of a solver body.
    pub linear: Vector<N>,
    // The angular velocity, multiplied by the inverse sqrt angular inertia, of a solver body.
    pub angular: AngVector<N>,
}

impl<N: Scalar + Copy> SolverVel<N> {
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

impl<N: SimdRealCopy> SolverVel<N> {
    pub fn zero() -> Self {
        Self {
            linear: na::zero(),
            angular: na::zero(),
        }
    }
}

impl<N: SimdRealCopy> AddAssign for SolverVel<N> {
    fn add_assign(&mut self, rhs: Self) {
        self.linear += rhs.linear;
        self.angular += rhs.angular;
    }
}

impl<N: SimdRealCopy> SubAssign for SolverVel<N> {
    fn sub_assign(&mut self, rhs: Self) {
        self.linear -= rhs.linear;
        self.angular -= rhs.angular;
    }
}

impl<N: SimdRealCopy> Sub for SolverVel<N> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        SolverVel {
            linear: self.linear - rhs.linear,
            angular: self.angular - rhs.angular,
        }
    }
}
