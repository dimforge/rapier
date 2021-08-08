use crate::math::{AngVector, Vector};
use na::{Scalar, SimdRealField};
use std::ops::AddAssign;

#[derive(Copy, Clone, Debug)]
//#[repr(align(64))]
pub(crate) struct DeltaVel<N: Scalar + Copy> {
    pub linear: Vector<N>,
    pub angular: AngVector<N>,
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
