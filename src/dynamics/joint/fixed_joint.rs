use crate::dynamics::{JointAxesMask, JointData};
use crate::math::{Isometry, Point, Real};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FixedJoint {
    data: JointData,
}

impl FixedJoint {
    pub fn new() -> Self {
        #[cfg(feature = "dim2")]
        let mask = JointAxesMask::X | JointAxesMask::Y | JointAxesMask::ANG_X;
        #[cfg(feature = "dim3")]
        let mask = JointAxesMask::X
            | JointAxesMask::Y
            | JointAxesMask::Z
            | JointAxesMask::ANG_X
            | JointAxesMask::ANG_Y
            | JointAxesMask::ANG_Z;

        let data = JointData::default().lock_axes(mask);
        Self { data }
    }

    #[must_use]
    pub fn local_frame1(mut self, local_frame: Isometry<Real>) -> Self {
        self.data = self.data.local_frame1(local_frame);
        self
    }

    #[must_use]
    pub fn local_frame2(mut self, local_frame: Isometry<Real>) -> Self {
        self.data = self.data.local_frame2(local_frame);
        self
    }

    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Point<Real>) -> Self {
        self.data = self.data.local_anchor1(anchor1);
        self
    }

    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Point<Real>) -> Self {
        self.data = self.data.local_anchor2(anchor2);
        self
    }
}

impl Into<JointData> for FixedJoint {
    fn into(self) -> JointData {
        self.data
    }
}
