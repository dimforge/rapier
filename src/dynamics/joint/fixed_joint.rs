use crate::dynamics::{GenericJoint, GenericJointBuilder, JointAxesMask};
use crate::math::{Isometry, Point, Real};

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(transparent)]
pub struct FixedJoint {
    data: GenericJoint,
}

impl Default for FixedJoint {
    fn default() -> Self {
        FixedJoint::new()
    }
}

impl FixedJoint {
    #[must_use]
    pub fn new() -> Self {
        let data = GenericJointBuilder::new(JointAxesMask::LOCKED_FIXED_AXES).build();
        Self { data }
    }

    #[must_use]
    pub fn local_frame1(&self) -> &Isometry<Real> {
        &self.data.local_frame1
    }

    pub fn set_local_frame1(&mut self, local_frame: Isometry<Real>) -> &mut Self {
        self.data.set_local_frame1(local_frame);
        self
    }

    #[must_use]
    pub fn local_frame2(&self) -> &Isometry<Real> {
        &self.data.local_frame2
    }

    pub fn set_local_frame2(&mut self, local_frame: Isometry<Real>) -> &mut Self {
        self.data.set_local_frame2(local_frame);
        self
    }

    #[must_use]
    pub fn local_anchor1(&self) -> Point<Real> {
        self.data.local_anchor1()
    }

    pub fn set_local_anchor1(&mut self, anchor1: Point<Real>) -> &mut Self {
        self.data.set_local_anchor1(anchor1);
        self
    }

    #[must_use]
    pub fn local_anchor2(&self) -> Point<Real> {
        self.data.local_anchor2()
    }

    pub fn set_local_anchor2(&mut self, anchor2: Point<Real>) -> &mut Self {
        self.data.set_local_anchor2(anchor2);
        self
    }
}

impl Into<GenericJoint> for FixedJoint {
    fn into(self) -> GenericJoint {
        self.data
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct FixedJointBuilder(FixedJoint);

impl FixedJointBuilder {
    pub fn new() -> Self {
        Self(FixedJoint::new())
    }

    #[must_use]
    pub fn local_frame1(mut self, local_frame: Isometry<Real>) -> Self {
        self.0.set_local_frame1(local_frame);
        self
    }

    #[must_use]
    pub fn local_frame2(mut self, local_frame: Isometry<Real>) -> Self {
        self.0.set_local_frame2(local_frame);
        self
    }

    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Point<Real>) -> Self {
        self.0.set_local_anchor1(anchor1);
        self
    }

    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Point<Real>) -> Self {
        self.0.set_local_anchor2(anchor2);
        self
    }

    #[must_use]
    pub fn build(self) -> FixedJoint {
        self.0
    }
}

impl Into<GenericJoint> for FixedJointBuilder {
    fn into(self) -> GenericJoint {
        self.0.into()
    }
}
