use crate::dynamics::{
    BallJoint, IntegrationParameters, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition,
};
#[cfg(feature = "dim2")]
use crate::math::SdpMatrix;
use crate::math::{AngularInertia, Isometry, Point, Real, Rotation, SimdReal, SIMD_WIDTH};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
use simba::simd::SimdValue;

#[derive(Debug)]
pub(crate) struct WBallPositionConstraint {
    position1: [usize; SIMD_WIDTH],
    position2: [usize; SIMD_WIDTH],

    local_com1: Point<SimdReal>,
    local_com2: Point<SimdReal>,

    im1: SimdReal,
    im2: SimdReal,

    ii1: AngularInertia<SimdReal>,
    ii2: AngularInertia<SimdReal>,

    local_anchor1: Point<SimdReal>,
    local_anchor2: Point<SimdReal>,
}

impl WBallPositionConstraint {
    pub fn from_params(
        rbs1: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        rbs2: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        cparams: [&BallJoint; SIMD_WIDTH],
    ) -> Self {
        let (mprops1, ids1) = rbs1;
        let (mprops2, ids2) = rbs2;

        let local_com1 = Point::from(gather![|ii| mprops1[ii].local_mprops.local_com]);
        let local_com2 = Point::from(gather![|ii| mprops2[ii].local_mprops.local_com]);
        let im1 = SimdReal::from(gather![|ii| mprops1[ii].effective_inv_mass]);
        let im2 = SimdReal::from(gather![|ii| mprops2[ii].effective_inv_mass]);
        let ii1 = AngularInertia::<SimdReal>::from(gather![
            |ii| mprops1[ii].effective_world_inv_inertia_sqrt
        ])
        .squared();
        let ii2 = AngularInertia::<SimdReal>::from(gather![
            |ii| mprops2[ii].effective_world_inv_inertia_sqrt
        ])
        .squared();
        let local_anchor1 = Point::from(gather![|ii| cparams[ii].local_anchor1]);
        let local_anchor2 = Point::from(gather![|ii| cparams[ii].local_anchor2]);
        let position1 = gather![|ii| ids1[ii].active_set_offset];
        let position2 = gather![|ii| ids2[ii].active_set_offset];

        Self {
            local_com1,
            local_com2,
            im1,
            im2,
            ii1,
            ii2,
            local_anchor1,
            local_anchor2,
            position1,
            position2,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position1 = Isometry::from(gather![|ii| positions[self.position1[ii]]]);
        let mut position2 = Isometry::from(gather![|ii| positions[self.position2[ii]]]);

        let anchor1 = position1 * self.local_anchor1;
        let anchor2 = position2 * self.local_anchor2;

        let com1 = position1 * self.local_com1;
        let com2 = position2 * self.local_com2;

        let err = anchor1 - anchor2;

        let centered_anchor1 = anchor1 - com1;
        let centered_anchor2 = anchor2 - com2;

        let cmat1 = centered_anchor1.gcross_matrix();
        let cmat2 = centered_anchor2.gcross_matrix();

        // NOTE: the -cmat1 is just a simpler way of doing cmat1.transpose()
        // because it is anti-symmetric.
        #[cfg(feature = "dim3")]
        let lhs = self.ii1.quadform(&cmat1).add_diagonal(self.im1)
            + self.ii2.quadform(&cmat2).add_diagonal(self.im2);

        // In 2D we just unroll the computation because
        // it's just easier that way.
        #[cfg(feature = "dim2")]
        let lhs = {
            let m11 =
                self.im1 + self.im2 + cmat1.x * cmat1.x * self.ii1 + cmat2.x * cmat2.x * self.ii2;
            let m12 = cmat1.x * cmat1.y * self.ii1 + cmat2.x * cmat2.y * self.ii2;
            let m22 =
                self.im1 + self.im2 + cmat1.y * cmat1.y * self.ii1 + cmat2.y * cmat2.y * self.ii2;
            SdpMatrix::new(m11, m12, m22)
        };

        let inv_lhs = lhs.inverse_unchecked();
        let impulse = inv_lhs * -(err * SimdReal::splat(params.joint_erp));

        position1.translation.vector += impulse * self.im1;
        position2.translation.vector -= impulse * self.im2;

        let angle1 = self.ii1.transform_vector(centered_anchor1.gcross(impulse));
        let angle2 = self.ii2.transform_vector(centered_anchor2.gcross(-impulse));

        position1.rotation = Rotation::new(angle1) * position1.rotation;
        position2.rotation = Rotation::new(angle2) * position2.rotation;

        for ii in 0..SIMD_WIDTH {
            positions[self.position1[ii]] = position1.extract(ii);
        }
        for ii in 0..SIMD_WIDTH {
            positions[self.position2[ii]] = position2.extract(ii);
        }
    }
}

#[derive(Debug)]
pub(crate) struct WBallPositionGroundConstraint {
    position2: [usize; SIMD_WIDTH],
    anchor1: Point<SimdReal>,
    im2: SimdReal,
    ii2: AngularInertia<SimdReal>,
    local_anchor2: Point<SimdReal>,
    local_com2: Point<SimdReal>,
}

impl WBallPositionGroundConstraint {
    pub fn from_params(
        rbs1: [&RigidBodyPosition; SIMD_WIDTH],
        rbs2: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        cparams: [&BallJoint; SIMD_WIDTH],
        flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        let poss1 = rbs1;
        let (mprops2, ids2) = rbs2;

        let position1 = Isometry::from(gather![|ii| poss1[ii].next_position]);
        let anchor1 = position1
            * Point::from(gather![|ii| if flipped[ii] {
                cparams[ii].local_anchor2
            } else {
                cparams[ii].local_anchor1
            }]);
        let im2 = SimdReal::from(gather![|ii| mprops2[ii].effective_inv_mass]);
        let ii2 = AngularInertia::<SimdReal>::from(gather![
            |ii| mprops2[ii].effective_world_inv_inertia_sqrt
        ])
        .squared();
        let local_anchor2 = Point::from(gather![|ii| if flipped[ii] {
            cparams[ii].local_anchor1
        } else {
            cparams[ii].local_anchor2
        }]);
        let position2 = gather![|ii| ids2[ii].active_set_offset];
        let local_com2 = Point::from(gather![|ii| mprops2[ii].local_mprops.local_com]);

        Self {
            anchor1,
            im2,
            ii2,
            local_anchor2,
            position2,
            local_com2,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position2 = Isometry::from(gather![|ii| positions[self.position2[ii]]]);

        let anchor2 = position2 * self.local_anchor2;
        let com2 = position2 * self.local_com2;

        let err = self.anchor1 - anchor2;
        let centered_anchor2 = anchor2 - com2;
        let cmat2 = centered_anchor2.gcross_matrix();

        #[cfg(feature = "dim3")]
        let lhs = self.ii2.quadform(&cmat2).add_diagonal(self.im2);

        #[cfg(feature = "dim2")]
        let lhs = {
            let m11 = self.im2 + cmat2.x * cmat2.x * self.ii2;
            let m12 = cmat2.x * cmat2.y * self.ii2;
            let m22 = self.im2 + cmat2.y * cmat2.y * self.ii2;
            SdpMatrix::new(m11, m12, m22)
        };

        let inv_lhs = lhs.inverse_unchecked();
        let impulse = inv_lhs * -(err * SimdReal::splat(params.joint_erp));
        position2.translation.vector -= impulse * self.im2;

        let angle2 = self.ii2.transform_vector(centered_anchor2.gcross(-impulse));
        position2.rotation = Rotation::new(angle2) * position2.rotation;

        for ii in 0..SIMD_WIDTH {
            positions[self.position2[ii]] = position2.extract(ii);
        }
    }
}
