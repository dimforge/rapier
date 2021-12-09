use crate::dynamics::{
    IntegrationParameters, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition, SpringJoint,
};
use crate::math::{
    AngularInertia, Isometry, Point, Real, Rotation, SimdBool, SimdReal, Translation, SIMD_WIDTH,
};
use crate::utils::{WAngularInertia, WCross};
use simba::simd::{SimdBool as _, SimdPartialOrd, SimdValue};

#[derive(Debug)]
pub(crate) struct WSpringPositionConstraint {
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

    limits_active: bool,
    limits_min_length: SimdReal,
    limits_max_length: SimdReal,
}

impl WSpringPositionConstraint {
    pub fn from_params(
        rbs1: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        rbs2: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        cparams: [&SpringJoint; SIMD_WIDTH],
    ) -> Self {
        let (mprops1, ids1) = rbs1;
        let (mprops2, ids2) = rbs2;

        let local_com1 = Point::from(gather![|ii| mprops1[ii].local_mprops.local_com]);
        let im1 = SimdReal::from(gather![|ii| mprops1[ii].effective_inv_mass]);
        let ii1 = AngularInertia::<SimdReal>::from(gather![|ii| mprops1[ii]
            .effective_world_inv_inertia_sqrt
            .squared()]);
        let local_anchor1 = Point::from(gather![|ii| cparams[ii].local_anchor1]);
        let position1 = gather![|ii| ids1[ii].active_set_offset];

        let local_com2 = Point::from(gather![|ii| mprops2[ii].local_mprops.local_com]);
        let im2 = SimdReal::from(gather![|ii| mprops2[ii].effective_inv_mass]);
        let ii2 = AngularInertia::<SimdReal>::from(gather![|ii| mprops2[ii]
            .effective_world_inv_inertia_sqrt
            .squared()]);
        let local_anchor2 = Point::from(gather![|ii| cparams[ii].local_anchor2]);
        let position2 = gather![|ii| ids2[ii].active_set_offset];

        let limits_min_length = SimdReal::from(gather![|ii| cparams[ii].limits_min_length]);
        let limits_max_length = SimdReal::from(gather![|ii| cparams[ii].limits_max_length]);
        let limits_enabled = SimdBool::from(gather![|ii| cparams[ii].limits_enabled]);
        let limits_active =
            limits_enabled.any() && limits_min_length.simd_lt(limits_max_length).any();

        Self {
            position1,
            position2,
            local_com1,
            local_com2,
            im1,
            im2,
            ii1,
            ii2,
            local_anchor1,
            local_anchor2,
            limits_active,
            limits_min_length,
            limits_max_length,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        if !self.limits_active {
            return;
        }

        let mut position1: Isometry<SimdReal> = Isometry {
            rotation: Rotation::from(gather![|ii| positions[self.position1[ii] as usize].rotation]),
            translation: Translation::from(gather![
                |ii| positions[self.position1[ii] as usize].translation
            ]),
        };

        let mut position2: Isometry<SimdReal> = Isometry {
            rotation: Rotation::from(gather![|ii| positions[self.position2[ii] as usize].rotation]),
            translation: Translation::from(gather![
                |ii| positions[self.position2[ii] as usize].translation
            ]),
        };

        let anchor1 = position1 * self.local_anchor1;
        let anchor2 = position2 * self.local_anchor2;

        let com1 = position1 * self.local_com1;
        let com2 = position2 * self.local_com2;

        let centered_anchor1 = anchor1 - com1;
        let centered_anchor2 = anchor2 - com2;

        let u = anchor2 - anchor1;
        let length = u.magnitude();
        let u = u.normalize();

        let c = SimdReal::from(gather![|ii| {
            let min_length = self.limits_min_length.extract(ii);
            let max_length = self.limits_max_length.extract(ii);
            let length = length.extract(ii);
            if min_length >= max_length || length < min_length {
                length - min_length
            } else if length > max_length {
                length - max_length
            } else {
                0.0
            }
        }]);

        let cr1u = centered_anchor1.gcross(u);
        let cr2u = centered_anchor2.gcross(u);

        let im1 = self.im1;
        let im2 = self.im2;
        let ii1 = self.ii1;
        let ii2 = self.ii2;

        let lhs: SimdReal;
        #[cfg(feature = "dim2")]
        {
            lhs = im1 + im2 + ii1 * cr1u * cr1u + ii2 * cr2u * cr2u;
        }
        #[cfg(feature = "dim3")]
        {
            let inv_i1: SimdReal = (cr1u.transpose() * ii1.into_matrix() * cr1u)[0];
            let inv_i2: SimdReal = (cr2u.transpose() * ii2.into_matrix() * cr2u)[0];
            lhs = im1 + im2 + inv_i1 + inv_i2;
        }
        let limits_inv_lhs = crate::utils::simd_inv(lhs);

        let impulse = -limits_inv_lhs * c * SimdReal::splat(params.joint_erp);
        let impulse = u * impulse;

        position1.translation.vector -= impulse * im1;
        position2.translation.vector += impulse * im2;

        let angle1 = ii1.transform_vector(centered_anchor1.gcross(-impulse));
        let angle2 = ii2.transform_vector(centered_anchor2.gcross(impulse));

        position1.rotation = Rotation::new(angle1) * position1.rotation;
        position2.rotation = Rotation::new(angle2) * position2.rotation;

        for ii in 0..SIMD_WIDTH {
            positions[self.position1[ii] as usize].translation = position1.translation.extract(ii);
            positions[self.position1[ii] as usize].rotation = position2.rotation.extract(ii);
        }

        for ii in 0..SIMD_WIDTH {
            positions[self.position2[ii] as usize].translation = position2.translation.extract(ii);
            positions[self.position2[ii] as usize].rotation = position2.rotation.extract(ii);
        }
    }
}

#[derive(Debug)]
pub(crate) struct WSpringPositionGroundConstraint {
    position2: [usize; SIMD_WIDTH],
    anchor1: Point<SimdReal>,
    im2: SimdReal,
    ii2: AngularInertia<SimdReal>,
    local_anchor2: Point<SimdReal>,
    local_com2: Point<SimdReal>,

    limits_active: bool,
    limits_min_length: SimdReal,
    limits_max_length: SimdReal,
}

impl WSpringPositionGroundConstraint {
    pub fn from_params(
        rbs1: [&RigidBodyPosition; SIMD_WIDTH],
        rbs2: (
            [&RigidBodyMassProps; SIMD_WIDTH],
            [&RigidBodyIds; SIMD_WIDTH],
        ),
        cparams: [&SpringJoint; SIMD_WIDTH],
        flipped: [bool; SIMD_WIDTH],
    ) -> Self {
        let poss1 = rbs1;
        let (mprops2, ids2) = rbs2;

        let local_com2 = Point::from(gather![|ii| mprops2[ii].local_mprops.local_com]);
        let im2 = SimdReal::from(gather![|ii| mprops2[ii].effective_inv_mass]);
        let ii2 = AngularInertia::<SimdReal>::from(gather![|ii| mprops2[ii]
            .effective_world_inv_inertia_sqrt
            .squared()]);
        let local_anchor2 = Point::<SimdReal>::from(gather![|ii| cparams[ii].local_anchor2]);
        let position2 = gather![|ii| ids2[ii].active_set_offset];

        let poss1_next_pos = Isometry::from(gather![|ii| poss1[ii].next_position]);
        let local_anchor1 = Point::<SimdReal>::from(gather![|ii| cparams[ii].local_anchor1]);

        let local_anchor = Point::from(gather![|ii| if flipped[ii] {
            local_anchor1.extract(ii)
        } else {
            local_anchor2.extract(ii)
        }]);

        let other_anchor = Point::from(gather![|ii| if flipped[ii] {
            local_anchor2.extract(ii)
        } else {
            local_anchor1.extract(ii)
        }]);

        let anchor1 = poss1_next_pos * other_anchor;

        let limits_min_length = SimdReal::from(gather![|ii| cparams[ii].limits_min_length]);
        let limits_max_length = SimdReal::from(gather![|ii| cparams[ii].limits_max_length]);
        let limits_enabled = SimdBool::from(gather![|ii| cparams[ii].limits_enabled]);
        let limits_active =
            limits_enabled.any() && limits_min_length.simd_lt(limits_max_length).any();

        Self {
            position2,
            anchor1,
            im2,
            ii2,
            local_anchor2: local_anchor,
            local_com2,

            limits_active,
            limits_min_length,
            limits_max_length,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        if !self.limits_active {
            return;
        }

        let mut position2: Isometry<SimdReal> = Isometry {
            rotation: Rotation::from(gather![|ii| positions[self.position2[ii] as usize].rotation]),
            translation: Translation::from(gather![
                |ii| positions[self.position2[ii] as usize].translation
            ]),
        };

        let anchor2 = position2 * self.local_anchor2;
        let com2 = position2 * self.local_com2;

        let centered_anchor2 = anchor2 - com2;

        let u = anchor2 - self.anchor1;
        let length = u.magnitude();
        let u = u.normalize();

        let c = SimdReal::from(gather![|ii| {
            let min_length = self.limits_min_length.extract(ii);
            let max_length = self.limits_max_length.extract(ii);
            let length = length.extract(ii);
            if min_length >= max_length || length < min_length {
                length - min_length
            } else if length > max_length {
                length - max_length
            } else {
                0.0
            }
        }]);

        let cr2u = centered_anchor2.gcross(u);

        let im2 = self.im2;
        let ii2 = self.ii2;

        let lhs: SimdReal;
        #[cfg(feature = "dim2")]
        {
            lhs = im2 + ii2 * cr2u * cr2u;
        }
        #[cfg(feature = "dim3")]
        {
            let inv_i2: SimdReal = (cr2u.transpose() * ii2.into_matrix() * cr2u)[0];
            lhs = im2 + inv_i2;
        }
        let limits_inv_lhs = crate::utils::simd_inv(lhs);

        let impulse = -limits_inv_lhs * c * SimdReal::splat(params.joint_erp);
        let impulse = u * impulse;

        position2.translation.vector += impulse * im2;

        let angle2 = ii2.transform_vector(centered_anchor2.gcross(impulse));

        position2.rotation = Rotation::new(angle2) * position2.rotation;

        for ii in 0..SIMD_WIDTH {
            positions[self.position2[ii] as usize].translation = position2.translation.extract(ii);
            positions[self.position2[ii] as usize].rotation = position2.rotation.extract(ii);
        }
    }
}
