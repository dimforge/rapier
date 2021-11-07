use crate::dynamics::{
    IntegrationParameters, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition, SpringJoint,
};
use crate::math::{AngularInertia, Isometry, Point, Real, Rotation};
use crate::utils::{WAngularInertia, WCross};

#[derive(Debug)]
pub(crate) struct SpringPositionConstraint {
    position1: usize,
    position2: usize,

    local_com1: Point<Real>,
    local_com2: Point<Real>,

    im1: Real,
    im2: Real,

    ii1: AngularInertia<Real>,
    ii2: AngularInertia<Real>,

    local_anchor1: Point<Real>,
    local_anchor2: Point<Real>,

    limits_enabled: bool,
    limits_min_length: Real,
    limits_max_length: Real,
}

impl SpringPositionConstraint {
    pub fn from_params(
        rb1: (&RigidBodyMassProps, &RigidBodyIds),
        rb2: (&RigidBodyMassProps, &RigidBodyIds),
        cparams: &SpringJoint,
    ) -> Self {
        let (mprops1, ids1) = rb1;
        let (mprops2, ids2) = rb2;

        let ii1 = mprops1.effective_world_inv_inertia_sqrt.squared();
        let ii2 = mprops2.effective_world_inv_inertia_sqrt.squared();

        Self {
            local_com1: mprops1.local_mprops.local_com,
            local_com2: mprops2.local_mprops.local_com,
            im1: mprops1.effective_inv_mass,
            im2: mprops2.effective_inv_mass,
            ii1,
            ii2,
            local_anchor1: cparams.local_anchor1,
            local_anchor2: cparams.local_anchor2,
            position1: ids1.active_set_offset,
            position2: ids2.active_set_offset,
            limits_enabled: cparams.limits_enabled,
            limits_min_length: cparams.limits_min_length,
            limits_max_length: cparams.limits_max_length,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        if !self.limits_enabled {
            return;
        }
        let mut position1 = positions[self.position1 as usize];
        let mut position2 = positions[self.position2 as usize];

        let anchor1 = position1 * self.local_anchor1;
        let anchor2 = position2 * self.local_anchor2;

        let com1 = position1 * self.local_com1;
        let com2 = position2 * self.local_com2;

        let centered_anchor1 = anchor1 - com1;
        let centered_anchor2 = anchor2 - com2;

        let u = anchor2 - anchor1;
        let length = u.magnitude();
        let u = u.normalize();

        let c: Real;
        if self.limits_min_length >= self.limits_max_length || length < self.limits_min_length {
            c = length - self.limits_min_length;
        } else if length > self.limits_max_length {
            c = length - self.limits_max_length;
        } else {
            return;
        }

        let cr1u = centered_anchor1.gcross(u);
        let cr2u = centered_anchor2.gcross(u);

        let im1 = self.im1;
        let im2 = self.im2;
        let ii1 = self.ii1;
        let ii2 = self.ii2;

        let lhs: Real;
        #[cfg(feature = "dim2")]
        {
            lhs = im1 + im2 + ii1 * cr1u * cr1u + ii2 * cr2u * cr2u;
        }
        #[cfg(feature = "dim3")]
        {
            let inv_i1: Real = (cr1u.transpose() * ii1.into_matrix() * cr1u)[0];
            let inv_i2: Real = (cr2u.transpose() * ii2.into_matrix() * cr2u)[0];
            lhs = im1 + im2 + inv_i1 + inv_i2;
        }
        let limits_inv_lhs = crate::utils::inv(lhs);

        let impulse = -limits_inv_lhs * c * params.joint_erp;
        let impulse = impulse * u;

        position1.translation.vector -= im1 * impulse;
        position2.translation.vector += im2 * impulse;

        let angle1 = ii1.transform_vector(centered_anchor1.gcross(-impulse));
        let angle2 = ii2.transform_vector(centered_anchor2.gcross(impulse));

        position1.rotation = Rotation::new(angle1) * position1.rotation;
        position2.rotation = Rotation::new(angle2) * position2.rotation;

        positions[self.position1 as usize] = position1;
        positions[self.position2 as usize] = position2;
    }
}

#[derive(Debug)]
pub(crate) struct SpringPositionGroundConstraint {
    position2: usize,
    anchor1: Point<Real>,
    im2: Real,
    ii2: AngularInertia<Real>,
    local_anchor2: Point<Real>,
    local_com2: Point<Real>,

    limits_enabled: bool,
    limits_min_length: Real,
    limits_max_length: Real,
}

impl SpringPositionGroundConstraint {
    pub fn from_params(
        rb1: &RigidBodyPosition,
        rb2: (&RigidBodyMassProps, &RigidBodyIds),
        cparams: &SpringJoint,
        flipped: bool,
    ) -> Self {
        let poss1 = rb1;
        let (mprops2, ids2) = rb2;

        if flipped {
            // Note the only thing that is flipped here
            // are the local_anchors. The rb1 and rb2 have
            // already been flipped by the caller.
            Self {
                anchor1: poss1.next_position * cparams.local_anchor2,
                im2: mprops2.effective_inv_mass,
                ii2: mprops2.effective_world_inv_inertia_sqrt.squared(),
                local_anchor2: cparams.local_anchor1,
                position2: ids2.active_set_offset,
                local_com2: mprops2.local_mprops.local_com,
                limits_enabled: cparams.limits_enabled,
                limits_min_length: cparams.limits_min_length,
                limits_max_length: cparams.limits_max_length,
            }
        } else {
            Self {
                anchor1: poss1.next_position * cparams.local_anchor1,
                im2: mprops2.effective_inv_mass,
                ii2: mprops2.effective_world_inv_inertia_sqrt.squared(),
                local_anchor2: cparams.local_anchor2,
                position2: ids2.active_set_offset,
                local_com2: mprops2.local_mprops.local_com,
                limits_enabled: cparams.limits_enabled,
                limits_min_length: cparams.limits_min_length,
                limits_max_length: cparams.limits_max_length,
            }
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        if !self.limits_enabled {
            return;
        }
        let mut position2 = positions[self.position2 as usize];

        let anchor2 = position2 * self.local_anchor2;
        let com2 = position2 * self.local_com2;

        let centered_anchor2 = anchor2 - com2;

        let u = anchor2 - self.anchor1;
        let length = u.magnitude();
        let u = u.normalize();

        let c: Real;
        if self.limits_min_length >= self.limits_max_length || length < self.limits_min_length {
            c = length - self.limits_min_length;
        } else if length > self.limits_max_length {
            c = length - self.limits_max_length;
        } else {
            return;
        }

        let cr2u = centered_anchor2.gcross(u);

        let im2 = self.im2;
        let ii2 = self.ii2;

        let lhs: Real;
        #[cfg(feature = "dim2")]
        {
            lhs = im2 + ii2 * cr2u * cr2u;
        }
        #[cfg(feature = "dim3")]
        {
            let inv_i2: Real = (cr2u.transpose() * ii2.into_matrix() * cr2u)[0];
            lhs = im2 + inv_i2;
        }
        let limits_inv_lhs = crate::utils::inv(lhs);

        let impulse = -limits_inv_lhs * c * params.joint_erp;
        let impulse = impulse * u;

        position2.translation.vector += im2 * impulse;

        let angle2 = ii2.transform_vector(centered_anchor2.gcross(impulse));

        position2.rotation = Rotation::new(angle2) * position2.rotation;

        positions[self.position2 as usize] = position2;
    }
}
