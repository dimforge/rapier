use crate::dynamics::{
    IntegrationParameters, PrismaticJoint, RigidBodyIds, RigidBodyMassProps, RigidBodyPosition,
};
use crate::math::{AngularInertia, Isometry, Point, Real, Rotation, Vector};
use crate::utils::WAngularInertia;
use na::Unit;

#[derive(Debug)]
pub(crate) struct PrismaticPositionConstraint {
    position1: usize,
    position2: usize,

    im1: Real,
    im2: Real,

    ii1: AngularInertia<Real>,
    ii2: AngularInertia<Real>,

    lin_inv_lhs: Real,
    ang_inv_lhs: AngularInertia<Real>,

    limits: [Real; 2],

    local_frame1: Isometry<Real>,
    local_frame2: Isometry<Real>,

    local_axis1: Unit<Vector<Real>>,
    local_axis2: Unit<Vector<Real>>,
}

impl PrismaticPositionConstraint {
    pub fn from_params(
        rb1: (&RigidBodyMassProps, &RigidBodyIds),
        rb2: (&RigidBodyMassProps, &RigidBodyIds),
        cparams: &PrismaticJoint,
    ) -> Self {
        let (mprops1, ids1) = rb1;
        let (mprops2, ids2) = rb2;

        let ii1 = mprops1.effective_world_inv_inertia_sqrt.squared();
        let ii2 = mprops2.effective_world_inv_inertia_sqrt.squared();
        let im1 = mprops1.effective_inv_mass;
        let im2 = mprops2.effective_inv_mass;
        let lin_inv_lhs = 1.0 / (im1 + im2);
        let ang_inv_lhs = (ii1 + ii2).inverse();

        Self {
            im1,
            im2,
            ii1,
            ii2,
            lin_inv_lhs,
            ang_inv_lhs,
            local_frame1: cparams.local_frame1(),
            local_frame2: cparams.local_frame2(),
            local_axis1: cparams.local_axis1,
            local_axis2: cparams.local_axis2,
            position1: ids1.active_set_offset,
            position2: ids2.active_set_offset,
            limits: cparams.limits,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position1 = positions[self.position1 as usize];
        let mut position2 = positions[self.position2 as usize];

        // Angular correction.
        let frame1 = position1 * self.local_frame1;
        let frame2 = position2 * self.local_frame2;
        let ang_err = frame2.rotation * frame1.rotation.inverse();
        #[cfg(feature = "dim2")]
        let ang_impulse = self
            .ang_inv_lhs
            .transform_vector(ang_err.angle() * params.joint_erp);
        #[cfg(feature = "dim3")]
        let ang_impulse = self
            .ang_inv_lhs
            .transform_vector(ang_err.scaled_axis() * params.joint_erp);
        position1.rotation =
            Rotation::new(self.ii1.transform_vector(ang_impulse)) * position1.rotation;
        position2.rotation =
            Rotation::new(self.ii2.transform_vector(-ang_impulse)) * position2.rotation;

        // Linear correction.
        let anchor1 = position1 * Point::from(self.local_frame1.translation.vector);
        let anchor2 = position2 * Point::from(self.local_frame2.translation.vector);
        let axis1 = position1 * self.local_axis1;
        let dpos = anchor2 - anchor1;
        let limit_err = dpos.dot(&axis1);
        let mut err = dpos - *axis1 * limit_err;

        if limit_err < self.limits[0] {
            err += *axis1 * (limit_err - self.limits[0]);
        } else if limit_err > self.limits[1] {
            err += *axis1 * (limit_err - self.limits[1]);
        }

        let impulse = err * (self.lin_inv_lhs * params.joint_erp);
        position1.translation.vector += self.im1 * impulse;
        position2.translation.vector -= self.im2 * impulse;

        positions[self.position1 as usize] = position1;
        positions[self.position2 as usize] = position2;
    }
}

#[derive(Debug)]
pub(crate) struct PrismaticPositionGroundConstraint {
    position2: usize,
    frame1: Isometry<Real>,
    local_frame2: Isometry<Real>,
    axis1: Unit<Vector<Real>>,
    local_axis2: Unit<Vector<Real>>,
    limits: [Real; 2],
}

impl PrismaticPositionGroundConstraint {
    pub fn from_params(
        rb1: &RigidBodyPosition,
        rb2: (&RigidBodyMassProps, &RigidBodyIds),
        cparams: &PrismaticJoint,
        flipped: bool,
    ) -> Self {
        let poss1 = rb1;
        let (_, ids2) = rb2;

        let frame1;
        let local_frame2;
        let axis1;
        let local_axis2;

        if flipped {
            frame1 = poss1.next_position * cparams.local_frame2();
            local_frame2 = cparams.local_frame1();
            axis1 = poss1.next_position * cparams.local_axis2;
            local_axis2 = cparams.local_axis1;
        } else {
            frame1 = poss1.next_position * cparams.local_frame1();
            local_frame2 = cparams.local_frame2();
            axis1 = poss1.next_position * cparams.local_axis1;
            local_axis2 = cparams.local_axis2;
        };

        Self {
            frame1,
            local_frame2,
            axis1,
            local_axis2,
            position2: ids2.active_set_offset,
            limits: cparams.limits,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position2 = positions[self.position2 as usize];

        // Angular correction.
        let frame2 = position2 * self.local_frame2;
        let ang_err = frame2.rotation * self.frame1.rotation.inverse();
        position2.rotation = ang_err.powf(-params.joint_erp) * position2.rotation;

        // Linear correction.
        let anchor1 = Point::from(self.frame1.translation.vector);
        let anchor2 = position2 * Point::from(self.local_frame2.translation.vector);
        let dpos = anchor2 - anchor1;
        let limit_err = dpos.dot(&self.axis1);
        let mut err = dpos - *self.axis1 * limit_err;

        if limit_err < self.limits[0] {
            err += *self.axis1 * (limit_err - self.limits[0]);
        } else if limit_err > self.limits[1] {
            err += *self.axis1 * (limit_err - self.limits[1]);
        }

        // NOTE: no need to divide by im2 just to multiply right after.
        let impulse = err * params.joint_erp;
        position2.translation.vector -= impulse;

        positions[self.position2 as usize] = position2;
    }
}
