use crate::dynamics::{FixedJoint, IntegrationParameters, RigidBody};
use crate::math::{AngularInertia, Isometry, Point, Real, Rotation};
use crate::utils::WAngularInertia;

#[derive(Debug)]
pub(crate) struct FixedPositionConstraint {
    position1: usize,
    position2: usize,
    local_anchor1: Isometry<Real>,
    local_anchor2: Isometry<Real>,
    local_com1: Point<Real>,
    local_com2: Point<Real>,
    im1: Real,
    im2: Real,
    ii1: AngularInertia<Real>,
    ii2: AngularInertia<Real>,

    lin_inv_lhs: Real,
    ang_inv_lhs: AngularInertia<Real>,
}

impl FixedPositionConstraint {
    pub fn from_params(rb1: &RigidBody, rb2: &RigidBody, cparams: &FixedJoint) -> Self {
        let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let im1 = rb1.effective_inv_mass;
        let im2 = rb2.effective_inv_mass;
        let lin_inv_lhs = 1.0 / (im1 + im2);
        let ang_inv_lhs = (ii1 + ii2).inverse();

        Self {
            local_anchor1: cparams.local_anchor1,
            local_anchor2: cparams.local_anchor2,
            position1: rb1.active_set_offset,
            position2: rb2.active_set_offset,
            im1,
            im2,
            ii1,
            ii2,
            local_com1: rb1.mass_properties.local_com,
            local_com2: rb2.mass_properties.local_com,
            lin_inv_lhs,
            ang_inv_lhs,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position1 = positions[self.position1 as usize];
        let mut position2 = positions[self.position2 as usize];

        // Angular correction.
        let anchor1 = position1 * self.local_anchor1;
        let anchor2 = position2 * self.local_anchor2;
        let ang_err = anchor2.rotation * anchor1.rotation.inverse();
        #[cfg(feature = "dim3")]
        let ang_impulse = self
            .ang_inv_lhs
            .transform_vector(ang_err.scaled_axis() * params.joint_erp);
        #[cfg(feature = "dim2")]
        let ang_impulse = self
            .ang_inv_lhs
            .transform_vector(ang_err.angle() * params.joint_erp);
        position1.rotation =
            Rotation::new(self.ii1.transform_vector(ang_impulse)) * position1.rotation;
        position2.rotation =
            Rotation::new(self.ii2.transform_vector(-ang_impulse)) * position2.rotation;

        // Linear correction.
        let anchor1 = position1 * Point::from(self.local_anchor1.translation.vector);
        let anchor2 = position2 * Point::from(self.local_anchor2.translation.vector);
        let err = anchor2 - anchor1;
        let impulse = err * (self.lin_inv_lhs * params.joint_erp);
        position1.translation.vector += self.im1 * impulse;
        position2.translation.vector -= self.im2 * impulse;

        positions[self.position1 as usize] = position1;
        positions[self.position2 as usize] = position2;
    }
}

#[derive(Debug)]
pub(crate) struct FixedPositionGroundConstraint {
    position2: usize,
    anchor1: Isometry<Real>,
    local_anchor2: Isometry<Real>,
    local_com2: Point<Real>,
    im2: Real,
    ii2: AngularInertia<Real>,
    impulse: Real,
}

impl FixedPositionGroundConstraint {
    pub fn from_params(
        rb1: &RigidBody,
        rb2: &RigidBody,
        cparams: &FixedJoint,
        flipped: bool,
    ) -> Self {
        let anchor1;
        let local_anchor2;

        if flipped {
            anchor1 = rb1.next_position * cparams.local_anchor2;
            local_anchor2 = cparams.local_anchor1;
        } else {
            anchor1 = rb1.next_position * cparams.local_anchor1;
            local_anchor2 = cparams.local_anchor2;
        };

        Self {
            anchor1,
            local_anchor2,
            position2: rb2.active_set_offset,
            im2: rb2.effective_inv_mass,
            ii2: rb2.effective_world_inv_inertia_sqrt.squared(),
            local_com2: rb2.mass_properties.local_com,
            impulse: 0.0,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position2 = positions[self.position2 as usize];

        // Angular correction.
        let anchor2 = position2 * self.local_anchor2;
        let ang_err = anchor2.rotation * self.anchor1.rotation.inverse();
        position2.rotation = ang_err.powf(-params.joint_erp) * position2.rotation;

        // Linear correction.
        let anchor1 = Point::from(self.anchor1.translation.vector);
        let anchor2 = position2 * Point::from(self.local_anchor2.translation.vector);
        let err = anchor2 - anchor1;
        // NOTE: no need to divide by im2 just to multiply right after.
        let impulse = err * params.joint_erp;
        position2.translation.vector -= impulse;

        positions[self.position2 as usize] = position2;
    }
}
