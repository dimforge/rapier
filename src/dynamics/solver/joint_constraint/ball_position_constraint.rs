use crate::dynamics::{BallJoint, IntegrationParameters, RigidBody};
#[cfg(feature = "dim2")]
use crate::math::SdpMatrix;
use crate::math::{AngularInertia, Isometry, Point, Real, Rotation};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};

#[derive(Debug)]
pub(crate) struct BallPositionConstraint {
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
}

impl BallPositionConstraint {
    pub fn from_params(rb1: &RigidBody, rb2: &RigidBody, cparams: &BallJoint) -> Self {
        Self {
            local_com1: rb1.mass_properties.local_com,
            local_com2: rb2.mass_properties.local_com,
            im1: rb1.effective_inv_mass,
            im2: rb2.effective_inv_mass,
            ii1: rb1.effective_world_inv_inertia_sqrt.squared(),
            ii2: rb2.effective_world_inv_inertia_sqrt.squared(),
            local_anchor1: cparams.local_anchor1,
            local_anchor2: cparams.local_anchor2,
            position1: rb1.active_set_offset,
            position2: rb2.active_set_offset,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position1 = positions[self.position1 as usize];
        let mut position2 = positions[self.position2 as usize];

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
        // it's just easier that way. It is also
        // faster because in 2D lhs will be symmetric.
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
        let impulse = inv_lhs * -(err * params.joint_erp);

        position1.translation.vector += self.im1 * impulse;
        position2.translation.vector -= self.im2 * impulse;

        let angle1 = self.ii1.transform_vector(centered_anchor1.gcross(impulse));
        let angle2 = self.ii2.transform_vector(centered_anchor2.gcross(-impulse));

        position1.rotation = Rotation::new(angle1) * position1.rotation;
        position2.rotation = Rotation::new(angle2) * position2.rotation;

        positions[self.position1 as usize] = position1;
        positions[self.position2 as usize] = position2;
    }
}

#[derive(Debug)]
pub(crate) struct BallPositionGroundConstraint {
    position2: usize,
    anchor1: Point<Real>,
    im2: Real,
    ii2: AngularInertia<Real>,
    local_anchor2: Point<Real>,
    local_com2: Point<Real>,
}

impl BallPositionGroundConstraint {
    pub fn from_params(
        rb1: &RigidBody,
        rb2: &RigidBody,
        cparams: &BallJoint,
        flipped: bool,
    ) -> Self {
        if flipped {
            // Note the only thing that is flipped here
            // are the local_anchors. The rb1 and rb2 have
            // already been flipped by the caller.
            Self {
                anchor1: rb1.next_position * cparams.local_anchor2,
                im2: rb2.effective_inv_mass,
                ii2: rb2.effective_world_inv_inertia_sqrt.squared(),
                local_anchor2: cparams.local_anchor1,
                position2: rb2.active_set_offset,
                local_com2: rb2.mass_properties.local_com,
            }
        } else {
            Self {
                anchor1: rb1.next_position * cparams.local_anchor1,
                im2: rb2.effective_inv_mass,
                ii2: rb2.effective_world_inv_inertia_sqrt.squared(),
                local_anchor2: cparams.local_anchor2,
                position2: rb2.active_set_offset,
                local_com2: rb2.mass_properties.local_com,
            }
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position2 = positions[self.position2 as usize];

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
        let impulse = inv_lhs * -(err * params.joint_erp);
        position2.translation.vector -= self.im2 * impulse;

        let angle2 = self.ii2.transform_vector(centered_anchor2.gcross(-impulse));
        position2.rotation = Rotation::new(angle2) * position2.rotation;
        positions[self.position2 as usize] = position2;
    }
}
