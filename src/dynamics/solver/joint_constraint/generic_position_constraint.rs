use super::{GenericVelocityConstraint, GenericVelocityGroundConstraint};
use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{GenericJoint, IntegrationParameters};
use crate::math::{
    AngDim, AngVector, AngularInertia, Dim, Isometry, Point, Real, Rotation, SpatialVector, Vector,
    DIM,
};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};
use na::{Vector3, Vector6};

// FIXME: review this code for the case where the center of masses are not the origin.
#[derive(Debug)]
pub(crate) struct GenericPositionConstraint {
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

    joint: GenericJoint,
}

impl GenericPositionConstraint {
    pub fn from_params(rb1: &RigidBody, rb2: &RigidBody, joint: &GenericJoint) -> Self {
        let ii1 = rb1.effective_world_inv_inertia_sqrt.squared();
        let ii2 = rb2.effective_world_inv_inertia_sqrt.squared();
        let im1 = rb1.effective_inv_mass;
        let im2 = rb2.effective_inv_mass;

        Self {
            local_anchor1: joint.local_anchor1,
            local_anchor2: joint.local_anchor2,
            position1: rb1.active_set_offset,
            position2: rb2.active_set_offset,
            im1,
            im2,
            ii1,
            ii2,
            local_com1: rb1.local_mprops.local_com,
            local_com2: rb2.local_mprops.local_com,
            joint: *joint,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position1 = positions[self.position1];
        let mut position2 = positions[self.position2];
        let mut params = *params;
        params.joint_erp = 0.8;

        /*
         *
         * Translation part.
         *
         */
        {
            let anchor1 = position1 * self.joint.local_anchor1;
            let anchor2 = position2 * self.joint.local_anchor2;
            let basis = anchor1.rotation;
            let r1 = Point::from(anchor1.translation.vector) - position1 * self.local_com1;
            let r2 = Point::from(anchor2.translation.vector) - position2 * self.local_com2;
            let mut min_pos_impulse = self.joint.min_pos_impulse.xyz();
            let mut max_pos_impulse = self.joint.max_pos_impulse.xyz();

            let pos_rhs = GenericVelocityConstraint::compute_lin_position_error(
                &anchor1,
                &anchor2,
                &basis,
                &self.joint.min_position.xyz(),
                &self.joint.max_position.xyz(),
            ) * params.joint_erp;

            for i in 0..3 {
                if pos_rhs[i] < 0.0 {
                    min_pos_impulse[i] = -Real::MAX;
                }
                if pos_rhs[i] > 0.0 {
                    max_pos_impulse[i] = Real::MAX;
                }
            }

            let rotmat = basis.to_rotation_matrix().into_inner();
            let rmat1 = r1.gcross_matrix() * rotmat;
            let rmat2 = r2.gcross_matrix() * rotmat;

            // Will be actually inverted right after.
            // TODO: we should keep the SdpMatrix3 type.
            let delassus = (self.ii1.quadform(&rmat1).add_diagonal(self.im1)
                + self.ii2.quadform(&rmat2).add_diagonal(self.im2))
            .into_matrix();

            let inv_delassus = GenericVelocityConstraint::invert_partial_delassus_matrix(
                &min_pos_impulse,
                &max_pos_impulse,
                &mut Vector3::zeros(),
                delassus,
            );

            let local_impulse = (inv_delassus * pos_rhs)
                .inf(&max_pos_impulse)
                .sup(&min_pos_impulse);
            let impulse = basis * local_impulse;

            let rot1 = self.ii1.transform_vector(r1.gcross(impulse));
            let rot2 = self.ii2.transform_vector(r2.gcross(impulse));

            position1.translation.vector += self.im1 * impulse;
            position1.rotation = position1.rotation.append_axisangle_linearized(&rot1);
            position2.translation.vector -= self.im2 * impulse;
            position2.rotation = position2.rotation.append_axisangle_linearized(&-rot2);
        }

        /*
         *
         * Rotation part
         *
         */
        {
            let anchor1 = position1 * self.joint.local_anchor1;
            let anchor2 = position2 * self.joint.local_anchor2;
            let basis = anchor1.rotation;
            let mut min_pos_impulse = self
                .joint
                .min_pos_impulse
                .fixed_rows::<Dim>(DIM)
                .into_owned();
            let mut max_pos_impulse = self
                .joint
                .max_pos_impulse
                .fixed_rows::<Dim>(DIM)
                .into_owned();

            let pos_rhs = GenericVelocityConstraint::compute_ang_position_error(
                &anchor1,
                &anchor2,
                &basis,
                &self.joint.min_position.fixed_rows::<Dim>(DIM).into_owned(),
                &self.joint.max_position.fixed_rows::<Dim>(DIM).into_owned(),
            ) * params.joint_erp;

            for i in 0..3 {
                if pos_rhs[i] < 0.0 {
                    min_pos_impulse[i] = -Real::MAX;
                }
                if pos_rhs[i] > 0.0 {
                    max_pos_impulse[i] = Real::MAX;
                }
            }

            // TODO: we should keep the SdpMatrix3 type.
            let rotmat = basis.to_rotation_matrix().into_inner();
            let delassus = (self.ii1.quadform(&rotmat) + self.ii2.quadform(&rotmat)).into_matrix();

            let inv_delassus = GenericVelocityConstraint::invert_partial_delassus_matrix(
                &min_pos_impulse,
                &max_pos_impulse,
                &mut Vector3::zeros(),
                delassus,
            );

            let local_impulse = (inv_delassus * pos_rhs)
                .inf(&max_pos_impulse)
                .sup(&min_pos_impulse);
            let impulse = basis * local_impulse;

            let rot1 = self.ii1.transform_vector(impulse);
            let rot2 = self.ii2.transform_vector(impulse);

            position1.rotation = position1.rotation.append_axisangle_linearized(&rot1);
            position2.rotation = position2.rotation.append_axisangle_linearized(&-rot2);
        }

        positions[self.position1] = position1;
        positions[self.position2] = position2;
    }
}

#[derive(Debug)]
pub(crate) struct GenericPositionGroundConstraint {
    position2: usize,
    anchor1: Isometry<Real>,
    local_anchor2: Isometry<Real>,
    local_com2: Point<Real>,
    im2: Real,
    ii2: AngularInertia<Real>,
    joint: GenericJoint,
}

impl GenericPositionGroundConstraint {
    pub fn from_params(
        rb1: &RigidBody,
        rb2: &RigidBody,
        joint: &GenericJoint,
        flipped: bool,
    ) -> Self {
        let anchor1;
        let local_anchor2;

        if flipped {
            anchor1 = rb1.predicted_position * joint.local_anchor2;
            local_anchor2 = joint.local_anchor1;
        } else {
            anchor1 = rb1.predicted_position * joint.local_anchor1;
            local_anchor2 = joint.local_anchor2;
        };

        Self {
            anchor1,
            local_anchor2,
            position2: rb2.active_set_offset,
            im2: rb2.effective_inv_mass,
            ii2: rb2.effective_world_inv_inertia_sqrt.squared(),
            local_com2: rb2.local_mprops.local_com,
            joint: *joint,
        }
    }

    pub fn solve(&self, params: &IntegrationParameters, positions: &mut [Isometry<Real>]) {
        let mut position2 = positions[self.position2];
        let mut params = *params;
        params.joint_erp = 0.8;

        /*
         *
         * Translation part.
         *
         */
        {
            let anchor1 = self.anchor1;
            let anchor2 = position2 * self.local_anchor2;
            let basis = anchor1.rotation;
            let r2 = Point::from(anchor2.translation.vector) - position2 * self.local_com2;
            let mut min_pos_impulse = self.joint.min_pos_impulse.xyz();
            let mut max_pos_impulse = self.joint.max_pos_impulse.xyz();

            let pos_rhs = GenericVelocityConstraint::compute_lin_position_error(
                &anchor1,
                &anchor2,
                &basis,
                &self.joint.min_position.xyz(),
                &self.joint.max_position.xyz(),
            ) * params.joint_erp;

            for i in 0..3 {
                if pos_rhs[i] < 0.0 {
                    min_pos_impulse[i] = -Real::MAX;
                }
                if pos_rhs[i] > 0.0 {
                    max_pos_impulse[i] = Real::MAX;
                }
            }

            let rotmat = basis.to_rotation_matrix().into_inner();
            let rmat2 = r2.gcross_matrix() * rotmat;

            // TODO: we should keep the SdpMatrix3 type.
            let delassus = self
                .ii2
                .quadform(&rmat2)
                .add_diagonal(self.im2)
                .into_matrix();

            let inv_delassus = GenericVelocityConstraint::invert_partial_delassus_matrix(
                &min_pos_impulse,
                &max_pos_impulse,
                &mut Vector3::zeros(),
                delassus,
            );

            let local_impulse = (inv_delassus * pos_rhs)
                .inf(&max_pos_impulse)
                .sup(&min_pos_impulse);
            let impulse = basis * local_impulse;

            let rot2 = self.ii2.transform_vector(r2.gcross(impulse));

            position2.translation.vector -= self.im2 * impulse;
            position2.rotation = position2.rotation.append_axisangle_linearized(&-rot2);
        }

        /*
         *
         * Rotation part
         *
         */
        {
            let anchor1 = self.anchor1;
            let anchor2 = position2 * self.local_anchor2;
            let basis = anchor1.rotation;
            let mut min_pos_impulse = self
                .joint
                .min_pos_impulse
                .fixed_rows::<Dim>(DIM)
                .into_owned();
            let mut max_pos_impulse = self
                .joint
                .max_pos_impulse
                .fixed_rows::<Dim>(DIM)
                .into_owned();

            let pos_rhs = GenericVelocityConstraint::compute_ang_position_error(
                &anchor1,
                &anchor2,
                &basis,
                &self.joint.min_position.fixed_rows::<Dim>(DIM).into_owned(),
                &self.joint.max_position.fixed_rows::<Dim>(DIM).into_owned(),
            ) * params.joint_erp;

            for i in 0..3 {
                if pos_rhs[i] < 0.0 {
                    min_pos_impulse[i] = -Real::MAX;
                }
                if pos_rhs[i] > 0.0 {
                    max_pos_impulse[i] = Real::MAX;
                }
            }

            // Will be actually inverted right after.
            // TODO: we should keep the SdpMatrix3 type.
            let rotmat = basis.to_rotation_matrix().into_inner();
            let delassus = self.ii2.quadform(&rotmat).into_matrix();

            let inv_delassus = GenericVelocityConstraint::invert_partial_delassus_matrix(
                &min_pos_impulse,
                &max_pos_impulse,
                &mut Vector3::zeros(),
                delassus,
            );

            let local_impulse = (inv_delassus * pos_rhs)
                .inf(&max_pos_impulse)
                .sup(&min_pos_impulse);
            let impulse = basis * local_impulse;
            let rot2 = self.ii2.transform_vector(impulse);

            position2.rotation = position2.rotation.append_axisangle_linearized(&-rot2);
        }

        positions[self.position2] = position2;
    }
}
