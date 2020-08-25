use crate::dynamics::solver::DeltaVel;
use crate::dynamics::{
    BallJoint, IntegrationParameters, JointGraphEdge, JointIndex, JointParams, RigidBody,
};
use crate::math::{SdpMatrix, Vector};
use crate::utils::{WAngularInertia, WCross, WCrossMatrix};

#[derive(Debug)]
pub(crate) struct BallVelocityConstraint {
    mj_lambda1: usize,
    mj_lambda2: usize,

    joint_id: JointIndex,

    rhs: Vector<f32>,
    pub(crate) impulse: Vector<f32>,

    gcross1: Vector<f32>,
    gcross2: Vector<f32>,

    inv_lhs: SdpMatrix<f32>,

    im1: f32,
    im2: f32,
}

impl BallVelocityConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        cparams: &BallJoint,
    ) -> Self {
        let anchor1 = rb1.position * cparams.local_anchor1 - rb1.world_com;
        let anchor2 = rb2.position * cparams.local_anchor2 - rb2.world_com;

        let vel1 = rb1.linvel + rb1.angvel.gcross(anchor1);
        let vel2 = rb2.linvel + rb2.angvel.gcross(anchor2);
        let im1 = rb1.mass_properties.inv_mass;
        let im2 = rb2.mass_properties.inv_mass;

        let rhs = -(vel1 - vel2);
        let lhs;

        let cmat1 = anchor1.gcross_matrix();
        let cmat2 = anchor2.gcross_matrix();

        #[cfg(feature = "dim3")]
        {
            lhs = rb2
                .world_inv_inertia_sqrt
                .squared()
                .quadform(&cmat2)
                .add_diagonal(im2)
                + rb1
                    .world_inv_inertia_sqrt
                    .squared()
                    .quadform(&cmat1)
                    .add_diagonal(im1);
        }

        // In 2D we just unroll the computation because
        // it's just easier that way.
        #[cfg(feature = "dim2")]
        {
            let ii1 = rb1.world_inv_inertia_sqrt.squared();
            let ii2 = rb2.world_inv_inertia_sqrt.squared();
            let m11 = im1 + im2 + cmat1.x * cmat1.x * ii1 + cmat2.x * cmat2.x * ii2;
            let m12 = cmat1.x * cmat1.y * ii1 + cmat2.x * cmat2.y * ii2;
            let m22 = im1 + im2 + cmat1.y * cmat1.y * ii1 + cmat2.y * cmat2.y * ii2;
            lhs = SdpMatrix::new(m11, m12, m22)
        }

        let gcross1 = rb1.world_inv_inertia_sqrt.transform_lin_vector(anchor1);
        let gcross2 = rb2.world_inv_inertia_sqrt.transform_lin_vector(anchor2);

        let inv_lhs = lhs.inverse_unchecked();

        BallVelocityConstraint {
            joint_id,
            mj_lambda1: rb1.active_set_offset,
            mj_lambda2: rb2.active_set_offset,
            im1,
            im2,
            impulse: cparams.impulse * params.warmstart_coeff,
            gcross1,
            gcross2,
            rhs,
            inv_lhs,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        mj_lambda1.linear += self.im1 * self.impulse;
        mj_lambda1.angular += self.gcross1.gcross(self.impulse);
        mj_lambda2.linear -= self.im2 * self.impulse;
        mj_lambda2.angular -= self.gcross2.gcross(self.impulse);

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda1 = mj_lambdas[self.mj_lambda1 as usize];
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let vel1 = mj_lambda1.linear + mj_lambda1.angular.gcross(self.gcross1);
        let vel2 = mj_lambda2.linear + mj_lambda2.angular.gcross(self.gcross2);
        let dvel = -vel1 + vel2 + self.rhs;

        let impulse = self.inv_lhs * dvel;
        self.impulse += impulse;

        mj_lambda1.linear += self.im1 * impulse;
        mj_lambda1.angular += self.gcross1.gcross(impulse);

        mj_lambda2.linear -= self.im2 * impulse;
        mj_lambda2.angular -= self.gcross2.gcross(impulse);

        mj_lambdas[self.mj_lambda1 as usize] = mj_lambda1;
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::BallJoint(ball) = &mut joint.params {
            ball.impulse = self.impulse
        }
    }
}

#[derive(Debug)]
pub(crate) struct BallVelocityGroundConstraint {
    mj_lambda2: usize,
    joint_id: JointIndex,
    rhs: Vector<f32>,
    impulse: Vector<f32>,
    gcross2: Vector<f32>,
    inv_lhs: SdpMatrix<f32>,
    im2: f32,
}

impl BallVelocityGroundConstraint {
    pub fn from_params(
        params: &IntegrationParameters,
        joint_id: JointIndex,
        rb1: &RigidBody,
        rb2: &RigidBody,
        cparams: &BallJoint,
        flipped: bool,
    ) -> Self {
        let (anchor1, anchor2) = if flipped {
            (
                rb1.position * cparams.local_anchor2 - rb1.world_com,
                rb2.position * cparams.local_anchor1 - rb2.world_com,
            )
        } else {
            (
                rb1.position * cparams.local_anchor1 - rb1.world_com,
                rb2.position * cparams.local_anchor2 - rb2.world_com,
            )
        };

        let im2 = rb2.mass_properties.inv_mass;
        let vel1 = rb1.linvel + rb1.angvel.gcross(anchor1);
        let vel2 = rb2.linvel + rb2.angvel.gcross(anchor2);
        let rhs = vel2 - vel1;

        let cmat2 = anchor2.gcross_matrix();
        let gcross2 = rb2.world_inv_inertia_sqrt.transform_lin_vector(anchor2);

        let lhs;

        #[cfg(feature = "dim3")]
        {
            lhs = rb2
                .world_inv_inertia_sqrt
                .squared()
                .quadform(&cmat2)
                .add_diagonal(im2);
        }

        #[cfg(feature = "dim2")]
        {
            let ii2 = rb2.world_inv_inertia_sqrt.squared();
            let m11 = im2 + cmat2.x * cmat2.x * ii2;
            let m12 = cmat2.x * cmat2.y * ii2;
            let m22 = im2 + cmat2.y * cmat2.y * ii2;
            lhs = SdpMatrix::new(m11, m12, m22)
        }

        let inv_lhs = lhs.inverse_unchecked();

        BallVelocityGroundConstraint {
            joint_id,
            mj_lambda2: rb2.active_set_offset,
            im2,
            impulse: cparams.impulse * params.warmstart_coeff,
            gcross2,
            rhs,
            inv_lhs,
        }
    }

    pub fn warmstart(&self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];
        mj_lambda2.linear -= self.im2 * self.impulse;
        mj_lambda2.angular -= self.gcross2.gcross(self.impulse);
        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    pub fn solve(&mut self, mj_lambdas: &mut [DeltaVel<f32>]) {
        let mut mj_lambda2 = mj_lambdas[self.mj_lambda2 as usize];

        let vel2 = mj_lambda2.linear + mj_lambda2.angular.gcross(self.gcross2);
        let dvel = vel2 + self.rhs;

        let impulse = self.inv_lhs * dvel;
        self.impulse += impulse;

        mj_lambda2.linear -= self.im2 * impulse;
        mj_lambda2.angular -= self.gcross2.gcross(impulse);

        mj_lambdas[self.mj_lambda2 as usize] = mj_lambda2;
    }

    // FIXME: duplicated code with the non-ground constraint.
    pub fn writeback_impulses(&self, joints_all: &mut [JointGraphEdge]) {
        let joint = &mut joints_all[self.joint_id].weight;
        if let JointParams::BallJoint(ball) = &mut joint.params {
            ball.impulse = self.impulse
        }
    }
}
