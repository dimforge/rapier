use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::{
    JointFixedSolverBody, JointOneBodyConstraint, JointTwoBodyConstraint,
};
use crate::dynamics::solver::joint_constraint::JointSolverBody;
use crate::dynamics::solver::solver_body::SolverBody;
use crate::dynamics::solver::ConstraintsCounts;
use crate::dynamics::{GenericJoint, ImpulseJoint, IntegrationParameters, JointIndex};
use crate::math::*;
use crate::prelude::RigidBodySet;

pub struct JointTwoBodyConstraintBuilder {
    body1: usize,
    body2: usize,
    joint_id: JointIndex,
    joint: GenericJoint,
    constraint_id: usize,
}

impl JointTwoBodyConstraintBuilder {
    pub fn generate(
        joint: &ImpulseJoint,
        bodies: &RigidBodySet,
        joint_id: JointIndex,
        out_builder: &mut Self,
        out_constraint_id: &mut usize,
    ) {
        let rb1 = &bodies[joint.body1];
        let rb2 = &bodies[joint.body2];
        *out_builder = Self {
            body1: rb1.ids.active_set_offset,
            body2: rb2.ids.active_set_offset,
            joint_id,
            joint: joint.data,
            constraint_id: *out_constraint_id,
        };

        let count = ConstraintsCounts::from_joint(joint);
        *out_constraint_id += count.num_constraints;
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        bodies: &[SolverBody],
        out: &mut [JointTwoBodyConstraint<Real, 1>],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?

        let rb1 = &bodies[self.body1];
        let rb2 = &bodies[self.body2];
        let frame1 = rb1.position * self.joint.local_frame1;
        let frame2 = rb2.position * self.joint.local_frame2;

        let joint_body1 = JointSolverBody {
            im: rb1.im,
            sqrt_ii: rb1.sqrt_ii,
            world_com: rb1.world_com,
            solver_vel: [self.body1],
        };
        let joint_body2 = JointSolverBody {
            im: rb2.im,
            sqrt_ii: rb2.sqrt_ii,
            world_com: rb2.world_com,
            solver_vel: [self.body2],
        };

        JointTwoBodyConstraint::<Real, 1>::lock_axes(
            params,
            self.joint_id,
            &joint_body1,
            &joint_body2,
            &frame1,
            &frame2,
            &self.joint,
            &mut out[self.constraint_id..],
        );
    }
}

#[cfg(feature = "simd-is-enabled")]
pub struct JointTwoBodyConstraintBuilderSimd {
    body1: [usize; SIMD_WIDTH],
    body2: [usize; SIMD_WIDTH],
    joint_body1: JointSolverBody<SimdReal, SIMD_WIDTH>,
    joint_body2: JointSolverBody<SimdReal, SIMD_WIDTH>,
    joint_id: [JointIndex; SIMD_WIDTH],
    local_frame1: SimdIsometry,
    local_frame2: SimdIsometry,
    locked_axes: u8,
    constraint_id: usize,
}

#[cfg(feature = "simd-is-enabled")]
impl JointTwoBodyConstraintBuilderSimd {
    pub fn generate(
        joint: [&ImpulseJoint; SIMD_WIDTH],
        bodies: &RigidBodySet,
        joint_id: [JointIndex; SIMD_WIDTH],
        out_builder: &mut Self,
        out_constraint_id: &mut usize,
    ) {
        let rb1 = gather![|ii| &bodies[joint[ii].body1]];
        let rb2 = gather![|ii| &bodies[joint[ii].body2]];

        let body1 = gather![|ii| rb1[ii].ids.active_set_offset];
        let body2 = gather![|ii| rb2[ii].ids.active_set_offset];

        let joint_body1 = JointSolverBody {
            im: gather![|ii| rb1[ii].mprops.effective_inv_mass].into(),
            sqrt_ii: gather![|ii| rb1[ii].mprops.effective_world_inv_inertia_sqrt].into(),
            world_com: SimdPoint::origin(),
            solver_vel: body1,
        };
        let joint_body2 = JointSolverBody {
            im: gather![|ii| rb2[ii].mprops.effective_inv_mass].into(),
            sqrt_ii: gather![|ii| rb2[ii].mprops.effective_world_inv_inertia_sqrt].into(),
            world_com: SimdPoint::origin(),
            solver_vel: body2,
        };

        *out_builder = Self {
            body1,
            body2,
            joint_body1,
            joint_body2,
            joint_id,
            local_frame1: gather![|ii| joint[ii].data.local_frame1].into(),
            local_frame2: gather![|ii| joint[ii].data.local_frame2].into(),
            locked_axes: joint[0].data.locked_axes.bits(),
            constraint_id: *out_constraint_id,
        };

        let count = ConstraintsCounts::from_joint(joint[0]);
        *out_constraint_id += count.num_constraints;
    }

    pub fn update(
        &mut self,
        params: &IntegrationParameters,
        bodies: &[SolverBody],
        out: &mut [JointTwoBodyConstraint<SimdReal, SIMD_WIDTH>],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?

        let rb1 = gather![|ii| &bodies[self.body1[ii]]];
        let rb2 = gather![|ii| &bodies[self.body2[ii]]];
        let frame1 = SimdIsometry::from(gather![|ii| rb1[ii].position]) * self.local_frame1;
        let frame2 = SimdIsometry::from(gather![|ii| rb2[ii].position]) * self.local_frame2;
        self.joint_body1.world_com = gather![|ii| rb1[ii].world_com].into();
        self.joint_body2.world_com = gather![|ii| rb2[ii].world_com].into();

        JointTwoBodyConstraint::<SimdReal, SIMD_WIDTH>::lock_axes(
            params,
            self.joint_id,
            &self.joint_body1,
            &self.joint_body2,
            &frame1,
            &frame2,
            self.locked_axes,
            &mut out[self.constraint_id..],
        );
    }
}

pub struct JointOneBodyConstraintBuilder {
    body1: JointFixedSolverBody<Real>,
    frame1: Isometry,
    body2: usize,
    joint_id: JointIndex,
    joint: GenericJoint,
    constraint_id: usize,
}

impl JointOneBodyConstraintBuilder {
    pub fn generate(
        joint: &ImpulseJoint,
        bodies: &RigidBodySet,
        joint_id: JointIndex,
        out_builder: &mut Self,
        out_constraint_id: &mut usize,
    ) {
        let mut joint_data = joint.data;
        let mut handle1 = joint.body1;
        let mut handle2 = joint.body2;
        let flipped = !bodies[handle2].is_dynamic();

        if flipped {
            std::mem::swap(&mut handle1, &mut handle2);
            std::mem::swap(&mut joint_data.local_frame1, &mut joint_data.local_frame2);
        };

        let rb1 = &bodies[handle1];
        let rb2 = &bodies[handle2];

        let frame1 = rb1.pos.position * joint_data.local_frame1;
        let joint_body1 = JointFixedSolverBody {
            linvel: rb1.vels.linvel,
            angvel: rb1.vels.angvel,
            world_com: rb1.mprops.world_com,
        };

        *out_builder = Self {
            body1: joint_body1,
            frame1,
            body2: rb2.ids.active_set_offset,
            joint_id,
            joint: joint_data,
            constraint_id: *out_constraint_id,
        };

        let count = ConstraintsCounts::from_joint(joint);
        *out_constraint_id += count.num_constraints;
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        bodies: &[SolverBody],
        out: &mut [JointOneBodyConstraint<Real, 1>],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?

        let rb2 = &bodies[self.body2];
        let frame2 = rb2.position * self.joint.local_frame2;

        let joint_body2 = JointSolverBody {
            im: rb2.im,
            sqrt_ii: rb2.sqrt_ii,
            world_com: rb2.world_com,
            solver_vel: [self.body2],
        };

        JointOneBodyConstraint::<Real, 1>::lock_axes(
            params,
            self.joint_id,
            &self.body1,
            &joint_body2,
            &self.frame1,
            &frame2,
            &self.joint,
            &mut out[self.constraint_id..],
        );
    }
}

#[cfg(feature = "simd-is-enabled")]
pub struct JointOneBodyConstraintBuilderSimd {
    body1: JointFixedSolverBody<SimdReal>,
    frame1: SimdIsometry,
    body2: [usize; SIMD_WIDTH],
    joint_id: [JointIndex; SIMD_WIDTH],
    local_frame2: SimdIsometry,
    locked_axes: u8,
    constraint_id: usize,
}

#[cfg(feature = "simd-is-enabled")]
impl JointOneBodyConstraintBuilderSimd {
    pub fn generate(
        joint: [&ImpulseJoint; SIMD_WIDTH],
        bodies: &RigidBodySet,
        joint_id: [JointIndex; SIMD_WIDTH],
        out_builder: &mut Self,
        out_constraint_id: &mut usize,
    ) {
        let mut rb1 = gather![|ii| &bodies[joint[ii].body1]];
        let mut rb2 = gather![|ii| &bodies[joint[ii].body2]];
        let mut local_frame1 = gather![|ii| joint[ii].data.local_frame1];
        let mut local_frame2 = gather![|ii| joint[ii].data.local_frame2];

        for ii in 0..SIMD_WIDTH {
            if !rb2[ii].is_dynamic() {
                std::mem::swap(&mut rb1[ii], &mut rb2[ii]);
                std::mem::swap(&mut local_frame1[ii], &mut local_frame2[ii]);
            }
        }

        let poss1 = SimdIsometry::from(gather![|ii| rb1[ii].pos.position]);

        let joint_body1 = JointFixedSolverBody {
            linvel: gather![|ii| rb1[ii].vels.linvel].into(),
            angvel: gather![|ii| rb1[ii].vels.angvel].into(),
            world_com: gather![|ii| rb1[ii].mprops.world_com].into(),
        };

        *out_builder = Self {
            body1: joint_body1,
            body2: gather![|ii| rb2[ii].ids.active_set_offset],
            joint_id,
            frame1: poss1 * SimdIsometry::from(local_frame1),
            local_frame2: local_frame2.into(),
            locked_axes: joint[0].data.locked_axes.bits(),
            constraint_id: *out_constraint_id,
        };

        let count = ConstraintsCounts::from_joint(joint[0]);
        *out_constraint_id += count.num_constraints;
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        bodies: &[SolverBody],
        out: &mut [JointOneBodyConstraint<SimdReal, SIMD_WIDTH>],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?

        let rb2 = gather![|ii| &bodies[self.body2[ii]]];
        let frame2 = SimdIsometry::from(gather![|ii| rb2[ii].position]) * self.local_frame2;

        let joint_body2 = JointSolverBody {
            im: gather![|ii| rb2[ii].im].into(),
            sqrt_ii: gather![|ii| rb2[ii].sqrt_ii].into(),
            world_com: gather![|ii| rb2[ii].world_com].into(),
            solver_vel: self.body2,
        };

        JointOneBodyConstraint::<SimdReal, SIMD_WIDTH>::lock_axes(
            params,
            self.joint_id,
            &self.body1,
            &joint_body2,
            &self.frame1,
            &frame2,
            self.locked_axes,
            &mut out[self.constraint_id..],
        );
    }
}
