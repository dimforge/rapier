use crate::dynamics::solver::joint_constraint::joint_velocity_constraint::{
    JointConstraint, WritebackId,
};
use crate::dynamics::solver::joint_constraint::{AngularLimitParams, JointSolverBody};
use crate::dynamics::solver::solver_body::SolverBodies;
use crate::dynamics::solver::{joint_data_num_constraints, joint_num_constraints};
use crate::dynamics::{GenericJoint, ImpulseJoint, IntegrationParameters, JointIndex};
use crate::math::{ANG_DIM, Real, SPATIAL_DIM};
use crate::prelude::RigidBodySet;

use {
    crate::dynamics::SpringCoefficients,
    crate::dynamics::solver::MotorParameters,
    crate::math::{DIM, SIMD_WIDTH, SimdPose, SimdReal},
    crate::na::SimdValue,
    crate::utils::ScalarType,
};

pub struct JointConstraintBuilder {
    body1: u32,
    body2: u32,
    joint_id: JointIndex,
    joint: GenericJoint,
    /// The limited angular axes' limits, in the form the rows consume. Pre-computed here so
    /// the per-substep row rebuild never calls `sin`/`cos`.
    ang_limits: [AngularLimitParams<Real>; ANG_DIM],
    constraint_id: usize,
    /// The per-dof impulses written back at the end of the previous step, used to
    /// seed the constraint impulses when joint warm-starting is enabled.
    prev_dof_impulses: crate::math::SpatialVector,
}

impl JointConstraintBuilder {
    pub fn generate(
        joint: &ImpulseJoint,
        bodies: &RigidBodySet,
        joint_id: JointIndex,
        out_builder: &mut Self,
        out_constraint_id: &mut usize,
    ) {
        let rb1 = &bodies[joint.body1];
        let rb2 = &bodies[joint.body2];
        // Stamped by `select_active_interactions` (`u32::MAX` = world-attached).
        let [solver_body1, solver_body2] = joint.solver_body_ids;

        *out_builder = Self {
            body1: solver_body1,
            body2: solver_body2,
            joint_id,
            joint: joint.data,
            ang_limits: core::array::from_fn(|ang_axis| {
                let limits = &joint.data.limits[crate::math::DIM + ang_axis];
                AngularLimitParams::new(limits.min, limits.max)
            }),
            constraint_id: *out_constraint_id,
            prev_dof_impulses: joint.impulses,
        };
        // Since solver body poses are given in center-of-mass space,
        // we need to transform the anchors to that space.
        out_builder.joint.transform_to_solver_body_space(rb1, rb2);

        *out_constraint_id += joint_num_constraints(joint);
    }

    /// Refreshes the warm-start seeds (the impulses written back at the end of
    /// the previous step) of a builder recycled across steps by the persistent
    /// joint assembly.
    pub fn refresh_warmstart_seeds(&mut self, joints_all: &[crate::dynamics::JointGraphEdge]) {
        let joint = &joints_all[self.joint_id].weight;
        self.prev_dof_impulses = joint.impulses;
        for i in 0..SPATIAL_DIM {
            self.joint.limits[i].impulse = joint.data.limits[i].impulse;
            self.joint.motors[i].impulse = joint.data.motors[i].impulse;
        }
    }

    pub fn update(
        &self,
        params: &IntegrationParameters,
        substep_id: usize,
        // `Some(warmstart_coefficient)` when joint warm-starting is enabled.
        warmstart: Option<Real>,
        bodies: &SolverBodies,
        out: &mut [JointConstraint<Real, 1>],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?

        let rb1 = bodies.get_pose(self.body1);
        let rb2 = bodies.get_pose(self.body2);
        let frame1 = rb1.pose() * self.joint.local_frame1;
        let frame2 = rb2.pose() * self.joint.local_frame2;
        let world_com1 = rb1.translation;
        let world_com2 = rb2.translation;

        let joint_body1 = JointSolverBody {
            im: rb1.im,
            ii: rb1.ii,
            world_com: world_com1,
            solver_vel: [self.body1],
        };
        let joint_body2 = JointSolverBody {
            im: rb2.im,
            ii: rb2.ii,
            world_com: world_com2,
            solver_vel: [self.body2],
        };

        let out_rows = &mut out[self.constraint_id..];

        // When warm-starting, carry the impulses accumulated by the previous substep
        // across the row rebuild (the row layout only depends on the static joint
        // configuration, so it is stable across the substeps of a step).
        const MAX_ROWS: usize = 4 * SPATIAL_DIM;
        let mut prev_impulses = [0.0; MAX_ROWS];
        if warmstart.is_some() && substep_id > 0 {
            let count = joint_data_num_constraints(&self.joint).min(MAX_ROWS);
            for (prev, row) in prev_impulses[..count].iter_mut().zip(out_rows.iter()) {
                *prev = row.impulse;
            }
        }

        let len = JointConstraint::<Real, 1>::update(
            params,
            self.joint_id,
            &joint_body1,
            &joint_body2,
            &frame1,
            &frame2,
            &self.joint,
            &self.ang_limits,
            out_rows,
        );

        if let Some(coeff) = warmstart {
            if substep_id == 0 {
                // Seed from the impulses written back at the end of the previous step.
                for row in &mut out_rows[..len] {
                    let seed = match row.writeback_id {
                        WritebackId::Dof(i) => self.prev_dof_impulses[i],
                        WritebackId::Limit(i) => self.joint.limits[i].impulse,
                        WritebackId::Motor(i) => self.joint.motors[i].impulse,
                        WritebackId::Friction(_) => 0.0,
                    };
                    row.impulse = seed * coeff;
                }
            } else {
                for (row, prev) in out_rows[..len].iter_mut().zip(prev_impulses.iter()) {
                    row.impulse = *prev * coeff;
                }
            }
        }
    }
}

pub struct JointConstraintBuilderSimd {
    body1: [u32; SIMD_WIDTH],
    body2: [u32; SIMD_WIDTH],
    joint_id: [JointIndex; SIMD_WIDTH],
    local_frame1: SimdPose<SimdReal>,
    local_frame2: SimdPose<SimdReal>,
    locked_axes: u8,
    /// Uncoupled limited axes (`limit_axes & !locked_axes`), identical across
    /// the lanes (guaranteed by the row-signature grouping).
    limit_axes: u8,
    /// Motorized axes (`motor_axes & !locked_axes`), identical across the
    /// lanes. Only the 2D angular motor has a wide row (see
    /// `GenericJoint::supports_simd_constraints`).
    #[cfg(feature = "dim2")]
    motor_axes: u8,
    /// The 2D angular motor's per-lane parameters (model shared per chunk via
    /// the row signature).
    #[cfg(feature = "dim2")]
    motor_model: crate::dynamics::MotorModel,
    #[cfg(feature = "dim2")]
    motor_stiffness: SimdReal,
    #[cfg(feature = "dim2")]
    motor_damping: SimdReal,
    #[cfg(feature = "dim2")]
    motor_target_pos: SimdReal,
    #[cfg(feature = "dim2")]
    motor_target_vel: SimdReal,
    #[cfg(feature = "dim2")]
    motor_max_force: SimdReal,
    /// Like `prev_dof_impulses`, for the 2D angular motor row.
    #[cfg(feature = "dim2")]
    prev_motor_impulse: SimdReal,
    /// Per-axis `[min, max]` limits of the limited linear axes (unset axes are zero).
    /// The angular axes go through `ang_limits`.
    limits: [[SimdReal; 2]; DIM],
    /// The limited angular axes' limits, in the form the rows consume — pre-computed so the
    /// per-substep row rebuild never calls `sin`/`cos`.
    ang_limits: [AngularLimitParams<SimdReal>; ANG_DIM],
    softness: SpringCoefficients<SimdReal>,
    constraint_id: usize,
    /// Per-dof impulses written back at the end of the previous step (one SIMD lane
    /// per joint), used to seed the constraint impulses when joint warm-starting is
    /// enabled. Only the locked axes are relevant for the SIMD builder.
    prev_dof_impulses: [SimdReal; SPATIAL_DIM],
    /// Like `prev_dof_impulses`, for the limit rows.
    prev_limit_impulses: [SimdReal; SPATIAL_DIM],
    /// The bodies' effective inverse masses/angular inertias, cached by the substep-0 update.
    /// Step-constant (solver-body mass properties refresh once per step), so later substeps
    /// only gather the transform part of the solver poses — about half the transposition work.
    im1: <SimdReal as ScalarType>::Vector,
    ii1: <SimdReal as ScalarType>::AngInertia,
    im2: <SimdReal as ScalarType>::Vector,
    ii2: <SimdReal as ScalarType>::AngInertia,
}

impl JointConstraintBuilderSimd {
    pub fn generate(
        joint: [&ImpulseJoint; SIMD_WIDTH],
        bodies: &RigidBodySet,
        joint_id: [JointIndex; SIMD_WIDTH],
        out_builder: &mut Self,
        out_constraint_id: &mut usize,
    ) {
        let rb1 = array![|ii| &bodies[joint[ii].body1]];
        let rb2 = array![|ii| &bodies[joint[ii].body2]];

        // Solver-body ids stamped by `select_active_interactions`
        // (`u32::MAX` = world-attached: fixed or — defensively — sleeping).
        let body1 = array![|ii| joint[ii].solver_body_ids[0]];
        let body2 = array![|ii| joint[ii].solver_body_ids[1]];

        let local_frame1 = array![|ii| if body1[ii] != u32::MAX {
            (joint[ii]
                .data
                .local_frame1
                .append_translation(-rb1[ii].mprops.local_mprops.local_com))
            .into()
        } else {
            (rb1[ii].pos.position * joint[ii].data.local_frame1).into()
        }]
        .into();
        let local_frame2 = array![|ii| if body2[ii] != u32::MAX {
            (joint[ii]
                .data
                .local_frame2
                .append_translation(-rb2[ii].mprops.local_mprops.local_com))
            .into()
        } else {
            (rb2[ii].pos.position * joint[ii].data.local_frame2).into()
        }]
        .into();

        let locked_axes = joint[0].data.locked_axes.bits();
        let limit_axes = joint[0].data.limit_axes.bits() & !locked_axes;
        debug_assert!(
            joint
                .iter()
                .all(|j| j.data.simd_row_signature() == joint[0].data.simd_row_signature())
        );

        #[cfg(feature = "dim2")]
        let ang_motor = |ii: usize| &joint[ii].data.motors[crate::math::DIM];

        let zero2 = [SimdReal::splat(0.0); 2];
        *out_builder = Self {
            body1,
            body2,
            joint_id,
            local_frame1,
            local_frame2,
            locked_axes,
            limit_axes,
            #[cfg(feature = "dim2")]
            motor_axes: joint[0].data.motor_axes.bits() & !locked_axes,
            #[cfg(feature = "dim2")]
            motor_model: ang_motor(0).model,
            #[cfg(feature = "dim2")]
            motor_stiffness: array![|ii| ang_motor(ii).stiffness].into(),
            #[cfg(feature = "dim2")]
            motor_damping: array![|ii| ang_motor(ii).damping].into(),
            #[cfg(feature = "dim2")]
            motor_target_pos: array![|ii| ang_motor(ii).target_pos].into(),
            #[cfg(feature = "dim2")]
            motor_target_vel: array![|ii| ang_motor(ii).target_vel].into(),
            #[cfg(feature = "dim2")]
            motor_max_force: array![|ii| ang_motor(ii).max_force].into(),
            #[cfg(feature = "dim2")]
            prev_motor_impulse: array![|ii| ang_motor(ii).impulse].into(),
            limits: core::array::from_fn(|axis| {
                if limit_axes & (1 << axis) != 0 {
                    [
                        array![|ii| joint[ii].data.limits[axis].min].into(),
                        array![|ii| joint[ii].data.limits[axis].max].into(),
                    ]
                } else {
                    zero2
                }
            }),
            ang_limits: core::array::from_fn(|ang_axis| {
                // The per-lane scalar `sin`/`cos` run once per assembly rebuild, not per substep.
                let per_lane = array![|ii| {
                    let limits = &joint[ii].data.limits[DIM + ang_axis];
                    AngularLimitParams::new(limits.min, limits.max)
                }];
                AngularLimitParams {
                    center: [
                        array![|ii| per_lane[ii].center[0]].into(),
                        array![|ii| per_lane[ii].center[1]].into(),
                    ],
                    half_range: array![|ii| per_lane[ii].half_range].into(),
                }
            }),
            softness: SpringCoefficients {
                natural_frequency: array![|ii| joint[ii].data.softness.natural_frequency].into(),
                damping_ratio: array![|ii| joint[ii].data.softness.damping_ratio].into(),
            },
            constraint_id: *out_constraint_id,
            prev_dof_impulses: core::array::from_fn(|axis| {
                array![|ii| joint[ii].impulses[axis]].into()
            }),
            prev_limit_impulses: core::array::from_fn(|axis| {
                if limit_axes & (1 << axis) != 0 {
                    array![|ii| joint[ii].data.limits[axis].impulse].into()
                } else {
                    SimdReal::splat(0.0)
                }
            }),
            im1: Default::default(),
            ii1: Default::default(),
            im2: Default::default(),
            ii2: Default::default(),
        };

        *out_constraint_id += joint_num_constraints(joint[0]);
    }

    /// Refreshes the warm-start seeds (the impulses written back at the end of
    /// the previous step) of a builder recycled across steps by the persistent
    /// joint assembly.
    pub fn refresh_warmstart_seeds(&mut self, joints_all: &[crate::dynamics::JointGraphEdge]) {
        let joint = array![|ii| &joints_all[self.joint_id[ii]].weight];
        self.prev_dof_impulses =
            core::array::from_fn(|axis| array![|ii| joint[ii].impulses[axis]].into());
        let limit_axes = self.limit_axes;
        self.prev_limit_impulses = core::array::from_fn(|axis| {
            if limit_axes & (1 << axis) != 0 {
                array![|ii| joint[ii].data.limits[axis].impulse].into()
            } else {
                SimdReal::splat(0.0)
            }
        });
        #[cfg(feature = "dim2")]
        {
            self.prev_motor_impulse =
                array![|ii| joint[ii].data.motors[crate::math::DIM].impulse].into();
        }
    }

    pub fn update(
        &mut self,
        params: &IntegrationParameters,
        substep_id: usize,
        // `Some(warmstart_coefficient)` when joint warm-starting is enabled.
        warmstart: Option<Real>,
        bodies: &SolverBodies,
        out: &mut [JointConstraint<SimdReal, SIMD_WIDTH>],
    ) {
        // NOTE: right now, the "update", is basically reconstructing all the
        //       constraints. Could we make this more incremental?

        let (frame1, frame2, joint_body1, joint_body2);
        if substep_id == 0 {
            let rb1 = bodies.gather_poses(self.body1);
            let rb2 = bodies.gather_poses(self.body2);
            frame1 = rb1.pose() * self.local_frame1;
            frame2 = rb2.pose() * self.local_frame2;

            // Cache the step-constant mass properties for the later substeps.
            self.im1 = rb1.im;
            self.ii1 = rb1.ii;
            self.im2 = rb2.im;
            self.ii2 = rb2.ii;

            joint_body1 = JointSolverBody {
                im: rb1.im,
                ii: rb1.ii,
                world_com: rb1.translation,
                solver_vel: self.body1,
            };
            joint_body2 = JointSolverBody {
                im: rb2.im,
                ii: rb2.ii,
                world_com: rb2.translation,
                solver_vel: self.body2,
            };
        } else {
            // Only the transform part of the poses changes across substeps: this
            // gather does about half the transposition work of a full pose gather.
            let t1 = bodies.gather_transforms(self.body1);
            let t2 = bodies.gather_transforms(self.body2);
            frame1 = <SimdReal as ScalarType>::Pose::from_parts(t1.translation.into(), t1.rotation)
                * self.local_frame1;
            frame2 = <SimdReal as ScalarType>::Pose::from_parts(t2.translation.into(), t2.rotation)
                * self.local_frame2;

            joint_body1 = JointSolverBody {
                im: self.im1,
                ii: self.ii1,
                world_com: t1.translation,
                solver_vel: self.body1,
            };
            joint_body2 = JointSolverBody {
                im: self.im2,
                ii: self.ii2,
                world_com: t2.translation,
                solver_vel: self.body2,
            };
        }

        let out_rows = &mut out[self.constraint_id..];

        // The (2D) angular motor row's wide parameters, from the gathered
        // per-lane motor data (`MotorModel::combine_coefficients` +
        // `JointMotor::motor_params`, wide).
        #[cfg(feature = "dim2")]
        let ang_motor_params = (self.motor_axes & (1 << DIM) != 0).then(|| {
            use crate::dynamics::MotorModel;
            let dt = SimdReal::splat(params.dt);
            let zero = SimdReal::splat(0.0);
            let erp_inv_dt = self.motor_stiffness
                * crate::utils::simd_inv(dt * self.motor_stiffness + self.motor_damping);
            let cfm =
                crate::utils::simd_inv(dt * dt * self.motor_stiffness + dt * self.motor_damping);
            let (cfm_coeff, cfm_gain) = match self.motor_model {
                MotorModel::AccelerationBased => (cfm, zero),
                MotorModel::ForceBased => (zero, cfm),
            };
            MotorParameters {
                erp_inv_dt,
                cfm_coeff,
                cfm_gain,
                target_pos: self.motor_target_pos,
                target_vel: self.motor_target_vel,
                max_impulse: self.motor_max_force * dt,
            }
        });
        #[cfg(feature = "dim3")]
        let ang_motor_params: Option<MotorParameters<SimdReal>> = None;

        // See the scalar builder: carry impulses across the row rebuild when warm-starting.
        // The SIMD builder emits at most one motor row, one row per locked axis and one per
        // (uncoupled) limited axis; the masks are disjoint.
        const MAX_WIDE_ROWS: usize = SPATIAL_DIM + 1;
        let mut prev_impulses = [SimdReal::splat(0.0); MAX_WIDE_ROWS];
        if warmstart.is_some() && substep_id > 0 {
            let count = ((self.locked_axes | self.limit_axes).count_ones() as usize
                + ang_motor_params.is_some() as usize)
                .min(MAX_WIDE_ROWS);
            for (prev, row) in prev_impulses[..count].iter_mut().zip(out_rows.iter()) {
                *prev = row.impulse;
            }
        }

        let len = JointConstraint::<SimdReal, SIMD_WIDTH>::update(
            params,
            self.joint_id,
            &joint_body1,
            &joint_body2,
            &frame1,
            &frame2,
            self.locked_axes,
            self.limit_axes,
            &self.limits,
            &self.ang_limits,
            self.softness,
            ang_motor_params.as_ref(),
            out_rows,
        );

        if let Some(coeff) = warmstart {
            let coeff = SimdReal::splat(coeff);
            if substep_id == 0 {
                for row in &mut out_rows[..len] {
                    match row.writeback_id {
                        WritebackId::Dof(i) => row.impulse = self.prev_dof_impulses[i] * coeff,
                        WritebackId::Limit(i) => row.impulse = self.prev_limit_impulses[i] * coeff,
                        #[cfg(feature = "dim2")]
                        WritebackId::Motor(_) => {
                            row.impulse = self.prev_motor_impulse * coeff;
                        }
                        #[cfg(feature = "dim3")]
                        WritebackId::Motor(_) => {}
                        // Impulse-joint rows only; friction rows are
                        // multibody-internal and are never built here.
                        WritebackId::Friction(_) => {}
                    }
                }
            } else {
                for (row, prev) in out_rows[..len].iter_mut().zip(prev_impulses.iter()) {
                    row.impulse = *prev * coeff;
                }
            }
        }
    }
}
