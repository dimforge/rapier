use crate::dynamics::Velocity;
use crate::math::{AngVector, Real, Rot, Vect};
use crate::utils;
use bevy::prelude::*;
use rapier::dynamics::{
    AxesMask as RapierAxesMask, RigidBody as RapierRigidBody, RigidBodyVelocity,
};

pub use rapier::control::PdErrors;

/// The axes affected by a [`PdController`] or a [`PidController`].
///
/// Only the axes with their flag set are taken into account when computing errors and
/// corrections.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Reflect)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[reflect(Default, PartialEq, Debug, Hash)]
pub struct AxesMask(u8);

bitflags::bitflags! {
    impl AxesMask: u8 {
        /// The translational X axis.
        const LIN_X = 1 << 0;
        /// The translational Y axis.
        const LIN_Y = 1 << 1;
        /// The translational Z axis.
        #[cfg(feature = "dim3")]
        const LIN_Z = 1 << 2;
        /// The rotational X axis.
        #[cfg(feature = "dim3")]
        const ANG_X = 1 << 3;
        /// The rotational Y axis.
        #[cfg(feature = "dim3")]
        const ANG_Y = 1 << 4;
        /// The rotational Z axis.
        const ANG_Z = 1 << 5;
    }
}

impl AxesMask {
    /// All the translational axes.
    #[cfg(feature = "dim2")]
    pub const LIN_AXES: Self = Self::LIN_X.union(Self::LIN_Y);
    /// All the translational axes.
    #[cfg(feature = "dim3")]
    pub const LIN_AXES: Self = Self::LIN_X.union(Self::LIN_Y).union(Self::LIN_Z);
    /// All the rotational axes.
    #[cfg(feature = "dim2")]
    pub const ANG_AXES: Self = Self::ANG_Z;
    /// All the rotational axes.
    #[cfg(feature = "dim3")]
    pub const ANG_AXES: Self = Self::ANG_X.union(Self::ANG_Y).union(Self::ANG_Z);
}

impl From<AxesMask> for RapierAxesMask {
    fn from(axes: AxesMask) -> RapierAxesMask {
        RapierAxesMask::from_bits_truncate(axes.bits())
    }
}

impl From<RapierAxesMask> for AxesMask {
    fn from(axes: RapierAxesMask) -> AxesMask {
        AxesMask::from_bits_truncate(axes.bits())
    }
}

/// The world-space pose and velocities a [`PdController`] or [`PidController`] drives its
/// rigid-body toward.
///
/// The target pose is the pose of the rigid-body itself (i.e. its `GlobalTransform` translation
/// and rotation), not the pose of its center of mass.
#[derive(Copy, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[reflect(Default, PartialEq, Debug)]
pub struct PidTarget {
    /// The target world-space translation.
    pub translation: Vect,
    /// The target world-space rotation.
    pub rotation: Rot,
    /// The target linear velocity.
    pub linvel: Vect,
    /// The target angular velocity.
    pub angvel: AngVector,
}

impl PidTarget {
    /// A target with the given pose and zero velocities.
    pub fn new(translation: Vect, rotation: Rot) -> Self {
        Self {
            translation,
            rotation,
            ..Default::default()
        }
    }

    /// A target with the given translation, an identity rotation and zero velocities.
    pub fn from_translation(translation: Vect) -> Self {
        Self {
            translation,
            ..Default::default()
        }
    }

    /// Sets the target velocities.
    pub fn with_velocities(mut self, linvel: Vect, angvel: AngVector) -> Self {
        self.linvel = linvel;
        self.angvel = angvel;
        self
    }

    fn to_rapier(self) -> (rapier::math::Pose, RigidBodyVelocity<Real>) {
        (
            utils::pose_from(self.translation, self.rotation),
            RigidBodyVelocity {
                linvel: self.linvel,
                angvel: self.angvel,
            },
        )
    }
}

fn to_velocity(vels: RigidBodyVelocity<Real>) -> Velocity {
    Velocity {
        linear: vels.linvel,
        angular: vels.angvel,
    }
}

#[cfg(feature = "dim2")]
fn splat_ang(value: Real) -> AngVector {
    value
}

#[cfg(feature = "dim3")]
fn splat_ang(value: Real) -> AngVector {
    AngVector::splat(value)
}

/// A Proportional-Derivative (PD) controller driving a rigid-body toward a [`PidTarget`].
///
/// When attached to an entity with a non-fixed rigid-body, the plugin computes a velocity
/// correction before each simulation step and adds it to the rigid-body's velocity. This is
/// the PID controller without its Integral part, which is generally sufficient for games.
/// Use either this or a [`PidController`] on a given entity, not both.
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[reflect(Component, Default, PartialEq, Debug)]
pub struct PdController {
    /// The pose and velocities the rigid-body is driven toward.
    pub target: PidTarget,
    /// The Proportional gain applied to the instantaneous linear position errors.
    ///
    /// This is usually set to a multiple of the inverse of the simulation timestep
    /// (e.g. `60` if the timestep is `1.0 / 60.0`).
    pub lin_kp: Vect,
    /// The Derivative gain applied to the instantaneous linear velocity errors.
    ///
    /// This is usually in `[0.0, 1.0]`, where `1.0` corrects velocity errors in a single step.
    pub lin_kd: Vect,
    /// The Proportional gain applied to the instantaneous angular position errors.
    pub ang_kp: AngVector,
    /// The Derivative gain applied to the instantaneous angular velocity errors.
    pub ang_kd: AngVector,
    /// The axes affected by this controller.
    pub axes: AxesMask,
}

impl Default for PdController {
    fn default() -> Self {
        Self::from_raw(&rapier::control::PdController::default())
    }
}

impl PdController {
    /// Initializes a PD controller with the same gains on all axes, and a default target.
    ///
    /// Only the axes specified in `axes` are controlled.
    pub fn new(kp: Real, kd: Real, axes: AxesMask) -> Self {
        Self::from_raw(&rapier::control::PdController::new(kp, kd, axes.into()))
    }

    /// Sets the target of this controller.
    pub fn with_target(mut self, target: PidTarget) -> Self {
        self.target = target;
        self
    }

    /// Converts a Rapier PD controller into this component, with a default target.
    pub fn from_raw(raw: &rapier::control::PdController) -> Self {
        Self {
            target: PidTarget::default(),
            lin_kp: raw.lin_kp,
            lin_kd: raw.lin_kd,
            ang_kp: raw.ang_kp,
            ang_kd: raw.ang_kd,
            axes: raw.axes.into(),
        }
    }

    /// Converts this component into a Rapier PD controller (the target is not part of it).
    pub fn to_raw(&self) -> rapier::control::PdController {
        rapier::control::PdController {
            lin_kp: self.lin_kp,
            lin_kd: self.lin_kd,
            ang_kp: self.ang_kp,
            ang_kd: self.ang_kd,
            axes: self.axes.into(),
        }
    }

    /// Computes the velocity correction from the given position and velocity errors.
    pub fn correction(&self, pose_errors: &PdErrors, vel_errors: &PdErrors) -> Velocity {
        to_velocity(self.to_raw().correction(pose_errors, vel_errors))
    }

    /// Computes the velocity correction driving the given Rapier rigid-body toward [`Self::target`].
    pub fn rigid_body_correction(&self, body: &RapierRigidBody) -> Velocity {
        let (pose, vels) = self.target.to_rapier();
        to_velocity(self.to_raw().rigid_body_correction(body, pose, vels))
    }
}

/// A Proportional-Integral-Derivative (PID) controller driving a rigid-body toward a [`PidTarget`].
///
/// When attached to an entity with a non-fixed rigid-body, the plugin computes a velocity
/// correction before each simulation step and adds it to the rigid-body's velocity. The
/// accumulated errors of the Integral part are updated at each step. Use either this or a
/// [`PdController`] on a given entity, not both.
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[reflect(Component, Default, PartialEq, Debug)]
pub struct PidController {
    /// The pose and velocities the rigid-body is driven toward.
    pub target: PidTarget,
    /// The Proportional gain applied to the instantaneous linear position errors.
    ///
    /// This is usually set to a multiple of the inverse of the simulation timestep
    /// (e.g. `60` if the timestep is `1.0 / 60.0`).
    pub lin_kp: Vect,
    /// The Integral gain applied to the accumulated linear position errors.
    pub lin_ki: Vect,
    /// The Derivative gain applied to the instantaneous linear velocity errors.
    ///
    /// This is usually in `[0.0, 1.0]`, where `1.0` corrects velocity errors in a single step.
    pub lin_kd: Vect,
    /// The Proportional gain applied to the instantaneous angular position errors.
    pub ang_kp: AngVector,
    /// The Integral gain applied to the accumulated angular position errors.
    pub ang_ki: AngVector,
    /// The Derivative gain applied to the instantaneous angular velocity errors.
    pub ang_kd: AngVector,
    /// The axes affected by this controller.
    pub axes: AxesMask,
    /// The linear position error accumulated through time (updated by the plugin).
    pub lin_integral: Vect,
    /// The angular position error accumulated through time (updated by the plugin).
    pub ang_integral: AngVector,
}

impl Default for PidController {
    fn default() -> Self {
        Self::from_raw(&rapier::control::PidController::default())
    }
}

impl PidController {
    /// Initializes a PID controller with the same gains on all axes, and a default target.
    ///
    /// Only the axes specified in `axes` are controlled.
    pub fn new(kp: Real, ki: Real, kd: Real, axes: AxesMask) -> Self {
        Self::from_raw(&rapier::control::PidController::new(
            kp,
            ki,
            kd,
            axes.into(),
        ))
    }

    /// Sets the target of this controller.
    pub fn with_target(mut self, target: PidTarget) -> Self {
        self.target = target;
        self
    }

    /// Resets to zero the accumulated errors used by the Integral part of the controller.
    pub fn reset_integrals(&mut self) {
        self.lin_integral = Vect::ZERO;
        self.ang_integral = splat_ang(0.0);
    }

    /// Converts a Rapier PID controller into this component, with a default target.
    pub fn from_raw(raw: &rapier::control::PidController) -> Self {
        Self {
            target: PidTarget::default(),
            lin_kp: raw.pd.lin_kp,
            lin_ki: raw.lin_ki,
            lin_kd: raw.pd.lin_kd,
            ang_kp: raw.pd.ang_kp,
            ang_ki: raw.ang_ki,
            ang_kd: raw.pd.ang_kd,
            axes: raw.pd.axes.into(),
            lin_integral: raw.lin_integral,
            ang_integral: raw.ang_integral,
        }
    }

    /// Converts this component into a Rapier PID controller (the target is not part of it).
    pub fn to_raw(&self) -> rapier::control::PidController {
        rapier::control::PidController {
            pd: rapier::control::PdController {
                lin_kp: self.lin_kp,
                lin_kd: self.lin_kd,
                ang_kp: self.ang_kp,
                ang_kd: self.ang_kd,
                axes: self.axes.into(),
            },
            lin_integral: self.lin_integral,
            ang_integral: self.ang_integral,
            lin_ki: self.lin_ki,
            ang_ki: self.ang_ki,
        }
    }

    fn read_integrals(&mut self, raw: &rapier::control::PidController) {
        self.lin_integral = raw.lin_integral;
        self.ang_integral = raw.ang_integral;
    }

    /// Computes the velocity correction from the given position and velocity errors, and
    /// accumulates the position errors for the Integral part.
    pub fn correction(
        &mut self,
        dt: Real,
        pose_errors: &PdErrors,
        vel_errors: &PdErrors,
    ) -> Velocity {
        let mut raw = self.to_raw();
        let result = raw.correction(dt, pose_errors, vel_errors);
        self.read_integrals(&raw);
        to_velocity(result)
    }

    /// Computes the velocity correction driving the given Rapier rigid-body toward
    /// [`Self::target`], and accumulates the position errors for the Integral part.
    pub fn rigid_body_correction(&mut self, dt: Real, body: &RapierRigidBody) -> Velocity {
        let (pose, vels) = self.target.to_rapier();
        let mut raw = self.to_raw();
        let result = raw.rigid_body_correction(dt, body, pose, vels);
        self.read_integrals(&raw);
        to_velocity(result)
    }
}
