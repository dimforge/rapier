use crate::math::Vect;
use bevy::prelude::*;
use rapier::prelude::{
    LockedAxes as RapierLockedAxes, Pose, RigidBodyActivation, RigidBodyHandle, RigidBodyType,
};
use std::ops::{Add, AddAssign, Sub, SubAssign};

#[cfg(doc)]
use rapier::dynamics::IntegrationParameters;

/// The Rapier handle of a [`RigidBody`] that was inserted to the physics scene.
#[derive(Copy, Clone, Debug, Component)]
pub struct RapierRigidBodyHandle(pub RigidBodyHandle);

/// A [`RigidBody`].
///
/// Related components:
/// - [`GlobalTransform`]: used as the ground truth for the bodies position.
/// - [`Velocity`]
/// - [`ExternalImpulse`]
/// - [`ExternalForce`]
/// - [`AdditionalMassProperties`]
/// - [`ReadMassProperties`]
/// - [`ReadWorldMassProperties`]
/// - [`Damping`]
/// - [`Dominance`]
/// - [`Ccd`]: Helps prevent fast bodies from tunneling through other moving bodies.
/// - [`SoftCcd`]
/// - [`LockedAxes`]
/// - [`RigidBodyDisabled`]
/// - [`GravityScale`]
/// - [`Sleeping`]
/// - [`AdditionalSolverIterations`]
/// - [`AdditionalPgsIterations`]
/// - [`AllowFastRotation`]
/// - `GyroscopicForces` (3D only)
#[derive(Copy, Clone, Debug, PartialEq, Eq, Component, Reflect, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[reflect(Component, Default, PartialEq)]
#[require(ReadMassProperties)]
pub enum RigidBody {
    /// A `RigidBody::Dynamic` body can be affected by all external forces.
    #[default]
    Dynamic,
    /// A `RigidBody::Fixed` body cannot be affected by external forces.
    Fixed,
    /// A `RigidBody::KinematicPositionBased` body cannot be affected by any external forces but can be controlled
    /// by the user at the position level while keeping realistic one-way interaction with dynamic bodies.
    ///
    /// One-way interaction means that a kinematic body can push a dynamic body, but a kinematic body
    /// cannot be pushed by anything. In other words, the trajectory of a kinematic body can only be
    /// modified by the user and is independent from any contact or joint it is involved in.
    KinematicPositionBased,
    /// A `RigidBody::KinematicVelocityBased` body cannot be affected by any external forces but can be controlled
    /// by the user at the velocity level while keeping realistic one-way interaction with dynamic bodies.
    ///
    /// One-way interaction means that a kinematic body can push a dynamic body, but a kinematic body
    /// cannot be pushed by anything. In other words, the trajectory of a kinematic body can only be
    /// modified by the user and is independent from any contact or joint it is involved in.
    KinematicVelocityBased,
}

impl From<RigidBody> for RigidBodyType {
    fn from(rigid_body: RigidBody) -> RigidBodyType {
        match rigid_body {
            RigidBody::Dynamic => RigidBodyType::Dynamic,
            RigidBody::Fixed => RigidBodyType::Fixed,
            RigidBody::KinematicPositionBased => RigidBodyType::KinematicPositionBased,
            RigidBody::KinematicVelocityBased => RigidBodyType::KinematicVelocityBased,
        }
    }
}

impl From<RigidBodyType> for RigidBody {
    fn from(rigid_body: RigidBodyType) -> RigidBody {
        match rigid_body {
            // Soft-body cluster proxies are simulated like dynamic bodies.
            RigidBodyType::Dynamic | RigidBodyType::SoftFrame => RigidBody::Dynamic,
            RigidBodyType::Fixed => RigidBody::Fixed,
            RigidBodyType::KinematicPositionBased => RigidBody::KinematicPositionBased,
            RigidBodyType::KinematicVelocityBased => RigidBody::KinematicVelocityBased,
        }
    }
}

/// The velocity of a [`RigidBody`].
///
/// Use this component to control and/or read the velocity of a dynamic or kinematic [`RigidBody`].
/// If this component isn’t present, a dynamic [`RigidBody`] will still be able to move (you will just
/// not be able to read/modify its velocity).
///
/// This only affects entities with a [`RigidBody`] component.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component, Reflect)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[reflect(Component, Default, PartialEq)]
pub struct Velocity {
    /// The linear velocity of the [`RigidBody`].
    pub linear: Vect,
    /// The angular velocity of the [`RigidBody`] in radian per second.
    #[cfg(feature = "dim2")]
    pub angular: f32,
    /// The angular velocity of the [`RigidBody`].
    #[cfg(feature = "dim3")]
    pub angular: Vect,
}

impl Velocity {
    /// Initialize a velocity set to zero.
    pub const fn zero() -> Self {
        Self {
            linear: Vect::ZERO,
            #[cfg(feature = "dim2")]
            angular: 0.0,
            #[cfg(feature = "dim3")]
            angular: Vect::ZERO,
        }
    }

    /// Initialize a velocity with the given linear velocity, and an angular velocity of zero.
    pub const fn linear(linear: Vect) -> Self {
        Self {
            linear,
            #[cfg(feature = "dim2")]
            angular: 0.0,
            #[cfg(feature = "dim3")]
            angular: Vect::ZERO,
        }
    }

    /// Initialize a velocity with the given angular velocity, and a linear velocity of zero.
    #[cfg(feature = "dim2")]
    pub const fn angular(angular: f32) -> Self {
        Self {
            linear: Vect::ZERO,
            angular,
        }
    }

    /// Initialize a velocity with the given angular velocity, and a linear velocity of zero.
    #[cfg(feature = "dim3")]
    pub const fn angular(angular: Vect) -> Self {
        Self {
            linear: Vect::ZERO,
            angular,
        }
    }

    /// Get linear velocity of specific world-space point of a [`RigidBody`].
    ///
    /// # Parameters
    /// - `point`: the point (world-space) to compute the velocity for.
    /// - `center_of_mass`: the center-of-mass (world-space) of the [`RigidBody`] the velocity belongs to.
    pub fn linear_velocity_at_point(&self, point: Vect, center_of_mass: Vect) -> Vect {
        #[cfg(feature = "dim2")]
        return self.linear + self.angular * (point - center_of_mass).perp();

        #[cfg(feature = "dim3")]
        return self.linear + self.angular.cross(point - center_of_mass);
    }
}

/// Mass-properties of a [`RigidBody`], added to the contributions of its attached colliders.
///
/// This only affects entities with a [`RigidBody`] component.
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub enum AdditionalMassProperties {
    /// This mass will be added to the [`RigidBody`]. The rigid-body’s total
    /// angular inertia tensor (obtained from its attached colliders) will
    /// be scaled accordingly.
    Mass(f32),
    /// These mass properties will be added to the [`RigidBody`].
    MassProperties(MassProperties),
}

impl Default for AdditionalMassProperties {
    fn default() -> Self {
        Self::MassProperties(MassProperties::default())
    }
}

/// Center-of-mass, mass, and angular inertia.
///
/// When this is used as a component, this lets you read the total mass properties of
/// a [`RigidBody`] (including the colliders contribution). Modifying this component won’t
/// affect the mass-properties of the [`RigidBody`] (the attached colliders’ `ColliderMassProperties`
/// and the `AdditionalMassProperties` should be modified instead).
///
/// This only reads the mass from entities with a [`RigidBody`] component. It is inserted
/// automatically with [`RigidBody`] (like the joint read-backs), since it is only updated when
/// the mass properties change.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct ReadMassProperties(MassProperties);

impl ReadMassProperties {
    /// Get the [`MassProperties`] of this [`RigidBody`].
    pub fn get(&self) -> &MassProperties {
        &self.0
    }

    pub(crate) fn set(&mut self, mass_props: MassProperties) {
        self.0 = mass_props;
    }
}

impl std::ops::Deref for ReadMassProperties {
    type Target = MassProperties;
    fn deref(&self) -> &Self::Target {
        self.get()
    }
}

/// World-space and locked-axes-aware mass properties of a [`RigidBody`].
///
/// When this is used as a component, it is updated after each physics step to reflect the
/// current state of the [`RigidBody`]. Modifying this component has no effect on the simulation.
///
/// This only reads the mass properties from entities with a [`RigidBody`] component. Unlike
/// [`ReadMassProperties`], it must be added manually: it changes at every step for every moving
/// body (its center of mass moves), so, like [`Velocity`], it is only written back for the bodies
/// that ask for it.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct ReadWorldMassProperties {
    /// The world-space center of mass of the [`RigidBody`].
    pub center_of_mass: Vect,
    /// The inverse mass along each world axis, set to zero along locked translation axes.
    ///
    /// This is zero for non-dynamic rigid-bodies.
    pub effective_inv_mass: Vect,
    /// The world-space inverse angular inertia, set to zero if rotations are locked.
    ///
    /// This is zero for non-dynamic rigid-bodies.
    #[cfg(feature = "dim2")]
    pub effective_world_inv_inertia: f32,
    /// The world-space inverse angular inertia tensor, with the rows and columns of locked
    /// rotation axes set to zero.
    ///
    /// This is zero for non-dynamic rigid-bodies.
    #[cfg(feature = "dim3")]
    pub effective_world_inv_inertia: Mat3,
}

impl ReadWorldMassProperties {
    /// Extracts the world-space mass properties from Rapier’s `RigidBodyMassProps` structure.
    pub fn from_rapier(mprops: &rapier::dynamics::RigidBodyMassProps) -> Self {
        #[cfg(feature = "dim3")]
        let effective_world_inv_inertia = {
            let i = &mprops.effective_world_inv_inertia;
            Mat3::from_cols(
                Vec3::new(i.m11, i.m12, i.m13),
                Vec3::new(i.m12, i.m22, i.m23),
                Vec3::new(i.m13, i.m23, i.m33),
            )
        };
        #[cfg(feature = "dim2")]
        let effective_world_inv_inertia = mprops.effective_world_inv_inertia;

        Self {
            center_of_mass: mprops.world_com,
            effective_inv_mass: mprops.effective_inv_mass,
            effective_world_inv_inertia,
        }
    }
}

/// Entity that likely had their mass properties changed this frame.
#[derive(Deref, Copy, Clone, Debug, PartialEq, Message)]
pub struct MassModifiedMessage(pub Entity);

impl From<Entity> for MassModifiedMessage {
    fn from(entity: Entity) -> Self {
        Self(entity)
    }
}

#[deprecated(
    since = "0.32.0",
    note = "MassModifiedMessage has been renamed to MassModifiedEvent for consistency with Bevy 0.17 naming conventions. "
)]
pub use MassModifiedMessage as MassModifiedEvent;

/// Center-of-mass, mass, and angular inertia.
///
/// This cannot be used as a component. Use the components `ReadMassProperties` to read a [`RigidBody`]’s
/// mass-properties or `AdditionalMassProperties` to set its additional mass-properties.
#[derive(Copy, Clone, Debug, Default, PartialEq, Reflect)]
#[reflect(Default, PartialEq)]
pub struct MassProperties {
    /// The center of mass of a [`RigidBody`] expressed in its local-space.
    pub local_center_of_mass: Vect,
    /// The mass of a [`RigidBody`].
    pub mass: f32,
    /// The principal angular inertia of the [`RigidBody`].
    #[cfg(feature = "dim2")]
    pub principal_inertia: f32,
    /// The principal vectors of the local angular inertia tensor of the [`RigidBody`].
    #[cfg(feature = "dim3")]
    pub principal_inertia_local_frame: crate::math::Rot,
    /// The principal angular inertia of the [`RigidBody`].
    #[cfg(feature = "dim3")]
    pub principal_inertia: Vect,
}

impl MassProperties {
    /// Converts these mass-properties to Rapier’s `MassProperties` structure.
    #[cfg(feature = "dim2")]
    pub fn into_rapier(self) -> rapier::dynamics::MassProperties {
        rapier::dynamics::MassProperties::new(
            self.local_center_of_mass,
            self.mass,
            #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
            self.principal_inertia.into(),
        )
    }

    /// Converts these mass-properties to Rapier’s `MassProperties` structure.
    #[cfg(feature = "dim3")]
    pub fn into_rapier(self) -> rapier::dynamics::MassProperties {
        rapier::dynamics::MassProperties::with_principal_inertia_frame(
            self.local_center_of_mass,
            self.mass,
            self.principal_inertia,
            self.principal_inertia_local_frame,
        )
    }

    /// Converts Rapier’s `MassProperties` structure to `Self`.
    pub fn from_rapier(mprops: rapier::dynamics::MassProperties) -> Self {
        #[allow(clippy::useless_conversion)] // Need to convert if dim3 enabled
        Self {
            mass: mprops.mass(),
            local_center_of_mass: mprops.local_com.into(),
            principal_inertia: mprops.principal_inertia().into(),
            #[cfg(feature = "dim3")]
            principal_inertia_local_frame: mprops.principal_inertia_local_frame.into(),
        }
    }
}

#[derive(Default, Debug, Component, Reflect, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
#[reflect(Component, Default, PartialEq)]
/// Flags affecting the behavior of the constraints solver for a given contact manifold.
pub struct LockedAxes(u8);

bitflags::bitflags! {
    impl LockedAxes: u8 {
        /// Flag indicating that the [`RigidBody`] cannot translate along the `X` axis.
        const TRANSLATION_LOCKED_X = 1 << 0;
        /// Flag indicating that the [`RigidBody`] cannot translate along the `Y` axis.
        const TRANSLATION_LOCKED_Y = 1 << 1;
        /// Flag indicating that the [`RigidBody`] cannot translate along the `Z` axis.
        const TRANSLATION_LOCKED_Z = 1 << 2;
        /// Flag indicating that the [`RigidBody`] cannot translate along any direction.
        const TRANSLATION_LOCKED = Self::TRANSLATION_LOCKED_X.bits() | Self::TRANSLATION_LOCKED_Y.bits() | Self::TRANSLATION_LOCKED_Z.bits();
        /// Flag indicating that the [`RigidBody`] cannot rotate along the `X` axis.
        const ROTATION_LOCKED_X = 1 << 3;
        /// Flag indicating that the [`RigidBody`] cannot rotate along the `Y` axis.
        const ROTATION_LOCKED_Y = 1 << 4;
        /// Flag indicating that the [`RigidBody`] cannot rotate along the `Z` axis.
        const ROTATION_LOCKED_Z = 1 << 5;
        /// Combination of flags indicating that the [`RigidBody`] cannot rotate along any axis.
        const ROTATION_LOCKED = Self::ROTATION_LOCKED_X.bits() | Self::ROTATION_LOCKED_Y.bits() | Self::ROTATION_LOCKED_Z.bits();
    }
}

impl From<LockedAxes> for RapierLockedAxes {
    fn from(locked_axes: LockedAxes) -> RapierLockedAxes {
        RapierLockedAxes::from_bits(locked_axes.bits()).expect("Internal conversion error.")
    }
}

/// Constant external forces applied continuously to a [`RigidBody`].
///
/// This force is applied at each timestep.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct ExternalForce {
    /// The linear force applied to the [`RigidBody`].
    pub force: Vect,
    /// The angular torque applied to the [`RigidBody`].
    #[cfg(feature = "dim2")]
    pub torque: f32,
    /// The angular torque applied to the [`RigidBody`].
    #[cfg(feature = "dim3")]
    pub torque: Vect,
}

impl ExternalForce {
    /// A force applied at a specific world-space point of a [`RigidBody`].
    ///
    /// # Parameters
    /// - `force`: the force to apply.
    /// - `point`: the point (world-space) where the impulse must be applied.
    /// - `center_of_mass`: the center-of-mass (world-space) of the [`RigidBody`] the impulse is being
    ///   applied to.
    pub fn at_point(force: Vect, point: Vect, center_of_mass: Vect) -> Self {
        Self {
            force,
            #[cfg(feature = "dim2")]
            torque: (point - center_of_mass).perp_dot(force),
            #[cfg(feature = "dim3")]
            torque: (point - center_of_mass).cross(force),
        }
    }
}

impl Add for ExternalForce {
    type Output = Self;

    #[inline]
    fn add(mut self, rhs: Self) -> Self::Output {
        self += rhs;
        self
    }
}

impl Sub for ExternalForce {
    type Output = Self;

    #[inline]
    fn sub(mut self, rhs: Self) -> Self::Output {
        self -= rhs;
        self
    }
}

impl AddAssign for ExternalForce {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.force += rhs.force;
        self.torque += rhs.torque;
    }
}

impl SubAssign for ExternalForce {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.force -= rhs.force;
        self.torque -= rhs.torque;
    }
}

/// Instantaneous external impulse applied continuously to a [`RigidBody`].
///
/// The impulse is only applied once, and whenever it it modified (based
/// on Bevy’s change detection).
#[derive(Copy, Clone, Debug, Default, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct ExternalImpulse {
    /// The linear impulse applied to the [`RigidBody`].
    pub impulse: Vect,
    /// The angular impulse applied to the [`RigidBody`].
    #[cfg(feature = "dim2")]
    pub torque_impulse: f32,
    /// The angular impulse applied to the [`RigidBody`].
    #[cfg(feature = "dim3")]
    pub torque_impulse: Vect,
}

impl ExternalImpulse {
    /// An impulse applied at a specific world-space point of a [`RigidBody`].
    ///
    /// # Parameters
    /// - `impulse`: the impulse to apply.
    /// - `point`: the point (world-space) where the impulse must be applied.
    /// - `center_of_mass`: the center-of-mass (world-space) of the [`RigidBody`] the impulse is being
    ///   applied to.
    pub fn at_point(impulse: Vect, point: Vect, center_of_mass: Vect) -> Self {
        Self {
            impulse,
            #[cfg(feature = "dim2")]
            torque_impulse: (point - center_of_mass).perp_dot(impulse),
            #[cfg(feature = "dim3")]
            torque_impulse: (point - center_of_mass).cross(impulse),
        }
    }

    /// Reset the external impulses to zero.
    pub fn reset(&mut self) {
        *self = Default::default();
    }
}

impl Add for ExternalImpulse {
    type Output = Self;

    #[inline]
    fn add(mut self, rhs: Self) -> Self::Output {
        self += rhs;
        self
    }
}

impl Sub for ExternalImpulse {
    type Output = Self;

    #[inline]
    fn sub(mut self, rhs: Self) -> Self::Output {
        self -= rhs;
        self
    }
}

impl AddAssign for ExternalImpulse {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.impulse += rhs.impulse;
        self.torque_impulse += rhs.torque_impulse;
    }
}

impl SubAssign for ExternalImpulse {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.impulse -= rhs.impulse;
        self.torque_impulse -= rhs.torque_impulse;
    }
}

/// Gravity is multiplied by this scaling factor before it's
/// applied to this [`RigidBody`].
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct GravityScale(pub f32);

impl Default for GravityScale {
    fn default() -> Self {
        Self(1.0)
    }
}

/// Enables full ("bullet") Continuous-Collision-Detection for a [`RigidBody`].
///
/// Fast dynamic bodies always sweep against fixed colliders automatically (unless
/// [`IntegrationParameters::max_ccd_substeps`] is `0`), even without this component. Enabling
/// CCD upgrades the body to also sweep against kinematic and dynamic bodies, at an extra CPU cost.
/// This is useful for projectiles that must not tunnel through other moving bodies. Note that a
/// bullet never sweeps another bullet, so two CCD-enabled bodies can still tunnel through each other.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct Ccd {
    /// Is full ("bullet") CCD enabled for this [`RigidBody`]?
    pub enabled: bool,
}

impl Ccd {
    /// Enable CCD for a [`RigidBody`].
    pub fn enabled() -> Self {
        Self { enabled: true }
    }

    /// Disable full CCD for a [`RigidBody`].
    ///
    /// Note that a [`RigidBody`] without the Ccd component attached
    /// has full CCD disabled by default (it still sweeps against fixed colliders when moving fast).
    pub fn disabled() -> Self {
        Self { enabled: false }
    }
}

/// Sets the maximum prediction distance Soft Continuous Collision-Detection.
///
/// When set to 0, soft-CCD is disabled. Soft-CCD helps prevent tunneling especially of
/// slow-but-thin to moderately fast objects. The soft CCD prediction distance indicates how
/// far in the object’s path the CCD algorithm is allowed to inspect. Large values can impact
/// performance badly by increasing the work needed from the broad-phase.
///
/// It is a generally cheaper variant of regular CCD (that can be enabled with
/// [`Ccd`]) since it relies on predictive constraints instead of shape-cast and substeps.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct SoftCcd {
    /// The soft CCD prediction distance.
    pub prediction: f32,
}

/// The dominance groups of a [`RigidBody`].
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct Dominance {
    // FIXME: rename this to `group` (no `s`).
    /// The dominance groups of a [`RigidBody`].
    pub groups: i8,
}

impl Dominance {
    /// Initialize the dominance to the given group.
    pub fn group(group: i8) -> Self {
        Self { groups: group }
    }
}

/// The activation status of a body.
///
/// This controls whether a body is sleeping or not.
/// If the threshold is negative, the body never sleeps.
///
/// Setting `sleeping` to `false` on a sleeping body wakes up its whole island (every body
/// connected to it through contacts or joints).
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct Sleeping {
    /// The linear velocity below which the body can fall asleep.
    ///
    /// The effictive threshold is obtained by multpilying this value by the
    /// [`IntegrationParameters::length_unit`].
    pub normalized_linear_threshold: f32,
    /// The angular velocity below which the body can fall asleep.
    pub angular_threshold: f32,
    /// How long (in seconds) the body must remain below the velocity thresholds before
    /// falling asleep.
    pub time_until_sleep: f32,
    /// Is this body sleeping?
    pub sleeping: bool,
}

impl Sleeping {
    /// Creates a components that disables sleeping for the associated [`RigidBody`].
    pub fn disabled() -> Self {
        Self {
            normalized_linear_threshold: -1.0,
            angular_threshold: -1.0,
            time_until_sleep: RigidBodyActivation::default_time_until_sleep(),
            sleeping: false,
        }
    }
}

impl Default for Sleeping {
    fn default() -> Self {
        Self {
            normalized_linear_threshold: RigidBodyActivation::default_normalized_linear_threshold(),
            angular_threshold: RigidBodyActivation::default_angular_threshold(),
            time_until_sleep: RigidBodyActivation::default_time_until_sleep(),
            sleeping: false,
        }
    }
}

/// Damping factors to gradually slow down a [`RigidBody`].
#[derive(Copy, Clone, Debug, PartialEq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct Damping {
    // TODO: rename these to "linear" and "angular"?
    /// Damping factor for gradually slowing down the translational motion of the [`RigidBody`].
    pub linear_damping: f32,
    /// Damping factor for gradually slowing down the angular motion of the [`RigidBody`].
    pub angular_damping: f32,
}

impl Default for Damping {
    fn default() -> Self {
        Self {
            linear_damping: 0.0,
            angular_damping: 0.0,
        }
    }
}

/// If the `TimestepMode::Interpolated` mode is set and this component is present,
/// the associated [`RigidBody`] will have its position automatically interpolated
/// between the last two [`RigidBody`] positions set by the physics engine.
#[derive(Copy, Clone, Debug, Default, PartialEq, Component)]
pub struct TransformInterpolation {
    /// The starting point of the interpolation.
    pub start: Option<Pose>,
    /// The end point of the interpolation.
    pub end: Option<Pose>,
}

impl TransformInterpolation {
    /// Interpolates between the start and end positions with `t` in the range `[0..1]`.
    pub fn lerp_slerp(&self, t: f32) -> Option<Pose> {
        if let (Some(start), Some(end)) = (self.start, self.end) {
            Some(start.lerp(&end, t))
        } else {
            None
        }
    }
}

/// Indicates whether or not the [`RigidBody`] is disabled explicitly by the user.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct RigidBodyDisabled;

/// Set the additional number of solver substeps run for the simulation island containing this
/// rigid-body.
///
/// Each extra substep re-derives the soft constraint bias at a smaller timestep, improving
/// accuracy for stiff couplings (joint chains, stacks with high mass ratios). The whole
/// connected component (every body linked to this one through contacts or joints) runs
/// [`IntegrationParameters::num_solver_iterations`] plus the largest
/// `AdditionalSolverIterations` among its bodies substeps. The cost thus scales with the size
/// of that component: attaching such a body to a large pile also substeps the whole pile.
///
/// The default value is 0, meaning exactly [`IntegrationParameters::num_solver_iterations`]
/// substeps will be used for this body.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct AdditionalSolverIterations(pub usize);

/// Set the additional number of internal PGS iterations run at each substep for the simulation
/// island containing this rigid-body.
///
/// The whole connected component (every body linked to this one through contacts or joints)
/// runs [`IntegrationParameters::num_internal_pgs_iterations`] plus the largest
/// `AdditionalPgsIterations` among its bodies. The default value is 0.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct AdditionalPgsIterations(pub usize);

/// Allows the [`RigidBody`] to exceed the angular speed cap.
///
/// By default, the angular velocity of a rigid-body is clamped at each substep (to roughly
/// 45 degrees per step) to keep CCD reliable. Add this marker to bodies that must spin fast,
/// e.g., wheels. Removing it restores the cap.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct AllowFastRotation;

/// Enables or disables gyroscopic forces on a [`RigidBody`].
///
/// Gyroscopic forces make spinning bodies resist changes of their rotation axis (e.g.,
/// spinning tops or flywheels), at a slight performance cost. Rapier enables them by default,
/// so a rigid-body without this component has gyroscopic forces enabled.
#[cfg(feature = "dim3")]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Component, Reflect)]
#[reflect(Component, Default, PartialEq)]
pub struct GyroscopicForces {
    /// Are gyroscopic forces enabled for this [`RigidBody`]?
    pub enabled: bool,
}

#[cfg(feature = "dim3")]
impl GyroscopicForces {
    /// Enable gyroscopic forces for a [`RigidBody`].
    pub fn enabled() -> Self {
        Self { enabled: true }
    }

    /// Disable gyroscopic forces for a [`RigidBody`].
    pub fn disabled() -> Self {
        Self { enabled: false }
    }
}

#[cfg(feature = "dim3")]
impl Default for GyroscopicForces {
    fn default() -> Self {
        Self::enabled()
    }
}
