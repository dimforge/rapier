use crate::dynamics::MassProperties;
use crate::geometry::{
    ColliderChanges, ColliderHandle, ColliderMassProperties, ColliderParent, ColliderPosition,
    ColliderSet, ColliderShape,
};
use crate::math::*;
use crate::parry::partitioning::IndexedData;
use crate::utils::{SimdAngularInertia, SimdCross, SimdDot};
use num::Zero;

use crate::data::Index;
#[cfg(feature = "bevy")]
use bevy::prelude::{Component, Reflect, ReflectComponent};

/// The unique handle of a rigid body added to a `RigidBodySet`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg(not(feature = "bevy"))]
#[repr(transparent)]
pub struct RigidBodyHandle(pub crate::data::arena::Index);

#[cfg(feature = "bevy")]
pub type RigidBodyHandle = bevy::prelude::Entity;

#[cfg(not(feature = "bevy"))]
impl RigidBodyHandle {
    pub const PLACEHOLDER: Self = Self(Index::from_raw_parts(
        crate::INVALID_U32,
        crate::INVALID_U32,
    ));

    /// Converts this handle into its (index, generation) components.
    pub fn into_raw_parts(self) -> (u32, u32) {
        self.0.into_raw_parts()
    }

    pub fn index(&self) -> u32 {
        self.0.into_raw_parts().0
    }

    /// Reconstructs an handle from its (index, generation) components.
    pub fn from_raw_parts(id: u32, generation: u32) -> Self {
        Self(crate::data::arena::Index::from_raw_parts(id, generation))
    }

    /// An always-invalid rigid-body handle.
    pub fn invalid() -> Self {
        Self::PLACEHOLDER
    }
}

#[cfg(not(feature = "bevy"))]
impl From<RigidBodyHandle> for crate::data::arena::Index {
    fn from(value: RigidBodyHandle) -> Self {
        value.0
    }
}

#[cfg(not(feature = "bevy"))]
impl From<Index> for RigidBodyHandle {
    fn from(value: Index) -> Self {
        Self(value)
    }
}

#[cfg(not(feature = "bevy"))]
impl IndexedData for RigidBodyHandle {
    fn default() -> Self {
        Self(IndexedData::default())
    }

    fn index(&self) -> usize {
        self.0.index()
    }
}

/// The type of a body, governing the way it is affected by external forces.
#[deprecated(note = "renamed as RigidBodyType")]
pub type BodyStatus = RigidBodyType;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// The status of a body, governing the way it is affected by external forces.
pub enum RigidBodyType {
    /// A `RigidBodyType::Dynamic` body can be affected by all external forces.
    Dynamic = 0,
    /// A `RigidBodyType::Fixed` body cannot be affected by external forces.
    Fixed = 1,
    /// A `RigidBodyType::KinematicPositionBased` body cannot be affected by any external forces but can be controlled
    /// by the user at the position level while keeping realistic one-way interaction with dynamic bodies.
    ///
    /// One-way interaction means that a kinematic body can push a dynamic body, but a kinematic body
    /// cannot be pushed by anything. In other words, the trajectory of a kinematic body can only be
    /// modified by the user and is independent from any contact or joint it is involved in.
    KinematicPositionBased = 2,
    /// A `RigidBodyType::KinematicVelocityBased` body cannot be affected by any external forces but can be controlled
    /// by the user at the velocity level while keeping realistic one-way interaction with dynamic bodies.
    ///
    /// One-way interaction means that a kinematic body can push a dynamic body, but a kinematic body
    /// cannot be pushed by anything. In other words, the trajectory of a kinematic body can only be
    /// modified by the user and is independent from any contact or joint it is involved in.
    KinematicVelocityBased = 3,
    // Semikinematic, // A kinematic that performs automatic CCD with the fixed environment to avoid traversing it?
    // Disabled,
}

impl RigidBodyType {
    /// Is this rigid-body fixed (i.e. cannot move)?
    pub fn is_fixed(self) -> bool {
        self == RigidBodyType::Fixed
    }

    /// Is this rigid-body dynamic (i.e. can move and be affected by forces)?
    pub fn is_dynamic(self) -> bool {
        self == RigidBodyType::Dynamic
    }

    /// Is this rigid-body kinematic (i.e. can move but is unaffected by forces)?
    pub fn is_kinematic(self) -> bool {
        self == RigidBodyType::KinematicPositionBased
            || self == RigidBodyType::KinematicVelocityBased
    }
}

bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags describing how the rigid-body has been modified by the user.
    pub struct RigidBodyChanges: u32 {
        /// Flag indicating that any component of this rigid-body has been modified.
        const MODIFIED    = 1 << 0;
        /// Flag indicating that the `RigidBodyPosition` component of this rigid-body has been modified.
        const POSITION    = 1 << 1;
        /// Flag indicating that the `SleepState` component of this rigid-body has been modified.
        const SLEEP       = 1 << 2;
        /// Flag indicating that the `RigidBodyColliders` component of this rigid-body has been modified.
        const COLLIDERS   = 1 << 3;
        /// Flag indicating that the `RigidBodyType` component of this rigid-body has been modified.
        const TYPE        = 1 << 4;
        /// Flag indicating that the `Dominance` component of this rigid-body has been modified.
        const DOMINANCE   = 1 << 5;
        /// Flag indicating that the local mass-properties of this rigid-body must be recomputed.
        const LOCAL_MASS_PROPERTIES = 1 << 6;
        /// Flag indicating that the rigid-body was enabled or disabled.
        const ENABLED_OR_DISABLED = 1 << 7;
    }
}

impl Default for RigidBodyChanges {
    fn default() -> Self {
        RigidBodyChanges::empty()
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, Copy, PartialEq)]
/// The position of this rigid-body.
pub struct RigidBodyPosition {
    /// The world-space position of the rigid-body.
    pub position: Isometry,
    /// The next position of the rigid-body.
    ///
    /// At the beginning of the timestep, and when the
    /// timestep is complete we must have position == next_position
    /// except for kinematic bodies.
    ///
    /// The next_position is updated after the velocity and position
    /// resolution. Then it is either validated (ie. we set position := set_position)
    /// or clamped by CCD.
    pub next_position: Isometry,
}

impl Default for RigidBodyPosition {
    fn default() -> Self {
        Self {
            position: Isometry::identity(),
            next_position: Isometry::identity(),
        }
    }
}

impl RigidBodyPosition {
    /// Computes the velocity need to travel from `self.position` to `self.next_position` in
    /// a time equal to `1.0 / inv_dt`.
    #[must_use]
    pub fn interpolate_velocity(&self, inv_dt: Real, local_com: &Point) -> Velocity {
        let com = self.position.transform_point(local_com);
        let shift = Translation::from(com);
        let dpos = shift.inverse() * self.next_position * self.position.inverse() * shift;

        let angvel;
        #[cfg(feature = "dim2")]
        {
            angvel = dpos.rotation.angle() * inv_dt;
        }
        #[cfg(feature = "dim3")]
        {
            angvel = dpos.rotation.scaled_axis() * inv_dt;
        }
        let linvel = dpos.translation.as_vector() * inv_dt;

        Velocity { linvel, angvel }
    }

    /// Compute new positions after integrating the given forces and velocities.
    ///
    /// This uses a symplectic Euler integration scheme.
    #[must_use]
    pub fn integrate_forces_and_velocities(
        &self,
        dt: Real,
        forces: &RigidBodyForces,
        vels: &Velocity,
        mprops: &RigidBodyMassProps,
    ) -> Isometry {
        let new_vels = forces.integrate(dt, vels, mprops);
        new_vels.integrate(dt, &self.position, &mprops.local_mprops.local_com)
    }
}

impl<T> From<T> for RigidBodyPosition
where
    Isometry: From<T>,
{
    fn from(position: T) -> Self {
        let position = position.into();
        Self {
            position,
            next_position: position,
        }
    }
}

bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting the behavior of the constraints solver for a given contact manifold.
    #[cfg_attr(
        feature = "bevy",
        derive(Component, Reflect),
        reflect(Component, PartialEq)
    )]
    pub struct LockedAxes: u8 {
        /// Flag indicating that the rigid-body cannot translate along the `X` axis.
        const TRANSLATION_LOCKED_X = 1 << 0;
        /// Flag indicating that the rigid-body cannot translate along the `Y` axis.
        const TRANSLATION_LOCKED_Y = 1 << 1;
        /// Flag indicating that the rigid-body cannot translate along the `Z` axis.
        const TRANSLATION_LOCKED_Z = 1 << 2;
        /// Flag indicating that the rigid-body cannot translate along any direction.
        const TRANSLATION_LOCKED = Self::TRANSLATION_LOCKED_X.bits | Self::TRANSLATION_LOCKED_Y.bits | Self::TRANSLATION_LOCKED_Z.bits;
        /// Flag indicating that the rigid-body cannot rotate along the `X` axis.
        const ROTATION_LOCKED_X = 1 << 3;
        /// Flag indicating that the rigid-body cannot rotate along the `Y` axis.
        const ROTATION_LOCKED_Y = 1 << 4;
        /// Flag indicating that the rigid-body cannot rotate along the `Z` axis.
        const ROTATION_LOCKED_Z = 1 << 5;
        /// Combination of flags indicating that the rigid-body cannot rotate along any axis.
        const ROTATION_LOCKED = Self::ROTATION_LOCKED_X.bits | Self::ROTATION_LOCKED_Y.bits | Self::ROTATION_LOCKED_Z.bits;
    }
}

/// Mass and angular inertia added to a rigid-body on top of its attached colliders’ contributions.
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "bevy",
    derive(Component, Reflect),
    reflect(Component, PartialEq)
)]
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AdditionalMassProperties {
    /// Mass properties to be added as-is to the rigid-body.
    MassProperties(MassProperties),
    /// Mass to be added to the rigid-body. This will also automatically scale
    /// the attached colliders total angular inertia to account for the added mass.
    Mass(Real),
}

impl Default for AdditionalMassProperties {
    fn default() -> Self {
        AdditionalMassProperties::MassProperties(MassProperties::default())
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, PartialEq)]
/// The mass properties of a rigid-body.
pub struct RigidBodyMassProps {
    /// Flags for locking rotation and translation.
    pub flags: LockedAxes,
    /// The local mass properties of the rigid-body.
    pub local_mprops: MassProperties,
    /// Mass-properties of this rigid-bodies, added to the contributions of its attached colliders.
    pub additional_local_mprops: Option<Box<AdditionalMassProperties>>,
    /// The world-space center of mass of the rigid-body.
    pub world_com: Point,
    /// The inverse mass taking into account translation locking.
    pub effective_inv_mass: Vector,
    /// The square-root of the world-space inverse angular inertia tensor of the rigid-body,
    /// taking into account rotation locking.
    pub effective_world_inv_inertia_sqrt: AngularInertia<Real>,
}

impl Default for RigidBodyMassProps {
    fn default() -> Self {
        Self {
            flags: LockedAxes::empty(),
            local_mprops: MassProperties::zero(),
            additional_local_mprops: None,
            world_com: Point::origin(),
            effective_inv_mass: Vector::default(),
            effective_world_inv_inertia_sqrt: AngularInertia::zero(),
        }
    }
}

impl From<LockedAxes> for RigidBodyMassProps {
    fn from(flags: LockedAxes) -> Self {
        Self {
            flags,
            ..Self::default()
        }
    }
}

impl From<MassProperties> for RigidBodyMassProps {
    fn from(local_mprops: MassProperties) -> Self {
        Self {
            local_mprops,
            ..Default::default()
        }
    }
}

impl RigidBodyMassProps {
    /// The mass of the rigid-body.
    #[must_use]
    pub fn mass(&self) -> Real {
        crate::utils::inv(self.local_mprops.inv_mass)
    }

    /// The effective mass (that takes the potential translation locking into account) of
    /// this rigid-body.
    #[must_use]
    pub fn effective_mass(&self) -> Vector {
        self.effective_inv_mass.map(crate::utils::inv)
    }

    /// The square root of the effective world-space angular inertia (that takes the potential rotation locking into account) of
    /// this rigid-body.
    #[must_use]
    pub fn effective_angular_inertia_sqrt(&self) -> AngularInertia<Real> {
        #[allow(unused_mut)] // mut needed in 3D.
        let mut ang_inertia = self.effective_world_inv_inertia_sqrt;

        // Make the matrix invertible.
        #[cfg(feature = "dim3")]
        {
            if self.flags.contains(LockedAxes::ROTATION_LOCKED_X) {
                ang_inertia.m11 = 1.0;
            }
            if self.flags.contains(LockedAxes::ROTATION_LOCKED_Y) {
                ang_inertia.m22 = 1.0;
            }
            if self.flags.contains(LockedAxes::ROTATION_LOCKED_Z) {
                ang_inertia.m33 = 1.0;
            }
        }

        #[allow(unused_mut)] // mut needed in 3D.
        let mut result = ang_inertia.inverse();

        // Remove the locked axes again.
        #[cfg(feature = "dim3")]
        {
            if self.flags.contains(LockedAxes::ROTATION_LOCKED_X) {
                result.m11 = 0.0;
            }
            if self.flags.contains(LockedAxes::ROTATION_LOCKED_Y) {
                result.m22 = 0.0;
            }
            if self.flags.contains(LockedAxes::ROTATION_LOCKED_Z) {
                result.m33 = 0.0;
            }
        }

        result
    }

    /// The effective world-space angular inertia (that takes the potential rotation locking into account) of
    /// this rigid-body.
    #[must_use]
    pub fn effective_angular_inertia(&self) -> AngularInertia<Real> {
        self.effective_angular_inertia_sqrt().squared()
    }

    /// Recompute the mass-properties of this rigid-bodies based on its currently attached colliders.
    pub fn recompute_mass_properties_from_colliders(
        &mut self,
        colliders: &ColliderSet,
        attached_colliders: &RigidBodyColliders,
        position: &Isometry,
    ) {
        let added_mprops = self
            .additional_local_mprops
            .as_ref()
            .map(|mprops| **mprops)
            .unwrap_or_else(|| AdditionalMassProperties::MassProperties(MassProperties::default()));

        self.local_mprops = MassProperties::default();

        for handle in &attached_colliders.0 {
            if let Some(co) = colliders.get(*handle) {
                if co.is_enabled() {
                    if let Some(co_parent) = co.parent {
                        let to_add = co
                            .mprops
                            .mass_properties(&*co.shape)
                            .transform_by(&co_parent.pos_wrt_parent);
                        self.local_mprops += to_add;
                    }
                }
            }
        }

        match added_mprops {
            AdditionalMassProperties::MassProperties(mprops) => {
                self.local_mprops += mprops;
            }
            AdditionalMassProperties::Mass(mass) => {
                let new_mass = self.local_mprops.mass() + mass;
                self.local_mprops.set_mass(new_mass, true);
            }
        }

        self.update_world_mass_properties(position);
    }

    /// Update the world-space mass properties of `self`, taking into account the new position.
    pub fn update_world_mass_properties(&mut self, position: &Isometry) {
        self.world_com = self.local_mprops.world_com(position);
        self.effective_inv_mass = Vector::repeat(self.local_mprops.inv_mass);
        self.effective_world_inv_inertia_sqrt =
            self.local_mprops.world_inv_inertia_sqrt(&position.rotation);

        // Take into account translation/rotation locking.
        if self.flags.contains(LockedAxes::TRANSLATION_LOCKED_X) {
            self.effective_inv_mass.x = 0.0;
        }

        if self.flags.contains(LockedAxes::TRANSLATION_LOCKED_Y) {
            self.effective_inv_mass.y = 0.0;
        }

        #[cfg(feature = "dim3")]
        if self.flags.contains(LockedAxes::TRANSLATION_LOCKED_Z) {
            self.effective_inv_mass.z = 0.0;
        }

        #[cfg(feature = "dim2")]
        {
            if self.flags.contains(LockedAxes::ROTATION_LOCKED_Z) {
                self.effective_world_inv_inertia_sqrt = 0.0;
            }
        }
        #[cfg(feature = "dim3")]
        {
            if self.flags.contains(LockedAxes::ROTATION_LOCKED_X) {
                self.effective_world_inv_inertia_sqrt.m11 = 0.0;
                self.effective_world_inv_inertia_sqrt.m12 = 0.0;
                self.effective_world_inv_inertia_sqrt.m13 = 0.0;
            }

            if self.flags.contains(LockedAxes::ROTATION_LOCKED_Y) {
                self.effective_world_inv_inertia_sqrt.m22 = 0.0;
                self.effective_world_inv_inertia_sqrt.m12 = 0.0;
                self.effective_world_inv_inertia_sqrt.m23 = 0.0;
            }
            if self.flags.contains(LockedAxes::ROTATION_LOCKED_Z) {
                self.effective_world_inv_inertia_sqrt.m33 = 0.0;
                self.effective_world_inv_inertia_sqrt.m13 = 0.0;
                self.effective_world_inv_inertia_sqrt.m23 = 0.0;
            }
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "bevy",
    derive(Component, Reflect),
    reflect(Component, PartialEq)
)]
#[derive(Clone, Debug, Copy, PartialEq, Default)]
/// The velocities of this rigid-body.
pub struct Velocity {
    /// The linear velocity of the rigid-body.
    pub linvel: Vector,
    /// The angular velocity of the rigid-body.
    pub angvel: AngVector,
}

impl Velocity {
    /// Create a new rigid-body velocity component.
    #[must_use]
    pub fn new(linvel: Vector, angvel: AngVector) -> Self {
        Self { linvel, angvel }
    }

    /// Converts a slice to a rigid-body velocity.
    ///
    /// The slice must contain at least 3 elements: the `slice[0..2]` contains
    /// the linear velocity and the `slice[2]` contains the angular velocity.
    #[must_use]
    #[cfg(feature = "dim2")]
    pub fn from_slice(slice: &[Real]) -> Self {
        Self {
            linvel: Vector::new(slice[0], slice[1]),
            angvel: slice[2],
        }
    }

    /// Converts a slice to a rigid-body velocity.
    ///
    /// The slice must contain at least 6 elements: the `slice[0..3]` contains
    /// the linear velocity and the `slice[3..6]` contains the angular velocity.
    #[must_use]
    #[cfg(feature = "dim3")]
    pub fn from_slice(slice: &[Real]) -> Self {
        Self {
            linvel: Vector::new(slice[0], slice[1], slice[2]),
            angvel: AngVector::new(slice[3], slice[4], slice[5]),
        }
    }

    /// Velocities set to zero.
    #[must_use]
    pub fn zero() -> Self {
        Self::default()
    }

    /// This velocity seen as a slice.
    ///
    /// The linear part is stored first.
    #[inline]
    pub fn as_slice(&self) -> &[Real] {
        self.as_vector().as_slice()
    }

    /// This velocity seen as a mutable slice.
    ///
    /// The linear part is stored first.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [Real] {
        self.as_vector_mut().as_mut_slice()
    }

    /// This velocity seen as a vector.
    ///
    /// The linear part is stored first.
    #[inline]
    #[cfg(feature = "dim2")]
    pub fn as_vector(&self) -> &na::Vector3<Real> {
        unsafe { std::mem::transmute(self) }
    }

    /// This velocity seen as a mutable vector.
    ///
    /// The linear part is stored first.
    #[inline]
    #[cfg(feature = "dim2")]
    pub fn as_vector_mut(&mut self) -> &mut na::Vector3<Real> {
        unsafe { std::mem::transmute(self) }
    }

    /// This velocity seen as a vector.
    ///
    /// The linear part is stored first.
    #[inline]
    #[cfg(feature = "dim3")]
    pub fn as_vector(&self) -> &na::Vector6<Real> {
        unsafe { std::mem::transmute(self) }
    }

    /// This velocity seen as a mutable vector.
    ///
    /// The linear part is stored first.
    #[inline]
    #[cfg(feature = "dim3")]
    pub fn as_vector_mut(&mut self) -> &mut na::Vector6<Real> {
        unsafe { std::mem::transmute(self) }
    }

    /// Return `self` rotated by `rotation`.
    #[must_use]
    pub fn transformed(self, rotation: &Rotation) -> Self {
        Self {
            linvel: *rotation * self.linvel,
            #[cfg(feature = "dim2")]
            angvel: self.angvel,
            #[cfg(feature = "dim3")]
            angvel: *rotation * self.angvel,
        }
    }

    /// The approximate kinetic energy of this rigid-body.
    ///
    /// This approximation does not take the rigid-body's mass and angular inertia
    /// into account.
    #[must_use]
    pub fn pseudo_kinetic_energy(&self) -> Real {
        self.linvel.norm_squared() + self.angvel.gdot(self.angvel)
    }

    /// Returns the update velocities after applying the given damping.
    #[must_use]
    pub fn apply_damping(&self, dt: Real, damping: &Damping) -> Self {
        Velocity {
            linvel: self.linvel * (1.0 / (1.0 + dt * damping.linear_damping)),
            angvel: self.angvel * (1.0 / (1.0 + dt * damping.angular_damping)),
        }
    }

    /// The velocity of the given world-space point on this rigid-body.
    #[must_use]
    pub fn velocity_at_point(&self, point: &Point, world_com: &Point) -> Vector {
        let dpt = *point - *world_com;
        self.linvel + self.angvel.gcross(dpt)
    }

    /// Integrate the velocities in `self` to compute obtain new positions when moving from the given
    /// inital position `init_pos`.
    #[must_use]
    pub fn integrate(&self, dt: Real, init_pos: &Isometry, local_com: &Point) -> Isometry {
        let com = init_pos.transform_point(local_com);
        let shift = Translation::from(com);
        let mut result =
            shift * Isometry::new(self.linvel * dt, self.angvel * dt) * shift.inverse() * init_pos;
        result.rotation.renormalize_fast();
        result
    }

    /// Are these velocities exactly equal to zero?
    #[must_use]
    pub fn is_zero(&self) -> bool {
        self.linvel.is_zero() && self.angvel.is_zero()
    }

    /// The kinetic energy of this rigid-body.
    #[must_use]
    pub fn kinetic_energy(&self, rb_mprops: &RigidBodyMassProps) -> Real {
        let mut energy = (rb_mprops.mass() * self.linvel.norm_squared()) / 2.0;

        #[cfg(feature = "dim2")]
        if !rb_mprops.effective_world_inv_inertia_sqrt.is_zero() {
            let inertia_sqrt = 1.0 / rb_mprops.effective_world_inv_inertia_sqrt;
            energy += (inertia_sqrt * self.angvel).powi(2) / 2.0;
        }

        #[cfg(feature = "dim3")]
        if !rb_mprops.effective_world_inv_inertia_sqrt.is_zero() {
            let inertia_sqrt = rb_mprops
                .effective_world_inv_inertia_sqrt
                .inverse_unchecked();
            energy += (inertia_sqrt * self.angvel).norm_squared() / 2.0;
        }

        energy
    }

    /// Applies an impulse at the center-of-mass of this rigid-body.
    /// The impulse is applied right away, changing the linear velocity.
    /// This does nothing on non-dynamic bodies.
    pub fn apply_impulse(&mut self, rb_mprops: &RigidBodyMassProps, impulse: Vector) {
        self.linvel += impulse.component_mul(&rb_mprops.effective_inv_mass);
    }

    /// Applies an angular impulse at the center-of-mass of this rigid-body.
    /// The impulse is applied right away, changing the angular velocity.
    /// This does nothing on non-dynamic bodies.
    #[cfg(feature = "dim2")]
    pub fn apply_torque_impulse(&mut self, rb_mprops: &RigidBodyMassProps, torque_impulse: Real) {
        self.angvel += rb_mprops.effective_world_inv_inertia_sqrt
            * (rb_mprops.effective_world_inv_inertia_sqrt * torque_impulse);
    }

    /// Applies an angular impulse at the center-of-mass of this rigid-body.
    /// The impulse is applied right away, changing the angular velocity.
    /// This does nothing on non-dynamic bodies.
    #[cfg(feature = "dim3")]
    pub fn apply_torque_impulse(&mut self, rb_mprops: &RigidBodyMassProps, torque_impulse: Vector) {
        self.angvel += rb_mprops.effective_world_inv_inertia_sqrt
            * (rb_mprops.effective_world_inv_inertia_sqrt * torque_impulse);
    }

    /// Applies an impulse at the given world-space point of this rigid-body.
    /// The impulse is applied right away, changing the linear and/or angular velocities.
    /// This does nothing on non-dynamic bodies.
    pub fn apply_impulse_at_point(
        &mut self,
        rb_mprops: &RigidBodyMassProps,
        impulse: Vector,
        point: Point,
    ) {
        let torque_impulse = (point - rb_mprops.world_com).gcross(impulse);
        self.apply_impulse(rb_mprops, impulse);
        self.apply_torque_impulse(rb_mprops, torque_impulse);
    }
}

impl std::ops::Mul<Real> for Velocity {
    type Output = Self;

    #[must_use]
    fn mul(self, rhs: Real) -> Self {
        Velocity {
            linvel: self.linvel * rhs,
            angvel: self.angvel * rhs,
        }
    }
}

impl std::ops::Add<Velocity> for Velocity {
    type Output = Self;

    #[must_use]
    fn add(self, rhs: Self) -> Self {
        Velocity {
            linvel: self.linvel + rhs.linvel,
            angvel: self.angvel + rhs.angvel,
        }
    }
}

impl std::ops::AddAssign<Velocity> for Velocity {
    fn add_assign(&mut self, rhs: Self) {
        self.linvel += rhs.linvel;
        self.angvel += rhs.angvel;
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "bevy",
    derive(Component, Reflect),
    reflect(Component, PartialEq)
)]
#[derive(Clone, Debug, Copy, PartialEq)]
/// Damping factors to progressively slow down a rigid-body.
pub struct Damping {
    /// Damping factor for gradually slowing down the translational motion of the rigid-body.
    pub linear_damping: Real,
    /// Damping factor for gradually slowing down the angular motion of the rigid-body.
    pub angular_damping: Real,
}

impl Default for Damping {
    fn default() -> Self {
        Self {
            linear_damping: 0.0,
            angular_damping: 0.0,
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, Copy, PartialEq)]
/// The user-defined external forces applied to this rigid-body.
pub struct RigidBodyForces {
    /// Accumulation of external forces (only for dynamic bodies).
    pub force: Vector,
    /// Accumulation of external torques (only for dynamic bodies).
    pub torque: AngVector,
    /// Gravity is multiplied by this scaling factor before it's
    /// applied to this rigid-body.
    pub gravity_scale: Real,
    /// Forces applied by the user.
    pub user_force: Vector,
    /// Torque applied by the user.
    pub user_torque: AngVector,
}

impl Default for RigidBodyForces {
    fn default() -> Self {
        Self {
            force: Default::default(),
            torque: Default::default(),
            gravity_scale: 1.0,
            user_force: Default::default(),
            user_torque: Default::default(),
        }
    }
}

impl RigidBodyForces {
    /// Integrate these forces to compute new velocities.
    #[must_use]
    pub fn integrate(
        &self,
        dt: Real,
        init_vels: &Velocity,
        mprops: &RigidBodyMassProps,
    ) -> Velocity {
        let linear_acc = self.force.component_mul(&mprops.effective_inv_mass);
        let angular_acc = mprops.effective_world_inv_inertia_sqrt
            * (mprops.effective_world_inv_inertia_sqrt * self.torque);

        Velocity {
            linvel: init_vels.linvel + linear_acc * dt,
            angvel: init_vels.angvel + angular_acc * dt,
        }
    }

    /// Adds to `self` the gravitational force that would result in a gravitational acceleration
    /// equal to `gravity`.
    pub fn compute_effective_force_and_torque(&mut self, gravity: &Vector, mass: &Vector) {
        self.force = self.user_force + gravity.component_mul(mass) * self.gravity_scale;
        self.torque = self.user_torque;
    }

    /// Applies a force at the given world-space point of the rigid-body with the given mass properties.
    pub fn apply_force_at_point(
        &mut self,
        rb_mprops: &RigidBodyMassProps,
        force: Vector,
        point: Point,
    ) {
        self.user_force += force;
        self.user_torque += (point - rb_mprops.world_com).gcross(force);
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "bevy",
    derive(Component, Reflect),
    reflect(Component, PartialEq)
)]
#[derive(Clone, Debug, Copy, PartialEq)]
/// Information used for Continuous-Collision-Detection.
pub struct Ccd {
    /// The distance used by the CCD solver to decide if a movement would
    /// result in a tunnelling problem.
    pub ccd_thickness: Real,
    /// The max distance between this rigid-body's center of mass and its
    /// furthest collider point.
    pub ccd_max_dist: Real,
    /// Is CCD active for this rigid-body?
    ///
    /// If `self.ccd_enabled` is `true`, then this is automatically set to
    /// `true` when the CCD solver detects that the rigid-body is moving fast
    /// enough to potential cause a tunneling problem.
    pub active: bool,
    /// Is CCD enabled for this rigid-body?
    pub enabled: bool,
}

impl Default for Ccd {
    fn default() -> Self {
        Self {
            ccd_thickness: Real::MAX,
            ccd_max_dist: 0.0,
            active: false,
            enabled: false,
        }
    }
}

impl Ccd {
    /// Enable CCD for a [`RigidBody`].
    pub fn enabled() -> Self {
        Self {
            enabled: true,
            ..Default::default()
        }
    }

    /// Disable CCD for a [`RigidBody`].
    ///
    /// Note that a [`RigidBody`] without the Ccd component attached
    /// has CCD disabled by default.
    pub fn disabled() -> Self {
        Self::default()
    }

    /// The maximum velocity any point of any collider attached to this rigid-body
    /// moving with the given velocity can have.
    pub fn max_point_velocity(&self, vels: &Velocity) -> Real {
        #[cfg(feature = "dim2")]
        return vels.linvel.norm() + vels.angvel.abs() * self.ccd_max_dist;
        #[cfg(feature = "dim3")]
        return vels.linvel.norm() + vels.angvel.norm() * self.ccd_max_dist;
    }

    /// Is this rigid-body moving fast enough so that it may cause a tunneling problem?
    pub fn is_moving_fast(
        &self,
        dt: Real,
        vels: &Velocity,
        forces: Option<&RigidBodyForces>,
    ) -> bool {
        // NOTE: for the threshold we don't use the exact CCD thickness. Theoretically, we
        //       should use `self.rb_ccd.ccd_thickness - smallest_contact_dist` where `smallest_contact_dist`
        //       is the deepest contact (the contact with the largest penetration depth, i.e., the
        //       negative `dist` with the largest absolute value.
        //       However, getting this penetration depth assumes querying the contact graph from
        //       the narrow-phase, which can be pretty expensive. So we use the CCD thickness
        //       divided by 10 right now. We will see in practice if this value is OK or if we
        //       should use a smaller (to be less conservative) or larger divisor (to be more conservative).
        let threshold = self.ccd_thickness / 10.0;

        if let Some(forces) = forces {
            let linear_part = (vels.linvel + forces.force * dt).norm();
            #[cfg(feature = "dim2")]
            let angular_part = (vels.angvel + forces.torque * dt).abs() * self.ccd_max_dist;
            #[cfg(feature = "dim3")]
            let angular_part = (vels.angvel + forces.torque * dt).norm() * self.ccd_max_dist;
            let vel_with_forces = linear_part + angular_part;
            vel_with_forces > threshold
        } else {
            self.max_point_velocity(vels) * dt > threshold
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Default, Clone, Debug, Copy, PartialEq, Eq, Hash)]
/// Internal identifiers used by the physics engine.
pub struct RigidBodyIds {
    pub(crate) active_island_id: usize,
    pub(crate) active_set_id: usize,
    pub(crate) active_set_offset: usize,
    pub(crate) active_set_timestamp: u32,
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Default, Clone, Debug, PartialEq, Eq)]
/// The set of colliders attached to this rigid-bodies.
///
/// This should not be modified manually unless you really know what
/// you are doing (for example if you are trying to integrate Rapier
/// to a game engine using its component-based interface).
pub struct RigidBodyColliders(pub Vec<ColliderHandle>);

impl RigidBodyColliders {
    /// Detach a collider from this rigid-body.
    pub fn detach_collider(
        &mut self,
        rb_changes: &mut RigidBodyChanges,
        co_handle: ColliderHandle,
    ) {
        if let Some(i) = self.0.iter().position(|e| *e == co_handle) {
            rb_changes.set(
                RigidBodyChanges::MODIFIED | RigidBodyChanges::COLLIDERS,
                true,
            );
            self.0.swap_remove(i);
        }
    }

    /// Attach a collider to this rigid-body.
    pub fn attach_collider(
        &mut self,
        rb_changes: &mut RigidBodyChanges,
        rb_ccd: &mut Ccd,
        rb_mprops: &mut RigidBodyMassProps,
        rb_pos: &RigidBodyPosition,
        co_handle: ColliderHandle,
        co_pos: &mut ColliderPosition,
        co_parent: &ColliderParent,
        co_shape: &ColliderShape,
        co_mprops: &ColliderMassProperties,
    ) {
        rb_changes.set(
            RigidBodyChanges::MODIFIED | RigidBodyChanges::COLLIDERS,
            true,
        );

        co_pos.0 = rb_pos.position * co_parent.pos_wrt_parent;
        rb_ccd.ccd_thickness = rb_ccd.ccd_thickness.min(co_shape.ccd_thickness());

        let shape_bsphere = co_shape.compute_bounding_sphere(&co_parent.pos_wrt_parent);
        rb_ccd.ccd_max_dist = rb_ccd
            .ccd_max_dist
            .max(shape_bsphere.center.as_vector().norm() + shape_bsphere.radius);

        let mass_properties = co_mprops
            .mass_properties(&**co_shape)
            .transform_by(&co_parent.pos_wrt_parent);
        self.0.push(co_handle);
        rb_mprops.local_mprops += mass_properties;
        rb_mprops.update_world_mass_properties(&rb_pos.position);
    }

    /// Update the positions of all the colliders attached to this rigid-body.
    pub fn update_positions(
        &self,
        colliders: &mut ColliderSet,
        modified_colliders: &mut Vec<ColliderHandle>,
        parent_pos: &Isometry,
    ) {
        for handle in &self.0 {
            // NOTE: the ColliderParent component must exist if we enter this method.
            let co = colliders.index_mut_internal(*handle);
            let new_pos = parent_pos * co.parent.as_ref().unwrap().pos_wrt_parent;

            if !co.changes.contains(ColliderChanges::MODIFIED) {
                modified_colliders.push(*handle);
            }

            // Set the modification flag so we can benefit from the modification-tracking
            // when updating the narrow-phase/broad-phase afterwards.
            co.changes |= ColliderChanges::POSITION;
            co.pos = ColliderPosition(new_pos);
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[cfg_attr(
    feature = "bevy",
    derive(Component, Reflect),
    reflect(Component, PartialEq)
)]
#[derive(Default, Clone, Debug, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// The dominance groups of a rigid-body.
pub struct Dominance {
    pub groups: i8,
}

impl Dominance {
    /// The actual dominance group of this rigid-body, after taking into account its type.
    pub fn effective_group(&self, status: &RigidBodyType) -> i16 {
        if status.is_dynamic() {
            self.groups as i16
        } else {
            i8::MAX as i16 + 1
        }
    }
}

/// The rb_activation status of a body.
///
/// This controls whether a body is sleeping or not.
/// If the threshold is negative, the body never sleeps.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(
    feature = "bevy",
    derive(Component, Reflect),
    reflect(Component, PartialEq)
)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct SleepState {
    /// The threshold linear velocity bellow which the body can fall asleep.
    pub linear_threshold: Real,
    /// The angular linear velocity bellow which the body can fall asleep.
    pub angular_threshold: Real,
    /// The amount of time the rigid-body must remain below the thresholds to be put to sleep.
    pub time_until_sleep: Real,
    /// Since how much time can this body sleep?
    pub time_since_can_sleep: Real,
    /// Is this body sleeping?
    pub sleeping: bool,
}

impl Default for SleepState {
    fn default() -> Self {
        Self::active()
    }
}

impl SleepState {
    /// The default linear velocity bellow which a body can be put to sleep.
    pub fn default_linear_threshold() -> Real {
        0.4
    }

    /// The default angular velocity bellow which a body can be put to sleep.
    pub fn default_angular_threshold() -> Real {
        0.5
    }

    /// The amount of time the rigid-body must remain bellow it’s linear and angular velocity
    /// threshold before falling to sleep.
    pub fn default_time_until_sleep() -> Real {
        2.0
    }

    /// Create a new rb_activation status initialised with the default rb_activation threshold and is active.
    pub fn active() -> Self {
        SleepState {
            linear_threshold: Self::default_linear_threshold(),
            angular_threshold: Self::default_angular_threshold(),
            time_until_sleep: Self::default_time_until_sleep(),
            time_since_can_sleep: 0.0,
            sleeping: false,
        }
    }

    /// Create a new rb_activation status initialised with the default rb_activation threshold and is inactive.
    pub fn inactive() -> Self {
        SleepState {
            linear_threshold: Self::default_linear_threshold(),
            angular_threshold: Self::default_angular_threshold(),
            time_until_sleep: Self::default_time_until_sleep(),
            time_since_can_sleep: Self::default_time_until_sleep(),
            sleeping: true,
        }
    }

    /// Create a new activation status that prevents the rigid-body from sleeping.
    pub fn cannot_sleep() -> Self {
        SleepState {
            linear_threshold: -1.0,
            angular_threshold: -1.0,
            ..Self::active()
        }
    }

    /// Returns `true` if the body is not asleep.
    #[inline]
    pub fn is_active(&self) -> bool {
        !self.sleeping
    }

    /// Wakes up this rigid-body.
    #[inline]
    pub fn wake_up(&mut self, strong: bool) {
        self.sleeping = false;
        if strong {
            self.time_since_can_sleep = 0.0;
        }
    }

    /// Put this rigid-body to sleep.
    #[inline]
    pub fn sleep(&mut self) {
        self.sleeping = true;
        self.time_since_can_sleep = self.time_until_sleep;
    }
}
