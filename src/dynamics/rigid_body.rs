use crate::dynamics::{
    LockedAxes, MassProperties, RigidBodyActivation, RigidBodyAdditionalMassProps, RigidBodyCcd,
    RigidBodyChanges, RigidBodyColliders, RigidBodyDamping, RigidBodyDominance, RigidBodyForces,
    RigidBodyIds, RigidBodyMassProps, RigidBodyPosition, RigidBodyType, RigidBodyVelocity,
};
use crate::geometry::{
    ColliderHandle, ColliderMassProps, ColliderParent, ColliderPosition, ColliderSet, ColliderShape,
};
use crate::math::{AngVector, Isometry, Point, Real, Rotation, Vector};
use crate::utils::WCross;
use num::Zero;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A rigid body.
///
/// To create a new rigid-body, use the [`RigidBodyBuilder`] structure.
#[derive(Debug, Clone)]
pub struct RigidBody {
    pub(crate) pos: RigidBodyPosition,
    pub(crate) mprops: RigidBodyMassProps,
    // NOTE: we need this so that the CCD can use the actual velocities obtained
    //       by the velocity solver with bias. If we switch to interpolation, we
    //       should remove this field.
    pub(crate) integrated_vels: RigidBodyVelocity,
    pub(crate) vels: RigidBodyVelocity,
    pub(crate) damping: RigidBodyDamping,
    pub(crate) forces: RigidBodyForces,
    pub(crate) ccd: RigidBodyCcd,
    pub(crate) ids: RigidBodyIds,
    pub(crate) colliders: RigidBodyColliders,
    /// Whether or not this rigid-body is sleeping.
    pub(crate) activation: RigidBodyActivation,
    pub(crate) changes: RigidBodyChanges,
    /// The status of the body, governing how it is affected by external forces.
    pub(crate) body_type: RigidBodyType,
    /// The dominance group this rigid-body is part of.
    pub(crate) dominance: RigidBodyDominance,
    pub(crate) enabled: bool,
    /// User-defined data associated to this rigid-body.
    pub user_data: u128,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self::new()
    }
}

impl RigidBody {
    fn new() -> Self {
        Self {
            pos: RigidBodyPosition::default(),
            mprops: RigidBodyMassProps::default(),
            integrated_vels: RigidBodyVelocity::default(),
            vels: RigidBodyVelocity::default(),
            damping: RigidBodyDamping::default(),
            forces: RigidBodyForces::default(),
            ccd: RigidBodyCcd::default(),
            ids: RigidBodyIds::default(),
            colliders: RigidBodyColliders::default(),
            activation: RigidBodyActivation::active(),
            changes: RigidBodyChanges::all(),
            body_type: RigidBodyType::Dynamic,
            dominance: RigidBodyDominance::default(),
            enabled: true,
            user_data: 0,
        }
    }

    pub(crate) fn reset_internal_references(&mut self) {
        self.colliders.0 = Vec::new();
        self.ids = Default::default();
    }

    /// The activation status of this rigid-body.
    pub fn activation(&self) -> &RigidBodyActivation {
        &self.activation
    }

    /// Mutable reference to the activation status of this rigid-body.
    pub fn activation_mut(&mut self) -> &mut RigidBodyActivation {
        self.changes |= RigidBodyChanges::SLEEP;
        &mut self.activation
    }

    /// Is this rigid-body enabled?
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Sets whether this rigid-body is enabled or not.
    pub fn set_enabled(&mut self, enabled: bool) {
        if enabled != self.enabled {
            if enabled {
                // NOTE: this is probably overkill, but it makes sure we don’t
                // forget anything that needs to be updated because the rigid-body
                // was basically interpreted as if it was removed while it was
                // disabled.
                self.changes = RigidBodyChanges::all();
            } else {
                self.changes |= RigidBodyChanges::ENABLED_OR_DISABLED;
            }

            self.enabled = enabled;
        }
    }

    /// The linear damping coefficient of this rigid-body.
    #[inline]
    pub fn linear_damping(&self) -> Real {
        self.damping.linear_damping
    }

    /// Sets the linear damping coefficient of this rigid-body.
    #[inline]
    pub fn set_linear_damping(&mut self, damping: Real) {
        self.damping.linear_damping = damping;
    }

    /// The angular damping coefficient of this rigid-body.
    #[inline]
    pub fn angular_damping(&self) -> Real {
        self.damping.angular_damping
    }

    /// Sets the angular damping coefficient of this rigid-body.
    #[inline]
    pub fn set_angular_damping(&mut self, damping: Real) {
        self.damping.angular_damping = damping
    }

    /// The type of this rigid-body.
    pub fn body_type(&self) -> RigidBodyType {
        self.body_type
    }

    /// Sets the type of this rigid-body.
    pub fn set_body_type(&mut self, status: RigidBodyType, wake_up: bool) {
        if status != self.body_type {
            self.changes.insert(RigidBodyChanges::TYPE);
            self.body_type = status;

            if status == RigidBodyType::Fixed {
                self.vels = RigidBodyVelocity::zero();
            }

            if self.is_dynamic() && wake_up {
                self.wake_up(true);
            }
        }
    }

    /// The world-space center-of-mass of this rigid-body.
    #[inline]
    pub fn center_of_mass(&self) -> &Point<Real> {
        &self.mprops.world_com
    }

    /// The mass-properties of this rigid-body.
    #[inline]
    pub fn mass_properties(&self) -> &RigidBodyMassProps {
        &self.mprops
    }

    /// The dominance group of this rigid-body.
    ///
    /// This method always returns `i8::MAX + 1` for non-dynamic
    /// rigid-bodies.
    #[inline]
    pub fn effective_dominance_group(&self) -> i16 {
        self.dominance.effective_group(&self.body_type)
    }

    /// Sets the axes along which this rigid-body cannot translate or rotate.
    #[inline]
    pub fn set_locked_axes(&mut self, locked_axes: LockedAxes, wake_up: bool) {
        if locked_axes != self.mprops.flags {
            if self.is_dynamic() && wake_up {
                self.wake_up(true);
            }

            self.mprops.flags = locked_axes;
            self.update_world_mass_properties();
        }
    }

    /// The axes along which this rigid-body cannot translate or rotate.
    #[inline]
    pub fn locked_axes(&self) -> LockedAxes {
        self.mprops.flags
    }

    #[inline]
    /// Locks or unlocks all the rotations of this rigid-body.
    pub fn lock_rotations(&mut self, locked: bool, wake_up: bool) {
        if locked != self.mprops.flags.contains(LockedAxes::ROTATION_LOCKED) {
            if self.is_dynamic() && wake_up {
                self.wake_up(true);
            }

            self.mprops.flags.set(LockedAxes::ROTATION_LOCKED_X, locked);
            self.mprops.flags.set(LockedAxes::ROTATION_LOCKED_Y, locked);
            self.mprops.flags.set(LockedAxes::ROTATION_LOCKED_Z, locked);
            self.update_world_mass_properties();
        }
    }

    #[inline]
    /// Locks or unlocks rotations of this rigid-body along each cartesian axes.
    pub fn set_enabled_rotations(
        &mut self,
        allow_rotations_x: bool,
        allow_rotations_y: bool,
        allow_rotations_z: bool,
        wake_up: bool,
    ) {
        if self.mprops.flags.contains(LockedAxes::ROTATION_LOCKED_X) != !allow_rotations_x
            || self.mprops.flags.contains(LockedAxes::ROTATION_LOCKED_Y) != !allow_rotations_y
            || self.mprops.flags.contains(LockedAxes::ROTATION_LOCKED_Z) != !allow_rotations_z
        {
            if self.is_dynamic() && wake_up {
                self.wake_up(true);
            }

            self.mprops
                .flags
                .set(LockedAxes::ROTATION_LOCKED_X, !allow_rotations_x);
            self.mprops
                .flags
                .set(LockedAxes::ROTATION_LOCKED_Y, !allow_rotations_y);
            self.mprops
                .flags
                .set(LockedAxes::ROTATION_LOCKED_Z, !allow_rotations_z);
            self.update_world_mass_properties();
        }
    }

    /// Locks or unlocks rotations of this rigid-body along each cartesian axes.
    #[deprecated(note = "Use `set_enabled_rotations` instead")]
    pub fn restrict_rotations(
        &mut self,
        allow_rotations_x: bool,
        allow_rotations_y: bool,
        allow_rotations_z: bool,
        wake_up: bool,
    ) {
        self.set_enabled_rotations(
            allow_rotations_x,
            allow_rotations_y,
            allow_rotations_z,
            wake_up,
        );
    }

    #[inline]
    /// Locks or unlocks all the rotations of this rigid-body.
    pub fn lock_translations(&mut self, locked: bool, wake_up: bool) {
        if locked != self.mprops.flags.contains(LockedAxes::TRANSLATION_LOCKED) {
            if self.is_dynamic() && wake_up {
                self.wake_up(true);
            }

            self.mprops
                .flags
                .set(LockedAxes::TRANSLATION_LOCKED, locked);
            self.update_world_mass_properties();
        }
    }

    #[inline]
    /// Locks or unlocks rotations of this rigid-body along each cartesian axes.
    pub fn set_enabled_translations(
        &mut self,
        allow_translation_x: bool,
        allow_translation_y: bool,
        #[cfg(feature = "dim3")] allow_translation_z: bool,
        wake_up: bool,
    ) {
        #[cfg(feature = "dim2")]
        if self.mprops.flags.contains(LockedAxes::TRANSLATION_LOCKED_X) == !allow_translation_x
            && self.mprops.flags.contains(LockedAxes::TRANSLATION_LOCKED_Y) == !allow_translation_y
        {
            // Nothing to change.
            return;
        }
        #[cfg(feature = "dim3")]
        if self.mprops.flags.contains(LockedAxes::TRANSLATION_LOCKED_X) == !allow_translation_x
            && self.mprops.flags.contains(LockedAxes::TRANSLATION_LOCKED_Y) == !allow_translation_y
            && self.mprops.flags.contains(LockedAxes::TRANSLATION_LOCKED_Z) == !allow_translation_z
        {
            // Nothing to change.
            return;
        }

        if self.is_dynamic() && wake_up {
            self.wake_up(true);
        }

        self.mprops
            .flags
            .set(LockedAxes::TRANSLATION_LOCKED_X, !allow_translation_x);
        self.mprops
            .flags
            .set(LockedAxes::TRANSLATION_LOCKED_Y, !allow_translation_y);
        #[cfg(feature = "dim3")]
        self.mprops
            .flags
            .set(LockedAxes::TRANSLATION_LOCKED_Z, !allow_translation_z);
        self.update_world_mass_properties();
    }

    #[inline]
    #[deprecated(note = "Use `set_enabled_translations` instead")]
    /// Locks or unlocks rotations of this rigid-body along each cartesian axes.
    pub fn restrict_translations(
        &mut self,
        allow_translation_x: bool,
        allow_translation_y: bool,
        #[cfg(feature = "dim3")] allow_translation_z: bool,
        wake_up: bool,
    ) {
        self.set_enabled_translations(
            allow_translation_x,
            allow_translation_y,
            #[cfg(feature = "dim3")]
            allow_translation_z,
            wake_up,
        )
    }

    /// Are the translations of this rigid-body locked?
    #[cfg(feature = "dim2")]
    pub fn is_translation_locked(&self) -> bool {
        self.mprops
            .flags
            .contains(LockedAxes::TRANSLATION_LOCKED_X | LockedAxes::TRANSLATION_LOCKED_Y)
    }

    /// Are the translations of this rigid-body locked?
    #[cfg(feature = "dim3")]
    pub fn is_translation_locked(&self) -> bool {
        self.mprops.flags.contains(LockedAxes::TRANSLATION_LOCKED)
    }

    /// Are the rotations of this rigid-body locked?
    #[cfg(feature = "dim2")]
    pub fn is_rotation_locked(&self) -> bool {
        self.mprops.flags.contains(LockedAxes::ROTATION_LOCKED_Z)
    }

    /// Returns `true` for each rotational degrees of freedom locked on this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn is_rotation_locked(&self) -> [bool; 3] {
        [
            self.mprops.flags.contains(LockedAxes::ROTATION_LOCKED_X),
            self.mprops.flags.contains(LockedAxes::ROTATION_LOCKED_Y),
            self.mprops.flags.contains(LockedAxes::ROTATION_LOCKED_Z),
        ]
    }

    /// Enables of disable CCD (continuous collision-detection) for this rigid-body.
    ///
    /// CCD prevents tunneling, but may still allow limited interpenetration of colliders.
    pub fn enable_ccd(&mut self, enabled: bool) {
        self.ccd.ccd_enabled = enabled;
    }

    /// Is CCD (continous collision-detection) enabled for this rigid-body?
    pub fn is_ccd_enabled(&self) -> bool {
        self.ccd.ccd_enabled
    }

    // This is different from `is_ccd_enabled`. This checks that CCD
    // is active for this rigid-body, i.e., if it was seen to move fast
    // enough to justify a CCD run.
    /// Is CCD active for this rigid-body?
    ///
    /// The CCD is considered active if the rigid-body is moving at
    /// a velocity greater than an automatically-computed threshold.
    ///
    /// This is not the same as `self.is_ccd_enabled` which only
    /// checks if CCD is enabled to run for this rigid-body or if
    /// it is completely disabled (independently from its velocity).
    pub fn is_ccd_active(&self) -> bool {
        self.ccd.ccd_active
    }

    /// Recompute the mass-properties of this rigid-bodies based on its currently attached colliders.
    pub fn recompute_mass_properties_from_colliders(&mut self, colliders: &ColliderSet) {
        self.mprops.recompute_mass_properties_from_colliders(
            colliders,
            &self.colliders,
            &self.pos.position,
        );
    }

    /// Sets the rigid-body's additional mass.
    ///
    /// The total angular inertia of the rigid-body will be scaled automatically based on this
    /// additional mass. If this scaling effect isn’t desired, use [`Self::set_additional_mass_properties`]
    /// instead of this method.
    ///
    /// This is only the "additional" mass because the total mass of the  rigid-body is
    /// equal to the sum of this additional mass and the mass computed from the colliders
    /// (with non-zero densities) attached to this rigid-body.
    ///
    /// That total mass (which includes the attached colliders’ contributions)
    /// will be updated at the name physics step, or can be updated manually with
    /// [`Self::recompute_mass_properties_from_colliders`].
    ///
    /// This will override any previous mass-properties set by [`Self::set_additional_mass`],
    /// [`Self::set_additional_mass_properties`], [`RigidBodyBuilder::additional_mass`], or
    /// [`RigidBodyBuilder::additional_mass_properties`] for this rigid-body.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    #[inline]
    pub fn set_additional_mass(&mut self, additional_mass: Real, wake_up: bool) {
        self.do_set_additional_mass_properties(
            RigidBodyAdditionalMassProps::Mass(additional_mass),
            wake_up,
        )
    }

    /// Sets the rigid-body's additional mass-properties.
    ///
    /// This is only the "additional" mass-properties because the total mass-properties of the
    /// rigid-body is equal to the sum of this additional mass-properties and the mass computed from
    /// the colliders (with non-zero densities) attached to this rigid-body.
    ///
    /// That total mass-properties (which include the attached colliders’ contributions)
    /// will be updated at the name physics step, or can be updated manually with
    /// [`Self::recompute_mass_properties_from_colliders`].
    ///
    /// This will override any previous mass-properties set by [`Self::set_additional_mass`],
    /// [`Self::set_additional_mass_properties`], [`RigidBodyBuilder::additional_mass`], or
    /// [`RigidBodyBuilder::additional_mass_properties`] for this rigid-body.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    #[inline]
    pub fn set_additional_mass_properties(&mut self, props: MassProperties, wake_up: bool) {
        self.do_set_additional_mass_properties(
            RigidBodyAdditionalMassProps::MassProps(props),
            wake_up,
        )
    }

    fn do_set_additional_mass_properties(
        &mut self,
        props: RigidBodyAdditionalMassProps,
        wake_up: bool,
    ) {
        let new_mprops = Some(Box::new(props));

        if self.mprops.additional_local_mprops != new_mprops {
            self.changes.insert(RigidBodyChanges::LOCAL_MASS_PROPERTIES);
            self.mprops.additional_local_mprops = new_mprops;

            if self.is_dynamic() && wake_up {
                self.wake_up(true);
            }
        }
    }

    /// The handles of colliders attached to this rigid body.
    pub fn colliders(&self) -> &[ColliderHandle] {
        &self.colliders.0[..]
    }

    /// Is this rigid body dynamic?
    ///
    /// A dynamic body can move freely and is affected by forces.
    pub fn is_dynamic(&self) -> bool {
        self.body_type == RigidBodyType::Dynamic
    }

    /// Is this rigid body kinematic?
    ///
    /// A kinematic body can move freely but is not affected by forces.
    pub fn is_kinematic(&self) -> bool {
        self.body_type.is_kinematic()
    }

    /// Is this rigid body fixed?
    ///
    /// A fixed body cannot move and is not affected by forces.
    pub fn is_fixed(&self) -> bool {
        self.body_type == RigidBodyType::Fixed
    }

    /// The mass of this rigid body.
    ///
    /// Returns zero if this rigid body has an infinite mass.
    pub fn mass(&self) -> Real {
        self.mprops.local_mprops.mass()
    }

    /// The predicted position of this rigid-body.
    ///
    /// If this rigid-body is kinematic this value is set by the `set_next_kinematic_position`
    /// method and is used for estimating the kinematic body velocity at the next timestep.
    /// For non-kinematic bodies, this value is currently unspecified.
    pub fn next_position(&self) -> &Isometry<Real> {
        &self.pos.next_position
    }

    /// The scale factor applied to the gravity affecting this rigid-body.
    pub fn gravity_scale(&self) -> Real {
        self.forces.gravity_scale
    }

    /// Sets the gravity scale facter for this rigid-body.
    pub fn set_gravity_scale(&mut self, scale: Real, wake_up: bool) {
        if self.forces.gravity_scale != scale {
            if wake_up && self.activation.sleeping {
                self.changes.insert(RigidBodyChanges::SLEEP);
                self.activation.sleeping = false;
            }

            self.forces.gravity_scale = scale;
        }
    }

    /// The dominance group of this rigid-body.
    pub fn dominance_group(&self) -> i8 {
        self.dominance.0
    }

    /// The dominance group of this rigid-body.
    pub fn set_dominance_group(&mut self, dominance: i8) {
        if self.dominance.0 != dominance {
            self.changes.insert(RigidBodyChanges::DOMINANCE);
            self.dominance.0 = dominance
        }
    }

    /// Adds a collider to this rigid-body.
    // TODO ECS: we keep this public for now just to simply our experiments on bevy_rapier.
    pub fn add_collider(
        &mut self,
        co_handle: ColliderHandle,
        co_parent: &ColliderParent,
        co_pos: &mut ColliderPosition,
        co_shape: &ColliderShape,
        co_mprops: &ColliderMassProps,
    ) {
        self.colliders.attach_collider(
            &mut self.changes,
            &mut self.ccd,
            &mut self.mprops,
            &self.pos,
            co_handle,
            co_pos,
            co_parent,
            co_shape,
            co_mprops,
        )
    }

    /// Removes a collider from this rigid-body.
    pub(crate) fn remove_collider_internal(&mut self, handle: ColliderHandle) {
        if let Some(i) = self.colliders.0.iter().position(|e| *e == handle) {
            self.changes.set(RigidBodyChanges::COLLIDERS, true);
            self.colliders.0.swap_remove(i);
        }
    }

    /// Put this rigid body to sleep.
    ///
    /// A sleeping body no longer moves and is no longer simulated by the physics engine unless
    /// it is waken up. It can be woken manually with `self.wake_up` or automatically due to
    /// external forces like contacts.
    pub fn sleep(&mut self) {
        self.activation.sleep();
        self.vels = RigidBodyVelocity::zero();
    }

    /// Wakes up this rigid body if it is sleeping.
    ///
    /// If `strong` is `true` then it is assured that the rigid-body will
    /// remain awake during multiple subsequent timesteps.
    pub fn wake_up(&mut self, strong: bool) {
        if self.activation.sleeping {
            self.changes.insert(RigidBodyChanges::SLEEP);
        }

        self.activation.wake_up(strong);
    }

    /// Is this rigid body sleeping?
    pub fn is_sleeping(&self) -> bool {
        // TODO: should we:
        // - return false for fixed bodies.
        // - return true for non-sleeping dynamic bodies.
        // - return true only for kinematic bodies with non-zero velocity?
        self.activation.sleeping
    }

    /// Is the velocity of this body not zero?
    pub fn is_moving(&self) -> bool {
        !self.vels.linvel.is_zero() || !self.vels.angvel.is_zero()
    }

    /// The linear velocity of this rigid-body.
    pub fn linvel(&self) -> &Vector<Real> {
        &self.vels.linvel
    }

    /// The angular velocity of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn angvel(&self) -> Real {
        self.vels.angvel
    }

    /// The angular velocity of this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn angvel(&self) -> &Vector<Real> {
        &self.vels.angvel
    }

    /// The linear velocity of this rigid-body.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    pub fn set_linvel(&mut self, linvel: Vector<Real>, wake_up: bool) {
        if self.vels.linvel != linvel {
            match self.body_type {
                RigidBodyType::Dynamic => {
                    self.vels.linvel = linvel;
                    if wake_up {
                        self.wake_up(true)
                    }
                }
                RigidBodyType::KinematicVelocityBased => {
                    self.vels.linvel = linvel;
                }
                RigidBodyType::Fixed | RigidBodyType::KinematicPositionBased => {}
            }
        }
    }

    /// The angular velocity of this rigid-body.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    #[cfg(feature = "dim2")]
    pub fn set_angvel(&mut self, angvel: Real, wake_up: bool) {
        if self.vels.angvel != angvel {
            match self.body_type {
                RigidBodyType::Dynamic => {
                    self.vels.angvel = angvel;
                    if wake_up {
                        self.wake_up(true)
                    }
                }
                RigidBodyType::KinematicVelocityBased => {
                    self.vels.angvel = angvel;
                }
                RigidBodyType::Fixed | RigidBodyType::KinematicPositionBased => {}
            }
        }
    }

    /// The angular velocity of this rigid-body.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    #[cfg(feature = "dim3")]
    pub fn set_angvel(&mut self, angvel: Vector<Real>, wake_up: bool) {
        if self.vels.angvel != angvel {
            match self.body_type {
                RigidBodyType::Dynamic => {
                    self.vels.angvel = angvel;
                    if wake_up {
                        self.wake_up(true)
                    }
                }
                RigidBodyType::KinematicVelocityBased => {
                    self.vels.angvel = angvel;
                }
                RigidBodyType::Fixed | RigidBodyType::KinematicPositionBased => {}
            }
        }
    }

    /// The world-space position of this rigid-body.
    #[inline]
    pub fn position(&self) -> &Isometry<Real> {
        &self.pos.position
    }

    /// The translational part of this rigid-body's position.
    #[inline]
    pub fn translation(&self) -> &Vector<Real> {
        &self.pos.position.translation.vector
    }

    /// Sets the translational part of this rigid-body's position.
    #[inline]
    pub fn set_translation(&mut self, translation: Vector<Real>, wake_up: bool) {
        if self.pos.position.translation.vector != translation
            || self.pos.next_position.translation.vector != translation
        {
            self.changes.insert(RigidBodyChanges::POSITION);
            self.pos.position.translation.vector = translation;
            self.pos.next_position.translation.vector = translation;

            // Update the world mass-properties so torque application remains valid.
            self.update_world_mass_properties();

            // TODO: Do we really need to check that the body isn't dynamic?
            if wake_up && self.is_dynamic() {
                self.wake_up(true)
            }
        }
    }

    /// The rotational part of this rigid-body's position.
    #[inline]
    pub fn rotation(&self) -> &Rotation<Real> {
        &self.pos.position.rotation
    }

    /// Sets the rotational part of this rigid-body's position.
    #[inline]
    pub fn set_rotation(&mut self, rotation: Rotation<Real>, wake_up: bool) {
        if self.pos.position.rotation != rotation || self.pos.next_position.rotation != rotation {
            self.changes.insert(RigidBodyChanges::POSITION);
            self.pos.position.rotation = rotation;
            self.pos.next_position.rotation = rotation;

            // Update the world mass-properties so torque application remains valid.
            self.update_world_mass_properties();

            // TODO: Do we really need to check that the body isn't dynamic?
            if wake_up && self.is_dynamic() {
                self.wake_up(true)
            }
        }
    }

    /// Sets the position and `next_kinematic_position` of this rigid body.
    ///
    /// This will teleport the rigid-body to the specified position/orientation,
    /// completely ignoring any physics rule. If this body is kinematic, this will
    /// also set the next kinematic position to the same value, effectively
    /// resetting to zero the next interpolated velocity of the kinematic body.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    pub fn set_position(&mut self, pos: Isometry<Real>, wake_up: bool) {
        if self.pos.position != pos || self.pos.next_position != pos {
            self.changes.insert(RigidBodyChanges::POSITION);
            self.pos.position = pos;
            self.pos.next_position = pos;

            // Update the world mass-properties so torque application remains valid.
            self.update_world_mass_properties();

            // TODO: Do we really need to check that the body isn't dynamic?
            if wake_up && self.is_dynamic() {
                self.wake_up(true)
            }
        }
    }

    /// If this rigid body is kinematic, sets its future orientation after the next timestep integration.
    pub fn set_next_kinematic_rotation(&mut self, rotation: Rotation<Real>) {
        if self.is_kinematic() {
            self.pos.next_position.rotation = rotation;
        }
    }

    /// If this rigid body is kinematic, sets its future translation after the next timestep integration.
    pub fn set_next_kinematic_translation(&mut self, translation: Vector<Real>) {
        if self.is_kinematic() {
            self.pos.next_position.translation = translation.into();
        }
    }

    /// If this rigid body is kinematic, sets its future position (translation and orientation) after
    /// the next timestep integration.
    pub fn set_next_kinematic_position(&mut self, pos: Isometry<Real>) {
        if self.is_kinematic() {
            self.pos.next_position = pos;
        }
    }

    /// Predicts the next position of this rigid-body, by integrating its velocity and forces
    /// by a time of `dt`.
    pub fn predict_position_using_velocity_and_forces(&self, dt: Real) -> Isometry<Real> {
        self.pos
            .integrate_forces_and_velocities(dt, &self.forces, &self.vels, &self.mprops)
    }

    pub(crate) fn update_world_mass_properties(&mut self) {
        self.mprops.update_world_mass_properties(&self.pos.position);
    }
}

/// ## Applying forces and torques
impl RigidBody {
    /// Resets to zero all the constant (linear) forces manually applied to this rigid-body.
    pub fn reset_forces(&mut self, wake_up: bool) {
        if !self.forces.user_force.is_zero() {
            self.forces.user_force = na::zero();

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Resets to zero all the constant torques manually applied to this rigid-body.
    pub fn reset_torques(&mut self, wake_up: bool) {
        if !self.forces.user_torque.is_zero() {
            self.forces.user_torque = na::zero();

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Adds to this rigid-body a constant force applied at its center-of-mass.ç
    ///
    /// This does nothing on non-dynamic bodies.
    pub fn add_force(&mut self, force: Vector<Real>, wake_up: bool) {
        if !force.is_zero() {
            if self.body_type == RigidBodyType::Dynamic {
                self.forces.user_force += force;

                if wake_up {
                    self.wake_up(true);
                }
            }
        }
    }

    /// Adds to this rigid-body a constant torque at its center-of-mass.
    ///
    /// This does nothing on non-dynamic bodies.
    #[cfg(feature = "dim2")]
    pub fn add_torque(&mut self, torque: Real, wake_up: bool) {
        if !torque.is_zero() {
            if self.body_type == RigidBodyType::Dynamic {
                self.forces.user_torque += torque;

                if wake_up {
                    self.wake_up(true);
                }
            }
        }
    }

    /// Adds to this rigid-body a constant torque at its center-of-mass.
    ///
    /// This does nothing on non-dynamic bodies.
    #[cfg(feature = "dim3")]
    pub fn add_torque(&mut self, torque: Vector<Real>, wake_up: bool) {
        if !torque.is_zero() {
            if self.body_type == RigidBodyType::Dynamic {
                self.forces.user_torque += torque;

                if wake_up {
                    self.wake_up(true);
                }
            }
        }
    }

    /// Adds to this rigid-body a constant force at the given world-space point of this rigid-body.
    ///
    /// This does nothing on non-dynamic bodies.
    pub fn add_force_at_point(&mut self, force: Vector<Real>, point: Point<Real>, wake_up: bool) {
        if !force.is_zero() {
            if self.body_type == RigidBodyType::Dynamic {
                self.forces.user_force += force;
                self.forces.user_torque += (point - self.mprops.world_com).gcross(force);

                if wake_up {
                    self.wake_up(true);
                }
            }
        }
    }
}

/// ## Applying impulses and angular impulses
impl RigidBody {
    /// Applies an impulse at the center-of-mass of this rigid-body.
    /// The impulse is applied right away, changing the linear velocity.
    /// This does nothing on non-dynamic bodies.
    pub fn apply_impulse(&mut self, impulse: Vector<Real>, wake_up: bool) {
        if !impulse.is_zero() && self.body_type == RigidBodyType::Dynamic {
            self.vels.linvel += impulse.component_mul(&self.mprops.effective_inv_mass);

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies an angular impulse at the center-of-mass of this rigid-body.
    /// The impulse is applied right away, changing the angular velocity.
    /// This does nothing on non-dynamic bodies.
    #[cfg(feature = "dim2")]
    pub fn apply_torque_impulse(&mut self, torque_impulse: Real, wake_up: bool) {
        if !torque_impulse.is_zero() && self.body_type == RigidBodyType::Dynamic {
            self.vels.angvel += self.mprops.effective_world_inv_inertia_sqrt
                * (self.mprops.effective_world_inv_inertia_sqrt * torque_impulse);

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies an angular impulse at the center-of-mass of this rigid-body.
    /// The impulse is applied right away, changing the angular velocity.
    /// This does nothing on non-dynamic bodies.
    #[cfg(feature = "dim3")]
    pub fn apply_torque_impulse(&mut self, torque_impulse: Vector<Real>, wake_up: bool) {
        if !torque_impulse.is_zero() && self.body_type == RigidBodyType::Dynamic {
            self.vels.angvel += self.mprops.effective_world_inv_inertia_sqrt
                * (self.mprops.effective_world_inv_inertia_sqrt * torque_impulse);

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies an impulse at the given world-space point of this rigid-body.
    /// The impulse is applied right away, changing the linear and/or angular velocities.
    /// This does nothing on non-dynamic bodies.
    pub fn apply_impulse_at_point(
        &mut self,
        impulse: Vector<Real>,
        point: Point<Real>,
        wake_up: bool,
    ) {
        let torque_impulse = (point - self.mprops.world_com).gcross(impulse);
        self.apply_impulse(impulse, wake_up);
        self.apply_torque_impulse(torque_impulse, wake_up);
    }

    /// Retrieves the constant force(s) that the user has added to the body.
    ///
    /// Returns zero if the rigid-body isn’t dynamic.
    pub fn user_force(&self) -> Vector<Real> {
        if self.body_type == RigidBodyType::Dynamic {
            self.forces.user_force
        } else {
            Vector::zeros()
        }
    }

    /// Retrieves the constant torque(s) that the user has added to the body.
    ///
    /// Returns zero if the rigid-body isn’t dynamic.
    pub fn user_torque(&self) -> AngVector<Real> {
        if self.body_type == RigidBodyType::Dynamic {
            self.forces.user_torque
        } else {
            AngVector::zero()
        }
    }
}

impl RigidBody {
    /// The velocity of the given world-space point on this rigid-body.
    pub fn velocity_at_point(&self, point: &Point<Real>) -> Vector<Real> {
        self.vels.velocity_at_point(point, &self.mprops.world_com)
    }

    /// The kinetic energy of this body.
    pub fn kinetic_energy(&self) -> Real {
        self.vels.kinetic_energy(&self.mprops)
    }

    /// The potential energy of this body in a gravity field.
    pub fn gravitational_potential_energy(&self, dt: Real, gravity: Vector<Real>) -> Real {
        let world_com = self
            .mprops
            .local_mprops
            .world_com(&self.pos.position)
            .coords;

        // Project position back along velocity vector one half-step (leap-frog)
        // to sync up the potential energy with the kinetic energy:
        let world_com = world_com - self.vels.linvel * (dt / 2.0);

        -self.mass() * self.forces.gravity_scale * gravity.dot(&world_com)
    }
}

/// A builder for rigid-bodies.
#[derive(Clone, Debug, PartialEq)]
#[must_use = "Builder functions return the updated builder"]
pub struct RigidBodyBuilder {
    /// The initial position of the rigid-body to be built.
    pub position: Isometry<Real>,
    /// The linear velocity of the rigid-body to be built.
    pub linvel: Vector<Real>,
    /// The angular velocity of the rigid-body to be built.
    pub angvel: AngVector<Real>,
    /// The scale factor applied to the gravity affecting the rigid-body to be built, `1.0` by default.
    pub gravity_scale: Real,
    /// Damping factor for gradually slowing down the translational motion of the rigid-body, `0.0` by default.
    pub linear_damping: Real,
    /// Damping factor for gradually slowing down the angular motion of the rigid-body, `0.0` by default.
    pub angular_damping: Real,
    body_type: RigidBodyType,
    mprops_flags: LockedAxes,
    /// The additional mass-properties of the rigid-body being built. See [`RigidBodyBuilder::additional_mass_properties`] for more information.
    additional_mass_properties: RigidBodyAdditionalMassProps,
    /// Whether or not the rigid-body to be created can sleep if it reaches a dynamic equilibrium.
    pub can_sleep: bool,
    /// Whether or not the rigid-body is to be created asleep.
    pub sleeping: bool,
    /// Whether continuous collision-detection is enabled for the rigid-body to be built.
    ///
    /// CCD prevents tunneling, but may still allow limited interpenetration of colliders.
    pub ccd_enabled: bool,
    /// The dominance group of the rigid-body to be built.
    pub dominance_group: i8,
    /// Will the rigid-body being built be enabled?
    pub enabled: bool,
    /// An arbitrary user-defined 128-bit integer associated to the rigid-bodies built by this builder.
    pub user_data: u128,
}

impl RigidBodyBuilder {
    /// Initialize a new builder for a rigid body which is either fixed, dynamic, or kinematic.
    pub fn new(body_type: RigidBodyType) -> Self {
        Self {
            position: Isometry::identity(),
            linvel: Vector::zeros(),
            angvel: na::zero(),
            gravity_scale: 1.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            body_type,
            mprops_flags: LockedAxes::empty(),
            additional_mass_properties: RigidBodyAdditionalMassProps::default(),
            can_sleep: true,
            sleeping: false,
            ccd_enabled: false,
            dominance_group: 0,
            enabled: true,
            user_data: 0,
        }
    }

    /// Initializes the builder of a new fixed rigid body.
    #[deprecated(note = "use `RigidBodyBuilder::fixed()` instead")]
    pub fn new_static() -> Self {
        Self::fixed()
    }
    /// Initializes the builder of a new velocity-based kinematic rigid body.
    #[deprecated(note = "use `RigidBodyBuilder::kinematic_velocity_based()` instead")]
    pub fn new_kinematic_velocity_based() -> Self {
        Self::kinematic_velocity_based()
    }
    /// Initializes the builder of a new position-based kinematic rigid body.
    #[deprecated(note = "use `RigidBodyBuilder::kinematic_position_based()` instead")]
    pub fn new_kinematic_position_based() -> Self {
        Self::kinematic_position_based()
    }

    /// Initializes the builder of a new fixed rigid body.
    pub fn fixed() -> Self {
        Self::new(RigidBodyType::Fixed)
    }

    /// Initializes the builder of a new velocity-based kinematic rigid body.
    pub fn kinematic_velocity_based() -> Self {
        Self::new(RigidBodyType::KinematicVelocityBased)
    }

    /// Initializes the builder of a new position-based kinematic rigid body.
    pub fn kinematic_position_based() -> Self {
        Self::new(RigidBodyType::KinematicPositionBased)
    }

    /// Initializes the builder of a new dynamic rigid body.
    pub fn dynamic() -> Self {
        Self::new(RigidBodyType::Dynamic)
    }

    /// Sets the scale applied to the gravity force affecting the rigid-body to be created.
    pub fn gravity_scale(mut self, scale_factor: Real) -> Self {
        self.gravity_scale = scale_factor;
        self
    }

    /// Sets the dominance group of this rigid-body.
    pub fn dominance_group(mut self, group: i8) -> Self {
        self.dominance_group = group;
        self
    }

    /// Sets the initial translation of the rigid-body to be created.
    pub fn translation(mut self, translation: Vector<Real>) -> Self {
        self.position.translation.vector = translation;
        self
    }

    /// Sets the initial orientation of the rigid-body to be created.
    pub fn rotation(mut self, angle: AngVector<Real>) -> Self {
        self.position.rotation = Rotation::new(angle);
        self
    }

    /// Sets the initial position (translation and orientation) of the rigid-body to be created.
    pub fn position(mut self, pos: Isometry<Real>) -> Self {
        self.position = pos;
        self
    }

    /// An arbitrary user-defined 128-bit integer associated to the rigid-bodies built by this builder.
    pub fn user_data(mut self, data: u128) -> Self {
        self.user_data = data;
        self
    }

    /// Sets the additional mass-properties of the rigid-body being built.
    ///
    /// This will be overridden by a call to [`Self::additional_mass`] so it only makes sense to call
    /// either [`Self::additional_mass`] or [`Self::additional_mass_properties`].    
    ///
    /// Note that "additional" means that the final mass-properties of the rigid-bodies depends
    /// on the initial mass-properties of the rigid-body (set by this method)
    /// to which is added the contributions of all the colliders with non-zero density
    /// attached to this rigid-body.
    ///
    /// Therefore, if you want your provided mass-properties to be the final
    /// mass-properties of your rigid-body, don't attach colliders to it, or
    /// only attach colliders with densities equal to zero.
    pub fn additional_mass_properties(mut self, mprops: MassProperties) -> Self {
        self.additional_mass_properties = RigidBodyAdditionalMassProps::MassProps(mprops);
        self
    }

    /// Sets the additional mass of the rigid-body being built.
    ///
    /// This will be overridden by a call to [`Self::additional_mass_properties`] so it only makes
    /// sense to call either [`Self::additional_mass`] or [`Self::additional_mass_properties`].    
    ///
    /// This is only the "additional" mass because the total mass of the  rigid-body is
    /// equal to the sum of this additional mass and the mass computed from the colliders
    /// (with non-zero densities) attached to this rigid-body.
    ///
    /// The total angular inertia of the rigid-body will be scaled automatically based on this
    /// additional mass. If this scaling effect isn’t desired, use [`Self::additional_mass_properties`]
    /// instead of this method.
    ///
    /// # Parameters
    /// * `mass`- The mass that will be added to the created rigid-body.
    pub fn additional_mass(mut self, mass: Real) -> Self {
        self.additional_mass_properties = RigidBodyAdditionalMassProps::Mass(mass);
        self
    }

    /// Sets the axes along which this rigid-body cannot translate or rotate.
    pub fn locked_axes(mut self, locked_axes: LockedAxes) -> Self {
        self.mprops_flags = locked_axes;
        self
    }

    /// Prevents this rigid-body from translating because of forces.
    pub fn lock_translations(mut self) -> Self {
        self.mprops_flags.set(LockedAxes::TRANSLATION_LOCKED, true);
        self
    }

    /// Only allow translations of this rigid-body around specific coordinate axes.
    pub fn enabled_translations(
        mut self,
        allow_translations_x: bool,
        allow_translations_y: bool,
        #[cfg(feature = "dim3")] allow_translations_z: bool,
    ) -> Self {
        self.mprops_flags
            .set(LockedAxes::TRANSLATION_LOCKED_X, !allow_translations_x);
        self.mprops_flags
            .set(LockedAxes::TRANSLATION_LOCKED_Y, !allow_translations_y);
        #[cfg(feature = "dim3")]
        self.mprops_flags
            .set(LockedAxes::TRANSLATION_LOCKED_Z, !allow_translations_z);
        self
    }

    #[deprecated(note = "Use `enabled_translations` instead")]
    /// Only allow translations of this rigid-body around specific coordinate axes.
    pub fn restrict_translations(
        self,
        allow_translations_x: bool,
        allow_translations_y: bool,
        #[cfg(feature = "dim3")] allow_translations_z: bool,
    ) -> Self {
        self.enabled_translations(
            allow_translations_x,
            allow_translations_y,
            #[cfg(feature = "dim3")]
            allow_translations_z,
        )
    }

    /// Prevents this rigid-body from rotating because of forces.
    pub fn lock_rotations(mut self) -> Self {
        self.mprops_flags.set(LockedAxes::ROTATION_LOCKED_X, true);
        self.mprops_flags.set(LockedAxes::ROTATION_LOCKED_Y, true);
        self.mprops_flags.set(LockedAxes::ROTATION_LOCKED_Z, true);
        self
    }

    /// Only allow rotations of this rigid-body around specific coordinate axes.
    #[cfg(feature = "dim3")]
    pub fn enabled_rotations(
        mut self,
        allow_rotations_x: bool,
        allow_rotations_y: bool,
        allow_rotations_z: bool,
    ) -> Self {
        self.mprops_flags
            .set(LockedAxes::ROTATION_LOCKED_X, !allow_rotations_x);
        self.mprops_flags
            .set(LockedAxes::ROTATION_LOCKED_Y, !allow_rotations_y);
        self.mprops_flags
            .set(LockedAxes::ROTATION_LOCKED_Z, !allow_rotations_z);
        self
    }

    /// Locks or unlocks rotations of this rigid-body along each cartesian axes.
    #[deprecated(note = "Use `enabled_rotations` instead")]
    #[cfg(feature = "dim3")]
    pub fn restrict_rotations(
        self,
        allow_rotations_x: bool,
        allow_rotations_y: bool,
        allow_rotations_z: bool,
    ) -> Self {
        self.enabled_rotations(allow_rotations_x, allow_rotations_y, allow_rotations_z)
    }

    /// Sets the damping factor for the linear part of the rigid-body motion.
    ///
    /// The higher the linear damping factor is, the more quickly the rigid-body
    /// will slow-down its translational movement.
    pub fn linear_damping(mut self, factor: Real) -> Self {
        self.linear_damping = factor;
        self
    }

    /// Sets the damping factor for the angular part of the rigid-body motion.
    ///
    /// The higher the angular damping factor is, the more quickly the rigid-body
    /// will slow-down its rotational movement.
    pub fn angular_damping(mut self, factor: Real) -> Self {
        self.angular_damping = factor;
        self
    }

    /// Sets the initial linear velocity of the rigid-body to be created.
    pub fn linvel(mut self, linvel: Vector<Real>) -> Self {
        self.linvel = linvel;
        self
    }

    /// Sets the initial angular velocity of the rigid-body to be created.
    pub fn angvel(mut self, angvel: AngVector<Real>) -> Self {
        self.angvel = angvel;
        self
    }

    /// Sets whether or not the rigid-body to be created can sleep if it reaches a dynamic equilibrium.
    pub fn can_sleep(mut self, can_sleep: bool) -> Self {
        self.can_sleep = can_sleep;
        self
    }

    /// Sets whether or not continuous collision-detection is enabled for this rigid-body.
    ///
    /// CCD prevents tunneling, but may still allow limited interpenetration of colliders.
    pub fn ccd_enabled(mut self, enabled: bool) -> Self {
        self.ccd_enabled = enabled;
        self
    }

    /// Sets whether or not the rigid-body is to be created asleep.
    pub fn sleeping(mut self, sleeping: bool) -> Self {
        self.sleeping = sleeping;
        self
    }

    /// Enable or disable the rigid-body after its creation.
    pub fn enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }

    /// Build a new rigid-body with the parameters configured with this builder.
    pub fn build(&self) -> RigidBody {
        let mut rb = RigidBody::new();
        rb.pos.next_position = self.position; // FIXME: compute the correct value?
        rb.pos.position = self.position;
        rb.vels.linvel = self.linvel;
        rb.vels.angvel = self.angvel;
        rb.body_type = self.body_type;
        rb.user_data = self.user_data;

        if self.additional_mass_properties
            != RigidBodyAdditionalMassProps::MassProps(MassProperties::zero())
            && self.additional_mass_properties != RigidBodyAdditionalMassProps::Mass(0.0)
        {
            rb.mprops.additional_local_mprops = Some(Box::new(self.additional_mass_properties));
        }

        rb.mprops.flags = self.mprops_flags;
        rb.damping.linear_damping = self.linear_damping;
        rb.damping.angular_damping = self.angular_damping;
        rb.forces.gravity_scale = self.gravity_scale;
        rb.dominance = RigidBodyDominance(self.dominance_group);
        rb.enabled = self.enabled;
        rb.enable_ccd(self.ccd_enabled);

        if self.can_sleep && self.sleeping {
            rb.sleep();
        }

        if !self.can_sleep {
            rb.activation.linear_threshold = -1.0;
            rb.activation.angular_threshold = -1.0;
        }

        rb
    }
}

impl Into<RigidBody> for RigidBodyBuilder {
    fn into(self) -> RigidBody {
        self.build()
    }
}
