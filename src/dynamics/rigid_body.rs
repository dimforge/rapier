use crate::dynamics::{
    MassProperties, RigidBodyActivation, RigidBodyCcd, RigidBodyChanges, RigidBodyColliders,
    RigidBodyDamping, RigidBodyDominance, RigidBodyForces, RigidBodyIds, RigidBodyMassProps,
    RigidBodyMassPropsFlags, RigidBodyPosition, RigidBodyType, RigidBodyVelocity,
};
use crate::geometry::{
    Collider, ColliderHandle, ColliderMassProps, ColliderParent, ColliderPosition, ColliderShape,
};
use crate::math::{AngVector, Isometry, Point, Real, Rotation, Vector};
use crate::utils::{self, WCross};
use na::ComplexField;
use num::Zero;

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A rigid body.
///
/// To create a new rigid-body, use the `RigidBodyBuilder` structure.
#[derive(Debug, Clone)]
pub struct RigidBody {
    pub(crate) rb_pos: RigidBodyPosition,
    pub(crate) rb_mprops: RigidBodyMassProps,
    pub(crate) rb_vels: RigidBodyVelocity,
    pub(crate) rb_damping: RigidBodyDamping,
    pub(crate) rb_forces: RigidBodyForces,
    pub(crate) rb_ccd: RigidBodyCcd,
    pub(crate) rb_ids: RigidBodyIds,
    pub(crate) rb_colliders: RigidBodyColliders,
    /// Whether or not this rigid-body is sleeping.
    pub(crate) rb_activation: RigidBodyActivation,
    pub(crate) changes: RigidBodyChanges,
    /// The status of the body, governing how it is affected by external forces.
    pub(crate) rb_type: RigidBodyType,
    /// The dominance group this rigid-body is part of.
    pub(crate) rb_dominance: RigidBodyDominance,
    /// User-defined data associated to this rigid-body.
    pub user_data: u128,
}

impl RigidBody {
    fn new() -> Self {
        Self {
            rb_pos: RigidBodyPosition::default(),
            rb_mprops: RigidBodyMassProps::default(),
            rb_vels: RigidBodyVelocity::default(),
            rb_damping: RigidBodyDamping::default(),
            rb_forces: RigidBodyForces::default(),
            rb_ccd: RigidBodyCcd::default(),
            rb_ids: RigidBodyIds::default(),
            rb_colliders: RigidBodyColliders::default(),
            rb_activation: RigidBodyActivation::active(),
            changes: RigidBodyChanges::all(),
            rb_type: RigidBodyType::Dynamic,
            rb_dominance: RigidBodyDominance::default(),
            user_data: 0,
        }
    }

    pub(crate) fn reset_internal_references(&mut self) {
        self.rb_colliders.0 = Vec::new();
        self.rb_ids = Default::default();
    }

    /// The activation status of this rigid-body.
    pub fn activation(&self) -> &RigidBodyActivation {
        &self.rb_activation
    }

    /// Mutable reference to the activation status of this rigid-body.
    pub fn activation_mut(&mut self) -> &mut RigidBodyActivation {
        self.changes |= RigidBodyChanges::SLEEP;
        &mut self.rb_activation
    }

    /// The linear damping coefficient of this rigid-body.
    #[inline]
    pub fn linear_damping(&self) -> Real {
        self.rb_damping.linear_damping
    }

    /// Sets the linear damping coefficient of this rigid-body.
    #[inline]
    pub fn set_linear_damping(&mut self, damping: Real) {
        self.rb_damping.linear_damping = damping;
    }

    /// The angular damping coefficient of this rigid-body.
    #[inline]
    pub fn angular_damping(&self) -> Real {
        self.rb_damping.angular_damping
    }

    /// Sets the angular damping coefficient of this rigid-body.
    #[inline]
    pub fn set_angular_damping(&mut self, damping: Real) {
        self.rb_damping.angular_damping = damping
    }

    /// The type of this rigid-body.
    pub fn body_type(&self) -> RigidBodyType {
        self.rb_type
    }

    /// Sets the type of this rigid-body.
    pub fn set_body_type(&mut self, status: RigidBodyType) {
        if status != self.rb_type {
            self.changes.insert(RigidBodyChanges::TYPE);
            self.rb_type = status;
        }
    }

    /// The mass properties of this rigid-body.
    #[inline]
    pub fn mass_properties(&self) -> &MassProperties {
        &self.rb_mprops.local_mprops
    }

    /// The dominance group of this rigid-body.
    ///
    /// This method always returns `i8::MAX + 1` for non-dynamic
    /// rigid-bodies.
    #[inline]
    pub fn effective_dominance_group(&self) -> i16 {
        self.rb_dominance.effective_group(&self.rb_type)
    }

    #[inline]
    /// Locks or unlocks all the rotations of this rigid-body.
    pub fn lock_rotations(&mut self, locked: bool, wake_up: bool) {
        if self.is_dynamic() {
            if wake_up {
                self.wake_up(true);
            }

            self.rb_mprops
                .flags
                .set(RigidBodyMassPropsFlags::ROTATION_LOCKED_X, locked);
            self.rb_mprops
                .flags
                .set(RigidBodyMassPropsFlags::ROTATION_LOCKED_Y, locked);
            self.rb_mprops
                .flags
                .set(RigidBodyMassPropsFlags::ROTATION_LOCKED_Z, locked);
            self.update_world_mass_properties();
        }
    }

    #[inline]
    /// Locks or unlocks rotations of this rigid-body along each cartesian axes.
    pub fn restrict_rotations(
        &mut self,
        allow_rotations_x: bool,
        allow_rotations_y: bool,
        allow_rotations_z: bool,
        wake_up: bool,
    ) {
        if self.is_dynamic() {
            if wake_up {
                self.wake_up(true);
            }

            self.rb_mprops.flags.set(
                RigidBodyMassPropsFlags::ROTATION_LOCKED_X,
                allow_rotations_x,
            );
            self.rb_mprops.flags.set(
                RigidBodyMassPropsFlags::ROTATION_LOCKED_Y,
                allow_rotations_y,
            );
            self.rb_mprops.flags.set(
                RigidBodyMassPropsFlags::ROTATION_LOCKED_Z,
                allow_rotations_z,
            );
            self.update_world_mass_properties();
        }
    }

    #[inline]
    /// Locks or unlocks all the rotations of this rigid-body.
    pub fn lock_translations(&mut self, locked: bool, wake_up: bool) {
        if self.is_dynamic() {
            if wake_up {
                self.wake_up(true);
            }

            self.rb_mprops
                .flags
                .set(RigidBodyMassPropsFlags::TRANSLATION_LOCKED, locked);
            self.update_world_mass_properties();
        }
    }

    /// Are the translations of this rigid-body locked?
    pub fn is_translation_locked(&self) -> bool {
        self.rb_mprops
            .flags
            .contains(RigidBodyMassPropsFlags::TRANSLATION_LOCKED)
    }

    /// Are the rotations of this rigid-body locked?
    #[cfg(feature = "dim2")]
    pub fn is_rotation_locked(&self) -> bool {
        self.rb_mprops
            .flags
            .contains(RigidBodyMassPropsFlags::ROTATION_LOCKED_Z)
    }

    /// Returns `true` for each rotational degrees of freedom locked on this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn is_rotation_locked(&self) -> [bool; 3] {
        [
            self.rb_mprops
                .flags
                .contains(RigidBodyMassPropsFlags::ROTATION_LOCKED_X),
            self.rb_mprops
                .flags
                .contains(RigidBodyMassPropsFlags::ROTATION_LOCKED_Y),
            self.rb_mprops
                .flags
                .contains(RigidBodyMassPropsFlags::ROTATION_LOCKED_Z),
        ]
    }

    /// Enables of disable CCD (continuous collision-detection) for this rigid-body.
    pub fn enable_ccd(&mut self, enabled: bool) {
        self.rb_ccd.ccd_enabled = enabled;
    }

    /// Is CCD (continous collision-detection) enabled for this rigid-body?
    pub fn is_ccd_enabled(&self) -> bool {
        self.rb_ccd.ccd_enabled
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
    /// checks if CCD is allowed to run for this rigid-body or if
    /// it is completely disabled (independently from its velocity).
    pub fn is_ccd_active(&self) -> bool {
        self.rb_ccd.ccd_active
    }

    /// Sets the rigid-body's initial mass properties.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    #[inline]
    pub fn set_mass_properties(&mut self, props: MassProperties, wake_up: bool) {
        if self.is_dynamic() && wake_up {
            self.wake_up(true);
        }

        self.rb_mprops.local_mprops = props;
        self.update_world_mass_properties();
    }

    /// The handles of colliders attached to this rigid body.
    pub fn colliders(&self) -> &[ColliderHandle] {
        &self.rb_colliders.0[..]
    }

    /// Is this rigid body dynamic?
    ///
    /// A dynamic body can move freely and is affected by forces.
    pub fn is_dynamic(&self) -> bool {
        self.rb_type == RigidBodyType::Dynamic
    }

    /// Is this rigid body kinematic?
    ///
    /// A kinematic body can move freely but is not affected by forces.
    pub fn is_kinematic(&self) -> bool {
        self.rb_type.is_kinematic()
    }

    /// Is this rigid body static?
    ///
    /// A static body cannot move and is not affected by forces.
    pub fn is_static(&self) -> bool {
        self.rb_type == RigidBodyType::Static
    }

    /// The mass of this rigid body.
    ///
    /// Returns zero if this rigid body has an infinite mass.
    pub fn mass(&self) -> Real {
        utils::inv(self.rb_mprops.local_mprops.inv_mass)
    }

    /// The predicted position of this rigid-body.
    ///
    /// If this rigid-body is kinematic this value is set by the `set_next_kinematic_position`
    /// method and is used for estimating the kinematic body velocity at the next timestep.
    /// For non-kinematic bodies, this value is currently unspecified.
    pub fn next_position(&self) -> &Isometry<Real> {
        &self.rb_pos.next_position
    }

    /// The scale factor applied to the gravity affecting this rigid-body.
    pub fn gravity_scale(&self) -> Real {
        self.rb_forces.gravity_scale
    }

    /// Sets the gravity scale facter for this rigid-body.
    pub fn set_gravity_scale(&mut self, scale: Real, wake_up: bool) {
        if wake_up && self.rb_activation.sleeping {
            self.changes.insert(RigidBodyChanges::SLEEP);
            self.rb_activation.sleeping = false;
        }

        self.rb_forces.gravity_scale = scale;
    }

    /// The dominance group of this rigid-body.
    pub fn dominance_group(&self) -> i8 {
        self.rb_dominance.0
    }

    /// The dominance group of this rigid-body.
    pub fn set_dominance_group(&mut self, dominance: i8) {
        self.rb_dominance.0 = dominance
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
        self.rb_colliders.attach_collider(
            &mut self.changes,
            &mut self.rb_ccd,
            &mut self.rb_mprops,
            &self.rb_pos,
            co_handle,
            co_pos,
            co_parent,
            co_shape,
            co_mprops,
        )
    }

    /// Removes a collider from this rigid-body.
    pub(crate) fn remove_collider_internal(&mut self, handle: ColliderHandle, coll: &Collider) {
        if let Some(i) = self.rb_colliders.0.iter().position(|e| *e == handle) {
            self.changes.set(RigidBodyChanges::COLLIDERS, true);
            self.rb_colliders.0.swap_remove(i);

            let mass_properties = coll
                .mass_properties()
                .transform_by(coll.position_wrt_parent().unwrap());
            self.rb_mprops.local_mprops -= mass_properties;
            self.update_world_mass_properties();
        }
    }

    /// Put this rigid body to sleep.
    ///
    /// A sleeping body no longer moves and is no longer simulated by the physics engine unless
    /// it is waken up. It can be woken manually with `self.wake_up` or automatically due to
    /// external forces like contacts.
    pub fn sleep(&mut self) {
        self.rb_activation.sleep();
        self.rb_vels = RigidBodyVelocity::zero();
    }

    /// Wakes up this rigid body if it is sleeping.
    ///
    /// If `strong` is `true` then it is assured that the rigid-body will
    /// remain awake during multiple subsequent timesteps.
    pub fn wake_up(&mut self, strong: bool) {
        if self.rb_activation.sleeping {
            self.changes.insert(RigidBodyChanges::SLEEP);
        }

        self.rb_activation.wake_up(strong);
    }

    /// Is this rigid body sleeping?
    pub fn is_sleeping(&self) -> bool {
        // TODO: should we:
        // - return false for static bodies.
        // - return true for non-sleeping dynamic bodies.
        // - return true only for kinematic bodies with non-zero velocity?
        self.rb_activation.sleeping
    }

    /// Is the velocity of this body not zero?
    pub fn is_moving(&self) -> bool {
        !self.rb_vels.linvel.is_zero() || !self.rb_vels.angvel.is_zero()
    }

    /// The linear velocity of this rigid-body.
    pub fn linvel(&self) -> &Vector<Real> {
        &self.rb_vels.linvel
    }

    /// The angular velocity of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn angvel(&self) -> Real {
        self.rb_vels.angvel
    }

    /// The angular velocity of this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn angvel(&self) -> &Vector<Real> {
        &self.rb_vels.angvel
    }

    /// The linear velocity of this rigid-body.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    pub fn set_linvel(&mut self, linvel: Vector<Real>, wake_up: bool) {
        self.rb_vels.linvel = linvel;

        if self.is_dynamic() && wake_up {
            self.wake_up(true)
        }
    }

    /// The angular velocity of this rigid-body.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    #[cfg(feature = "dim2")]
    pub fn set_angvel(&mut self, angvel: Real, wake_up: bool) {
        self.rb_vels.angvel = angvel;

        if self.is_dynamic() && wake_up {
            self.wake_up(true)
        }
    }

    /// The angular velocity of this rigid-body.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    #[cfg(feature = "dim3")]
    pub fn set_angvel(&mut self, angvel: Vector<Real>, wake_up: bool) {
        self.rb_vels.angvel = angvel;

        if self.is_dynamic() && wake_up {
            self.wake_up(true)
        }
    }

    /// The world-space position of this rigid-body.
    #[inline]
    pub fn position(&self) -> &Isometry<Real> {
        &self.rb_pos.position
    }

    /// The translational part of this rigid-body's position.
    #[inline]
    pub fn translation(&self) -> &Vector<Real> {
        &self.rb_pos.position.translation.vector
    }

    /// Sets the translational part of this rigid-body's position.
    #[inline]
    pub fn set_translation(&mut self, translation: Vector<Real>, wake_up: bool) {
        self.changes.insert(RigidBodyChanges::POSITION);
        self.rb_pos.position.translation.vector = translation;
        self.rb_pos.next_position.translation.vector = translation;

        // TODO: Do we really need to check that the body isn't dynamic?
        if wake_up && self.is_dynamic() {
            self.wake_up(true)
        }
    }

    /// The translational part of this rigid-body's position.
    #[inline]
    pub fn rotation(&self) -> &Rotation<Real> {
        &self.rb_pos.position.rotation
    }

    /// Sets the rotational part of this rigid-body's position.
    #[inline]
    pub fn set_rotation(&mut self, rotation: AngVector<Real>, wake_up: bool) {
        self.changes.insert(RigidBodyChanges::POSITION);
        let rotation = Rotation::new(rotation);
        self.rb_pos.position.rotation = rotation;
        self.rb_pos.next_position.rotation = rotation;

        // TODO: Do we really need to check that the body isn't dynamic?
        if wake_up && self.is_dynamic() {
            self.wake_up(true)
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
        self.changes.insert(RigidBodyChanges::POSITION);
        self.rb_pos.position = pos;
        self.rb_pos.next_position = pos;

        // TODO: Do we really need to check that the body isn't dynamic?
        if wake_up && self.is_dynamic() {
            self.wake_up(true)
        }
    }

    /// If this rigid body is kinematic, sets its future translation after the next timestep integration.
    pub fn set_next_kinematic_rotation(&mut self, rotation: AngVector<Real>) {
        if self.is_kinematic() {
            self.rb_pos.next_position.rotation = Rotation::new(rotation);
        }
    }

    /// If this rigid body is kinematic, sets its future orientation after the next timestep integration.
    pub fn set_next_kinematic_translation(&mut self, translation: Vector<Real>) {
        if self.is_kinematic() {
            self.rb_pos.next_position.translation = translation.into();
        }
    }

    /// If this rigid body is kinematic, sets its future position after the next timestep integration.
    pub fn set_next_kinematic_position(&mut self, pos: Isometry<Real>) {
        if self.is_kinematic() {
            self.rb_pos.next_position = pos;
        }
    }

    /// Predicts the next position of this rigid-body, by integrating its velocity and forces
    /// by a time of `dt`.
    pub fn predict_position_using_velocity_and_forces(&self, dt: Real) -> Isometry<Real> {
        self.rb_pos.integrate_forces_and_velocities(
            dt,
            &self.rb_forces,
            &self.rb_vels,
            &self.rb_mprops,
        )
    }

    pub(crate) fn update_world_mass_properties(&mut self) {
        self.rb_mprops
            .update_world_mass_properties(&self.rb_pos.position);
    }
}

/// ## Applying forces and torques
impl RigidBody {
    /// Applies a force at the center-of-mass of this rigid-body.
    /// The force will be applied in the next simulation step.
    /// This does nothing on non-dynamic bodies.
    pub fn apply_force(&mut self, force: Vector<Real>, wake_up: bool) {
        if self.rb_type == RigidBodyType::Dynamic {
            self.rb_forces.force += force;

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies a torque at the center-of-mass of this rigid-body.
    /// The torque will be applied in the next simulation step.
    /// This does nothing on non-dynamic bodies.
    #[cfg(feature = "dim2")]
    pub fn apply_torque(&mut self, torque: Real, wake_up: bool) {
        if self.rb_type == RigidBodyType::Dynamic {
            self.rb_forces.torque += torque;

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies a torque at the center-of-mass of this rigid-body.
    /// The torque will be applied in the next simulation step.
    /// This does nothing on non-dynamic bodies.
    #[cfg(feature = "dim3")]
    pub fn apply_torque(&mut self, torque: Vector<Real>, wake_up: bool) {
        if self.rb_type == RigidBodyType::Dynamic {
            self.rb_forces.torque += torque;

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies a force at the given world-space point of this rigid-body.
    /// The force will be applied in the next simulation step.
    /// This does nothing on non-dynamic bodies.
    pub fn apply_force_at_point(&mut self, force: Vector<Real>, point: Point<Real>, wake_up: bool) {
        if self.rb_type == RigidBodyType::Dynamic {
            self.rb_forces.force += force;
            self.rb_forces.torque += (point - self.rb_mprops.world_com).gcross(force);

            if wake_up {
                self.wake_up(true);
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
        if self.rb_type == RigidBodyType::Dynamic {
            self.rb_vels.linvel += impulse * self.rb_mprops.effective_inv_mass;

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
        if self.rb_type == RigidBodyType::Dynamic {
            self.rb_vels.angvel += self.rb_mprops.effective_world_inv_inertia_sqrt
                * (self.rb_mprops.effective_world_inv_inertia_sqrt * torque_impulse);

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
        if self.rb_type == RigidBodyType::Dynamic {
            self.rb_vels.angvel += self.rb_mprops.effective_world_inv_inertia_sqrt
                * (self.rb_mprops.effective_world_inv_inertia_sqrt * torque_impulse);

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
        let torque_impulse = (point - self.rb_mprops.world_com).gcross(impulse);
        self.apply_impulse(impulse, wake_up);
        self.apply_torque_impulse(torque_impulse, wake_up);
    }
}

impl RigidBody {
    /// The velocity of the given world-space point on this rigid-body.
    pub fn velocity_at_point(&self, point: &Point<Real>) -> Vector<Real> {
        self.rb_vels
            .velocity_at_point(point, &self.rb_mprops.world_com)
    }

    /// The kinetic energy of this body.
    pub fn kinetic_energy(&self) -> Real {
        self.rb_vels.kinetic_energy(&self.rb_mprops)
    }

    /// The potential energy of this body in a gravity field.
    pub fn gravitational_potential_energy(&self, dt: Real, gravity: Vector<Real>) -> Real {
        let world_com = self
            .rb_mprops
            .local_mprops
            .world_com(&self.rb_pos.position)
            .coords;

        // Project position back along velocity vector one half-step (leap-frog)
        // to sync up the potential energy with the kinetic energy:
        let world_com = world_com - self.rb_vels.linvel * (dt / 2.0);

        -self.mass() * self.rb_forces.gravity_scale * gravity.dot(&world_com)
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
    rb_type: RigidBodyType,
    mprops_flags: RigidBodyMassPropsFlags,
    /// The additional mass properties of the rigid-body being built. See [`RigidBodyBuilder::additional_mass_properties`] for more information.
    pub mass_properties: MassProperties,
    /// Whether or not the rigid-body to be created can sleep if it reaches a dynamic equilibrium.
    pub can_sleep: bool,
    /// Whether or not the rigid-body is to be created asleep.
    pub sleeping: bool,
    /// Whether continuous collision-detection is enabled for the rigid-body to be built.
    pub ccd_enabled: bool,
    /// The dominance group of the rigid-body to be built.
    pub dominance_group: i8,
    /// An arbitrary user-defined 128-bit integer associated to the rigid-bodies built by this builder.
    pub user_data: u128,
}

impl RigidBodyBuilder {
    /// Initialize a new builder for a rigid body which is either static, dynamic, or kinematic.
    pub fn new(rb_type: RigidBodyType) -> Self {
        Self {
            position: Isometry::identity(),
            linvel: Vector::zeros(),
            angvel: na::zero(),
            gravity_scale: 1.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            rb_type,
            mprops_flags: RigidBodyMassPropsFlags::empty(),
            mass_properties: MassProperties::zero(),
            can_sleep: true,
            sleeping: false,
            ccd_enabled: false,
            dominance_group: 0,
            user_data: 0,
        }
    }

    /// Initializes the builder of a new static rigid body.
    pub fn new_static() -> Self {
        Self::new(RigidBodyType::Static)
    }

    /// Initializes the builder of a new kinematic rigid body.
    pub fn new_kinematic_velocity_based() -> Self {
        Self::new(RigidBodyType::KinematicVelocityBased)
    }

    /// Initializes the builder of a new kinematic rigid body.
    pub fn new_kinematic_position_based() -> Self {
        Self::new(RigidBodyType::KinematicPositionBased)
    }

    /// Initializes the builder of a new dynamic rigid body.
    pub fn new_dynamic() -> Self {
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

    /// Sets the additional mass properties of the rigid-body being built.
    ///
    /// Note that "additional" means that the final mass properties of the rigid-bodies depends
    /// on the initial mass-properties of the rigid-body (set by this method)
    /// to which is added the contributions of all the colliders with non-zero density
    /// attached to this rigid-body.
    ///
    /// Therefore, if you want your provided mass properties to be the final
    /// mass properties of your rigid-body, don't attach colliders to it, or
    /// only attach colliders with densities equal to zero.
    pub fn additional_mass_properties(mut self, props: MassProperties) -> Self {
        self.mass_properties = props;
        self
    }

    /// Prevents this rigid-body from translating because of forces.
    pub fn lock_translations(mut self) -> Self {
        self.mprops_flags
            .set(RigidBodyMassPropsFlags::TRANSLATION_LOCKED, true);
        self
    }

    /// Prevents this rigid-body from rotating because of forces.
    pub fn lock_rotations(mut self) -> Self {
        self.mprops_flags
            .set(RigidBodyMassPropsFlags::ROTATION_LOCKED_X, true);
        self.mprops_flags
            .set(RigidBodyMassPropsFlags::ROTATION_LOCKED_Y, true);
        self.mprops_flags
            .set(RigidBodyMassPropsFlags::ROTATION_LOCKED_Z, true);
        self
    }

    /// Only allow rotations of this rigid-body around specific coordinate axes.
    #[cfg(feature = "dim3")]
    pub fn restrict_rotations(
        mut self,
        allow_rotations_x: bool,
        allow_rotations_y: bool,
        allow_rotations_z: bool,
    ) -> Self {
        self.mprops_flags.set(
            RigidBodyMassPropsFlags::ROTATION_LOCKED_X,
            !allow_rotations_x,
        );
        self.mprops_flags.set(
            RigidBodyMassPropsFlags::ROTATION_LOCKED_Y,
            !allow_rotations_y,
        );
        self.mprops_flags.set(
            RigidBodyMassPropsFlags::ROTATION_LOCKED_Z,
            !allow_rotations_z,
        );
        self
    }

    /// Sets the additional mass of the rigid-body being built.
    ///
    /// This is only the "additional" mass because the total mass of the  rigid-body is
    /// equal to the sum of this additional mass and the mass computed from the colliders
    /// (with non-zero densities) attached to this rigid-body.
    pub fn additional_mass(mut self, mass: Real) -> Self {
        self.mass_properties.set_mass(mass, false);
        self
    }

    /// Sets the additional mass of the rigid-body being built.
    ///
    /// This is only the "additional" mass because the total mass of the  rigid-body is
    /// equal to the sum of this additional mass and the mass computed from the colliders
    /// (with non-zero densities) attached to this rigid-body.
    #[deprecated(note = "renamed to `additional_mass`.")]
    pub fn mass(self, mass: Real) -> Self {
        self.additional_mass(mass)
    }

    /// Sets the additional angular inertia of this rigid-body.
    ///
    /// This is only the "additional" angular inertia because the total angular inertia of
    /// the rigid-body is equal to the sum of this additional value and the angular inertia
    /// computed from the colliders (with non-zero densities) attached to this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn additional_principal_angular_inertia(mut self, inertia: Real) -> Self {
        self.mass_properties.inv_principal_inertia_sqrt =
            utils::inv(ComplexField::sqrt(inertia.max(0.0)));
        self
    }

    /// Sets the angular inertia of this rigid-body.
    #[cfg(feature = "dim2")]
    #[deprecated(note = "renamed to `additional_principal_angular_inertia`.")]
    pub fn principal_angular_inertia(self, inertia: Real) -> Self {
        self.additional_principal_angular_inertia(inertia)
    }

    /// Use `self.principal_angular_inertia` instead.
    #[cfg(feature = "dim2")]
    #[deprecated(note = "renamed to `additional_principal_angular_inertia`.")]
    pub fn principal_inertia(self, inertia: Real) -> Self {
        self.additional_principal_angular_inertia(inertia)
    }

    /// Sets the additional principal angular inertia of this rigid-body.
    ///
    /// This is only the "additional" angular inertia because the total angular inertia of
    /// the rigid-body is equal to the sum of this additional value and the angular inertia
    /// computed from the colliders (with non-zero densities) attached to this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn additional_principal_angular_inertia(mut self, inertia: AngVector<Real>) -> Self {
        self.mass_properties.inv_principal_inertia_sqrt =
            inertia.map(|e| utils::inv(ComplexField::sqrt(e.max(0.0))));
        self
    }

    /// Sets the principal angular inertia of this rigid-body.
    #[cfg(feature = "dim3")]
    #[deprecated(note = "renamed to `additional_principal_angular_inertia`.")]
    pub fn principal_angular_inertia(self, inertia: AngVector<Real>) -> Self {
        self.additional_principal_angular_inertia(inertia)
    }

    /// Use `self.principal_angular_inertia` instead.
    #[cfg(feature = "dim3")]
    #[deprecated(note = "renamed to `additional_principal_angular_inertia`.")]
    pub fn principal_inertia(self, inertia: AngVector<Real>) -> Self {
        self.additional_principal_angular_inertia(inertia)
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
    pub fn ccd_enabled(mut self, enabled: bool) -> Self {
        self.ccd_enabled = enabled;
        self
    }

    /// Sets whether or not the rigid-body is to be created asleep.
    pub fn sleeping(mut self, sleeping: bool) -> Self {
        self.sleeping = sleeping;
        self
    }

    /// Build a new rigid-body with the parameters configured with this builder.
    pub fn build(&self) -> RigidBody {
        let mut rb = RigidBody::new();
        rb.rb_pos.next_position = self.position; // FIXME: compute the correct value?
        rb.rb_pos.position = self.position;
        rb.rb_vels.linvel = self.linvel;
        rb.rb_vels.angvel = self.angvel;
        rb.rb_type = self.rb_type;
        rb.user_data = self.user_data;
        rb.rb_mprops.local_mprops = self.mass_properties;
        rb.rb_mprops.flags = self.mprops_flags;
        rb.rb_damping.linear_damping = self.linear_damping;
        rb.rb_damping.angular_damping = self.angular_damping;
        rb.rb_forces.gravity_scale = self.gravity_scale;
        rb.rb_dominance = RigidBodyDominance(self.dominance_group);
        rb.enable_ccd(self.ccd_enabled);

        if self.can_sleep && self.sleeping {
            rb.sleep();
        }

        if !self.can_sleep {
            rb.rb_activation.threshold = -1.0;
        }

        rb
    }
}
