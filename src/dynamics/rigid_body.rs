use crate::dynamics::MassProperties;
use crate::geometry::{Collider, ColliderHandle, InteractionGraph, RigidBodyGraphIndex};
use crate::math::{AngVector, AngularInertia, Isometry, Point, Rotation, Translation, Vector};
use crate::utils::{WCross, WDot};
use num::Zero;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// The status of a body, governing the way it is affected by external forces.
pub enum BodyStatus {
    /// A `BodyStatus::Dynamic` body can be affected by all external forces.
    Dynamic,
    /// A `BodyStatus::Static` body cannot be affected by external forces.
    Static,
    /// A `BodyStatus::Kinematic` body cannot be affected by any external forces but can be controlled
    /// by the user at the position level while keeping realistic one-way interaction with dynamic bodies.
    ///
    /// One-way interaction means that a kinematic body can push a dynamic body, but a kinematic body
    /// cannot be pushed by anything. In other words, the trajectory of a kinematic body can only be
    /// modified by the user and is independent from any contact or joint it is involved in.
    Kinematic,
    // Semikinematic, // A kinematic that performs automatic CCD with the static environment toi avoid traversing it?
    // Disabled,
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A rigid body.
///
/// To create a new rigid-body, use the `RigidBodyBuilder` structure.
#[derive(Debug)]
pub struct RigidBody {
    /// The world-space position of the rigid-body.
    pub position: Isometry<f32>,
    pub(crate) predicted_position: Isometry<f32>,
    /// The local mass properties of the rigid-body.
    pub mass_properties: MassProperties,
    /// The world-space center of mass of the rigid-body.
    pub world_com: Point<f32>,
    /// The square-root of the inverse angular inertia tensor of the rigid-body.
    pub world_inv_inertia_sqrt: AngularInertia<f32>,
    /// The linear velocity of the rigid-body.
    pub linvel: Vector<f32>,
    /// The angular velocity of the rigid-body.
    pub angvel: AngVector<f32>,
    pub(crate) linacc: Vector<f32>,
    pub(crate) angacc: AngVector<f32>,
    pub(crate) colliders: Vec<ColliderHandle>,
    /// Whether or not this rigid-body is sleeping.
    pub activation: ActivationStatus,
    pub(crate) joint_graph_index: RigidBodyGraphIndex,
    pub(crate) active_island_id: usize,
    pub(crate) active_set_id: usize,
    pub(crate) active_set_offset: usize,
    pub(crate) active_set_timestamp: u32,
    /// The status of the body, governing how it is affected by external forces.
    pub body_status: BodyStatus,
}

impl Clone for RigidBody {
    fn clone(&self) -> Self {
        Self {
            colliders: Vec::new(),
            joint_graph_index: RigidBodyGraphIndex::new(crate::INVALID_U32),
            active_island_id: crate::INVALID_USIZE,
            active_set_id: crate::INVALID_USIZE,
            active_set_offset: crate::INVALID_USIZE,
            active_set_timestamp: crate::INVALID_U32,
            ..*self
        }
    }
}

impl RigidBody {
    fn new() -> Self {
        Self {
            position: Isometry::identity(),
            predicted_position: Isometry::identity(),
            mass_properties: MassProperties::zero(),
            world_com: Point::origin(),
            world_inv_inertia_sqrt: AngularInertia::zero(),
            linvel: Vector::zeros(),
            angvel: na::zero(),
            linacc: Vector::zeros(),
            angacc: na::zero(),
            colliders: Vec::new(),
            activation: ActivationStatus::new_active(),
            joint_graph_index: InteractionGraph::<()>::invalid_graph_index(),
            active_island_id: 0,
            active_set_id: 0,
            active_set_offset: 0,
            active_set_timestamp: 0,
            body_status: BodyStatus::Dynamic,
        }
    }

    pub(crate) fn integrate_accelerations(&mut self, dt: f32, gravity: Vector<f32>) {
        if self.mass_properties.inv_mass != 0.0 {
            self.linvel += (gravity + self.linacc) * dt;
            self.angvel += self.angacc * dt;

            // Reset the accelerations.
            self.linacc = na::zero();
            self.angacc = na::zero();
        }
    }

    /// The handles of colliders attached to this rigid body.
    pub fn colliders(&self) -> &[ColliderHandle] {
        &self.colliders[..]
    }

    /// Is this rigid body dynamic?
    ///
    /// A dynamic body can move freely and is affected by forces.
    pub fn is_dynamic(&self) -> bool {
        self.body_status == BodyStatus::Dynamic
    }

    /// Is this rigid body kinematic?
    ///
    /// A kinematic body can move freely but is not affected by forces.
    pub fn is_kinematic(&self) -> bool {
        self.body_status == BodyStatus::Kinematic
    }

    /// Is this rigid body static?
    ///
    /// A static body cannot move and is not affected by forces.
    pub fn is_static(&self) -> bool {
        self.body_status == BodyStatus::Static
    }

    /// The mass of this rigid body.
    ///
    /// Returns zero if this rigid body has an infinite mass.
    pub fn mass(&self) -> f32 {
        crate::utils::inv(self.mass_properties.inv_mass)
    }

    /// The predicted position of this rigid-body.
    ///
    /// If this rigid-body is kinematic this value is set by the `set_next_kinematic_position`
    /// method and is used for estimating the kinematic body velocity at the next timestep.
    /// For non-kinematic bodies, this value is currently unspecified.
    pub fn predicted_position(&self) -> &Isometry<f32> {
        &self.predicted_position
    }

    /// Adds a collider to this rigid-body.
    pub(crate) fn add_collider_internal(&mut self, handle: ColliderHandle, coll: &Collider) {
        let mass_properties = coll
            .mass_properties()
            .transform_by(coll.position_wrt_parent());
        self.colliders.push(handle);
        self.mass_properties += mass_properties;
        self.update_world_mass_properties();
    }

    /// Removes a collider from this rigid-body.
    pub(crate) fn remove_collider_internal(&mut self, handle: ColliderHandle, coll: &Collider) {
        if let Some(i) = self.colliders.iter().position(|e| *e == handle) {
            self.colliders.swap_remove(i);
            let mass_properties = coll
                .mass_properties()
                .transform_by(coll.position_wrt_parent());
            self.mass_properties -= mass_properties;
            self.update_world_mass_properties();
        }
    }

    /// Put this rigid body to sleep.
    ///
    /// A sleeping body no longer moves and is no longer simulated by the physics engine unless
    /// it is waken up. It can be woken manually with `self.wake_up` or automatically due to
    /// external forces like contacts.
    pub fn sleep(&mut self) {
        self.activation.energy = 0.0;
        self.activation.sleeping = true;
        self.linvel = na::zero();
        self.angvel = na::zero();
    }

    /// Wakes up this rigid body if it is sleeping.
    ///
    /// If `strong` is `true` then it is assured that the rigid-body will
    /// remain awake during multiple subsequent timesteps.
    pub fn wake_up(&mut self, strong: bool) {
        self.activation.sleeping = false;

        if (strong || self.activation.energy == 0.0) && self.is_dynamic() {
            self.activation.energy = self.activation.threshold.abs() * 2.0;
        }
    }

    pub(crate) fn update_energy(&mut self) {
        let mix_factor = 0.01;
        let new_energy = (1.0 - mix_factor) * self.activation.energy
            + mix_factor * (self.linvel.norm_squared() + self.angvel.gdot(self.angvel));
        self.activation.energy = new_energy.min(self.activation.threshold.abs() * 4.0);
    }

    /// Is this rigid body sleeping?
    pub fn is_sleeping(&self) -> bool {
        // TODO: should we:
        // - return false for static bodies.
        // - return true for non-sleeping dynamic bodies.
        // - return true only for kinematic bodies with non-zero velocity?
        self.activation.sleeping
    }

    /// Is the velocity of this body not zero?
    pub fn is_moving(&self) -> bool {
        !self.linvel.is_zero() || !self.angvel.is_zero()
    }

    fn integrate_velocity(&self, dt: f32) -> Isometry<f32> {
        let com = &self.position * self.mass_properties.local_com;
        let shift = Translation::from(com.coords);
        shift * Isometry::new(self.linvel * dt, self.angvel * dt) * shift.inverse()
    }
    pub(crate) fn integrate(&mut self, dt: f32) {
        self.position = self.integrate_velocity(dt) * self.position;
    }

    /// Sets the position and `next_kinematic_position` of this rigid body.
    ///
    /// This will teleport the rigid-body to the specified position/orientation,
    /// completely ignoring any physics rule. If this body is kinematic, this will
    /// also set the next kinematic position to the same value, effectively
    /// resetting to zero the next interpolated velocity of the kinematic body.
    pub fn set_position(&mut self, pos: Isometry<f32>) {
        self.position = pos;

        // TODO: update the predicted position for dynamic bodies too?
        if self.is_static() || self.is_kinematic() {
            self.predicted_position = pos;
        }
    }

    /// If this rigid body is kinematic, sets its future position after the next timestep integration.
    pub fn set_next_kinematic_position(&mut self, pos: Isometry<f32>) {
        if self.is_kinematic() {
            self.predicted_position = pos;
        }
    }

    pub(crate) fn compute_velocity_from_predicted_position(&mut self, inv_dt: f32) {
        let dpos = self.predicted_position * self.position.inverse();
        #[cfg(feature = "dim2")]
        {
            self.angvel = dpos.rotation.angle() * inv_dt;
        }
        #[cfg(feature = "dim3")]
        {
            self.angvel = dpos.rotation.scaled_axis() * inv_dt;
        }
        self.linvel = dpos.translation.vector * inv_dt;
    }

    pub(crate) fn update_predicted_position(&mut self, dt: f32) {
        self.predicted_position = self.integrate_velocity(dt) * self.position;
    }

    pub(crate) fn update_world_mass_properties(&mut self) {
        self.world_com = self.mass_properties.world_com(&self.position);
        self.world_inv_inertia_sqrt = self
            .mass_properties
            .world_inv_inertia_sqrt(&self.position.rotation);
    }

    /*
     * Application of forces/impulses.
     */
    /// Applies a force at the center-of-mass of this rigid-body.
    pub fn apply_force(&mut self, force: Vector<f32>) {
        if self.body_status == BodyStatus::Dynamic {
            self.linacc += force * self.mass_properties.inv_mass;
        }
    }

    /// Applies an impulse at the center-of-mass of this rigid-body.
    pub fn apply_impulse(&mut self, impulse: Vector<f32>) {
        if self.body_status == BodyStatus::Dynamic {
            self.linvel += impulse * self.mass_properties.inv_mass;
        }
    }

    /// Applies a torque at the center-of-mass of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn apply_torque(&mut self, torque: f32) {
        if self.body_status == BodyStatus::Dynamic {
            self.angacc += self.world_inv_inertia_sqrt * (self.world_inv_inertia_sqrt * torque);
        }
    }

    /// Applies a torque at the center-of-mass of this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn apply_torque(&mut self, torque: Vector<f32>) {
        if self.body_status == BodyStatus::Dynamic {
            self.angacc += self.world_inv_inertia_sqrt * (self.world_inv_inertia_sqrt * torque);
        }
    }

    /// Applies an impulsive torque at the center-of-mass of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn apply_torque_impulse(&mut self, torque_impulse: f32) {
        if self.body_status == BodyStatus::Dynamic {
            self.angvel +=
                self.world_inv_inertia_sqrt * (self.world_inv_inertia_sqrt * torque_impulse);
        }
    }

    /// Applies an impulsive torque at the center-of-mass of this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn apply_torque_impulse(&mut self, torque_impulse: Vector<f32>) {
        if self.body_status == BodyStatus::Dynamic {
            self.angvel +=
                self.world_inv_inertia_sqrt * (self.world_inv_inertia_sqrt * torque_impulse);
        }
    }

    /// Applies a force at the given world-space point of this rigid-body.
    pub fn apply_force_at_point(&mut self, force: Vector<f32>, point: Point<f32>) {
        let torque = (point - self.world_com).gcross(force);
        self.apply_force(force);
        self.apply_torque(torque);
    }

    /// Applies an impulse at the given world-space point of this rigid-body.
    pub fn apply_impulse_at_point(&mut self, impulse: Vector<f32>, point: Point<f32>) {
        let torque_impulse = (point - self.world_com).gcross(impulse);
        self.apply_impulse(impulse);
        self.apply_torque_impulse(torque_impulse);
    }
}

/// A builder for rigid-bodies.
pub struct RigidBodyBuilder {
    position: Isometry<f32>,
    linvel: Vector<f32>,
    angvel: AngVector<f32>,
    body_status: BodyStatus,
    can_sleep: bool,
}

impl RigidBodyBuilder {
    /// Initialize a new builder for a rigid body which is either static, dynamic, or kinematic.
    pub fn new(body_status: BodyStatus) -> Self {
        Self {
            position: Isometry::identity(),
            linvel: Vector::zeros(),
            angvel: na::zero(),
            body_status,
            can_sleep: true,
        }
    }

    /// Initializes the builder of a new static rigid body.
    pub fn new_static() -> Self {
        Self::new(BodyStatus::Static)
    }

    /// Initializes the builder of a new kinematic rigid body.
    pub fn new_kinematic() -> Self {
        Self::new(BodyStatus::Kinematic)
    }

    /// Initializes the builder of a new dynamic rigid body.
    pub fn new_dynamic() -> Self {
        Self::new(BodyStatus::Dynamic)
    }

    /// Sets the initial translation of the rigid-body to be created.
    #[cfg(feature = "dim2")]
    pub fn translation(mut self, x: f32, y: f32) -> Self {
        self.position.translation.x = x;
        self.position.translation.y = y;
        self
    }

    /// Sets the initial translation of the rigid-body to be created.
    #[cfg(feature = "dim3")]
    pub fn translation(mut self, x: f32, y: f32, z: f32) -> Self {
        self.position.translation.x = x;
        self.position.translation.y = y;
        self.position.translation.z = z;
        self
    }

    /// Sets the initial orientation of the rigid-body to be created.
    pub fn rotation(mut self, angle: AngVector<f32>) -> Self {
        self.position.rotation = Rotation::new(angle);
        self
    }

    /// Sets the initial position (translation and orientation) of the rigid-body to be created.
    pub fn position(mut self, pos: Isometry<f32>) -> Self {
        self.position = pos;
        self
    }

    /// Sets the initial linear velocity of the rigid-body to be created.
    #[cfg(feature = "dim2")]
    pub fn linvel(mut self, x: f32, y: f32) -> Self {
        self.linvel = Vector::new(x, y);
        self
    }

    /// Sets the initial linear velocity of the rigid-body to be created.
    #[cfg(feature = "dim3")]
    pub fn linvel(mut self, x: f32, y: f32, z: f32) -> Self {
        self.linvel = Vector::new(x, y, z);
        self
    }

    /// Sets the initial angular velocity of the rigid-body to be created.
    pub fn angvel(mut self, angvel: AngVector<f32>) -> Self {
        self.angvel = angvel;
        self
    }

    /// Sets whether or not the rigid-body to be created can sleep if it reaches a dynamic equilibrium.
    pub fn can_sleep(mut self, can_sleep: bool) -> Self {
        self.can_sleep = can_sleep;
        self
    }

    /// Build a new rigid-body with the parameters configured with this builder.
    pub fn build(&self) -> RigidBody {
        let mut rb = RigidBody::new();
        rb.predicted_position = self.position; // FIXME: compute the correct value?
        rb.set_position(self.position);
        rb.linvel = self.linvel;
        rb.angvel = self.angvel;
        rb.body_status = self.body_status;

        if !self.can_sleep {
            rb.activation.threshold = -1.0;
        }

        rb
    }
}

/// The activation status of a body.
///
/// This controls whether a body is sleeping or not.
/// If the threshold is negative, the body never sleeps.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct ActivationStatus {
    /// The threshold pseudo-kinetic energy bellow which the body can fall asleep.
    pub threshold: f32,
    /// The current pseudo-kinetic energy of the body.
    pub energy: f32,
    /// Is this body already sleeping?
    pub sleeping: bool,
}

impl ActivationStatus {
    /// The default amount of energy bellow which a body can be put to sleep by nphysics.
    pub fn default_threshold() -> f32 {
        0.01
    }

    /// Create a new activation status initialised with the default activation threshold and is active.
    pub fn new_active() -> Self {
        ActivationStatus {
            threshold: Self::default_threshold(),
            energy: Self::default_threshold() * 4.0,
            sleeping: false,
        }
    }

    /// Create a new activation status initialised with the default activation threshold and is inactive.
    pub fn new_inactive() -> Self {
        ActivationStatus {
            threshold: Self::default_threshold(),
            energy: 0.0,
            sleeping: true,
        }
    }

    /// Returns `true` if the body is not asleep.
    #[inline]
    pub fn is_active(&self) -> bool {
        self.energy != 0.0
    }
}
