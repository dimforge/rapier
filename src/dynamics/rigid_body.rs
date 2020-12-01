use crate::dynamics::MassProperties;
use crate::geometry::{
    Collider, ColliderHandle, ColliderSet, InteractionGraph, RigidBodyGraphIndex,
};
use crate::math::{AngVector, AngularInertia, Isometry, Point, Rotation, Translation, Vector};
use crate::utils::{self, WCross, WDot};
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

bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting the behavior of the constraints solver for a given contact manifold.
    pub(crate) struct RigidBodyFlags: u8 {
        const IGNORE_COLLIDER_MASS              = 1 << 0;
        const IGNORE_COLLIDER_ANGULAR_INERTIA_X = 1 << 1;
        const IGNORE_COLLIDER_ANGULAR_INERTIA_Y = 1 << 2;
        const IGNORE_COLLIDER_ANGULAR_INERTIA_Z = 1 << 3;
    }
}

bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags affecting the behavior of the constraints solver for a given contact manifold.
    pub(crate) struct RigidBodyChanges: u32 {
        const MODIFIED  = 1 << 0;
        const POSITION  = 1 << 1;
        const SLEEP     = 1 << 2;
        const COLLIDERS = 1 << 3;
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
/// A rigid body.
///
/// To create a new rigid-body, use the `RigidBodyBuilder` structure.
#[derive(Debug, Clone)]
pub struct RigidBody {
    /// The world-space position of the rigid-body.
    pub(crate) position: Isometry<f32>,
    pub(crate) predicted_position: Isometry<f32>,
    /// The local mass properties of the rigid-body.
    pub(crate) mass_properties: MassProperties,
    /// The world-space center of mass of the rigid-body.
    pub world_com: Point<f32>,
    /// The square-root of the inverse angular inertia tensor of the rigid-body.
    pub world_inv_inertia_sqrt: AngularInertia<f32>,
    /// The linear velocity of the rigid-body.
    pub(crate) linvel: Vector<f32>,
    /// The angular velocity of the rigid-body.
    pub(crate) angvel: AngVector<f32>,
    /// Damping factor for gradually slowing down the translational motion of the rigid-body.
    pub linear_damping: f32,
    /// Damping factor for gradually slowing down the angular motion of the rigid-body.
    pub angular_damping: f32,
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
    flags: RigidBodyFlags,
    pub(crate) changes: RigidBodyChanges,
    /// The status of the body, governing how it is affected by external forces.
    pub body_status: BodyStatus,
    /// User-defined data associated to this rigid-body.
    pub user_data: u128,
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
            linear_damping: 0.0,
            angular_damping: 0.0,
            colliders: Vec::new(),
            activation: ActivationStatus::new_active(),
            joint_graph_index: InteractionGraph::<()>::invalid_graph_index(),
            active_island_id: 0,
            active_set_id: 0,
            active_set_offset: 0,
            active_set_timestamp: 0,
            flags: RigidBodyFlags::empty(),
            changes: RigidBodyChanges::all(),
            body_status: BodyStatus::Dynamic,
            user_data: 0,
        }
    }

    pub(crate) fn reset_internal_references(&mut self) {
        self.colliders = Vec::new();
        self.joint_graph_index = InteractionGraph::<()>::invalid_graph_index();
        self.active_island_id = 0;
        self.active_set_id = 0;
        self.active_set_offset = 0;
        self.active_set_timestamp = 0;
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

    /// The mass properties of this rigid-body.
    #[inline]
    pub fn mass_properties(&self) -> &MassProperties {
        &self.mass_properties
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
        utils::inv(self.mass_properties.inv_mass)
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
    pub(crate) fn add_collider(&mut self, handle: ColliderHandle, coll: &Collider) {
        self.changes.set(
            RigidBodyChanges::MODIFIED | RigidBodyChanges::COLLIDERS,
            true,
        );

        let mass_properties = coll
            .mass_properties()
            .transform_by(coll.position_wrt_parent());
        self.colliders.push(handle);
        self.mass_properties += Self::filter_collider_mass_props(mass_properties, self.flags);
        self.update_world_mass_properties();
    }

    fn filter_collider_mass_props(
        mut props: MassProperties,
        flags: RigidBodyFlags,
    ) -> MassProperties {
        if flags.contains(RigidBodyFlags::IGNORE_COLLIDER_MASS) {
            props.inv_mass = 0.0;
        }

        #[cfg(feature = "dim2")]
        {
            if flags.contains(RigidBodyFlags::IGNORE_COLLIDER_ANGULAR_INERTIA_Z) {
                props.inv_principal_inertia_sqrt = 0.0;
            }
        }
        #[cfg(feature = "dim3")]
        {
            if flags.contains(RigidBodyFlags::IGNORE_COLLIDER_ANGULAR_INERTIA_X) {
                props.inv_principal_inertia_sqrt.x = 0.0;
            }
            if flags.contains(RigidBodyFlags::IGNORE_COLLIDER_ANGULAR_INERTIA_Y) {
                props.inv_principal_inertia_sqrt.y = 0.0;
            }
            if flags.contains(RigidBodyFlags::IGNORE_COLLIDER_ANGULAR_INERTIA_Z) {
                props.inv_principal_inertia_sqrt.z = 0.0;
            }
        }

        props
    }

    pub(crate) fn update_colliders_positions(&mut self, colliders: &mut ColliderSet) {
        for handle in &self.colliders {
            let collider = &mut colliders[*handle];
            collider.position = self.position * collider.delta;
            collider.predicted_position = self.predicted_position * collider.delta;
        }
    }

    /// Removes a collider from this rigid-body.
    pub(crate) fn remove_collider_internal(&mut self, handle: ColliderHandle, coll: &Collider) {
        if let Some(i) = self.colliders.iter().position(|e| *e == handle) {
            self.changes.set(RigidBodyChanges::COLLIDERS, true);
            self.colliders.swap_remove(i);
            let mass_properties = coll
                .mass_properties()
                .transform_by(coll.position_wrt_parent());
            self.mass_properties -= Self::filter_collider_mass_props(mass_properties, self.flags);
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
        if self.activation.sleeping {
            self.changes.insert(RigidBodyChanges::SLEEP);
            self.activation.sleeping = false;
        }

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
        // TODO: do we want to apply damping before or after the velocity integration?
        self.linvel *= 1.0 / (1.0 + dt * self.linear_damping);
        self.angvel *= 1.0 / (1.0 + dt * self.angular_damping);

        self.position = self.integrate_velocity(dt) * self.position;
    }

    /// The linear velocity of this rigid-body.
    pub fn linvel(&self) -> &Vector<f32> {
        &self.linvel
    }

    /// The angular velocity of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn angvel(&self) -> f32 {
        self.angvel
    }

    /// The angular velocity of this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn angvel(&self) -> &Vector<f32> {
        &self.angvel
    }

    /// The linear velocity of this rigid-body.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    pub fn set_linvel(&mut self, linvel: Vector<f32>, wake_up: bool) {
        self.linvel = linvel;

        if self.is_dynamic() && wake_up {
            self.wake_up(true)
        }
    }

    /// The angular velocity of this rigid-body.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    #[cfg(feature = "dim2")]
    pub fn set_angvel(&mut self, angvel: f32, wake_up: bool) {
        self.angvel = angvel;

        if self.is_dynamic() && wake_up {
            self.wake_up(true)
        }
    }

    /// The angular velocity of this rigid-body.
    ///
    /// If `wake_up` is `true` then the rigid-body will be woken up if it was
    /// put to sleep because it did not move for a while.
    #[cfg(feature = "dim3")]
    pub fn set_angvel(&mut self, angvel: Vector<f32>, wake_up: bool) {
        self.angvel = angvel;

        if self.is_dynamic() && wake_up {
            self.wake_up(true)
        }
    }

    /// The world-space position of this rigid-body.
    pub fn position(&self) -> &Isometry<f32> {
        &self.position
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
    pub fn set_position(&mut self, pos: Isometry<f32>, wake_up: bool) {
        self.changes.insert(RigidBodyChanges::POSITION);
        self.set_position_internal(pos);

        // TODO: Do we really need to check that the body isn't dynamic?
        if wake_up && self.is_dynamic() {
            self.wake_up(true)
        }
    }

    pub(crate) fn set_position_internal(&mut self, pos: Isometry<f32>) {
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
    pub fn apply_force(&mut self, force: Vector<f32>, wake_up: bool) {
        if self.body_status == BodyStatus::Dynamic {
            self.linacc += force * self.mass_properties.inv_mass;

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies an impulse at the center-of-mass of this rigid-body.
    pub fn apply_impulse(&mut self, impulse: Vector<f32>, wake_up: bool) {
        if self.body_status == BodyStatus::Dynamic {
            self.linvel += impulse * self.mass_properties.inv_mass;

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies a torque at the center-of-mass of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn apply_torque(&mut self, torque: f32, wake_up: bool) {
        if self.body_status == BodyStatus::Dynamic {
            self.angacc += self.world_inv_inertia_sqrt * (self.world_inv_inertia_sqrt * torque);

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies a torque at the center-of-mass of this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn apply_torque(&mut self, torque: Vector<f32>, wake_up: bool) {
        if self.body_status == BodyStatus::Dynamic {
            self.angacc += self.world_inv_inertia_sqrt * (self.world_inv_inertia_sqrt * torque);

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies an impulsive torque at the center-of-mass of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn apply_torque_impulse(&mut self, torque_impulse: f32, wake_up: bool) {
        if self.body_status == BodyStatus::Dynamic {
            self.angvel +=
                self.world_inv_inertia_sqrt * (self.world_inv_inertia_sqrt * torque_impulse);

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies an impulsive torque at the center-of-mass of this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn apply_torque_impulse(&mut self, torque_impulse: Vector<f32>, wake_up: bool) {
        if self.body_status == BodyStatus::Dynamic {
            self.angvel +=
                self.world_inv_inertia_sqrt * (self.world_inv_inertia_sqrt * torque_impulse);

            if wake_up {
                self.wake_up(true);
            }
        }
    }

    /// Applies a force at the given world-space point of this rigid-body.
    pub fn apply_force_at_point(&mut self, force: Vector<f32>, point: Point<f32>, wake_up: bool) {
        let torque = (point - self.world_com).gcross(force);
        self.apply_force(force, wake_up);
        self.apply_torque(torque, wake_up);
    }

    /// Applies an impulse at the given world-space point of this rigid-body.
    pub fn apply_impulse_at_point(
        &mut self,
        impulse: Vector<f32>,
        point: Point<f32>,
        wake_up: bool,
    ) {
        let torque_impulse = (point - self.world_com).gcross(impulse);
        self.apply_impulse(impulse, wake_up);
        self.apply_torque_impulse(torque_impulse, wake_up);
    }

    /// The velocity of the given world-space point on this rigid-body.
    pub fn velocity_at_point(&self, point: &Point<f32>) -> Vector<f32> {
        let dpt = point - self.world_com;
        self.linvel + self.angvel.gcross(dpt)
    }
}

/// A builder for rigid-bodies.
pub struct RigidBodyBuilder {
    position: Isometry<f32>,
    linvel: Vector<f32>,
    angvel: AngVector<f32>,
    linear_damping: f32,
    angular_damping: f32,
    body_status: BodyStatus,
    flags: RigidBodyFlags,
    mass_properties: MassProperties,
    can_sleep: bool,
    sleeping: bool,
    user_data: u128,
}

impl RigidBodyBuilder {
    /// Initialize a new builder for a rigid body which is either static, dynamic, or kinematic.
    pub fn new(body_status: BodyStatus) -> Self {
        Self {
            position: Isometry::identity(),
            linvel: Vector::zeros(),
            angvel: na::zero(),
            linear_damping: 0.0,
            angular_damping: 0.0,
            body_status,
            flags: RigidBodyFlags::empty(),
            mass_properties: MassProperties::zero(),
            can_sleep: true,
            sleeping: false,
            user_data: 0,
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

    /// An arbitrary user-defined 128-bit integer associated to the rigid-bodies built by this builder.
    pub fn user_data(mut self, data: u128) -> Self {
        self.user_data = data;
        self
    }

    /// Sets the mass properties of the rigid-body being built.
    ///
    /// Note that the final mass properties of the rigid-bodies depends
    /// on the initial mass-properties of the rigid-body (set by this method)
    /// to which is added the contributions of all the colliders with non-zero density
    /// attached to this rigid-body.
    ///
    /// Therefore, if you want your provided mass properties to be the final
    /// mass properties of your rigid-body, don't attach colliders to it, or
    /// only attach colliders with densities equal to zero.
    pub fn mass_properties(mut self, props: MassProperties) -> Self {
        self.mass_properties = props;
        self
    }

    /// Prevents this rigid-body from translating because of forces.
    ///
    /// This is equivalent to `self.mass(0.0, false)`. See the
    /// documentation of [`RigidBodyBuilder::mass`] for more details.
    pub fn lock_translations(self) -> Self {
        self.mass(0.0, false)
    }

    /// Prevents this rigid-body from rotating because of forces.
    ///
    /// This is equivalent to `self.principal_inertia(0.0, false)` (in 2D) or
    /// `self.principal_inertia(Vector3::zeros(), Vector3::repeat(false))` (in 3D).
    ///
    /// See the documentation of [`RigidBodyBuilder::principal_inertia`] for more details.
    pub fn lock_rotations(self) -> Self {
        #[cfg(feature = "dim2")]
        return self.principal_angular_inertia(0.0, false);
        #[cfg(feature = "dim3")]
        return self.principal_angular_inertia(Vector::zeros(), Vector::repeat(false));
    }

    /// Sets the mass of the rigid-body being built.
    ///
    /// In order to lock the translations of this rigid-body (by
    /// making them kinematic), call `.mass(0.0, false)`.
    ///
    /// If `colliders_contribution_enabled` is `false`, then the mass specified here
    /// will be the final mass of the rigid-body created by this builder.
    /// If `colliders_contribution_enabled` is `true`, then the final mass of the rigid-body
    /// will depends on the initial mass set by this method to which is added
    /// the contributions of all the colliders with non-zero density attached to
    /// this rigid-body.
    pub fn mass(mut self, mass: f32, colliders_contribution_enabled: bool) -> Self {
        self.mass_properties.inv_mass = utils::inv(mass);
        self.flags.set(
            RigidBodyFlags::IGNORE_COLLIDER_MASS,
            !colliders_contribution_enabled,
        );
        self
    }
    /// Sets the angular inertia of this rigid-body.
    ///
    /// In order to lock the rotations of this rigid-body (by
    /// making them kinematic), call `.principal_inertia(0.0, false)`.
    ///
    /// If `colliders_contribution_enabled` is `false`, then the principal inertia specified here
    /// will be the final principal inertia of the rigid-body created by this builder.
    /// If `colliders_contribution_enabled` is `true`, then the final principal of the rigid-body
    /// will depend on the initial principal inertia set by this method to which is added
    /// the contributions of all the colliders with non-zero density attached to this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn principal_angular_inertia(
        mut self,
        inertia: f32,
        colliders_contribution_enabled: bool,
    ) -> Self {
        self.mass_properties.inv_principal_inertia_sqrt = utils::inv(inertia);
        self.flags.set(
            RigidBodyFlags::IGNORE_COLLIDER_ANGULAR_INERTIA_X
                | RigidBodyFlags::IGNORE_COLLIDER_ANGULAR_INERTIA_Y
                | RigidBodyFlags::IGNORE_COLLIDER_ANGULAR_INERTIA_Z,
            !colliders_contribution_enabled,
        );
        self
    }

    /// Use `self.principal_angular_inertia` instead.
    #[cfg(feature = "dim2")]
    #[deprecated(note = "renamed to `principal_angular_inertia`.")]
    pub fn principal_inertia(self, inertia: f32, colliders_contribution_enabled: bool) -> Self {
        self.principal_angular_inertia(inertia, colliders_contribution_enabled)
    }

    /// Sets the principal angular inertia of this rigid-body.
    ///
    /// In order to lock the rotations of this rigid-body (by
    /// making them kinematic), call `.principal_inertia(Vector3::zeros(), Vector3::repeat(false))`.
    ///
    /// If `colliders_contribution_enabled[i]` is `false`, then the principal inertia specified here
    /// along the `i`-th local axis of the rigid-body, will be the final principal inertia along
    /// the `i`-th local axis of the rigid-body created by this builder.
    /// If `colliders_contribution_enabled[i]` is `true`, then the final principal of the rigid-body
    /// along its `i`-th local axis will depend on the initial principal inertia set by this method
    /// to which is added the contributions of all the colliders with non-zero density
    /// attached to this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn principal_angular_inertia(
        mut self,
        inertia: AngVector<f32>,
        colliders_contribution_enabled: AngVector<bool>,
    ) -> Self {
        self.mass_properties.inv_principal_inertia_sqrt = inertia.map(utils::inv);
        self.flags.set(
            RigidBodyFlags::IGNORE_COLLIDER_ANGULAR_INERTIA_X,
            !colliders_contribution_enabled.x,
        );
        self.flags.set(
            RigidBodyFlags::IGNORE_COLLIDER_ANGULAR_INERTIA_Y,
            !colliders_contribution_enabled.y,
        );
        self.flags.set(
            RigidBodyFlags::IGNORE_COLLIDER_ANGULAR_INERTIA_Z,
            !colliders_contribution_enabled.z,
        );
        self
    }

    /// Use `self.principal_angular_inertia` instead.
    #[cfg(feature = "dim3")]
    #[deprecated(note = "renamed to `principal_angular_inertia`.")]
    pub fn principal_inertia(
        self,
        inertia: AngVector<f32>,
        colliders_contribution_enabled: AngVector<bool>,
    ) -> Self {
        self.principal_angular_inertia(inertia, colliders_contribution_enabled)
    }

    /// Sets the damping factor for the linear part of the rigid-body motion.
    ///
    /// The higher the linear damping factor is, the more quickly the rigid-body
    /// will slow-down its translational movement.
    pub fn linear_damping(mut self, factor: f32) -> Self {
        self.linear_damping = factor;
        self
    }

    /// Sets the damping factor for the angular part of the rigid-body motion.
    ///
    /// The higher the angular damping factor is, the more quickly the rigid-body
    /// will slow-down its rotational movement.
    pub fn angular_damping(mut self, factor: f32) -> Self {
        self.angular_damping = factor;
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

    /// Sets whether or not the rigid-body is to be created asleep.
    pub fn sleeping(mut self, sleeping: bool) -> Self {
        self.sleeping = sleeping;
        self
    }

    /// Build a new rigid-body with the parameters configured with this builder.
    pub fn build(&self) -> RigidBody {
        let mut rb = RigidBody::new();
        rb.predicted_position = self.position; // FIXME: compute the correct value?
        rb.set_position_internal(self.position);
        rb.linvel = self.linvel;
        rb.angvel = self.angvel;
        rb.body_status = self.body_status;
        rb.user_data = self.user_data;
        rb.mass_properties = self.mass_properties;
        rb.linear_damping = self.linear_damping;
        rb.angular_damping = self.angular_damping;
        rb.flags = self.flags;

        if self.can_sleep && self.sleeping {
            rb.sleep();
        }

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
