use crate::data::{ComponentSetMut, ComponentSetOption};
use crate::dynamics::MassProperties;
use crate::geometry::{
    ColliderChanges, ColliderHandle, ColliderMassProperties, ColliderParent, ColliderPosition,
    ColliderShape, InteractionGraph, RigidBodyGraphIndex,
};
use crate::math::{AngVector, AngularInertia, Isometry, Point, Real, Translation, Vector};
use crate::parry::partitioning::IndexedData;
use crate::utils::WDot;
use num::Zero;

/// The unique handle of a rigid body added to a `RigidBodySet`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[repr(transparent)]
pub struct RigidBodyHandle(pub crate::data::arena::Index);

impl RigidBodyHandle {
    /// Converts this handle into its (index, generation) components.
    pub fn into_raw_parts(self) -> (u32, u32) {
        self.0.into_raw_parts()
    }

    /// Reconstructs an handle from its (index, generation) components.
    pub fn from_raw_parts(id: u32, generation: u32) -> Self {
        Self(crate::data::arena::Index::from_raw_parts(id, generation))
    }

    /// An always-invalid rigid-body handle.
    pub fn invalid() -> Self {
        Self(crate::data::arena::Index::from_raw_parts(
            crate::INVALID_U32,
            crate::INVALID_U32,
        ))
    }
}

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
    Dynamic,
    /// A `RigidBodyType::Static` body cannot be affected by external forces.
    Static,
    /// A `RigidBodyType::Kinematic` body cannot be affected by any external forces but can be controlled
    /// by the user at the position level while keeping realistic one-way interaction with dynamic bodies.
    ///
    /// One-way interaction means that a kinematic body can push a dynamic body, but a kinematic body
    /// cannot be pushed by anything. In other words, the trajectory of a kinematic body can only be
    /// modified by the user and is independent from any contact or joint it is involved in.
    Kinematic,
    // Semikinematic, // A kinematic that performs automatic CCD with the static environment to avoid traversing it?
    // Disabled,
}

impl RigidBodyType {
    pub fn is_static(self) -> bool {
        self == RigidBodyType::Static
    }

    pub fn is_dynamic(self) -> bool {
        self == RigidBodyType::Dynamic
    }

    pub fn is_kinematic(self) -> bool {
        self == RigidBodyType::Kinematic
    }
}

bitflags::bitflags! {
    #[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
    /// Flags describing how the rigid-body has been modified by the user.
    pub struct RigidBodyChanges: u32 {
        const MODIFIED    = 1 << 0;
        const POSITION    = 1 << 1;
        const SLEEP       = 1 << 2;
        const COLLIDERS   = 1 << 3;
        const TYPE        = 1 << 4;
    }
}

impl Default for RigidBodyChanges {
    fn default() -> Self {
        RigidBodyChanges::empty()
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, Copy)]
pub struct RigidBodyPosition {
    /// The world-space position of the rigid-body.
    pub position: Isometry<Real>,
    /// The next position of the rigid-body.
    ///
    /// At the beginning of the timestep, and when the
    /// timestep is complete we must have position == next_position
    /// except for kinematic bodies.
    ///
    /// The next_position is updated after the velocity and position
    /// resolution. Then it is either validated (ie. we set position := set_position)
    /// or clamped by CCD.
    pub next_position: Isometry<Real>,
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
    #[must_use]
    pub fn interpolate_velocity(&self, inv_dt: Real) -> RigidBodyVelocity {
        let dpos = self.next_position * self.position.inverse();
        let angvel;
        #[cfg(feature = "dim2")]
        {
            angvel = dpos.rotation.angle() * inv_dt;
        }
        #[cfg(feature = "dim3")]
        {
            angvel = dpos.rotation.scaled_axis() * inv_dt;
        }
        let linvel = dpos.translation.vector * inv_dt;
        RigidBodyVelocity { linvel, angvel }
    }

    #[must_use]
    pub fn integrate_forces_and_velocities(
        &self,
        dt: Real,
        forces: &RigidBodyForces,
        vels: &RigidBodyVelocity,
        mprops: &RigidBodyMassProps,
    ) -> Isometry<Real> {
        let new_vels = forces.integrate(dt, vels, mprops);
        new_vels.integrate(dt, &self.position, &mprops.mass_properties.local_com)
    }
}

impl<T> From<T> for RigidBodyPosition
where
    Isometry<Real>: From<T>,
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
    pub struct RigidBodyMassPropsFlags: u8 {
        const TRANSLATION_LOCKED = 1 << 0;
        const ROTATION_LOCKED_X = 1 << 1;
        const ROTATION_LOCKED_Y = 1 << 2;
        const ROTATION_LOCKED_Z = 1 << 3;
        const ROTATION_LOCKED = Self::ROTATION_LOCKED_X.bits | Self::ROTATION_LOCKED_Y.bits | Self::ROTATION_LOCKED_Z.bits;
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, Copy)]
pub struct RigidBodyMassProps {
    /// Flags for locking rotation and translation.
    pub flags: RigidBodyMassPropsFlags,
    /// The local mass properties of the rigid-body.
    pub mass_properties: MassProperties,
    /// The world-space center of mass of the rigid-body.
    pub world_com: Point<Real>,
    /// The inverse mass taking into account translation locking.
    pub effective_inv_mass: Real,
    /// The square-root of the world-space inverse angular inertia tensor of the rigid-body,
    /// taking into account rotation locking.
    pub effective_world_inv_inertia_sqrt: AngularInertia<Real>,
}

impl Default for RigidBodyMassProps {
    fn default() -> Self {
        Self {
            flags: RigidBodyMassPropsFlags::empty(),
            mass_properties: MassProperties::zero(),
            world_com: Point::origin(),
            effective_inv_mass: 0.0,
            effective_world_inv_inertia_sqrt: AngularInertia::zero(),
        }
    }
}

impl From<RigidBodyMassPropsFlags> for RigidBodyMassProps {
    fn from(flags: RigidBodyMassPropsFlags) -> Self {
        Self {
            flags,
            ..Self::default()
        }
    }
}

impl RigidBodyMassProps {
    #[must_use]
    pub fn with_translations_locked(mut self) -> Self {
        self.flags |= RigidBodyMassPropsFlags::TRANSLATION_LOCKED;
        self
    }

    pub fn effective_mass(&self) -> Real {
        crate::utils::inv(self.effective_inv_mass)
    }

    pub fn update_world_mass_properties(&mut self, position: &Isometry<Real>) {
        self.world_com = self.mass_properties.world_com(&position);
        self.effective_inv_mass = self.mass_properties.inv_mass;
        self.effective_world_inv_inertia_sqrt = self
            .mass_properties
            .world_inv_inertia_sqrt(&position.rotation);

        // Take into account translation/rotation locking.
        if self
            .flags
            .contains(RigidBodyMassPropsFlags::TRANSLATION_LOCKED)
        {
            self.effective_inv_mass = 0.0;
        }

        #[cfg(feature = "dim2")]
        {
            if self
                .flags
                .contains(RigidBodyMassPropsFlags::ROTATION_LOCKED_Z)
            {
                self.effective_world_inv_inertia_sqrt = 0.0;
            }
        }
        #[cfg(feature = "dim3")]
        {
            if self
                .flags
                .contains(RigidBodyMassPropsFlags::ROTATION_LOCKED_X)
            {
                self.effective_world_inv_inertia_sqrt.m11 = 0.0;
                self.effective_world_inv_inertia_sqrt.m12 = 0.0;
                self.effective_world_inv_inertia_sqrt.m13 = 0.0;
            }

            if self
                .flags
                .contains(RigidBodyMassPropsFlags::ROTATION_LOCKED_Y)
            {
                self.effective_world_inv_inertia_sqrt.m22 = 0.0;
                self.effective_world_inv_inertia_sqrt.m12 = 0.0;
                self.effective_world_inv_inertia_sqrt.m23 = 0.0;
            }
            if self
                .flags
                .contains(RigidBodyMassPropsFlags::ROTATION_LOCKED_Z)
            {
                self.effective_world_inv_inertia_sqrt.m33 = 0.0;
                self.effective_world_inv_inertia_sqrt.m13 = 0.0;
                self.effective_world_inv_inertia_sqrt.m23 = 0.0;
            }
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, Copy)]
pub struct RigidBodyVelocity {
    /// The linear velocity of the rigid-body.
    pub linvel: Vector<Real>,
    /// The angular velocity of the rigid-body.
    pub angvel: AngVector<Real>,
}

impl Default for RigidBodyVelocity {
    fn default() -> Self {
        Self::zero()
    }
}

impl RigidBodyVelocity {
    #[must_use]
    pub fn zero() -> Self {
        Self {
            linvel: na::zero(),
            angvel: na::zero(),
        }
    }

    #[must_use]
    pub fn pseudo_kinetic_energy(&self) -> Real {
        self.linvel.norm_squared() + self.angvel.gdot(self.angvel)
    }

    #[must_use]
    pub fn apply_damping(&self, dt: Real, damping: &RigidBodyDamping) -> Self {
        RigidBodyVelocity {
            linvel: self.linvel * (1.0 / (1.0 + dt * damping.linear_damping)),
            angvel: self.angvel * (1.0 / (1.0 + dt * damping.angular_damping)),
        }
    }

    #[must_use]
    pub fn integrate(
        &self,
        dt: Real,
        init_pos: &Isometry<Real>,
        local_com: &Point<Real>,
    ) -> Isometry<Real> {
        let com = init_pos * local_com;
        let shift = Translation::from(com.coords);
        let mut result =
            shift * Isometry::new(self.linvel * dt, self.angvel * dt) * shift.inverse() * init_pos;
        result.rotation.renormalize_fast();
        result
    }

    #[must_use]
    pub fn is_zero(&self) -> bool {
        self.linvel.is_zero() && self.angvel.is_zero()
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, Copy)]
pub struct RigidBodyDamping {
    /// Damping factor for gradually slowing down the translational motion of the rigid-body.
    pub linear_damping: Real,
    /// Damping factor for gradually slowing down the angular motion of the rigid-body.
    pub angular_damping: Real,
}

impl Default for RigidBodyDamping {
    fn default() -> Self {
        Self {
            linear_damping: 0.0,
            angular_damping: 0.0,
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, Copy)]
pub struct RigidBodyForces {
    /// Accumulation of external forces (only for dynamic bodies).
    pub force: Vector<Real>,
    /// Accumulation of external torques (only for dynamic bodies).
    pub torque: AngVector<Real>,
    pub gravity_scale: Real,
}

impl Default for RigidBodyForces {
    fn default() -> Self {
        Self {
            force: na::zero(),
            torque: na::zero(),
            gravity_scale: 1.0,
        }
    }
}

impl RigidBodyForces {
    #[must_use]
    pub fn integrate(
        &self,
        dt: Real,
        init_vels: &RigidBodyVelocity,
        mprops: &RigidBodyMassProps,
    ) -> RigidBodyVelocity {
        let linear_acc = self.force * mprops.effective_inv_mass;
        let angular_acc = mprops.effective_world_inv_inertia_sqrt
            * (mprops.effective_world_inv_inertia_sqrt * self.torque);

        RigidBodyVelocity {
            linvel: init_vels.linvel + linear_acc * dt,
            angvel: init_vels.angvel + angular_acc * dt,
        }
    }

    pub fn add_linear_acceleration(&mut self, gravity: &Vector<Real>, mass: Real) {
        self.force += gravity * self.gravity_scale * mass;
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, Copy)]
pub struct RigidBodyCcd {
    pub ccd_thickness: Real,
    pub ccd_max_dist: Real,
    pub ccd_active: bool,
    pub ccd_enabled: bool,
}

impl Default for RigidBodyCcd {
    fn default() -> Self {
        Self {
            ccd_thickness: 0.0,
            ccd_max_dist: 0.0,
            ccd_active: false,
            ccd_enabled: false,
        }
    }
}

impl RigidBodyCcd {
    pub fn max_point_velocity(&self, vels: &RigidBodyVelocity) -> Real {
        #[cfg(feature = "dim2")]
        return vels.linvel.norm() + vels.angvel.abs() * self.ccd_max_dist;
        #[cfg(feature = "dim3")]
        return vels.linvel.norm() + vels.angvel.norm() * self.ccd_max_dist;
    }

    pub fn is_moving_fast(
        &self,
        dt: Real,
        vels: &RigidBodyVelocity,
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
#[derive(Clone, Debug, Copy)]
pub struct RigidBodyIds {
    pub joint_graph_index: RigidBodyGraphIndex,
    pub active_island_id: usize,
    pub active_set_id: usize,
    pub active_set_offset: usize,
    pub active_set_timestamp: u32,
}

impl Default for RigidBodyIds {
    fn default() -> Self {
        Self {
            joint_graph_index: InteractionGraph::<(), ()>::invalid_graph_index(),
            active_island_id: 0,
            active_set_id: 0,
            active_set_offset: 0,
            active_set_timestamp: 0,
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug)]
pub struct RigidBodyColliders(pub Vec<ColliderHandle>);

impl Default for RigidBodyColliders {
    fn default() -> Self {
        Self(vec![])
    }
}

impl RigidBodyColliders {
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

    pub fn attach_collider(
        &mut self,
        rb_changes: &mut RigidBodyChanges,
        rb_ccd: &mut RigidBodyCcd,
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
            .max(shape_bsphere.center.coords.norm() + shape_bsphere.radius);

        let mass_properties = co_mprops
            .mass_properties(&**co_shape)
            .transform_by(&co_parent.pos_wrt_parent);
        self.0.push(co_handle);
        rb_mprops.mass_properties += mass_properties;
        rb_mprops.update_world_mass_properties(&rb_pos.position);
    }

    pub fn update_positions<Colliders>(
        &self,
        colliders: &mut Colliders,
        modified_colliders: &mut Vec<ColliderHandle>,
        parent_pos: &Isometry<Real>,
    ) where
        Colliders: ComponentSetMut<ColliderPosition>
            + ComponentSetMut<ColliderChanges>
            + ComponentSetOption<ColliderParent>,
    {
        for handle in &self.0 {
            // NOTE: the ColliderParent component must exist if we enter this method.
            let co_parent: &ColliderParent = colliders
                .get(handle.0)
                .expect("Could not find the ColliderParent component.");
            let new_pos = parent_pos * co_parent.pos_wrt_parent;

            // Set the modification flag so we can benefit from the modification-tracking
            // when updating the narrow-phase/broad-phase afterwards.
            colliders.map_mut_internal(handle.0, |co_changes: &mut ColliderChanges| {
                if !co_changes.contains(ColliderChanges::MODIFIED) {
                    modified_colliders.push(*handle);
                }

                *co_changes |= ColliderChanges::POSITION;
            });
            colliders.set_internal(handle.0, ColliderPosition(new_pos));
        }
    }
}

#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
#[derive(Clone, Debug, Copy)]
pub struct RigidBodyDominance(pub i8);

impl Default for RigidBodyDominance {
    fn default() -> Self {
        RigidBodyDominance(0)
    }
}

impl RigidBodyDominance {
    pub fn effective_group(&self, status: &RigidBodyType) -> i16 {
        if status.is_dynamic() {
            self.0 as i16
        } else {
            i8::MAX as i16 + 1
        }
    }
}

/// The rb_activation status of a body.
///
/// This controls whether a body is sleeping or not.
/// If the threshold is negative, the body never sleeps.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "serde-serialize", derive(Serialize, Deserialize))]
pub struct RigidBodyActivation {
    /// The threshold pseudo-kinetic energy bellow which the body can fall asleep.
    pub threshold: Real,
    /// The current pseudo-kinetic energy of the body.
    pub energy: Real,
    /// Is this body already sleeping?
    pub sleeping: bool,
}

impl Default for RigidBodyActivation {
    fn default() -> Self {
        Self::new_active()
    }
}

impl RigidBodyActivation {
    /// The default amount of energy bellow which a body can be put to sleep by nphysics.
    pub fn default_threshold() -> Real {
        0.01
    }

    /// Create a new rb_activation status initialised with the default rb_activation threshold and is active.
    pub fn new_active() -> Self {
        RigidBodyActivation {
            threshold: Self::default_threshold(),
            energy: Self::default_threshold() * 4.0,
            sleeping: false,
        }
    }

    /// Create a new rb_activation status initialised with the default rb_activation threshold and is inactive.
    pub fn new_inactive() -> Self {
        RigidBodyActivation {
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

    #[inline]
    pub fn wake_up(&mut self, strong: bool) {
        self.sleeping = false;
        if strong || self.energy == 0.0 {
            self.energy = self.threshold.abs() * 2.0;
        }
    }

    #[inline]
    pub fn sleep(&mut self) {
        self.energy = 0.0;
        self.sleeping = true;
    }
}
