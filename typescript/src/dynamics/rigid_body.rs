use crate::dynamics::{RawRigidBodySet, RawRigidBodyType};
use crate::geometry::RawColliderSet;
#[cfg(feature = "dim3")]
use crate::math::RawRotation;
use crate::math::RawVector;
use crate::utils::{self, FlatHandle};
use na::Point;
use rapier::dynamics::MassProperties;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
impl RawRigidBodySet {
    /// The world-space translation of this rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim2")]
    pub fn rbTranslation(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.position().translation.vector;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
        });
    }

    /// The world-space translation of this rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbTranslation(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.position().translation.vector;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        });
    }

    /// The world-space orientation of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbRotation(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.position().rotation.angle())
    }

    /// The world-space orientation of this rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbRotation(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.position().rotation;
            let inner = u.into_inner();
            scratch_buffer.set_index(0, inner.i);
            scratch_buffer.set_index(1, inner.j);
            scratch_buffer.set_index(2, inner.k);
            scratch_buffer.set_index(3, inner.w);
        });
    }

    /// Put the given rigid-body to sleep.
    pub fn rbSleep(&mut self, handle: FlatHandle) {
        self.map_mut(handle, |rb| rb.sleep());
    }

    /// Is this rigid-body sleeping?
    pub fn rbIsSleeping(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_sleeping())
    }

    /// Is the velocity of this rigid-body not zero?
    pub fn rbIsMoving(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_moving())
    }

    /// The world-space predicted translation of this rigid-body.
    ///
    /// If this rigid-body is kinematic this value is set by the `setNextKinematicTranslation`
    /// method and is used for estimating the kinematic body velocity at the next timestep.
    /// For non-kinematic bodies, this value is currently unspecified.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim2")]
    pub fn rbNextTranslation(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.next_position().translation.vector;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
        });
    }

    /// The world-space predicted translation of this rigid-body.
    ///
    /// If this rigid-body is kinematic this value is set by the `setNextKinematicTranslation`
    /// method and is used for estimating the kinematic body velocity at the next timestep.
    /// For non-kinematic bodies, this value is currently unspecified.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbNextTranslation(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.next_position().translation.vector;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        });
    }

    /// The world-space predicted orientation of this rigid-body.
    ///
    /// If this rigid-body is kinematic this value is set by the `setNextKinematicRotation`
    /// method and is used for estimating the kinematic body velocity at the next timestep.
    /// For non-kinematic bodies, this value is currently unspecified.
    #[cfg(feature = "dim2")]
    pub fn rbNextRotation(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.next_position().rotation.angle())
    }

    /// The world-space predicted orientation of this rigid-body.
    ///
    /// If this rigid-body is kinematic this value is set by the `setNextKinematicRotation`
    /// method and is used for estimating the kinematic body velocity at the next timestep.
    /// For non-kinematic bodies, this value is currently unspecified.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbNextRotation(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.next_position().rotation;
            let inner = u.into_inner();
            scratch_buffer.set_index(0, inner.i);
            scratch_buffer.set_index(1, inner.j);
            scratch_buffer.set_index(2, inner.k);
            scratch_buffer.set_index(3, inner.w);
        });
    }

    /// Sets the translation of this rigid-body.
    ///
    /// # Parameters
    /// - `x`: the world-space position of the rigid-body along the `x` axis.
    /// - `y`: the world-space position of the rigid-body along the `y` axis.
    /// - `z`: the world-space position of the rigid-body along the `z` axis.
    /// - `wakeUp`: forces the rigid-body to wake-up so it is properly affected by forces if it
    /// wasn't moving before modifying its position.
    #[cfg(feature = "dim3")]
    pub fn rbSetTranslation(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.set_translation(na::Vector3::new(x, y, z), wakeUp);
        })
    }

    /// Sets the translation of this rigid-body.
    ///
    /// # Parameters
    /// - `x`: the world-space position of the rigid-body along the `x` axis.
    /// - `y`: the world-space position of the rigid-body along the `y` axis.
    /// - `wakeUp`: forces the rigid-body to wake-up so it is properly affected by forces if it
    /// wasn't moving before modifying its position.
    #[cfg(feature = "dim2")]
    pub fn rbSetTranslation(&mut self, handle: FlatHandle, x: f32, y: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.set_translation(na::Vector2::new(x, y), wakeUp);
        })
    }

    /// Sets the rotation quaternion of this rigid-body.
    ///
    /// This does nothing if a zero quaternion is provided.
    ///
    /// # Parameters
    /// - `x`: the first vector component of the quaternion.
    /// - `y`: the second vector component of the quaternion.
    /// - `z`: the third vector component of the quaternion.
    /// - `w`: the scalar component of the quaternion.
    /// - `wakeUp`: forces the rigid-body to wake-up so it is properly affected by forces if it
    /// wasn't moving before modifying its position.
    #[cfg(feature = "dim3")]
    pub fn rbSetRotation(
        &mut self,
        handle: FlatHandle,
        x: f32,
        y: f32,
        z: f32,
        w: f32,
        wakeUp: bool,
    ) {
        if let Some(q) = na::Unit::try_new(na::Quaternion::new(w, x, y, z), 0.0) {
            self.map_mut(handle, |rb| rb.set_rotation(q, wakeUp))
        }
    }

    /// Sets the rotation angle of this rigid-body.
    ///
    /// # Parameters
    /// - `angle`: the rotation angle, in radians.
    /// - `wakeUp`: forces the rigid-body to wake-up so it is properly affected by forces if it
    /// wasn't moving before modifying its position.
    #[cfg(feature = "dim2")]
    pub fn rbSetRotation(&mut self, handle: FlatHandle, angle: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.set_rotation(na::UnitComplex::new(angle), wakeUp)
        })
    }

    /// Sets the linear velocity of this rigid-body.
    pub fn rbSetLinvel(&mut self, handle: FlatHandle, linvel: &RawVector, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.set_linvel(linvel.0, wakeUp);
        });
    }

    /// Sets the angular velocity of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbSetAngvel(&mut self, handle: FlatHandle, angvel: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.set_angvel(angvel, wakeUp);
        });
    }

    /// Sets the angular velocity of this rigid-body.
    #[cfg(feature = "dim3")]
    pub fn rbSetAngvel(&mut self, handle: FlatHandle, angvel: &RawVector, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.set_angvel(angvel.0, wakeUp);
        });
    }

    /// If this rigid body is kinematic, sets its future translation after the next timestep integration.
    ///
    /// This should be used instead of `rigidBody.setTranslation` to make the dynamic object
    /// interacting with this kinematic body behave as expected. Internally, Rapier will compute
    /// an artificial velocity for this rigid-body from its current position and its next kinematic
    /// position. This velocity will be used to compute forces on dynamic bodies interacting with
    /// this body.
    ///
    /// # Parameters
    /// - `x`: the world-space position of the rigid-body along the `x` axis.
    /// - `y`: the world-space position of the rigid-body along the `y` axis.
    /// - `z`: the world-space position of the rigid-body along the `z` axis.
    #[cfg(feature = "dim3")]
    pub fn rbSetNextKinematicTranslation(&mut self, handle: FlatHandle, x: f32, y: f32, z: f32) {
        self.map_mut(handle, |rb| {
            rb.set_next_kinematic_translation(na::Vector3::new(x, y, z));
        })
    }

    /// If this rigid body is kinematic, sets its future translation after the next timestep integration.
    ///
    /// This should be used instead of `rigidBody.setTranslation` to make the dynamic object
    /// interacting with this kinematic body behave as expected. Internally, Rapier will compute
    /// an artificial velocity for this rigid-body from its current position and its next kinematic
    /// position. This velocity will be used to compute forces on dynamic bodies interacting with
    /// this body.
    ///
    /// # Parameters
    /// - `x`: the world-space position of the rigid-body along the `x` axis.
    /// - `y`: the world-space position of the rigid-body along the `y` axis.
    #[cfg(feature = "dim2")]
    pub fn rbSetNextKinematicTranslation(&mut self, handle: FlatHandle, x: f32, y: f32) {
        self.map_mut(handle, |rb| {
            rb.set_next_kinematic_translation(na::Vector2::new(x, y));
        })
    }

    /// If this rigid body is kinematic, sets its future rotation after the next timestep integration.
    ///
    /// This should be used instead of `rigidBody.setRotation` to make the dynamic object
    /// interacting with this kinematic body behave as expected. Internally, Rapier will compute
    /// an artificial velocity for this rigid-body from its current position and its next kinematic
    /// position. This velocity will be used to compute forces on dynamic bodies interacting with
    /// this body.
    ///
    /// # Parameters
    /// - `x`: the first vector component of the quaternion.
    /// - `y`: the second vector component of the quaternion.
    /// - `z`: the third vector component of the quaternion.
    /// - `w`: the scalar component of the quaternion.
    #[cfg(feature = "dim3")]
    pub fn rbSetNextKinematicRotation(
        &mut self,
        handle: FlatHandle,
        x: f32,
        y: f32,
        z: f32,
        w: f32,
    ) {
        if let Some(q) = na::Unit::try_new(na::Quaternion::new(w, x, y, z), 0.0) {
            self.map_mut(handle, |rb| {
                rb.set_next_kinematic_rotation(q);
            })
        }
    }

    /// If this rigid body is kinematic, sets its future rotation after the next timestep integration.
    ///
    /// This should be used instead of `rigidBody.setRotation` to make the dynamic object
    /// interacting with this kinematic body behave as expected. Internally, Rapier will compute
    /// an artificial velocity for this rigid-body from its current position and its next kinematic
    /// position. This velocity will be used to compute forces on dynamic bodies interacting with
    /// this body.
    ///
    /// # Parameters
    /// - `angle`: the rotation angle, in radians.
    #[cfg(feature = "dim2")]
    pub fn rbSetNextKinematicRotation(&mut self, handle: FlatHandle, angle: f32) {
        self.map_mut(handle, |rb| {
            rb.set_next_kinematic_rotation(na::UnitComplex::new(angle));
        })
    }

    pub fn rbRecomputeMassPropertiesFromColliders(
        &mut self,
        handle: FlatHandle,
        colliders: &RawColliderSet,
    ) {
        self.map_mut(handle, |rb| {
            rb.recompute_mass_properties_from_colliders(&colliders.0)
        })
    }

    pub fn rbSetAdditionalMass(&mut self, handle: FlatHandle, mass: f32, wake_up: bool) {
        self.map_mut(handle, |rb| {
            rb.set_additional_mass(mass, wake_up);
        })
    }

    #[cfg(feature = "dim3")]
    pub fn rbSetAdditionalMassProperties(
        &mut self,
        handle: FlatHandle,
        mass: f32,
        centerOfMass: &RawVector,
        principalAngularInertia: &RawVector,
        angularInertiaFrame: &RawRotation,
        wake_up: bool,
    ) {
        self.map_mut(handle, |rb| {
            let mprops = MassProperties::with_principal_inertia_frame(
                centerOfMass.0.into(),
                mass,
                principalAngularInertia.0,
                angularInertiaFrame.0,
            );
            rb.set_additional_mass_properties(mprops, wake_up)
        })
    }

    #[cfg(feature = "dim2")]
    pub fn rbSetAdditionalMassProperties(
        &mut self,
        handle: FlatHandle,
        mass: f32,
        centerOfMass: &RawVector,
        principalAngularInertia: f32,
        wake_up: bool,
    ) {
        self.map_mut(handle, |rb| {
            let props = MassProperties::new(centerOfMass.0.into(), mass, principalAngularInertia);
            rb.set_additional_mass_properties(props, wake_up)
        })
    }

    /// The linear velocity of this rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim2")]
    pub fn rbLinvel(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.linvel();
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
        });
    }

    /// The linear velocity of this rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbLinvel(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.linvel();
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        });
    }

    /// The angular velocity of this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbAngvel(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.angvel())
    }

    /// The angular velocity of this rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbAngvel(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.angvel();
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        });
    }

    /// The velocity of the given world-space point on this rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim2")]
    pub fn rbVelocityAtPoint(
        &self,
        handle: FlatHandle,
        point: &RawVector,
        scratch_buffer: &js_sys::Float32Array,
    ) {
        self.map(handle, |rb| {
            let u = rb.velocity_at_point(&Point::from(point.0));
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
        });
    }

    /// The velocity of the given world-space point on this rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbVelocityAtPoint(
        &self,
        handle: FlatHandle,
        point: &RawVector,
        scratch_buffer: &js_sys::Float32Array,
    ) {
        self.map(handle, |rb| {
            let u = rb.velocity_at_point(&Point::from(point.0));
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        });
    }

    pub fn rbLockTranslations(&mut self, handle: FlatHandle, locked: bool, wake_up: bool) {
        self.map_mut(handle, |rb| rb.lock_translations(locked, wake_up))
    }

    #[cfg(feature = "dim2")]
    pub fn rbSetEnabledTranslations(
        &mut self,
        handle: FlatHandle,
        allow_x: bool,
        allow_y: bool,
        wake_up: bool,
    ) {
        self.map_mut(handle, |rb| {
            rb.set_enabled_translations(allow_x, allow_y, wake_up)
        })
    }

    #[cfg(feature = "dim3")]
    pub fn rbSetEnabledTranslations(
        &mut self,
        handle: FlatHandle,
        allow_x: bool,
        allow_y: bool,
        allow_z: bool,
        wake_up: bool,
    ) {
        self.map_mut(handle, |rb| {
            rb.set_enabled_translations(allow_x, allow_y, allow_z, wake_up)
        })
    }

    pub fn rbLockRotations(&mut self, handle: FlatHandle, locked: bool, wake_up: bool) {
        self.map_mut(handle, |rb| rb.lock_rotations(locked, wake_up))
    }

    #[cfg(feature = "dim3")]
    pub fn rbSetEnabledRotations(
        &mut self,
        handle: FlatHandle,
        allow_x: bool,
        allow_y: bool,
        allow_z: bool,
        wake_up: bool,
    ) {
        self.map_mut(handle, |rb| {
            rb.set_enabled_rotations(allow_x, allow_y, allow_z, wake_up)
        })
    }

    pub fn rbDominanceGroup(&self, handle: FlatHandle) -> i8 {
        self.map(handle, |rb| rb.dominance_group())
    }

    pub fn rbSetDominanceGroup(&mut self, handle: FlatHandle, group: i8) {
        self.map_mut(handle, |rb| rb.set_dominance_group(group))
    }

    pub fn rbEnableCcd(&mut self, handle: FlatHandle, enabled: bool) {
        self.map_mut(handle, |rb| rb.enable_ccd(enabled))
    }

    pub fn rbSetSoftCcdPrediction(&mut self, handle: FlatHandle, prediction: f32) {
        self.map_mut(handle, |rb| rb.set_soft_ccd_prediction(prediction))
    }

    /// The mass of this rigid-body.
    pub fn rbMass(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.mass())
    }

    /// The inverse of the mass of a rigid-body.
    ///
    /// If this is zero, the rigid-body is assumed to have infinite mass.
    pub fn rbInvMass(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.mass_properties().local_mprops.inv_mass)
    }

    /// The inverse mass taking into account translation locking.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim2")]
    pub fn rbEffectiveInvMass(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.mass_properties().effective_inv_mass;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
        });
    }

    /// The inverse mass taking into account translation locking.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbEffectiveInvMass(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.mass_properties().effective_inv_mass;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        });
    }

    /// The center of mass of a rigid-body expressed in its local-space.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim2")]
    pub fn rbLocalCom(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.mass_properties().local_mprops.local_com;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
        });
    }

    /// The center of mass of a rigid-body expressed in its local-space.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbLocalCom(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.mass_properties().local_mprops.local_com;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        });
    }

    /// The world-space center of mass of the rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim2")]
    pub fn rbWorldCom(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.mass_properties().world_com;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
        });
    }

    /// The world-space center of mass of the rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbWorldCom(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.mass_properties().world_com;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        });
    }

    /// The inverse of the principal angular inertia of the rigid-body.
    ///
    /// Components set to zero are assumed to be infinite along the corresponding principal axis.
    #[cfg(feature = "dim2")]
    pub fn rbInvPrincipalInertia(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| {
            rb.mass_properties().local_mprops.inv_principal_inertia
        })
    }

    /// The inverse of the principal angular inertia of the rigid-body.
    ///
    /// Components set to zero are assumed to be infinite along the corresponding principal axis.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbInvPrincipalInertia(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.mass_properties().local_mprops.inv_principal_inertia;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        });
    }

    /// The principal vectors of the local angular inertia tensor of the rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbPrincipalInertiaLocalFrame(
        &self,
        handle: FlatHandle,
        scratch_buffer: &js_sys::Float32Array,
    ) {
        self.map(handle, |rb| {
            let u = rb
                .mass_properties()
                .local_mprops
                .principal_inertia_local_frame;
            let inner = u.into_inner();
            scratch_buffer.set_index(0, inner.i);
            scratch_buffer.set_index(1, inner.j);
            scratch_buffer.set_index(2, inner.k);
            scratch_buffer.set_index(3, inner.w);
        });
    }

    /// The angular inertia along the principal inertia axes of the rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbPrincipalInertia(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| {
            rb.mass_properties().local_mprops.principal_inertia()
        })
    }

    /// The angular inertia along the principal inertia axes of the rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbPrincipalInertia(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.mass_properties().local_mprops.principal_inertia();
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        });
    }

    /// The world-space inverse angular inertia tensor of the rigid-body,
    /// taking into account rotation locking.
    #[cfg(feature = "dim2")]
    pub fn rbEffectiveWorldInvInertia(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| {
            rb.mass_properties().effective_world_inv_inertia
        })
    }

    /// The world-space inverse angular inertia tensor of the rigid-body,
    /// taking into account rotation locking.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbEffectiveWorldInvInertia(
        &self,
        handle: FlatHandle,
        scratch_buffer: &js_sys::Float32Array,
    ) {
        self.map(handle, |rb| {
            let u = rb.mass_properties().effective_world_inv_inertia;
            scratch_buffer.set_index(0, u.m11);
            scratch_buffer.set_index(1, u.m12);
            scratch_buffer.set_index(2, u.m13);
            scratch_buffer.set_index(3, u.m22);
            scratch_buffer.set_index(4, u.m23);
            scratch_buffer.set_index(5, u.m33);
        });
    }

    /// The effective world-space angular inertia (that takes the potential rotation locking into account) of
    /// this rigid-body.
    #[cfg(feature = "dim2")]
    pub fn rbEffectiveAngularInertia(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| {
            rb.mass_properties().effective_angular_inertia()
        })
    }

    /// The effective world-space angular inertia (that takes the potential rotation locking into account) of
    /// this rigid-body.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbEffectiveAngularInertia(
        &self,
        handle: FlatHandle,
        scratch_buffer: &js_sys::Float32Array,
    ) {
        self.map(handle, |rb| {
            let u = rb.mass_properties().effective_angular_inertia();
            scratch_buffer.set_index(0, u.m11);
            scratch_buffer.set_index(1, u.m12);
            scratch_buffer.set_index(2, u.m13);
            scratch_buffer.set_index(3, u.m22);
            scratch_buffer.set_index(4, u.m23);
            scratch_buffer.set_index(5, u.m33);
        });
    }

    /// Wakes this rigid-body up.
    ///
    /// A dynamic rigid-body that does not move during several consecutive frames will
    /// be put to sleep by the physics engine, i.e., it will stop being simulated in order
    /// to avoid useless computations.
    /// This method forces a sleeping rigid-body to wake-up. This is useful, e.g., before modifying
    /// the position of a dynamic body so that it is properly simulated afterwards.
    pub fn rbWakeUp(&mut self, handle: FlatHandle) {
        self.map_mut(handle, |rb| rb.wake_up(true))
    }

    /// Is Continuous Collision Detection enabled for this rigid-body?
    pub fn rbIsCcdEnabled(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_ccd_enabled())
    }
    pub fn rbSoftCcdPrediction(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.soft_ccd_prediction())
    }

    /// The number of colliders attached to this rigid-body.
    pub fn rbNumColliders(&self, handle: FlatHandle) -> usize {
        self.map(handle, |rb| rb.colliders().len())
    }

    /// Retrieves the `i-th` collider attached to this rigid-body.
    ///
    /// # Parameters
    /// - `at`: The index of the collider to retrieve. Must be a number in `[0, this.numColliders()[`.
    ///         This index is **not** the same as the unique identifier of the collider.
    pub fn rbCollider(&self, handle: FlatHandle, at: usize) -> FlatHandle {
        self.map(handle, |rb| utils::flat_handle(rb.colliders()[at].0))
    }

    /// The status of this rigid-body: fixed, dynamic, or kinematic.
    pub fn rbBodyType(&self, handle: FlatHandle) -> RawRigidBodyType {
        self.map(handle, |rb| rb.body_type().into())
    }

    /// Set a new status for this rigid-body: fixed, dynamic, or kinematic.
    pub fn rbSetBodyType(&mut self, handle: FlatHandle, status: RawRigidBodyType, wake_up: bool) {
        self.map_mut(handle, |rb| rb.set_body_type(status.into(), wake_up));
    }

    /// Is this rigid-body fixed?
    pub fn rbIsFixed(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_fixed())
    }

    /// Is this rigid-body kinematic?
    pub fn rbIsKinematic(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_kinematic())
    }

    /// Is this rigid-body dynamic?
    pub fn rbIsDynamic(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_dynamic())
    }

    /// The linear damping coefficient of this rigid-body.
    pub fn rbLinearDamping(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.linear_damping())
    }

    /// The angular damping coefficient of this rigid-body.
    pub fn rbAngularDamping(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.angular_damping())
    }

    pub fn rbSetLinearDamping(&mut self, handle: FlatHandle, factor: f32) {
        self.map_mut(handle, |rb| rb.set_linear_damping(factor));
    }

    pub fn rbSetAngularDamping(&mut self, handle: FlatHandle, factor: f32) {
        self.map_mut(handle, |rb| rb.set_angular_damping(factor));
    }

    pub fn rbSetEnabled(&mut self, handle: FlatHandle, enabled: bool) {
        self.map_mut(handle, |rb| rb.set_enabled(enabled))
    }

    pub fn rbIsEnabled(&self, handle: FlatHandle) -> bool {
        self.map(handle, |rb| rb.is_enabled())
    }

    pub fn rbGravityScale(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.gravity_scale())
    }

    pub fn rbSetGravityScale(&mut self, handle: FlatHandle, factor: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| rb.set_gravity_scale(factor, wakeUp));
    }

    /// Resets to zero all user-added forces added to this rigid-body.
    pub fn rbResetForces(&mut self, handle: FlatHandle, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.reset_forces(wakeUp);
        })
    }

    /// Resets to zero all user-added torques added to this rigid-body.
    pub fn rbResetTorques(&mut self, handle: FlatHandle, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.reset_torques(wakeUp);
        })
    }

    /// Adds a force at the center-of-mass of this rigid-body.
    ///
    /// # Parameters
    /// - `force`: the world-space force to apply on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    pub fn rbAddForce(&mut self, handle: FlatHandle, force: &RawVector, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.add_force(force.0, wakeUp);
        })
    }

    /// Applies an impulse at the center-of-mass of this rigid-body.
    ///
    /// # Parameters
    /// - `impulse`: the world-space impulse to apply on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    pub fn rbApplyImpulse(&mut self, handle: FlatHandle, impulse: &RawVector, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.apply_impulse(impulse.0, wakeUp);
        })
    }

    /// Adds a torque at the center-of-mass of this rigid-body.
    ///
    /// # Parameters
    /// - `torque`: the torque to apply on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    #[cfg(feature = "dim2")]
    pub fn rbAddTorque(&mut self, handle: FlatHandle, torque: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.add_torque(torque, wakeUp);
        })
    }

    /// Adds a torque at the center-of-mass of this rigid-body.
    ///
    /// # Parameters
    /// - `torque`: the world-space torque to apply on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    #[cfg(feature = "dim3")]
    pub fn rbAddTorque(&mut self, handle: FlatHandle, torque: &RawVector, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.add_torque(torque.0, wakeUp);
        })
    }

    /// Applies an impulsive torque at the center-of-mass of this rigid-body.
    ///
    /// # Parameters
    /// - `torque impulse`: the torque impulse to apply on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    #[cfg(feature = "dim2")]
    pub fn rbApplyTorqueImpulse(&mut self, handle: FlatHandle, torque_impulse: f32, wakeUp: bool) {
        self.map_mut(handle, |rb| {
            rb.apply_torque_impulse(torque_impulse, wakeUp);
        })
    }

    /// Applies an impulsive torque at the center-of-mass of this rigid-body.
    ///
    /// # Parameters
    /// - `torque impulse`: the world-space torque impulse to apply on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    #[cfg(feature = "dim3")]
    pub fn rbApplyTorqueImpulse(
        &mut self,
        handle: FlatHandle,
        torque_impulse: &RawVector,
        wakeUp: bool,
    ) {
        self.map_mut(handle, |rb| {
            rb.apply_torque_impulse(torque_impulse.0, wakeUp);
        })
    }

    /// Adds a force at the given world-space point of this rigid-body.
    ///
    /// # Parameters
    /// - `force`: the world-space force to apply on the rigid-body.
    /// - `point`: the world-space point where the impulse is to be applied on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    pub fn rbAddForceAtPoint(
        &mut self,
        handle: FlatHandle,
        force: &RawVector,
        point: &RawVector,
        wakeUp: bool,
    ) {
        self.map_mut(handle, |rb| {
            rb.add_force_at_point(force.0, point.0.into(), wakeUp);
        })
    }

    /// Applies an impulse at the given world-space point of this rigid-body.
    ///
    /// # Parameters
    /// - `impulse`: the world-space impulse to apply on the rigid-body.
    /// - `point`: the world-space point where the impulse is to be applied on the rigid-body.
    /// - `wakeUp`: should the rigid-body be automatically woken-up?
    pub fn rbApplyImpulseAtPoint(
        &mut self,
        handle: FlatHandle,
        impulse: &RawVector,
        point: &RawVector,
        wakeUp: bool,
    ) {
        self.map_mut(handle, |rb| {
            rb.apply_impulse_at_point(impulse.0, point.0.into(), wakeUp);
        })
    }

    pub fn rbAdditionalSolverIterations(&self, handle: FlatHandle) -> usize {
        self.map(handle, |rb| rb.additional_solver_iterations())
    }

    pub fn rbSetAdditionalSolverIterations(&mut self, handle: FlatHandle, iters: usize) {
        self.map_mut(handle, |rb| {
            rb.set_additional_solver_iterations(iters as usize);
        })
    }

    /// An arbitrary user-defined 32-bit integer
    pub fn rbUserData(&self, handle: FlatHandle) -> u32 {
        self.map(handle, |rb| rb.user_data as u32)
    }

    /// Sets the user-defined 32-bit integer of this rigid-body.
    ///
    /// # Parameters
    /// - `data`: an arbitrary user-defined 32-bit integer.
    pub fn rbSetUserData(&mut self, handle: FlatHandle, data: u32) {
        self.map_mut(handle, |rb| {
            rb.user_data = data as u128;
        })
    }

    /// Retrieves the constant force(s) the user added to this rigid-body.
    /// Returns zero if the rigid-body is not dynamic.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim2")]
    pub fn rbUserForce(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.user_force();
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
        });
    }

    /// Retrieves the constant force(s) the user added to this rigid-body.
    /// Returns zero if the rigid-body is not dynamic.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbUserForce(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.user_force();
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        });
    }

    /// Retrieves the constant torque(s) the user added to this rigid-body.
    /// Returns zero if the rigid-body is not dynamic.
    #[cfg(feature = "dim2")]
    pub fn rbUserTorque(&self, handle: FlatHandle) -> f32 {
        self.map(handle, |rb| rb.user_torque())
    }

    /// Retrieves the constant torque(s) the user added to this rigid-body.
    /// Returns zero if the rigid-body is not dynamic.
    ///
    /// # Parameters
    /// - `scratch_buffer`: The array to be populated.
    #[cfg(feature = "dim3")]
    pub fn rbUserTorque(&self, handle: FlatHandle, scratch_buffer: &js_sys::Float32Array) {
        self.map(handle, |rb| {
            let u = rb.user_torque();
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        });
    }
}
