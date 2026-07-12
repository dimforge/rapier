use crate::dynamics::{RawImpulseJointSet, RawIslandManager, RawMultibodyJointSet};
use crate::geometry::RawColliderSet;
use crate::math::{RawRotation, RawVector};
use crate::utils::{self, FlatHandle};
use rapier::dynamics::{MassProperties, RigidBody, RigidBodyBuilder, RigidBodySet, RigidBodyType};
use rapier::math::Pose;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub enum RawRigidBodyType {
    Dynamic,
    Fixed,
    KinematicPositionBased,
    KinematicVelocityBased,
}

impl Into<RigidBodyType> for RawRigidBodyType {
    fn into(self) -> RigidBodyType {
        match self {
            RawRigidBodyType::Dynamic => RigidBodyType::Dynamic,
            RawRigidBodyType::Fixed => RigidBodyType::Fixed,
            RawRigidBodyType::KinematicPositionBased => RigidBodyType::KinematicPositionBased,
            RawRigidBodyType::KinematicVelocityBased => RigidBodyType::KinematicVelocityBased,
        }
    }
}

impl Into<RawRigidBodyType> for RigidBodyType {
    fn into(self) -> RawRigidBodyType {
        match self {
            RigidBodyType::Dynamic => RawRigidBodyType::Dynamic,
            RigidBodyType::Fixed => RawRigidBodyType::Fixed,
            RigidBodyType::KinematicPositionBased => RawRigidBodyType::KinematicPositionBased,
            RigidBodyType::KinematicVelocityBased => RawRigidBodyType::KinematicVelocityBased,
        }
    }
}

#[wasm_bindgen]
pub struct RawRigidBodySet(pub(crate) RigidBodySet);

impl RawRigidBodySet {
    pub(crate) fn map<T>(&self, handle: FlatHandle, f: impl FnOnce(&RigidBody) -> T) -> T {
        let body = self.0.get(utils::body_handle(handle)).expect(
            "Invalid RigidBody reference. It may have been removed from the physics World.",
        );
        f(body)
    }

    pub(crate) fn map_mut<T>(
        &mut self,
        handle: FlatHandle,
        f: impl FnOnce(&mut RigidBody) -> T,
    ) -> T {
        let body = self.0.get_mut(utils::body_handle(handle)).expect(
            "Invalid RigidBody reference. It may have been removed from the physics World.",
        );
        f(body)
    }
}

#[wasm_bindgen]
impl RawRigidBodySet {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawRigidBodySet(RigidBodySet::new())
    }

    #[cfg(feature = "dim3")]
    pub fn createRigidBody(
        &mut self,
        enabled: bool,
        translation: &RawVector,
        rotation: &RawRotation,
        gravityScale: f32,
        mass: f32,
        massOnly: bool,
        centerOfMass: &RawVector,
        linvel: &RawVector,
        angvel: &RawVector,
        principalAngularInertia: &RawVector,
        angularInertiaFrame: &RawRotation,
        translationEnabledX: bool,
        translationEnabledY: bool,
        translationEnabledZ: bool,
        rotationEnabledX: bool,
        rotationEnabledY: bool,
        rotationEnabledZ: bool,
        linearDamping: f32,
        angularDamping: f32,
        rb_type: RawRigidBodyType,
        canSleep: bool,
        sleeping: bool,
        softCcdPrediction: f32,
        ccdEnabled: bool,
        dominanceGroup: i8,
        additional_solver_iterations: usize,
    ) -> FlatHandle {
        let pos = Pose::from_parts(translation.0, rotation.0);

        let mut rigid_body = RigidBodyBuilder::new(rb_type.into())
            .enabled(enabled)
            .pose(pos)
            .gravity_scale(gravityScale)
            .enabled_translations(
                translationEnabledX,
                translationEnabledY,
                translationEnabledZ,
            )
            .enabled_rotations(rotationEnabledX, rotationEnabledY, rotationEnabledZ)
            .linvel(linvel.0)
            .angvel(angvel.0)
            .linear_damping(linearDamping)
            .angular_damping(angularDamping)
            .can_sleep(canSleep)
            .sleeping(sleeping)
            .ccd_enabled(ccdEnabled)
            .dominance_group(dominanceGroup)
            .additional_solver_iterations(additional_solver_iterations)
            .soft_ccd_prediction(softCcdPrediction);

        rigid_body = if massOnly {
            rigid_body.additional_mass(mass)
        } else {
            let props = MassProperties::with_principal_inertia_frame(
                centerOfMass.0.into(),
                mass,
                principalAngularInertia.0,
                angularInertiaFrame.0,
            );
            rigid_body.additional_mass_properties(props)
        };

        utils::flat_handle(self.0.insert(rigid_body.build()).0)
    }

    #[cfg(feature = "dim2")]
    pub fn createRigidBody(
        &mut self,
        enabled: bool,
        translation: &RawVector,
        rotation: &RawRotation,
        gravityScale: f32,
        mass: f32,
        massOnly: bool,
        centerOfMass: &RawVector,
        linvel: &RawVector,
        angvel: f32,
        principalAngularInertia: f32,
        translationEnabledX: bool,
        translationEnabledY: bool,
        rotationsEnabled: bool,
        linearDamping: f32,
        angularDamping: f32,
        rb_type: RawRigidBodyType,
        canSleep: bool,
        sleeping: bool,
        softCcdPrediciton: f32,
        ccdEnabled: bool,
        dominanceGroup: i8,
        additional_solver_iterations: usize,
    ) -> FlatHandle {
        let pos = Pose::from_parts(translation.0, rotation.0);
        let mut rigid_body = RigidBodyBuilder::new(rb_type.into())
            .enabled(enabled)
            .pose(pos)
            .gravity_scale(gravityScale)
            .enabled_translations(translationEnabledX, translationEnabledY)
            .linvel(linvel.0)
            .angvel(angvel)
            .linear_damping(linearDamping)
            .angular_damping(angularDamping)
            .can_sleep(canSleep)
            .sleeping(sleeping)
            .ccd_enabled(ccdEnabled)
            .dominance_group(dominanceGroup)
            .additional_solver_iterations(additional_solver_iterations)
            .soft_ccd_prediction(softCcdPrediciton);

        rigid_body = if massOnly {
            rigid_body.additional_mass(mass)
        } else {
            let props = MassProperties::new(centerOfMass.0.into(), mass, principalAngularInertia);
            rigid_body.additional_mass_properties(props)
        };

        if !rotationsEnabled {
            rigid_body = rigid_body.lock_rotations();
        }

        utils::flat_handle(self.0.insert(rigid_body.build()).0)
    }

    pub fn remove(
        &mut self,
        handle: FlatHandle,
        islands: &mut RawIslandManager,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
    ) {
        let handle = utils::body_handle(handle);
        self.0.remove(
            handle,
            &mut islands.0,
            &mut colliders.0,
            &mut joints.0,
            &mut articulations.0,
            true,
        );
    }

    /// The number of rigid-bodies on this set.
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Checks if a rigid-body with the given integer handle exists.
    pub fn contains(&self, handle: FlatHandle) -> bool {
        self.0.get(utils::body_handle(handle)).is_some()
    }

    /// Applies the given JavaScript function to the integer handle of each rigid-body managed by this set.
    ///
    /// # Parameters
    /// - `f(handle)`: the function to apply to the integer handle of each rigid-body managed by this set. Called as `f(collider)`.
    pub fn forEachRigidBodyHandle(&self, f: &js_sys::Function) {
        let this = JsValue::null();
        for (handle, _) in self.0.iter() {
            let _ = f.call1(&this, &JsValue::from(utils::flat_handle(handle.0)));
        }
    }

    pub fn propagateModifiedBodyPositionsToColliders(&mut self, colliders: &mut RawColliderSet) {
        self.0
            .propagate_modified_body_positions_to_colliders(&mut colliders.0);
    }
}
