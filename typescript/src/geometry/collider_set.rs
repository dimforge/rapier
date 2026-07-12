use crate::dynamics::{RawIslandManager, RawRigidBodySet};
use crate::geometry::RawShape;
use crate::math::{RawRotation, RawVector};
use crate::utils::{self, FlatHandle};
use rapier::prelude::*;
use wasm_bindgen::prelude::*;

// NOTE: this MUST match the same enum on the TS side.
enum MassPropsMode {
    Density = 0,
    Mass,
    MassProps,
}

#[wasm_bindgen]
pub struct RawColliderSet(pub(crate) ColliderSet);

impl RawColliderSet {
    pub(crate) fn map<T>(&self, handle: FlatHandle, f: impl FnOnce(&Collider) -> T) -> T {
        let collider = self
            .0
            .get(utils::collider_handle(handle))
            .expect("Invalid Collider reference. It may have been removed from the physics World.");
        f(collider)
    }

    pub(crate) fn map_mut<T>(
        &mut self,
        handle: FlatHandle,
        f: impl FnOnce(&mut Collider) -> T,
    ) -> T {
        let collider = self
            .0
            .get_mut(utils::collider_handle(handle))
            .expect("Invalid Collider reference. It may have been removed from the physics World.");
        f(collider)
    }

    pub(crate) fn map_pair_mut<T>(
        &mut self,
        handle1: FlatHandle,
        handle2: FlatHandle,
        f: impl FnOnce(Option<&mut Collider>, Option<&mut Collider>) -> T,
    ) -> T {
        let (collider1, collider2) = self.0.get_pair_mut(
            utils::collider_handle(handle1),
            utils::collider_handle(handle2),
        );
        f(collider1, collider2)
    }
}

impl RawColliderSet {
    // This is a workaround because wasm-bindgen doesn't support the `cfg(feature = ...)`
    // for the method arguments.
    pub fn do_create_collider(
        &mut self,
        enabled: bool,
        shape: &RawShape,
        translation: &RawVector,
        rotation: &RawRotation,
        massPropsMode: u32,
        mass: f32,
        centerOfMass: &RawVector,
        #[cfg(feature = "dim2")] principalAngularInertia: f32,
        #[cfg(feature = "dim3")] principalAngularInertia: &RawVector,
        #[cfg(feature = "dim3")] angularInertiaFrame: &RawRotation,
        density: f32,
        friction: f32,
        restitution: f32,
        frictionCombineRule: u32,
        restitutionCombineRule: u32,
        isSensor: bool,
        collisionGroups: u32,
        solverGroups: u32,
        activeCollisionTypes: u16,
        activeHooks: u32,
        activeEvents: u32,
        contactForceEventThreshold: f32,
        contactSkin: f32,
        hasParent: bool,
        parent: FlatHandle,
        bodies: &mut RawRigidBodySet,
    ) -> Option<FlatHandle> {
        let pos = Isometry::from_parts(translation.0.into(), rotation.0);
        let mut builder = ColliderBuilder::new(shape.0.clone())
            .enabled(enabled)
            .position(pos)
            .friction(friction)
            .restitution(restitution)
            .collision_groups(super::unpack_interaction_groups(collisionGroups))
            .solver_groups(super::unpack_interaction_groups(solverGroups))
            .active_hooks(ActiveHooks::from_bits(activeHooks).unwrap_or(ActiveHooks::empty()))
            .active_events(ActiveEvents::from_bits(activeEvents).unwrap_or(ActiveEvents::empty()))
            .active_collision_types(
                ActiveCollisionTypes::from_bits(activeCollisionTypes)
                    .unwrap_or(ActiveCollisionTypes::empty()),
            )
            .sensor(isSensor)
            .friction_combine_rule(super::combine_rule_from_u32(frictionCombineRule))
            .restitution_combine_rule(super::combine_rule_from_u32(restitutionCombineRule))
            .contact_force_event_threshold(contactForceEventThreshold)
            .contact_skin(contactSkin);

        if massPropsMode == MassPropsMode::MassProps as u32 {
            #[cfg(feature = "dim2")]
            let mprops = MassProperties::new(centerOfMass.0.into(), mass, principalAngularInertia);
            #[cfg(feature = "dim3")]
            let mprops = MassProperties::with_principal_inertia_frame(
                centerOfMass.0.into(),
                mass,
                principalAngularInertia.0,
                angularInertiaFrame.0,
            );
            builder = builder.mass_properties(mprops);
        } else if massPropsMode == MassPropsMode::Density as u32 {
            builder = builder.density(density);
        } else {
            assert_eq!(massPropsMode, MassPropsMode::Mass as u32);
            builder = builder.mass(mass);
        };

        let collider = builder.build();

        if hasParent {
            Some(utils::flat_handle(
                self.0
                    .insert_with_parent(collider, utils::body_handle(parent), &mut bodies.0)
                    .0,
            ))
        } else {
            Some(utils::flat_handle(self.0.insert(collider).0))
        }
    }
}

#[wasm_bindgen]
impl RawColliderSet {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawColliderSet(ColliderSet::new())
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn contains(&self, handle: FlatHandle) -> bool {
        self.0.get(utils::collider_handle(handle)).is_some()
    }

    #[cfg(feature = "dim2")]
    pub fn createCollider(
        &mut self,
        enabled: bool,
        shape: &RawShape,
        translation: &RawVector,
        rotation: &RawRotation,
        massPropsMode: u32,
        mass: f32,
        centerOfMass: &RawVector,
        principalAngularInertia: f32,
        density: f32,
        friction: f32,
        restitution: f32,
        frictionCombineRule: u32,
        restitutionCombineRule: u32,
        isSensor: bool,
        collisionGroups: u32,
        solverGroups: u32,
        activeCollisionTypes: u16,
        activeHooks: u32,
        activeEvents: u32,
        contactForceEventThreshold: f32,
        contactSkin: f32,
        hasParent: bool,
        parent: FlatHandle,
        bodies: &mut RawRigidBodySet,
    ) -> Option<FlatHandle> {
        self.do_create_collider(
            enabled,
            shape,
            translation,
            rotation,
            massPropsMode,
            mass,
            centerOfMass,
            principalAngularInertia,
            density,
            friction,
            restitution,
            frictionCombineRule,
            restitutionCombineRule,
            isSensor,
            collisionGroups,
            solverGroups,
            activeCollisionTypes,
            activeHooks,
            activeEvents,
            contactForceEventThreshold,
            contactSkin,
            hasParent,
            parent,
            bodies,
        )
    }

    #[cfg(feature = "dim3")]
    pub fn createCollider(
        &mut self,
        enabled: bool,
        shape: &RawShape,
        translation: &RawVector,
        rotation: &RawRotation,
        massPropsMode: u32,
        mass: f32,
        centerOfMass: &RawVector,
        principalAngularInertia: &RawVector,
        angularInertiaFrame: &RawRotation,
        density: f32,
        friction: f32,
        restitution: f32,
        frictionCombineRule: u32,
        restitutionCombineRule: u32,
        isSensor: bool,
        collisionGroups: u32,
        solverGroups: u32,
        activeCollisionTypes: u16,
        activeHooks: u32,
        activeEvents: u32,
        contactForceEventThreshold: f32,
        contactSkin: f32,
        hasParent: bool,
        parent: FlatHandle,
        bodies: &mut RawRigidBodySet,
    ) -> Option<FlatHandle> {
        self.do_create_collider(
            enabled,
            shape,
            translation,
            rotation,
            massPropsMode,
            mass,
            centerOfMass,
            principalAngularInertia,
            angularInertiaFrame,
            density,
            friction,
            restitution,
            frictionCombineRule,
            restitutionCombineRule,
            isSensor,
            collisionGroups,
            solverGroups,
            activeCollisionTypes,
            activeHooks,
            activeEvents,
            contactForceEventThreshold,
            contactSkin,
            hasParent,
            parent,
            bodies,
        )
    }

    /// Removes a collider from this set and wake-up the rigid-body it is attached to.
    pub fn remove(
        &mut self,
        handle: FlatHandle,
        islands: &mut RawIslandManager,
        bodies: &mut RawRigidBodySet,
        wakeUp: bool,
    ) {
        let handle = utils::collider_handle(handle);
        self.0.remove(handle, &mut islands.0, &mut bodies.0, wakeUp);
    }

    /// Checks if a collider with the given integer handle exists.
    pub fn isHandleValid(&self, handle: FlatHandle) -> bool {
        self.0.get(utils::collider_handle(handle)).is_some()
    }

    /// Applies the given JavaScript function to the integer handle of each collider managed by this collider set.
    ///
    /// # Parameters
    /// - `f(handle)`: the function to apply to the integer handle of each collider managed by this collider set. Called as `f(handle)`.
    pub fn forEachColliderHandle(&self, f: &js_sys::Function) {
        let this = JsValue::null();
        for (handle, _) in self.0.iter() {
            let _ = f.call1(&this, &JsValue::from(utils::flat_handle(handle.0)));
        }
    }
}
