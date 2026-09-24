use crate::dynamics::RawGenericJoint;
use crate::utils::{self, FlatHandle};
use rapier::dynamics::{ImpulseJoint, ImpulseJointSet};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawImpulseJointSet(pub(crate) ImpulseJointSet);

impl RawImpulseJointSet {
    pub(crate) fn map<T>(&self, handle: FlatHandle, f: impl FnOnce(&ImpulseJoint) -> T) -> T {
        let body = self.0.get(utils::impulse_joint_handle(handle)).expect(
            "Invalid ImpulseJoint reference. It may have been removed from the physics World.",
        );
        f(body)
    }

    pub(crate) fn map_mut<T>(
        &mut self,
        handle: FlatHandle,
        f: impl FnOnce(&mut ImpulseJoint) -> T,
    ) -> T {
        let body = self
            .0
            .get_mut(utils::impulse_joint_handle(handle), true)
            .expect(
                "Invalid ImpulseJoint reference. It may have been removed from the physics World.",
            );
        f(body)
    }
}

#[wasm_bindgen]
impl RawImpulseJointSet {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawImpulseJointSet(ImpulseJointSet::new())
    }

    pub fn createJoint(
        &mut self,
        params: &RawGenericJoint,
        parent1: FlatHandle,
        parent2: FlatHandle,
        wake_up: bool,
    ) -> FlatHandle {
        utils::flat_handle(
            self.0
                .insert(
                    utils::body_handle(parent1),
                    utils::body_handle(parent2),
                    params.0.clone(),
                    wake_up,
                )
                .0,
        )
    }

    pub fn remove(&mut self, handle: FlatHandle, wakeUp: bool) {
        let handle = utils::impulse_joint_handle(handle);
        self.0.remove(handle, wakeUp);
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn contains(&self, handle: FlatHandle) -> bool {
        self.0.get(utils::impulse_joint_handle(handle)).is_some()
    }

    /// Applies the given JavaScript function to the integer handle of each joint managed by this physics world.
    ///
    /// # Parameters
    /// - `f(handle)`: the function to apply to the integer handle of each joint managed by this set. Called as `f(collider)`.
    pub fn forEachJointHandle(&self, f: &js_sys::Function) {
        let this = JsValue::null();
        for (handle, _) in self.0.iter() {
            let _ = f.call1(&this, &JsValue::from(utils::flat_handle(handle.0)));
        }
    }

    /// Applies the given JavaScript function to the integer handle of each joint attached to the given rigid-body.
    ///
    /// # Parameters
    /// - `f(handle)`: the function to apply to the integer handle of each joint attached to the rigid-body. Called as `f(collider)`.
    pub fn forEachJointAttachedToRigidBody(&self, body: FlatHandle, f: &js_sys::Function) {
        let this = JsValue::null();
        for (_, _, handle, _) in self.0.attached_joints(utils::body_handle(body)) {
            let _ = f.call1(&this, &JsValue::from(utils::flat_handle(handle.0)));
        }
    }
}
