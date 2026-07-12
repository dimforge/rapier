use crate::utils;
use rapier::dynamics::IslandManager;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawIslandManager(pub(crate) IslandManager);

#[wasm_bindgen]
impl RawIslandManager {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawIslandManager(IslandManager::new())
    }

    /// Applies the given JavaScript function to the integer handle of each active rigid-body
    /// managed by this island manager.
    ///
    /// After a short time of inactivity, a rigid-body is automatically deactivated ("asleep") by
    /// the physics engine in order to save computational power. A sleeping rigid-body never moves
    /// unless it is moved manually by the user.
    ///
    /// # Parameters
    /// - `f(handle)`: the function to apply to the integer handle of each active rigid-body managed by this
    ///   set. Called as `f(collider)`.
    pub fn forEachActiveRigidBodyHandle(&self, f: &js_sys::Function) {
        let this = JsValue::null();
        for handle in self.0.active_bodies() {
            let _ = f.call1(&this, &JsValue::from(utils::flat_handle(handle.0)));
        }
    }
}
