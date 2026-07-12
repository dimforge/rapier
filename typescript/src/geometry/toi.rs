use crate::utils::{self, FlatHandle};
use rapier::geometry::{ColliderHandle, ShapeCastHit};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawShapeCastHit {
    pub(crate) hit: ShapeCastHit,
}

#[wasm_bindgen]
impl RawShapeCastHit {
    #[cfg(feature = "dim2")]
    pub fn getComponents(&self, scratch_buffer: &js_sys::Float32Array) {
        scratch_buffer.set_index(0, self.hit.time_of_impact);
        let mut u = self.hit.witness1;
        scratch_buffer.set_index(1, u.x);
        scratch_buffer.set_index(2, u.y);
        u = self.hit.witness2;
        scratch_buffer.set_index(3, u.x);
        scratch_buffer.set_index(4, u.y);
        u = self.hit.normal1;
        scratch_buffer.set_index(5, u.x);
        scratch_buffer.set_index(6, u.y);
        u = self.hit.normal2;
        scratch_buffer.set_index(7, u.x);
        scratch_buffer.set_index(8, u.y);
    }
    #[cfg(feature = "dim3")]
    pub fn getComponents(&self, scratch_buffer: &js_sys::Float32Array) {
        scratch_buffer.set_index(0, self.hit.time_of_impact);
        let mut u = self.hit.witness1;
        scratch_buffer.set_index(1, u.x);
        scratch_buffer.set_index(2, u.y);
        scratch_buffer.set_index(3, u.z);
        u = self.hit.witness2;
        scratch_buffer.set_index(4, u.x);
        scratch_buffer.set_index(5, u.y);
        scratch_buffer.set_index(6, u.z);
        u = self.hit.normal1;
        scratch_buffer.set_index(7, u.x);
        scratch_buffer.set_index(8, u.y);
        scratch_buffer.set_index(9, u.z);
        u = self.hit.normal2;
        scratch_buffer.set_index(10, u.x);
        scratch_buffer.set_index(11, u.y);
        scratch_buffer.set_index(12, u.z);
    }
}

#[wasm_bindgen]
pub struct RawColliderShapeCastHit {
    pub(crate) handle: ColliderHandle,
    pub(crate) hit: ShapeCastHit,
}

#[wasm_bindgen]
impl RawColliderShapeCastHit {
    pub fn colliderHandle(&self) -> FlatHandle {
        utils::flat_handle(self.handle.0)
    }
    #[cfg(feature = "dim2")]
    pub fn getComponents(&self, scratch_buffer: &js_sys::Float32Array) {
        scratch_buffer.set_index(0, self.hit.time_of_impact);
        let mut u = self.hit.witness1;
        scratch_buffer.set_index(1, u.x);
        scratch_buffer.set_index(2, u.y);
        u = self.hit.witness2;
        scratch_buffer.set_index(3, u.x);
        scratch_buffer.set_index(4, u.y);
        u = self.hit.normal1;
        scratch_buffer.set_index(5, u.x);
        scratch_buffer.set_index(6, u.y);
        u = self.hit.normal2;
        scratch_buffer.set_index(7, u.x);
        scratch_buffer.set_index(8, u.y);
    }
    #[cfg(feature = "dim3")]
    pub fn getComponents(&self, scratch_buffer: &js_sys::Float32Array) {
        scratch_buffer.set_index(0, self.hit.time_of_impact);
        let mut u = self.hit.witness1;
        scratch_buffer.set_index(1, u.x);
        scratch_buffer.set_index(2, u.y);
        scratch_buffer.set_index(3, u.z);
        u = self.hit.witness2;
        scratch_buffer.set_index(4, u.x);
        scratch_buffer.set_index(5, u.y);
        scratch_buffer.set_index(6, u.z);
        u = self.hit.normal1;
        scratch_buffer.set_index(7, u.x);
        scratch_buffer.set_index(8, u.y);
        scratch_buffer.set_index(9, u.z);
        u = self.hit.normal2;
        scratch_buffer.set_index(10, u.x);
        scratch_buffer.set_index(11, u.y);
        scratch_buffer.set_index(12, u.z);
    }
}
