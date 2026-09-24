use crate::geometry::feature::IntoTypeValue;
use crate::geometry::RawFeatureType;
use crate::utils::{self, FlatHandle};
use rapier::geometry::{ColliderHandle, RayIntersection};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawRayIntersection(pub(crate) RayIntersection);

#[wasm_bindgen]
impl RawRayIntersection {
    /// Writes the hit normal components into the given scratch buffer.
    #[cfg(feature = "dim2")]
    pub fn normal(&self, scratch_buffer: &js_sys::Float32Array) {
        let u = self.0.normal;
        scratch_buffer.set_index(0, u.x);
        scratch_buffer.set_index(1, u.y);
    }

    /// Writes the hit normal components into the given scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn normal(&self, scratch_buffer: &js_sys::Float32Array) {
        let u = self.0.normal;
        scratch_buffer.set_index(0, u.x);
        scratch_buffer.set_index(1, u.y);
        scratch_buffer.set_index(2, u.z);
    }

    pub fn time_of_impact(&self) -> f32 {
        self.0.time_of_impact
    }

    pub fn featureType(&self) -> RawFeatureType {
        self.0.feature.into_type()
    }

    pub fn featureId(&self) -> Option<u32> {
        self.0.feature.into_value()
    }
}

#[wasm_bindgen]
pub struct RawRayColliderIntersection {
    pub(crate) handle: ColliderHandle,
    pub(crate) inter: RayIntersection,
}

#[wasm_bindgen]
impl RawRayColliderIntersection {
    pub fn colliderHandle(&self) -> FlatHandle {
        utils::flat_handle(self.handle.0)
    }

    /// Writes the hit normal components into the given scratch buffer.
    #[cfg(feature = "dim2")]
    pub fn normal(&self, scratch_buffer: &js_sys::Float32Array) {
        let u = self.inter.normal;
        scratch_buffer.set_index(0, u.x);
        scratch_buffer.set_index(1, u.y);
    }

    /// Writes the hit normal components into the given scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn normal(&self, scratch_buffer: &js_sys::Float32Array) {
        let u = self.inter.normal;
        scratch_buffer.set_index(0, u.x);
        scratch_buffer.set_index(1, u.y);
        scratch_buffer.set_index(2, u.z);
    }

    pub fn time_of_impact(&self) -> f32 {
        self.inter.time_of_impact
    }

    pub fn featureType(&self) -> RawFeatureType {
        self.inter.feature.into_type()
    }

    pub fn featureId(&self) -> Option<u32> {
        self.inter.feature.into_value()
    }
}

#[wasm_bindgen]
pub struct RawRayColliderHit {
    pub(crate) handle: ColliderHandle,
    pub(crate) timeOfImpact: f32,
}

#[wasm_bindgen]
impl RawRayColliderHit {
    pub fn colliderHandle(&self) -> FlatHandle {
        utils::flat_handle(self.handle.0)
    }

    pub fn timeOfImpact(&self) -> f32 {
        self.timeOfImpact
    }
}
