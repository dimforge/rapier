use crate::geometry::feature::IntoTypeValue;
use crate::geometry::RawFeatureType;
use crate::utils::{self, FlatHandle};
use rapier::{
    geometry::{ColliderHandle, PointProjection},
    prelude::FeatureId,
};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawPointProjection(pub(crate) PointProjection);

#[wasm_bindgen]
impl RawPointProjection {
    /// Writes the projected point components into the given scratch buffer.
    #[cfg(feature = "dim2")]
    pub fn point(&self, scratch_buffer: &js_sys::Float32Array) {
        let u = self.0.point.coords;
        scratch_buffer.set_index(0, u.x);
        scratch_buffer.set_index(1, u.y);
    }

    /// Writes the projected point components into the given scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn point(&self, scratch_buffer: &js_sys::Float32Array) {
        let u = self.0.point.coords;
        scratch_buffer.set_index(0, u.x);
        scratch_buffer.set_index(1, u.y);
        scratch_buffer.set_index(2, u.z);
    }

    pub fn isInside(&self) -> bool {
        self.0.is_inside
    }
}

#[wasm_bindgen]
pub struct RawPointColliderProjection {
    pub(crate) handle: ColliderHandle,
    pub(crate) proj: PointProjection,
    pub(crate) feature: FeatureId,
}

#[wasm_bindgen]
impl RawPointColliderProjection {
    pub fn colliderHandle(&self) -> FlatHandle {
        utils::flat_handle(self.handle.0)
    }

    /// Writes the projected point components into the given scratch buffer.
    #[cfg(feature = "dim2")]
    pub fn point(&self, scratch_buffer: &js_sys::Float32Array) {
        let u = self.proj.point.coords;
        scratch_buffer.set_index(0, u.x);
        scratch_buffer.set_index(1, u.y);
    }

    /// Writes the projected point components into the given scratch buffer.
    #[cfg(feature = "dim3")]
    pub fn point(&self, scratch_buffer: &js_sys::Float32Array) {
        let u = self.proj.point.coords;
        scratch_buffer.set_index(0, u.x);
        scratch_buffer.set_index(1, u.y);
        scratch_buffer.set_index(2, u.z);
    }

    pub fn isInside(&self) -> bool {
        self.proj.is_inside
    }

    pub fn featureType(&self) -> RawFeatureType {
        self.feature.into_type()
    }

    pub fn featureId(&self) -> Option<u32> {
        self.feature.into_value()
    }
}
