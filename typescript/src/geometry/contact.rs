use rapier::parry::query;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawShapeContact {
    pub(crate) contact: query::Contact,
}

#[wasm_bindgen]
impl RawShapeContact {
    /// Writes the contact components into the given scratch buffer.
    ///
    /// Layout: `[distance, point1, point2, normal1, normal2]`.
    #[cfg(feature = "dim2")]
    pub fn getComponents(&self, scratch_buffer: &js_sys::Float32Array) {
        scratch_buffer.set_index(0, self.contact.dist);
        let p1 = self.contact.point1.coords;
        scratch_buffer.set_index(1, p1.x);
        scratch_buffer.set_index(2, p1.y);
        let p2 = self.contact.point2.coords;
        scratch_buffer.set_index(3, p2.x);
        scratch_buffer.set_index(4, p2.y);
        let n1 = self.contact.normal1.into_inner();
        scratch_buffer.set_index(5, n1.x);
        scratch_buffer.set_index(6, n1.y);
        let n2 = self.contact.normal2.into_inner();
        scratch_buffer.set_index(7, n2.x);
        scratch_buffer.set_index(8, n2.y);
    }

    /// Writes the contact components into the given scratch buffer.
    ///
    /// Layout: `[distance, point1, point2, normal1, normal2]`.
    #[cfg(feature = "dim3")]
    pub fn getComponents(&self, scratch_buffer: &js_sys::Float32Array) {
        scratch_buffer.set_index(0, self.contact.dist);
        let p1 = self.contact.point1.coords;
        scratch_buffer.set_index(1, p1.x);
        scratch_buffer.set_index(2, p1.y);
        scratch_buffer.set_index(3, p1.z);
        let p2 = self.contact.point2.coords;
        scratch_buffer.set_index(4, p2.x);
        scratch_buffer.set_index(5, p2.y);
        scratch_buffer.set_index(6, p2.z);
        let n1 = self.contact.normal1.into_inner();
        scratch_buffer.set_index(7, n1.x);
        scratch_buffer.set_index(8, n1.y);
        scratch_buffer.set_index(9, n1.z);
        let n2 = self.contact.normal2.into_inner();
        scratch_buffer.set_index(10, n2.x);
        scratch_buffer.set_index(11, n2.y);
        scratch_buffer.set_index(12, n2.z);
    }
}
