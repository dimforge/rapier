use crate::utils::{self, FlatHandle};
use rapier::geometry::{ContactManifold, ContactPair, NarrowPhase};
use rapier::math::Real;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct RawNarrowPhase(pub(crate) NarrowPhase);

#[wasm_bindgen]
impl RawNarrowPhase {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        RawNarrowPhase(NarrowPhase::new())
    }

    pub fn contact_pairs_with(&self, handle1: FlatHandle, f: js_sys::Function) {
        let this = JsValue::null();
        let handle1 = utils::collider_handle(handle1);
        for pair in self.0.contact_pairs_with(handle1) {
            let handle2 = if pair.collider1 == handle1 {
                utils::flat_handle(pair.collider2.0)
            } else {
                utils::flat_handle(pair.collider1.0)
            };

            let _ = f.call1(&this, &JsValue::from(handle2));
        }
    }

    pub fn contact_pair(&self, handle1: FlatHandle, handle2: FlatHandle) -> Option<RawContactPair> {
        let handle1 = utils::collider_handle(handle1);
        let handle2 = utils::collider_handle(handle2);
        self.0
            .contact_pair(handle1, handle2)
            .map(|p| RawContactPair(p as *const ContactPair))
    }

    pub fn intersection_pairs_with(&self, handle1: FlatHandle, f: js_sys::Function) {
        let this = JsValue::null();
        let handle1 = utils::collider_handle(handle1);
        for (h1, h2, inter) in self.0.intersection_pairs_with(handle1) {
            if inter {
                let handle2 = if h1 == handle1 {
                    utils::flat_handle(h2.0)
                } else {
                    utils::flat_handle(h1.0)
                };

                let _ = f.call1(&this, &JsValue::from(handle2));
            }
        }
    }

    pub fn intersection_pair(&self, handle1: FlatHandle, handle2: FlatHandle) -> bool {
        let handle1 = utils::collider_handle(handle1);
        let handle2 = utils::collider_handle(handle2);
        self.0.intersection_pair(handle1, handle2) == Some(true)
    }
}

#[wasm_bindgen]
pub struct RawContactPair(*const ContactPair);
#[wasm_bindgen]
pub struct RawContactManifold(*const ContactManifold);

// SAFETY: the use of a raw pointer is very unsafe.
//         We need this because wasm-bindgen doesn't support
//         lifetimes. So for the moment, we have to make sure
//         that our TypeScript wrapper properly free the pair
//         before the user has a chance to invalidate this pointer.
#[wasm_bindgen]
impl RawContactPair {
    pub fn collider1(&self) -> FlatHandle {
        unsafe { utils::flat_handle((*self.0).collider1.0) }
    }

    pub fn collider2(&self) -> FlatHandle {
        unsafe { utils::flat_handle((*self.0).collider2.0) }
    }

    pub fn numContactManifolds(&self) -> usize {
        unsafe { (*self.0).manifolds.len() }
    }
    pub fn contactManifold(&self, i: usize) -> Option<RawContactManifold> {
        unsafe {
            (&(*self.0).manifolds)
                .get(i)
                .map(|m| RawContactManifold(m as *const ContactManifold))
        }
    }
}

#[wasm_bindgen]
impl RawContactManifold {
    #[cfg(feature = "dim2")]
    pub fn normal(&self, scratch_buffer: &js_sys::Float32Array) {
        unsafe {
            let u = (*self.0).data.normal;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
        }
    }

    #[cfg(feature = "dim3")]
    pub fn normal(&self, scratch_buffer: &js_sys::Float32Array) {
        unsafe {
            let u = (*self.0).data.normal;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        }
    }

    // pub fn user_data(&self) -> u32 {
    //     unsafe { (*self.0).data.user_data }
    // }

    #[cfg(feature = "dim2")]
    pub fn local_n1(&self, scratch_buffer: &js_sys::Float32Array) {
        unsafe {
            let u = (*self.0).local_n1;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
        }
    }

    #[cfg(feature = "dim3")]
    pub fn local_n1(&self, scratch_buffer: &js_sys::Float32Array) {
        unsafe {
            let u = (*self.0).local_n1;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        }
    }

    #[cfg(feature = "dim2")]
    pub fn local_n2(&self, scratch_buffer: &js_sys::Float32Array) {
        unsafe {
            let u = (*self.0).local_n2;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
        }
    }

    #[cfg(feature = "dim3")]
    pub fn local_n2(&self, scratch_buffer: &js_sys::Float32Array) {
        unsafe {
            let u = (*self.0).local_n2;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        }
    }

    pub fn subshape1(&self) -> u32 {
        unsafe { (*self.0).subshape1 }
    }

    pub fn subshape2(&self) -> u32 {
        unsafe { (*self.0).subshape2 }
    }

    pub fn num_contacts(&self) -> usize {
        unsafe { (*self.0).points.len() }
    }

    #[cfg(feature = "dim2")]
    pub fn contact_local_p1(&self, i: usize, scratch_buffer: &js_sys::Float32Array) -> bool {
        unsafe {
            (&(*self.0).points).get(i).map_or(false, |c| {
                let u = c.local_p1.coords;
                scratch_buffer.set_index(0, u.x);
                scratch_buffer.set_index(1, u.y);
                true
            })
        }
    }

    #[cfg(feature = "dim3")]
    pub fn contact_local_p1(&self, i: usize, scratch_buffer: &js_sys::Float32Array) -> bool {
        unsafe {
            (&(*self.0).points).get(i).map_or(false, |c| {
                let u = c.local_p1.coords;
                scratch_buffer.set_index(0, u.x);
                scratch_buffer.set_index(1, u.y);
                scratch_buffer.set_index(2, u.z);
                true
            })
        }
    }

    #[cfg(feature = "dim2")]
    pub fn contact_local_p2(&self, i: usize, scratch_buffer: &js_sys::Float32Array) -> bool {
        unsafe {
            (&(*self.0).points).get(i).map_or(false, |c| {
                let u = c.local_p2.coords;
                scratch_buffer.set_index(0, u.x);
                scratch_buffer.set_index(1, u.y);
                true
            })
        }
    }

    #[cfg(feature = "dim3")]
    pub fn contact_local_p2(&self, i: usize, scratch_buffer: &js_sys::Float32Array) -> bool {
        unsafe {
            (&(*self.0).points).get(i).map_or(false, |c| {
                let u = c.local_p2.coords;
                scratch_buffer.set_index(0, u.x);
                scratch_buffer.set_index(1, u.y);
                scratch_buffer.set_index(2, u.z);
                true
            })
        }
    }

    pub fn contact_dist(&self, i: usize) -> Real {
        unsafe { (&(*self.0).points).get(i).map(|c| c.dist).unwrap_or(0.0) }
    }

    pub fn contact_fid1(&self, i: usize) -> u32 {
        unsafe { (&(*self.0).points).get(i).map(|c| c.fid1.0).unwrap_or(0) }
    }

    pub fn contact_fid2(&self, i: usize) -> u32 {
        unsafe { (&(*self.0).points).get(i).map(|c| c.fid2.0).unwrap_or(0) }
    }

    pub fn contact_impulse(&self, i: usize) -> Real {
        unsafe {
            (&(*self.0).points)
                .get(i)
                .map(|c| c.data.impulse)
                .unwrap_or(0.0)
        }
    }

    #[cfg(feature = "dim2")]
    pub fn contact_tangent_impulse(&self, i: usize) -> Real {
        unsafe {
            (&(*self.0).points)
                .get(i)
                .map(|c| c.data.tangent_impulse.x)
                .unwrap_or(0.0)
        }
    }

    #[cfg(feature = "dim3")]
    pub fn contact_tangent_impulse_x(&self, i: usize) -> Real {
        unsafe {
            (&(*self.0).points)
                .get(i)
                .map(|c| c.data.tangent_impulse.x)
                .unwrap_or(0.0)
        }
    }

    #[cfg(feature = "dim3")]
    pub fn contact_tangent_impulse_y(&self, i: usize) -> Real {
        unsafe {
            (&(*self.0).points)
                .get(i)
                .map(|c| c.data.tangent_impulse.y)
                .unwrap_or(0.0)
        }
    }

    pub fn num_solver_contacts(&self) -> usize {
        unsafe { (*self.0).data.solver_contacts.len() }
    }

    #[cfg(feature = "dim2")]
    pub fn solver_contact_point(&self, i: usize, scratch_buffer: &js_sys::Float32Array) -> bool {
        unsafe {
            (&(*self.0).data).solver_contacts.get(i).map_or(false, |c| {
                let u = c.point.coords;
                scratch_buffer.set_index(0, u.x);
                scratch_buffer.set_index(1, u.y);
                true
            })
        }
    }

    #[cfg(feature = "dim3")]
    pub fn solver_contact_point(&self, i: usize, scratch_buffer: &js_sys::Float32Array) -> bool {
        unsafe {
            (&(*self.0).data).solver_contacts.get(i).map_or(false, |c| {
                let u = c.point.coords;
                scratch_buffer.set_index(0, u.x);
                scratch_buffer.set_index(1, u.y);
                scratch_buffer.set_index(2, u.z);
                true
            })
        }
    }

    pub fn solver_contact_dist(&self, i: usize) -> Real {
        unsafe {
            (&(*self.0).data)
                .solver_contacts
                .get(i)
                .map(|c| c.dist)
                .unwrap_or(0.0)
        }
    }

    pub fn solver_contact_friction(&self, i: usize) -> Real {
        unsafe { (&(*self.0).data).solver_contacts[i].friction }
    }

    pub fn solver_contact_restitution(&self, i: usize) -> Real {
        unsafe { (&(*self.0).data).solver_contacts[i].restitution }
    }

    #[cfg(feature = "dim2")]
    pub fn solver_contact_tangent_velocity(&self, i: usize, scratch_buffer: &js_sys::Float32Array) {
        unsafe {
            let u = (&(*self.0).data).solver_contacts[i].tangent_velocity;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
        }
    }

    #[cfg(feature = "dim3")]
    pub fn solver_contact_tangent_velocity(&self, i: usize, scratch_buffer: &js_sys::Float32Array) {
        unsafe {
            let u = (&(*self.0).data).solver_contacts[i].tangent_velocity;
            scratch_buffer.set_index(0, u.x);
            scratch_buffer.set_index(1, u.y);
            scratch_buffer.set_index(2, u.z);
        }
    }
}
