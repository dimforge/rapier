use crate::dynamics::RawRigidBodySet;
use crate::math::RawVector;
use crate::utils::{self, FlatHandle};
use rapier::control::PidController;
use rapier::dynamics::AxesMask;
use rapier::math::Vector;
use wasm_bindgen::prelude::*;

#[cfg(feature = "dim3")]
use crate::math::RawRotation;
#[cfg(feature = "dim2")]
use rapier::math::Rotation;

#[wasm_bindgen]
pub struct RawPidController {
    controller: PidController,
}

#[wasm_bindgen]
impl RawPidController {
    #[wasm_bindgen(constructor)]
    pub fn new(kp: f32, ki: f32, kd: f32, axes_mask: u8) -> Self {
        let controller = PidController::new(
            kp,
            ki,
            kd,
            AxesMask::from_bits(axes_mask).unwrap_or(AxesMask::all()),
        );
        Self { controller }
    }

    pub fn set_kp(&mut self, kp: f32, axes: u8) {
        let axes = AxesMask::from_bits(axes).unwrap_or(AxesMask::all());
        if axes.contains(AxesMask::LIN_X) {
            self.controller.pd.lin_kp.x = kp;
        }
        if axes.contains(AxesMask::LIN_Y) {
            self.controller.pd.lin_kp.y = kp;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::LIN_Z) {
            self.controller.pd.lin_kp.z = kp;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::ANG_X) {
            self.controller.pd.ang_kp.x = kp;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::ANG_Y) {
            self.controller.pd.ang_kp.y = kp;
        }
        if axes.contains(AxesMask::ANG_Z) {
            #[cfg(feature = "dim2")]
            {
                self.controller.pd.ang_kp = kp;
            }
            #[cfg(feature = "dim3")]
            {
                self.controller.pd.ang_kp.z = kp;
            }
        }
    }

    pub fn set_ki(&mut self, ki: f32, axes: u8) {
        let axes = AxesMask::from_bits(axes).unwrap_or(AxesMask::all());
        if axes.contains(AxesMask::LIN_X) {
            self.controller.lin_ki.x = ki;
        }
        if axes.contains(AxesMask::LIN_Y) {
            self.controller.lin_ki.y = ki;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::LIN_Z) {
            self.controller.lin_ki.z = ki;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::ANG_X) {
            self.controller.ang_ki.x = ki;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::ANG_Y) {
            self.controller.ang_ki.y = ki;
        }
        if axes.contains(AxesMask::ANG_Z) {
            #[cfg(feature = "dim2")]
            {
                self.controller.ang_ki = ki;
            }
            #[cfg(feature = "dim3")]
            {
                self.controller.ang_ki.z = ki;
            }
        }
    }

    pub fn set_kd(&mut self, kd: f32, axes: u8) {
        let axes = AxesMask::from_bits(axes).unwrap_or(AxesMask::all());
        if axes.contains(AxesMask::LIN_X) {
            self.controller.pd.lin_kd.x = kd;
        }
        if axes.contains(AxesMask::LIN_Y) {
            self.controller.pd.lin_kd.x = kd;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::LIN_Z) {
            self.controller.pd.lin_kd.x = kd;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::ANG_X) {
            self.controller.pd.ang_kd.x = kd;
        }
        #[cfg(feature = "dim3")]
        if axes.contains(AxesMask::ANG_Y) {
            self.controller.pd.ang_kd.y = kd;
        }
        if axes.contains(AxesMask::ANG_Z) {
            #[cfg(feature = "dim2")]
            {
                self.controller.pd.ang_kd = kd;
            }
            #[cfg(feature = "dim3")]
            {
                self.controller.pd.ang_kd.z = kd;
            }
        }
    }

    pub fn set_axes_mask(&mut self, axes_mask: u8) {
        if let Some(mask) = AxesMask::from_bits(axes_mask) {
            self.controller.pd.axes = mask;
        }
    }

    pub fn reset_integrals(&mut self) {
        self.controller.reset_integrals();
    }

    pub fn apply_linear_correction(
        &mut self,
        dt: f32,
        bodies: &mut RawRigidBodySet,
        rb_handle: FlatHandle,
        target_translation: &RawVector,
        target_linvel: &RawVector,
    ) {
        let rb_handle = utils::body_handle(rb_handle);
        let Some(rb) = bodies.0.get_mut(rb_handle) else {
            return;
        };

        let correction = self.controller.linear_rigid_body_correction(
            dt,
            rb,
            target_translation.0.into(),
            target_linvel.0,
        );
        rb.set_linvel(rb.linvel() + correction, true);
    }

    #[cfg(feature = "dim2")]
    pub fn apply_angular_correction(
        &mut self,
        dt: f32,
        bodies: &mut RawRigidBodySet,
        rb_handle: FlatHandle,
        target_rotation: f32,
        target_angvel: f32,
    ) {
        let rb_handle = crate::utils::body_handle(rb_handle);
        let Some(rb) = bodies.0.get_mut(rb_handle) else {
            return;
        };

        let correction = self.controller.angular_rigid_body_correction(
            dt,
            rb,
            Rotation::new(target_rotation),
            target_angvel,
        );
        rb.set_angvel(rb.angvel() + correction, true);
    }

    #[cfg(feature = "dim3")]
    pub fn apply_angular_correction(
        &mut self,
        dt: f32,
        bodies: &mut RawRigidBodySet,
        rb_handle: FlatHandle,
        target_rotation: &RawRotation,
        target_angvel: &RawVector,
    ) {
        let rb_handle = crate::utils::body_handle(rb_handle);
        let Some(rb) = bodies.0.get_mut(rb_handle) else {
            return;
        };

        let correction = self.controller.angular_rigid_body_correction(
            dt,
            rb,
            target_rotation.0,
            target_angvel.0,
        );
        rb.set_angvel(rb.angvel() + correction, true);
    }

    pub fn linear_correction(
        &mut self,
        dt: f32,
        bodies: &RawRigidBodySet,
        rb_handle: FlatHandle,
        target_translation: &RawVector,
        target_linvel: &RawVector,
        scratch_buffer: &js_sys::Float32Array,
    ) {
        let rb_handle = crate::utils::body_handle(rb_handle);
        let Some(rb) = bodies.0.get(rb_handle) else {
            scratch_buffer.set_index(0, 0.0);
            scratch_buffer.set_index(1, 0.0);
            #[cfg(feature = "dim3")]
            scratch_buffer.set_index(2, 0.0);
            return;
        };

        let u: Vector = self.controller.linear_rigid_body_correction(
            dt,
            rb,
            target_translation.0.into(),
            target_linvel.0,
        );
        scratch_buffer.set_index(0, u.x);
        scratch_buffer.set_index(1, u.y);
        #[cfg(feature = "dim3")]
        scratch_buffer.set_index(2, u.z);
    }

    #[cfg(feature = "dim2")]
    pub fn angular_correction(
        &mut self,
        dt: f32,
        bodies: &RawRigidBodySet,
        rb_handle: FlatHandle,
        target_rotation: f32,
        target_angvel: f32,
    ) -> f32 {
        let rb_handle = crate::utils::body_handle(rb_handle);
        let Some(rb) = bodies.0.get(rb_handle) else {
            return 0.0;
        };

        self.controller.angular_rigid_body_correction(
            dt,
            rb,
            Rotation::new(target_rotation),
            target_angvel,
        )
    }

    #[cfg(feature = "dim3")]
    pub fn angular_correction(
        &mut self,
        dt: f32,
        bodies: &RawRigidBodySet,
        rb_handle: FlatHandle,
        target_rotation: &RawRotation,
        target_angvel: &RawVector,
        scratch_buffer: &js_sys::Float32Array,
    ) {
        let rb_handle = crate::utils::body_handle(rb_handle);
        let Some(rb) = bodies.0.get(rb_handle) else {
            scratch_buffer.set_index(0, 0.0);
            scratch_buffer.set_index(1, 0.0);
            scratch_buffer.set_index(2, 0.0);
            return;
        };

        let u: Vector = self.controller.angular_rigid_body_correction(
            dt,
            rb,
            target_rotation.0,
            target_angvel.0,
        );
        scratch_buffer.set_index(0, u.x);
        scratch_buffer.set_index(1, u.y);
        scratch_buffer.set_index(2, u.z);
    }
}
